#include "touch.h"

#include <string.h>

#include "driver/touch_sens.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

static const char *TAG = "touch";

/* Touch channel per zone. On the ESP32-S3 the touch channel number is the
 * GPIO number, so these are GPIO4, GPIO5 and GPIO6. */
static const int s_channel[TOUCH_ZONE_COUNT] = { 4, 5, 6 };

/* Charge times and the voltage window the pad swings between, matching the
 * values Espressif's touch examples use for V2 hardware. They suit a plain
 * PCB pad behind a few millimetres of panel. */
#define TOUCH_CHARGE_TIMES          500
#define TOUCH_VOLT_LOW              TOUCH_VOLT_LIM_L_0V5
#define TOUCH_VOLT_HIGH             TOUCH_VOLT_LIM_H_2V2

/* A finger adds capacitance, which raises the smooth reading above the
 * hardware benchmark. The pad counts as touched once that difference exceeds
 * benchmark * this. Espressif's examples start at 1.5% and the documented
 * useful range for V2 is roughly 0.1% to 20%, but those assume a far less
 * responsive pad than this board has. Measured here: benchmark ~70000,
 * deviation 0-17 untouched and 46500-55000 pressed, i.e. a press moves the
 * signal by about 70-80% of benchmark. At the 2% this started on, the trip
 * point sat 40x below a real press, so any common-mode bleed from a second
 * finger cleared it and a released pad stayed active far down its decay
 * tail. 20% puts the trip point near 14000: ~3x clear of the crosstalk
 * residue and ~3x below a genuine press.
 * Raise it if the pads trigger on a near-hand or if touching one zone also
 * trips its neighbours, lower it if a firm press is missed. */
#define TOUCH_THRESHOLD_RATIO       0.20f

/* Benchmark samples averaged during calibration, and the settling delay
 * between them. The hardware needs a few scan periods to re-settle its
 * benchmark after the controller restarts, hence the discarded warm-up. */
#define TOUCH_BENCHMARK_SAMPLES     16
#define TOUCH_WARMUP_MS             200
#define TOUCH_SAMPLE_DELAY_MS       20

/* A pad whose benchmark swings this much across the calibration samples is
 * being touched or is not connected; calibrating against it would bake a bad
 * threshold into NVS. Permille of the benchmark.
 *
 * This started at 10 permille, which was a guess and rejected healthy pads:
 * measured idle jitter on this board reaches ~1150 counts on a benchmark of
 * ~68900, i.e. 16.7 permille. The separation from a real touch is what makes
 * a loose limit safe here - a press moves the signal by 46500-55000 counts,
 * around 700 permille, so 50 still catches a touched or disconnected pad by
 * an order of magnitude while leaving room for normal jitter. */
#define TOUCH_BENCHMARK_MAX_SPREAD_PERMILLE 50

/* The hardware freezes the benchmark of an active channel, so a pad that
 * activates spuriously can never recover on its own: its benchmark stays
 * pinned wherever it was, the deviation against the settled smooth value
 * stays above the threshold, and the pad reads touched forever. Observed in
 * the field with benchmarks stuck ~26000 counts below smooth.
 * A pad held longer than this is therefore assumed stuck rather than
 * pressed, and its benchmark is reset to break the latch. Well above any
 * real press, well below a timescale a user would notice. */
#define TOUCH_STUCK_TIMEOUT_US      (30 * 1000 * 1000)

/* Threshold meaning "never trigger", used before a calibration exists.
 * A channel is active when (smooth - benchmark) exceeds its threshold, so a
 * threshold of zero would make an uncalibrated pad permanently active - and
 * the hardware stops updating the benchmark of an active channel, which
 * would leave the benchmark pinned and make the first calibration read a
 * meaningless value. TOUCH_LL_ACTIVE_THRESH_MAX can never be exceeded. */
#define TOUCH_THRESHOLD_NEVER       0x3FFFFF

/* The internal denoise channel (touch channel 0, not a usable pad) measures
 * the system's own background noise, and the hardware subtracts it from
 * every other channel automatically. Without it, noise that shifts all the
 * channels together - a supply ripple, or the charge injected by touching
 * any one pad - can push the neighbouring pads over their thresholds too,
 * which shows up as one zone triggering the others.
 * The reference capacitance should sit near that of a real pad.
 * Resolution trades sensitivity for suppression: BIT12 suppresses the most
 * but attenuates the real signal the most too, which can leave a genuine
 * press below its threshold. BIT8 is the middle ground - start here, move to
 * BIT10/BIT12 only if crosstalk returns, and to BIT4 if presses are weak. */
#define TOUCH_DENOISE_REF_CAP       TOUCH_DENOISE_CHAN_CAP_9PF
#define TOUCH_DENOISE_RESOLUTION    TOUCH_DENOISE_CHAN_RESOLUTION_BIT8

#define TOUCH_NVS_NAMESPACE         "heating"
/* Versioned: enabling the denoise channel attenuates every reading, so
 * thresholds stored by an earlier build no longer apply. Bumping the key
 * makes those stale values simply not load, forcing a fresh calibration
 * rather than silently using numbers from a different measurement regime. */
#define TOUCH_NVS_KEY               "touch_thresh2"

static touch_sensor_handle_t s_sensor;
static touch_channel_handle_t s_chan[TOUCH_ZONE_COUNT];
static touch_sensor_sample_config_t s_sample_cfg[TOUCH_SAMPLE_CFG_NUM];

/* Applied activation thresholds, mirrored here so they can be stored and
 * logged without reading them back out of the driver. */
static uint32_t s_threshold[TOUCH_ZONE_COUNT];
static bool s_calibrated;

/* Written from the touch ISR callbacks, read by the control task. One byte
 * per zone, so a read never tears; no lock needed for a single flag. */
static volatile bool s_touched[TOUCH_ZONE_COUNT];

/* When each pad last went active, in microseconds, for the stuck-pad
 * watchdog below. Written in the ISR, read by the control task; a 64-bit
 * value is not read atomically on a 32-bit core, so the reader takes a
 * snapshot and tolerates a torn value - the worst case is one watchdog
 * cycle of delay. */
static volatile int64_t s_active_since_us[TOUCH_ZONE_COUNT];

static int zone_of_channel(int chan_id)
{
    for (int z = 0; z < TOUCH_ZONE_COUNT; z++) {
        if (s_channel[z] == chan_id) {
            return z;
        }
    }
    return -1;
}

/* Both callbacks run in ISR context: they may only touch the flag array.
 * They are not marked IRAM_ATTR because CONFIG_TOUCH_ISR_IRAM_SAFE is off;
 * enable that Kconfig option and add the attribute together, never one
 * without the other. */
static bool on_active(touch_sensor_handle_t sens,
                      const touch_active_event_data_t *event,
                      void *user_ctx)
{
    (void)sens; (void)user_ctx;
    int z = zone_of_channel(event->chan_id);
    if (z >= 0) {
        s_touched[z] = true;
        /* esp_timer_get_time() is ISR-safe. */
        s_active_since_us[z] = esp_timer_get_time();
    }
    return false;
}

static bool on_inactive(touch_sensor_handle_t sens,
                        const touch_inactive_event_data_t *event,
                        void *user_ctx)
{
    (void)sens; (void)user_ctx;
    int z = zone_of_channel(event->chan_id);
    if (z >= 0) {
        s_touched[z] = false;
    }
    return false;
}

/* Fills a channel config with the zone's current threshold. The hardware
 * applies the same threshold to every sample configuration.
 * Without a calibration the pad is configured never to trigger, which keeps
 * it inactive and so lets the hardware keep tracking its benchmark - that
 * benchmark is exactly what the first calibration needs to read. */
static void chan_config_for(int zone, touch_channel_config_t *cfg)
{
    *cfg = (touch_channel_config_t){
        .charge_speed = TOUCH_CHARGE_SPEED_7,
        .init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
    };
    uint32_t thresh = s_threshold[zone] ? s_threshold[zone] : TOUCH_THRESHOLD_NEVER;
    for (unsigned s = 0; s < TOUCH_SAMPLE_CFG_NUM; s++) {
        cfg->active_thresh[s] = thresh;
    }
}

/* Applies s_threshold[] to the hardware. The controller must be stopped and
 * disabled first: touch_sensor_reconfig_channel() is only legal in the INIT
 * state. Leaves the controller scanning again on success. */
static esp_err_t thresholds_apply(void)
{
    /* From the first stop onwards the controller is down, so no failure may
     * return early - every path has to fall through to the restart below, or
     * touch sensing would stay dead until the next reboot. */
    esp_err_t err = ESP_OK;

    esp_err_t stop_err = touch_sensor_stop_continuous_scanning(s_sensor);
    if (stop_err != ESP_OK) {
        ESP_LOGE(TAG, "stop scan failed: %s", esp_err_to_name(stop_err));
        err = stop_err;
    }
    esp_err_t dis_err = touch_sensor_disable(s_sensor);
    if (dis_err != ESP_OK) {
        ESP_LOGE(TAG, "disable failed: %s", esp_err_to_name(dis_err));
        if (err == ESP_OK) err = dis_err;
    }

    /* Reconfiguring is only legal in the INIT state. If the controller could
     * not be brought down, skip it rather than issue calls the driver will
     * reject, and go straight to restoring the running state. */
    if (dis_err != ESP_OK) {
        goto restart;
    }

    for (int z = 0; z < TOUCH_ZONE_COUNT; z++) {
        touch_channel_config_t chan_cfg;
        chan_config_for(z, &chan_cfg);
        esp_err_t e = touch_sensor_reconfig_channel(s_chan[z], &chan_cfg);
        if (e != ESP_OK && err == ESP_OK) {
            ESP_LOGE(TAG, "zone %d: reconfig failed: %s", z + 1, esp_err_to_name(e));
            err = e;
        }
    }

restart:
    /* Restarting is subject to the same rule: report a failure, but always
     * attempt the next step, so the controller is never left half-down.
     * Enabling is skipped when the disable never took effect, since the
     * controller is then already enabled. */
    if (dis_err == ESP_OK) {
        esp_err_t en_err = touch_sensor_enable(s_sensor);
        if (en_err != ESP_OK) {
            ESP_LOGE(TAG, "enable failed: %s", esp_err_to_name(en_err));
            if (err == ESP_OK) err = en_err;
        }
    }
    esp_err_t start_err = touch_sensor_start_continuous_scanning(s_sensor);
    if (start_err != ESP_OK) {
        ESP_LOGE(TAG, "start scan failed: %s", esp_err_to_name(start_err));
        if (err == ESP_OK) err = start_err;
    }
    return err;
}

static bool thresholds_load(void)
{
    nvs_handle_t nvs;
    if (nvs_open(TOUCH_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return false;
    }

    size_t len = sizeof(s_threshold);
    esp_err_t err = nvs_get_blob(nvs, TOUCH_NVS_KEY, s_threshold, &len);
    nvs_close(nvs);

    if (err != ESP_OK || len != sizeof(s_threshold)) {
        memset(s_threshold, 0, sizeof(s_threshold));
        return false;
    }
    return true;
}

static void thresholds_store(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(TOUCH_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot open NVS to store calibration: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_blob(nvs, TOUCH_NVS_KEY, s_threshold, sizeof(s_threshold));
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot store calibration: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "calibration stored in NVS");
    }
}

esp_err_t touch_init(void)
{
    s_sample_cfg[0] = (touch_sensor_sample_config_t)
        TOUCH_SENSOR_V2_DEFAULT_SAMPLE_CONFIG(TOUCH_CHARGE_TIMES,
                                              TOUCH_VOLT_LOW, TOUCH_VOLT_HIGH);

    touch_sensor_config_t sens_cfg =
        TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(TOUCH_SAMPLE_CFG_NUM, s_sample_cfg);
    ESP_RETURN_ON_ERROR(touch_sensor_new_controller(&sens_cfg, &s_sensor),
                        TAG, "touch controller");

    /* The filter supplies the smooth data the thresholds work against, and
     * its hysteresis and debounce counters mean a pad state arrives already
     * clean - no software debouncing needed on top.
     *
     * These are deliberately not the DEFAULT_FILTER_CONFIG values. That
     * macro uses TOUCH_BM_IIR_FILTER_4, which makes the benchmark chase the
     * smooth value quickly: a slow press is then partly absorbed into the
     * benchmark and never registers. The header recommends IIR_16, which
     * tracks slow drift (temperature, humidity) while leaving a press
     * standing out against it. */
    touch_sensor_filter_config_t filter_cfg = {
        .benchmark = {
            .filter_mode = TOUCH_BM_IIR_FILTER_16,
            .jitter_step = 4,       /* unused unless filter_mode is JITTER */
            /* Noise below this is not allowed to move the benchmark.
             * Range 0-4; 2 suppresses more coupling than the default 1. */
            .denoise_lvl = 2,
        },
        .data = {
            .smooth_filter = TOUCH_SMOOTH_IIR_FILTER_4,
            /* Added to the threshold on activation and subtracted on
             * release, so it widens the dead band around the trip point. */
            .active_hysteresis = 2,
            /* Consecutive readings past the threshold before the state
             * flips - the hardware debounce that replaces a software one. */
            .debounce_cnt = 2,
        },
    };
    ESP_RETURN_ON_ERROR(touch_sensor_config_filter(s_sensor, &filter_cfg),
                        TAG, "touch filter");

    /* Must be configured while the controller is disabled, which it still is
     * at this point. */
    touch_denoise_chan_config_t denoise_cfg = {
        .charge_speed = TOUCH_CHARGE_SPEED_7,
        .init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        .ref_cap = TOUCH_DENOISE_REF_CAP,
        .resolution = TOUCH_DENOISE_RESOLUTION,
    };
    ESP_RETURN_ON_ERROR(touch_sensor_config_denoise_channel(s_sensor, &denoise_cfg),
                        TAG, "touch denoise");

    /* A stored calibration is applied as the channels are created; without
     * one the thresholds stay zero, which would make every pad read as
     * permanently active, so the reported state is forced false until a
     * calibrate command arrives. */
    s_calibrated = thresholds_load();

    for (int z = 0; z < TOUCH_ZONE_COUNT; z++) {
        touch_channel_config_t chan_cfg;
        chan_config_for(z, &chan_cfg);
        ESP_RETURN_ON_ERROR(touch_sensor_new_channel(s_sensor, s_channel[z],
                                                     &chan_cfg, &s_chan[z]),
                            TAG, "touch channel %d", z);
        if (s_calibrated) {
            ESP_LOGI(TAG, "touch zone %d on GPIO%d (touch ch %d), threshold %lu",
                     z + 1, s_channel[z], s_channel[z], (unsigned long)s_threshold[z]);
        } else {
            ESP_LOGI(TAG, "touch zone %d on GPIO%d (touch ch %d), uncalibrated",
                     z + 1, s_channel[z], s_channel[z]);
        }
    }

    touch_event_callbacks_t cbs = {
        .on_active = on_active,
        .on_inactive = on_inactive,
    };
    ESP_RETURN_ON_ERROR(touch_sensor_register_callbacks(s_sensor, &cbs, NULL),
                        TAG, "touch callbacks");

    ESP_RETURN_ON_ERROR(touch_sensor_enable(s_sensor), TAG, "touch enable");
    ESP_RETURN_ON_ERROR(touch_sensor_start_continuous_scanning(s_sensor), TAG, "touch scan");

    if (!s_calibrated) {
        ESP_LOGW(TAG, "no stored calibration - publish \"calibrate\" on the "
                      "command topic with the pads untouched");
    }
    return ESP_OK;
}

esp_err_t touch_calibrate(void)
{
    uint64_t sum[TOUCH_ZONE_COUNT] = {0};
    uint32_t lo[TOUCH_ZONE_COUNT];
    uint32_t hi[TOUCH_ZONE_COUNT] = {0};

    for (int z = 0; z < TOUCH_ZONE_COUNT; z++) {
        lo[z] = UINT32_MAX;
    }

    ESP_LOGI(TAG, "calibrating, keep the pads untouched");

    /* Force each benchmark back to the pad's current raw reading. Without
     * this a pad that is currently active would keep a stale benchmark,
     * because the hardware freezes the benchmark of an active channel.
     * This call is legal while scanning, so the controller keeps running. */
    for (int z = 0; z < TOUCH_ZONE_COUNT; z++) {
        touch_chan_benchmark_config_t bm_cfg = { .do_reset = true };
        ESP_RETURN_ON_ERROR(touch_channel_config_benchmark(s_chan[z], &bm_cfg),
                            TAG, "reset benchmark zone %d", z);
    }

    /* Let the reset benchmark re-settle through the filter before it is
     * trusted. */
    vTaskDelay(pdMS_TO_TICKS(TOUCH_WARMUP_MS));

    for (int i = 0; i < TOUCH_BENCHMARK_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(TOUCH_SAMPLE_DELAY_MS));
        for (int z = 0; z < TOUCH_ZONE_COUNT; z++) {
            uint32_t bench = 0;
            ESP_RETURN_ON_ERROR(touch_channel_read_data(s_chan[z],
                                    TOUCH_CHAN_DATA_TYPE_BENCHMARK, &bench),
                                TAG, "read zone %d", z);
            sum[z] += bench;
            if (bench < lo[z]) lo[z] = bench;
            if (bench > hi[z]) hi[z] = bench;
        }
    }

    /* Validate every pad before committing any of them, so one bad pad does
     * not leave half the zones on a fresh threshold and half on an old one. */
    uint32_t fresh[TOUCH_ZONE_COUNT];
    for (int z = 0; z < TOUCH_ZONE_COUNT; z++) {
        uint32_t bench = (uint32_t)(sum[z] / TOUCH_BENCHMARK_SAMPLES);

        if (bench == 0) {
            ESP_LOGE(TAG, "zone %d: pad reads zero, not connected?", z + 1);
            return ESP_ERR_INVALID_STATE;
        }

        uint32_t spread = hi[z] - lo[z];
        if (spread > (uint64_t)bench * TOUCH_BENCHMARK_MAX_SPREAD_PERMILLE / 1000) {
            ESP_LOGE(TAG, "zone %d: benchmark unstable (spread %lu on %lu), "
                          "pad touched during calibration?",
                     z + 1, (unsigned long)spread, (unsigned long)bench);
            return ESP_ERR_INVALID_STATE;
        }

        fresh[z] = (uint32_t)(bench * TOUCH_THRESHOLD_RATIO);
        if (fresh[z] == 0) {
            ESP_LOGE(TAG, "zone %d: benchmark %lu too small for a usable threshold",
                     z + 1, (unsigned long)bench);
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGI(TAG, "zone %d: benchmark %lu, threshold %lu",
                 z + 1, (unsigned long)bench, (unsigned long)fresh[z]);
    }

    uint32_t previous[TOUCH_ZONE_COUNT];
    memcpy(previous, s_threshold, sizeof(previous));
    memcpy(s_threshold, fresh, sizeof(s_threshold));

    esp_err_t err = thresholds_apply();
    if (err != ESP_OK) {
        /* Put the previous thresholds back rather than keeping values the
         * hardware never accepted, and try to restore them so a failed
         * calibration leaves the pads working as they did before. */
        memcpy(s_threshold, previous, sizeof(s_threshold));
        if (thresholds_apply() != ESP_OK) {
            ESP_LOGE(TAG, "could not restore the previous thresholds");
        }
        for (int z = 0; z < TOUCH_ZONE_COUNT; z++) {
            s_touched[z] = false;
        }
        return err;
    }

    s_calibrated = true;
    thresholds_store();
    return ESP_OK;
}

bool touch_is_calibrated(void)
{
    return s_calibrated;
}

void touch_log_diagnostics(void)
{
    for (int z = 0; z < TOUCH_ZONE_COUNT; z++) {
        uint32_t smooth = 0;
        uint32_t bench = 0;
        if (touch_channel_read_data(s_chan[z], TOUCH_CHAN_DATA_TYPE_SMOOTH, &smooth) != ESP_OK ||
            touch_channel_read_data(s_chan[z], TOUCH_CHAN_DATA_TYPE_BENCHMARK, &bench) != ESP_OK) {
            ESP_LOGW(TAG, "zone %d: cannot read touch data", z + 1);
            continue;
        }

        /* A negative deviation is normal for an untouched pad; report it as
         * zero rather than wrapping the unsigned subtraction. */
        uint32_t deviation = smooth > bench ? smooth - bench : 0;

        ESP_LOGI(TAG, "zone %d: benchmark=%lu smooth=%lu deviation=%lu "
                      "threshold=%lu active=%d",
                 z + 1, (unsigned long)bench, (unsigned long)smooth,
                 (unsigned long)deviation,
                 (unsigned long)(s_calibrated ? s_threshold[z] : 0),
                 s_touched[z] ? 1 : 0);
    }
    if (!s_calibrated) {
        ESP_LOGW(TAG, "not calibrated - pads never trigger until \"calibrate\" is sent");
    }
}

bool touch_is_touched(int zone)
{
    if (zone < 0 || zone >= TOUCH_ZONE_COUNT || !s_calibrated) {
        return false;
    }
    return s_touched[zone];
}

void touch_service(void)
{
    if (!s_calibrated) {
        return;
    }

    int64_t now = esp_timer_get_time();
    for (int z = 0; z < TOUCH_ZONE_COUNT; z++) {
        if (!s_touched[z]) {
            continue;
        }
        int64_t since = s_active_since_us[z];
        if (since == 0 || now - since < TOUCH_STUCK_TIMEOUT_US) {
            continue;
        }

        /* Reset the benchmark to the pad's current smooth value. That clears
         * the deviation, so the channel goes inactive and the hardware
         * resumes tracking the benchmark normally. Legal while scanning. */
        touch_chan_benchmark_config_t bm_cfg = { .do_reset = true };
        esp_err_t err = touch_channel_config_benchmark(s_chan[z], &bm_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "zone %d: stuck, benchmark reset failed: %s",
                     z + 1, esp_err_to_name(err));
            /* Back off a full timeout rather than retrying every cycle. */
            s_active_since_us[z] = now;
            continue;
        }

        ESP_LOGW(TAG, "zone %d: active for over %d s with no release, "
                      "benchmark reset", z + 1,
                 (int)(TOUCH_STUCK_TIMEOUT_US / 1000000));

        /* The reset makes the hardware deactivate the channel, which fires
         * on_inactive and clears the flag. Clear it here too so a missed
         * event cannot leave the zone latched. */
        s_touched[z] = false;
        s_active_since_us[z] = 0;
    }
}
