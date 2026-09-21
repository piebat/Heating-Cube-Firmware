#include <math.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "heater.h"
#include "ntc.h"
#include "pid.h"
#include "touch.h"
#include "wifi_mqtt.h"

#include <string.h>

static const char *TAG = "main";

/* Three independent zones, each an NTC and a touch pad paired with the
 * heater they belong to:
 *   zone 0: NTC GPIO1, touch GPIO4 -> heater GPIO7
 *   zone 1: NTC GPIO2, touch GPIO5 -> heater GPIO8
 *   zone 2: NTC GPIO3, touch GPIO6 -> heater GPIO9
 * The zone index is shared by ntc.c, touch.c, heater.c and the MQTT
 * payloads. */
#define ZONE_COUNT          3
_Static_assert(ZONE_COUNT == NTC_ZONE_COUNT, "zone count must match ntc.h");
_Static_assert(ZONE_COUNT == HEATER_ZONE_COUNT, "zone count must match heater.h");
_Static_assert(ZONE_COUNT == TOUCH_ZONE_COUNT, "zone count must match touch.h");
_Static_assert(ZONE_COUNT == WIFI_MQTT_ZONE_COUNT, "zone count must match wifi_mqtt.h");

#define CONTROL_PERIOD_MS   100
#define STATUS_EVERY_N      20      /* publish every 2 s */
#define SETPOINT_MIN_C      0.0f
#define SETPOINT_MAX_C      ((float)CONFIG_HEATING_MAX_TEMP_C)
#define MAX_TEMP_C          ((float)CONFIG_HEATING_MAX_TEMP_C)
#define MAX_BAD_READS       10      /* 1 s of bad reads before cut-out */

/* Starting gains for a small thermal mass: the element responds quickly,
 * so kp carries most of the work and kd stays small - a large kd would
 * just amplify ADC noise into gate chatter at this loop rate.
 * Tune on the real load: raise kp until it oscillates, back off to about
 * half, then add ki slowly until the steady-state offset closes. */
#define PID_KP              2.0f
#define PID_KI              0.5f
#define PID_KD              0.5f

static pid_t s_pid[ZONE_COUNT];
static portMUX_TYPE s_setpoint_lock = portMUX_INITIALIZER_UNLOCKED;
static float s_setpoint_c[ZONE_COUNT];          /* guarded by s_setpoint_lock */
static bool  s_setpoint_changed[ZONE_COUNT];

/* Runs on the MQTT client task, not the control task. */
static void on_setpoint(int zone, float value)
{
    if (zone < 0 || zone >= ZONE_COUNT) {
        return;
    }
    if (isnan(value) || value < SETPOINT_MIN_C || value > SETPOINT_MAX_C) {
        ESP_LOGW(TAG, "zone %d setpoint %.2f out of range [%.1f, %.1f], ignored",
                 zone + 1, value, SETPOINT_MIN_C, SETPOINT_MAX_C);
        return;
    }
    portENTER_CRITICAL(&s_setpoint_lock);
    s_setpoint_c[zone] = value;
    s_setpoint_changed[zone] = true;
    portEXIT_CRITICAL(&s_setpoint_lock);
}

/* Set by the MQTT task, serviced by the control task. Touch calibration
 * blocks for the better part of a second and must not stall the MQTT
 * client, so the command only raises this flag. */
static volatile bool s_calibrate_requested;

/* Runs on the MQTT client task, not the control task. */
static void on_command(const char *command)
{
    if (strcmp(command, "calibrate") == 0) {
        s_calibrate_requested = true;
    } else if (strcmp(command, "diag") == 0) {
        /* Only reads registers and logs, so it is safe to run here. */
        touch_log_diagnostics();
    } else {
        ESP_LOGW(TAG, "unknown command \"%s\"", command);
    }
}

/* A touch-gated heating session.
 *
 * IDLE   -> a non-zero setpoint on "commands" arms it
 * ARMED  -> heating starts once every pad is touched at the same time
 * RUNNING-> heating stops the moment any pad is released, or a zero
 *           setpoint arrives; the result is published and it returns to
 *           IDLE. A session is consumed by running, so the next one needs a
 *           fresh "commands" message.
 *
 * Only the control task changes the state. The MQTT task writes a pending
 * request, which the control task picks up at the top of its next cycle,
 * so the state machine never runs from two tasks at once. */
typedef enum {
    SESSION_IDLE,
    SESSION_ARMED,
    SESSION_RUNNING,
} session_state_t;

static portMUX_TYPE s_session_lock = portMUX_INITIALIZER_UNLOCKED;
static bool  s_session_pending;                  /* guarded by s_session_lock */
static float s_session_pending_setpoint;
static bool  s_session_pending_channels[ZONE_COUNT];

/* Runs on the MQTT client task, not the control task. */
static void on_session(float setpoint_c, const bool *channels)
{
    if (isnan(setpoint_c) || setpoint_c < SETPOINT_MIN_C || setpoint_c > SETPOINT_MAX_C) {
        ESP_LOGW(TAG, "session setpoint %.2f out of range [%.1f, %.1f], ignored",
                 setpoint_c, SETPOINT_MIN_C, SETPOINT_MAX_C);
        return;
    }

    portENTER_CRITICAL(&s_session_lock);
    s_session_pending = true;
    s_session_pending_setpoint = setpoint_c;
    for (int z = 0; z < ZONE_COUNT; z++) {
        s_session_pending_channels[z] = channels[z];
    }
    portEXIT_CRITICAL(&s_session_lock);
}

/* Session state, touched only by the control task. */
static session_state_t s_session_state = SESSION_IDLE;
static float s_session_setpoint;
static bool  s_session_channels[ZONE_COUNT];
static int64_t s_session_start_us;
static float s_session_max_c[ZONE_COUNT];
static double s_session_sum_c[ZONE_COUNT];
static int   s_session_samples[ZONE_COUNT];

static void session_publish(const bool *touched)
{
    wifi_mqtt_values_t v = {0};

    v.touch_time_ms = s_session_start_us
        ? (int)((esp_timer_get_time() - s_session_start_us) / 1000)
        : 0;
    v.temp_setpoint_c = s_session_setpoint;

    for (int z = 0; z < ZONE_COUNT; z++) {
        v.touched[z] = touched[z] ? 1 : 0;
        v.channels[z] = s_session_channels[z] ? 1 : 0;
        /* A zone with no valid reading for the whole session has no
         * statistics to report; NAN is published as null. */
        v.temps_max_c[z] = s_session_samples[z] ? s_session_max_c[z] : NAN;
        v.temps_avg_c[z] = s_session_samples[z]
            ? (float)(s_session_sum_c[z] / s_session_samples[z])
            : NAN;
    }

    ESP_LOGI(TAG, "session ended after %d ms", v.touch_time_ms);
    wifi_mqtt_publish_values(&v);
}

/* Advances the session one control cycle. May rewrite setpoints[] for the
 * channels the session owns, and sets changed[] when it does, so the PID is
 * reset on the transition exactly as a manual setpoint change would. */
static void session_step(const bool *touched, const float *temps,
                         float *setpoints, bool *changed)
{
    /* Pick up a request from the MQTT task. A zero setpoint cancels; a
     * non-zero one arms a fresh session, replacing any already in flight. */
    bool  pending = false;
    float pending_setpoint = 0.0f;
    bool  pending_channels[ZONE_COUNT];

    portENTER_CRITICAL(&s_session_lock);
    if (s_session_pending) {
        pending = true;
        s_session_pending = false;
        pending_setpoint = s_session_pending_setpoint;
        for (int z = 0; z < ZONE_COUNT; z++) {
            pending_channels[z] = s_session_pending_channels[z];
        }
    }
    portEXIT_CRITICAL(&s_session_lock);

    if (pending) {
        if (pending_setpoint <= SETPOINT_MIN_C) {
            /* Cancel. A running session still reports what it did. */
            if (s_session_state == SESSION_RUNNING) {
                session_publish(touched);
            } else if (s_session_state == SESSION_ARMED) {
                ESP_LOGI(TAG, "session cancelled before it started");
            }
            s_session_state = SESSION_IDLE;
        } else {
            s_session_setpoint = pending_setpoint;
            for (int z = 0; z < ZONE_COUNT; z++) {
                s_session_channels[z] = pending_channels[z];
                s_session_max_c[z] = -INFINITY;
                s_session_sum_c[z] = 0.0;
                s_session_samples[z] = 0;
            }
            s_session_start_us = 0;
            s_session_state = SESSION_ARMED;
            ESP_LOGI(TAG, "session armed at %.2f C, waiting for all pads",
                     s_session_setpoint);
        }
    }

    if (s_session_state == SESSION_IDLE) {
        return;
    }

    bool all_touched = true;
    bool any_released = false;
    for (int z = 0; z < ZONE_COUNT; z++) {
        if (!touched[z]) {
            all_touched = false;
            any_released = true;
        }
    }

    if (s_session_state == SESSION_ARMED) {
        if (all_touched) {
            s_session_state = SESSION_RUNNING;
            s_session_start_us = esp_timer_get_time();
            ESP_LOGI(TAG, "session started");
        }
    } else if (any_released) {
        /* Stop on the first release, before the statistics take this cycle:
         * the pads are no longer all held, so this cycle is outside the
         * session. */
        session_publish(touched);
        s_session_state = SESSION_IDLE;
    }

    /* Accumulate only while running, so the statistics cover the heating
     * period and nothing else. */
    if (s_session_state == SESSION_RUNNING) {
        for (int z = 0; z < ZONE_COUNT; z++) {
            if (isnan(temps[z])) {
                continue;
            }
            if (temps[z] > s_session_max_c[z]) {
                s_session_max_c[z] = temps[z];
            }
            s_session_sum_c[z] += temps[z];
            s_session_samples[z]++;
        }
    }

    /* Drive the session's channels while it holds them. On the way out the
     * channels are released and forced off once, so the zones return to
     * manual control cold rather than inheriting the session's setpoint. */
    if (s_session_state == SESSION_IDLE) {
        for (int z = 0; z < ZONE_COUNT; z++) {
            if (!s_session_channels[z]) {
                continue;
            }
            s_session_channels[z] = false;
            setpoints[z] = 0.0f;
            changed[z] = true;
            /* The manual setpoint is zeroed too, so a later manual message
             * is what brings the zone back rather than a stale value. */
            portENTER_CRITICAL(&s_setpoint_lock);
            s_setpoint_c[z] = 0.0f;
            portEXIT_CRITICAL(&s_setpoint_lock);
        }
        return;
    }

    for (int z = 0; z < ZONE_COUNT; z++) {
        if (!s_session_channels[z]) {
            continue;
        }
        float target = (s_session_state == SESSION_RUNNING) ? s_session_setpoint : 0.0f;
        if (setpoints[z] != target) {
            changed[z] = true;
        }
        setpoints[z] = target;
    }
}

static void control_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_us = esp_timer_get_time();
    int cycle = 0;
    int bad_reads[ZONE_COUNT] = {0};
    float temps[ZONE_COUNT];
    float setpoints[ZONE_COUNT];
    float duties[ZONE_COUNT];
    bool  touched[ZONE_COUNT];

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

        if (s_calibrate_requested) {
            s_calibrate_requested = false;
            /* The heaters keep their current duty: calibration only reads
             * the touch peripheral and takes well under a second, so the
             * loop resumes before the thermal state moves appreciably. */
            esp_err_t cal = touch_calibrate();
            if (cal != ESP_OK) {
                ESP_LOGE(TAG, "touch calibration failed: %s", esp_err_to_name(cal));
            } else {
                ESP_LOGI(TAG, "touch calibration complete");
            }
            /* Calibration blocked for many control periods; resynchronise so
             * vTaskDelayUntil does not try to catch up with a burst of
             * immediate iterations. */
            last_wake = xTaskGetTickCount();
            last_us = esp_timer_get_time();
            continue;
        }

        /* Breaks the latch if a pad has activated spuriously; see touch.h. */
        touch_service();

        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) / 1e6f;
        last_us = now_us;

        bool changed[ZONE_COUNT];
        portENTER_CRITICAL(&s_setpoint_lock);
        for (int z = 0; z < ZONE_COUNT; z++) {
            setpoints[z] = s_setpoint_c[z];
            changed[z] = s_setpoint_changed[z];
            s_setpoint_changed[z] = false;
        }
        portEXIT_CRITICAL(&s_setpoint_lock);

        /* Touch and temperature are read for every zone first: the session
         * needs both before the PID runs, since it decides the setpoints
         * the PID will act on and accumulates the temperature statistics
         * it will publish. */
        for (int z = 0; z < ZONE_COUNT; z++) {
            touched[z] = touch_is_touched(z);
            temps[z] = ntc_read_celsius(z);
        }

        /* The session owns the setpoints of the channels it runs, so it is
         * resolved before the PID sees them: an armed or stopped session
         * holds its channels at zero, a running one drives them to the
         * session setpoint. Zones outside the session keep whatever the
         * manual setpoint topic last set. */
        session_step(touched, temps, setpoints, changed);

        for (int z = 0; z < ZONE_COUNT; z++) {
            if (changed[z]) {
                /* Drop the accumulated integral so an old wind-up does not
                 * overshoot against the new target. */
                pid_reset(&s_pid[z]);
            }

            float temp = temps[z];

            if (isnan(temp)) {
                if (++bad_reads[z] >= MAX_BAD_READS) {
                    ESP_LOGE(TAG, "zone %d: no valid temperature, heater off", z + 1);
                    heater_off(z);
                    pid_reset(&s_pid[z]);
                }
                duties[z] = heater_get_duty(z);
                continue;
            }
            bad_reads[z] = 0;

            if (temp >= MAX_TEMP_C) {
                ESP_LOGE(TAG, "zone %d: over-temperature %.1f C, heater off", z + 1, temp);
                heater_off(z);
                pid_reset(&s_pid[z]);
            } else if (setpoints[z] <= SETPOINT_MIN_C) {
                /* No setpoint received yet, or explicitly switched off. */
                heater_off(z);
            } else {
                float duty = pid_compute(&s_pid[z], setpoints[z], temp, dt);
                heater_set_duty(z, duty);
            }
            duties[z] = heater_get_duty(z);
        }

        if (++cycle >= STATUS_EVERY_N) {
            cycle = 0;
            for (int z = 0; z < ZONE_COUNT; z++) {
                ESP_LOGI(TAG, "zone %d: T=%.2f C  SP=%.2f C  duty=%.1f%%  touch=%d",
                         z + 1, temps[z], setpoints[z], duties[z] * 100.0f,
                         touched[z] ? 1 : 0);
            }
            wifi_mqtt_publish_status(temps, setpoints, duties, touched);
        }
    }
}

void app_main(void)
{
    /* Heaters first: every gate must be driven low before anything can fail. */
    ESP_ERROR_CHECK(heater_init());

    /* NVS holds the stored touch calibration, so it must be up before
     * touch_init() looks for it. wifi_mqtt_start() relies on it too. */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    ESP_ERROR_CHECK(ntc_init());
    ESP_ERROR_CHECK(touch_init());

    for (int z = 0; z < ZONE_COUNT; z++) {
        pid_init(&s_pid[z], PID_KP, PID_KI, PID_KD, 0.0f, 1.0f);
        s_setpoint_c[z] = 0.0f;
    }

    esp_err_t err = wifi_mqtt_start(on_setpoint, on_command, on_session);
    if (err != ESP_OK) {
        /* Keep regulating on the last setpoints rather than rebooting into
         * an uncontrolled state; the MQTT client retries on its own. */
        ESP_LOGE(TAG, "network start failed: %s", esp_err_to_name(err));
    }

    xTaskCreate(control_task, "control", 4096, NULL, 5, NULL);
}
