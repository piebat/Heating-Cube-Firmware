#include "ntc.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "ntc";

/* Divider, one per zone: 3.3V --- R_SERIES --- GPIOn --- NTC --- GND
 * With the NTC on the low side the ADC voltage falls as the temperature
 * rises. Flip NTC_ON_LOW_SIDE if your board wires it the other way. */
#define NTC_ON_LOW_SIDE     1
#define NTC_R_SERIES        10000.0f   /* ohm  */
#define NTC_R_NOMINAL       10000.0f   /* ohm at NTC_T_NOMINAL */
#define NTC_T_NOMINAL       25.0f      /* degC */
#define NTC_BETA            3950.0f    /* B25/85, check your datasheet */
#define NTC_VSUPPLY_MV      3300.0f

#define NTC_ADC_UNIT        ADC_UNIT_1
#define NTC_ADC_ATTEN       ADC_ATTEN_DB_12
/* Odd count so the median is a real sample. Kept small because the control
 * loop runs every 100 ms; noise is handled by the EMA in ntc_read_celsius. */
#define NTC_SAMPLES         5

/* Exponential moving average over successive reads. At a 100 ms loop this
 * gives a time constant of roughly half a second - enough to keep the
 * derivative term quiet without hiding a real temperature ramp. */
#define NTC_EMA_ALPHA       0.2f

/* ADC1 channel per zone, in zone order. */
static const adc_channel_t s_channel[NTC_ZONE_COUNT] = {
    ADC_CHANNEL_0,  /* GPIO1 */
    ADC_CHANNEL_1,  /* GPIO2 */
    ADC_CHANNEL_2,  /* GPIO3 */
};
static const int s_gpio[NTC_ZONE_COUNT] = { 1, 2, 3 };

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali[NTC_ZONE_COUNT];
static bool s_cali_ok[NTC_ZONE_COUNT];
static float s_filtered[NTC_ZONE_COUNT];

static bool cali_init(adc_channel_t chan, adc_cali_handle_t *out)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id = NTC_ADC_UNIT,
        .chan = chan,
        .atten = NTC_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cfg, out) == ESP_OK) {
        return true;
    }
#else
    (void)chan;
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg2 = {
        .unit_id = NTC_ADC_UNIT,
        .atten = NTC_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cfg2, out) == ESP_OK) {
        return true;
    }
#endif
    return false;
}

esp_err_t ntc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = NTC_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc), TAG, "adc unit");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = NTC_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    for (int z = 0; z < NTC_ZONE_COUNT; z++) {
        ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, s_channel[z], &chan_cfg),
                            TAG, "adc channel %d", z);

        s_cali_ok[z] = cali_init(s_channel[z], &s_cali[z]);
        if (!s_cali_ok[z]) {
            ESP_LOGW(TAG, "zone %d: no ADC calibration scheme, falling back to raw scaling", z);
        }
        s_filtered[z] = NAN;
        ESP_LOGI(TAG, "NTC zone %d ready on GPIO%d (ADC1_CH%d)", z, s_gpio[z], (int)s_channel[z]);
    }
    return ESP_OK;
}

/* Median-of-samples keeps a single noisy conversion from moving the PID. */
static int cmp_int(const void *a, const void *b)
{
    return (*(const int *)a) - (*(const int *)b);
}

float ntc_read_celsius(int zone)
{
    if (zone < 0 || zone >= NTC_ZONE_COUNT) {
        return NAN;
    }

    int raw[NTC_SAMPLES];

    for (int i = 0; i < NTC_SAMPLES; i++) {
        if (adc_oneshot_read(s_adc, s_channel[zone], &raw[i]) != ESP_OK) {
            ESP_LOGE(TAG, "zone %d: adc read failed", zone);
            return NAN;
        }
    }
    qsort(raw, NTC_SAMPLES, sizeof(raw[0]), cmp_int);
    int median = raw[NTC_SAMPLES / 2];

    float mv;
    if (s_cali_ok[zone]) {
        int v = 0;
        if (adc_cali_raw_to_voltage(s_cali[zone], median, &v) != ESP_OK) {
            return NAN;
        }
        mv = (float)v;
    } else {
        /* 12-bit default width, ~3.1V full scale at 12 dB. */
        mv = (float)median * 3100.0f / 4095.0f;
    }

    /* An open or shorted sensor lands at the rails; report it instead of
     * feeding a bogus temperature to the PID. */
    if (mv < 50.0f || mv > NTC_VSUPPLY_MV - 50.0f) {
        ESP_LOGE(TAG, "zone %d: NTC out of range (%.0f mV) - open or shorted?", zone, mv);
        return NAN;
    }

    /* Solve the divider for the thermistor resistance. */
    float r_ntc;
#if NTC_ON_LOW_SIDE
    r_ntc = NTC_R_SERIES * mv / (NTC_VSUPPLY_MV - mv);
#else
    r_ntc = NTC_R_SERIES * (NTC_VSUPPLY_MV - mv) / mv;
#endif

    /* Beta equation: 1/T = 1/T0 + (1/B) * ln(R/R0), T in kelvin. */
    const float t0_k = NTC_T_NOMINAL + 273.15f;
    float inv_t = 1.0f / t0_k + logf(r_ntc / NTC_R_NOMINAL) / NTC_BETA;
    float celsius = 1.0f / inv_t - 273.15f;

    if (isnan(s_filtered[zone])) {
        s_filtered[zone] = celsius;   /* first good read seeds the filter */
    } else {
        s_filtered[zone] += NTC_EMA_ALPHA * (celsius - s_filtered[zone]);
    }
    return s_filtered[zone];
}
