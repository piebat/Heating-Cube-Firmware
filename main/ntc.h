#pragma once

#include "esp_err.h"

/* Three NTC 10K thermistors, each read through its own voltage divider:
 *   zone 0 -> GPIO1 (ADC1_CH0)
 *   zone 1 -> GPIO2 (ADC1_CH1)
 *   zone 2 -> GPIO3 (ADC1_CH2)
 * All channels share one ADC unit; every zone keeps its own filter state. */

#define NTC_ZONE_COUNT      3

esp_err_t ntc_init(void);

/* Returns the temperature of zone (0..NTC_ZONE_COUNT-1) in degrees Celsius.
 * On error, or for an out-of-range zone, returns NAN. */
float ntc_read_celsius(int zone);
