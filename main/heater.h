#pragma once

#include "esp_err.h"

/* Three DC heating elements, each driven through its own MOSFET gate:
 *   zone 0 -> GPIO7   (pairs with the NTC on GPIO1)
 *   zone 1 -> GPIO8   (pairs with the NTC on GPIO2)
 *   zone 2 -> GPIO9   (pairs with the NTC on GPIO3)
 * The PID duty (0..1) is applied as LEDC hardware PWM at HEATER_FREQ_HZ;
 * all three outputs share one LEDC timer, one channel each. */

#define HEATER_ZONE_COUNT   3

esp_err_t heater_init(void);

/* duty is clamped to [0, 1]. Out-of-range zones are ignored. */
void heater_set_duty(int zone, float duty);

float heater_get_duty(int zone);

/* Cuts one zone's output immediately and forces its duty to zero. */
void heater_off(int zone);

/* Cuts every zone. */
void heater_off_all(void);
