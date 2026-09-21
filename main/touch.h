#pragma once

#include <stdbool.h>
#include "esp_err.h"

/* Three capacitive touch pads, one per heating zone:
 *   zone 0 -> GPIO4 (touch channel 4)
 *   zone 1 -> GPIO5 (touch channel 5)
 *   zone 2 -> GPIO6 (touch channel 6)
 * Zone indices match ntc.h and heater.h. */

#define TOUCH_ZONE_COUNT    3

esp_err_t touch_init(void);

/* True while the pad of the given zone reads as touched.
 * An out-of-range zone, or a zone that has never been calibrated, reads
 * false. */
bool touch_is_touched(int zone);

/* Re-measures the untouched baseline of every pad and derives the activation
 * thresholds from it, then stores them in NVS so they survive a reboot.
 * The pads must not be touched while this runs - it takes about
 * TOUCH_CALIBRATE_MS to complete and blocks the calling task.
 *
 * Returns ESP_ERR_INVALID_STATE if a pad reads as already active (which
 * usually means it was being touched), leaving the previous thresholds in
 * place. */
esp_err_t touch_calibrate(void);

/* True once every pad has a threshold, whether loaded from NVS at init or
 * produced by touch_calibrate(). */
bool touch_is_calibrated(void);

/* Housekeeping, to be called periodically from the control loop.
 *
 * The touch hardware freezes the benchmark of an active channel, so a pad
 * that activates spuriously stays latched forever: its benchmark cannot
 * move while it reads touched, and it cannot stop reading touched while the
 * benchmark is stale. This detects a pad held implausibly long and resets
 * its benchmark to break that deadlock.
 *
 * Cheap, and safe to call every cycle. */
void touch_service(void);

/* Logs the live benchmark, smooth reading, deviation and threshold of every
 * pad at INFO level. Use it to see how far a real press actually moves a pad
 * relative to its threshold when tuning sensitivity. */
void touch_log_diagnostics(void);
