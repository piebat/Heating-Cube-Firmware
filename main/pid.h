#pragma once

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float out_min;
    float out_max;

    /* internal state */
    float integral;
    float prev_input;
    bool  has_prev;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float out_min, float out_max);

/* Clears the integral term and the derivative history. */
void pid_reset(pid_t *pid);

/* Runs one PID step. dt is in seconds and must be > 0.
 * The derivative is taken on the measurement (not the error) so that a
 * setpoint change does not produce a derivative kick. The integral is
 * clamped against the output limits to avoid wind-up. */
float pid_compute(pid_t *pid, float setpoint, float input, float dt);
