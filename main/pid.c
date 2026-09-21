#include "pid.h"

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pid_init(pid_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid_reset(pid);
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_input = 0.0f;
    pid->has_prev = false;
}

float pid_compute(pid_t *pid, float setpoint, float input, float dt)
{
    if (dt <= 0.0f) {
        return clampf(pid->kp * (setpoint - input), pid->out_min, pid->out_max);
    }

    float error = setpoint - input;

    /* Integrate, then clamp the accumulated term itself so that a long
     * saturation does not leave the controller unable to come back down. */
    pid->integral += pid->ki * error * dt;
    pid->integral = clampf(pid->integral, pid->out_min, pid->out_max);

    /* Derivative on measurement: -d(input)/dt instead of d(error)/dt. */
    float derivative = 0.0f;
    if (pid->has_prev) {
        derivative = -pid->kd * (input - pid->prev_input) / dt;
    }
    pid->prev_input = input;
    pid->has_prev = true;

    return clampf(pid->kp * error + pid->integral + derivative,
                  pid->out_min, pid->out_max);
}
