#include "pid.h"

static float limit_float(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

int pid_update(pid_t *pid, int target, int measure)
{
    float error;
    float derivative;
    float output;

    error = (float)(target - measure);
    pid->integral += error;
    pid->integral = limit_float(pid->integral, pid->integral_limit);

    derivative = error - pid->last_error;
    pid->last_error = error;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = limit_float(output, pid->output_limit);

    return (int)output;
}
