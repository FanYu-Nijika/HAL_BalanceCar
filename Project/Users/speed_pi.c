#include "speed_pi.h"

static int limit_int(int value, int limit)
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

void speed_pi_init(speed_pi_t *pi, int kp_x100, int ki_x100, int output_limit)
{
    pi->kp_x100 = kp_x100;
    pi->ki_x100 = ki_x100;
    pi->output_limit = output_limit;
    speed_pi_reset(pi);
}

void speed_pi_reset(speed_pi_t *pi)
{
    pi->last_error = 0;
    pi->output = 0;
}

int speed_pi_update(speed_pi_t *pi, int target_speed, int measure_speed)
{
    int error;
    int delta;

    error = target_speed - measure_speed;
    delta = pi->kp_x100 * (error - pi->last_error) / 100;
    delta += pi->ki_x100 * error / 100;

    pi->output += delta;
    pi->output = limit_int(pi->output, pi->output_limit);
    pi->last_error = error;

    return pi->output;
}
