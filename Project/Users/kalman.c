#include "kalman.h"

#define KALMAN_Q_ANGLE   0.001f
#define KALMAN_Q_BIAS    0.003f
#define KALMAN_R_MEASURE 0.03f

void kalman_init(kalman_t *kalman, float angle)
{
    kalman->angle = angle;
    kalman->bias = 0.0f;
    kalman->rate = 0.0f;
    kalman->p00 = 0.0f;
    kalman->p01 = 0.0f;
    kalman->p10 = 0.0f;
    kalman->p11 = 0.0f;
}

float kalman_update(kalman_t *kalman, float new_angle, float new_rate, float dt)
{
    float s;
    float k0;
    float k1;
    float y;
    float p00_temp;
    float p01_temp;

    kalman->rate = new_rate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    kalman->p00 += dt * (dt * kalman->p11 - kalman->p01 - kalman->p10 + KALMAN_Q_ANGLE);
    kalman->p01 -= dt * kalman->p11;
    kalman->p10 -= dt * kalman->p11;
    kalman->p11 += KALMAN_Q_BIAS * dt;

    s = kalman->p00 + KALMAN_R_MEASURE;
    k0 = kalman->p00 / s;
    k1 = kalman->p10 / s;

    y = new_angle - kalman->angle;
    kalman->angle += k0 * y;
    kalman->bias += k1 * y;

    p00_temp = kalman->p00;
    p01_temp = kalman->p01;

    kalman->p00 -= k0 * p00_temp;
    kalman->p01 -= k0 * p01_temp;
    kalman->p10 -= k1 * p00_temp;
    kalman->p11 -= k1 * p01_temp;

    return kalman->angle;
}
