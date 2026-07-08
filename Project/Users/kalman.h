#ifndef __KALMAN_H
#define __KALMAN_H

typedef struct
{
    float angle;
    float bias;
    float rate;
    float p00;
    float p01;
    float p10;
    float p11;
} kalman_t;

void kalman_init(kalman_t *kalman, float angle);
float kalman_update(kalman_t *kalman, float new_angle, float new_rate, float dt);

#endif
