#ifndef __SPEED_PI_H
#define __SPEED_PI_H

typedef struct
{
    int kp_x100;
    int ki_x100;
    int last_error;
    int output;
    int output_limit;
} speed_pi_t;

void speed_pi_init(speed_pi_t *pi, int kp_x100, int ki_x100, int output_limit);
void speed_pi_reset(speed_pi_t *pi);
int speed_pi_update(speed_pi_t *pi, int target_speed, int measure_speed);

#endif
