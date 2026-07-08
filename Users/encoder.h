#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f1xx_hal.h"

#define ENCODER_PULSE_PER_ROUND 63000
#define WHEEL_DIAMETER_MM 80

void encoder_init(void);
int encoder_read_left_count(void);
int encoder_read_right_count(void);
int encoder_count_to_speed_mm_s(int count);
int encoder_count_to_speed_mm_s_by_period(int count, int sample_period_ms);

#endif
