#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"

#define MOTOR_PWM_MAX 6900

void motor_init(void);
void motor_set_left(int pwm);
void motor_set_right(int pwm);
void motor_stop_all(void);

#endif
