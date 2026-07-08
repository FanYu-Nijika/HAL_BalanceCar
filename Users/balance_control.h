#ifndef __BALANCE_CONTROL_H
#define __BALANCE_CONTROL_H

#include <stdint.h>

extern volatile float Balance_Kp;
extern volatile float Balance_Kd;
extern volatile float Velocity_Kp;
extern volatile float Velocity_Ki;
extern volatile float Balance_Target_Angle;
extern volatile float Balance_Target_Offset;
extern volatile int Balance_Filter_Mode;
extern volatile int Balance_Output_Dir;
extern volatile int Balance_Left_Motor_Dir;
extern volatile int Balance_Right_Motor_Dir;

void balance_control_init(void);
void balance_control_update(void);
void balance_control_mpu_irq(void);
float balance_control_get_angle(void);
float balance_control_get_gyro_rate(void);
int balance_control_get_pwm(void);
int balance_control_get_left_pwm(void);
int balance_control_get_right_pwm(void);
uint8_t balance_control_is_fallen(void);

#endif
