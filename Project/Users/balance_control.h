#ifndef __BALANCE_CONTROL_H
#define __BALANCE_CONTROL_H

void balance_control_init(void);
void balance_control_set_target_angle(float angle);
void balance_control_set_speed_target(float speed);
void balance_control_set_turn_target(float turn);
void balance_control_set_angle_pid(float kp, float kd);
void balance_control_set_output_dir(int dir);
void balance_control_update(void);

float balance_control_get_pitch(void);
int balance_control_get_pwm(void);
int balance_control_is_fallen(void);

extern volatile float g_balance_angle_kp;
extern volatile float g_balance_gyro_kd;
extern volatile float g_balance_target_angle_watch;
extern volatile int g_balance_output_dir;
extern volatile int g_balance_left_motor_dir;
extern volatile int g_balance_right_motor_dir;

#endif
