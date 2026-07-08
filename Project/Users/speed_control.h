#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

void speed_control_init(void);
void speed_control_set_target(int left_mm_s, int right_mm_s);
void speed_control_update(void);
void speed_control_stop(void);

int speed_control_get_left_speed(void);
int speed_control_get_right_speed(void);

#endif
