#ifndef __PID_H
#define __PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float integral_limit;
    float output_limit;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
void pid_reset(pid_t *pid);
int pid_update(pid_t *pid, int target, int measure);

#endif
