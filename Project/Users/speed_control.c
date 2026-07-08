#include "speed_control.h"
#include "encoder.h"
#include "motor.h"
#include "speed_pi.h"
#include <stdio.h>

/* ---------- hardware direction calibration ---------- */
#define LEFT_MOTOR_DIR    1
#define RIGHT_MOTOR_DIR  -1
#define LEFT_ENCODER_DIR  1
#define RIGHT_ENCODER_DIR 1

/* ---------- PI default parameters (x100, from experiment) ---------- */
#define SPEED_KP_X100      140
#define SPEED_KI_X100      16
#define SPEED_PI_LIMIT     MOTOR_PWM_MAX

/* ---------- internal state ---------- */
static speed_pi_t g_spd_left_pi;
static speed_pi_t g_spd_right_pi;
static int g_target_left  = 0;
static int g_target_right = 0;
static int g_left_speed   = 0;
static int g_right_speed  = 0;

/* ---------- public API ---------- */

void speed_control_init(void)
{
    motor_init();
    encoder_init();

    speed_pi_init(&g_spd_left_pi,  SPEED_KP_X100, SPEED_KI_X100, SPEED_PI_LIMIT);
    speed_pi_init(&g_spd_right_pi, SPEED_KP_X100, SPEED_KI_X100, SPEED_PI_LIMIT);
}

void speed_control_set_target(int left_mm_s, int right_mm_s)
{
    g_target_left  = left_mm_s;
    g_target_right = right_mm_s;
}

void speed_control_update(void)
{
    int left_count;
    int right_count;
    int left_pwm;
    int right_pwm;

    /* 1. read encoder counts, convert to speed (mm/s, fixed 10 ms window) */
    left_count  = encoder_read_left_count()  * LEFT_ENCODER_DIR;
    right_count = encoder_read_right_count() * RIGHT_ENCODER_DIR;

    g_left_speed  = encoder_count_to_speed_mm_s(left_count);
    g_right_speed = encoder_count_to_speed_mm_s(right_count);

    /* 2. PI speed control */
    left_pwm  = speed_pi_update(&g_spd_left_pi,  g_target_left,  g_left_speed);
    right_pwm = speed_pi_update(&g_spd_right_pi, g_target_right, g_right_speed);

    /* 3. motor output (with direction calibration) */
    motor_set_left(left_pwm   * LEFT_MOTOR_DIR);
    motor_set_right(right_pwm * RIGHT_MOTOR_DIR);

    /* 4. VOFA+ telemetry */
    // printf("%d,%d,%d,%d,%d,%d\r\n",
    //        g_target_left,
    //        g_left_speed,
    //        g_right_speed,
    //        left_pwm,
    //        right_pwm,
    //        g_target_right);
}

void speed_control_stop(void)
{
    motor_stop_all();
    speed_pi_reset(&g_spd_left_pi);
    speed_pi_reset(&g_spd_right_pi);
}

int speed_control_get_left_speed(void)
{
    return g_left_speed;
}

int speed_control_get_right_speed(void)
{
    return g_right_speed;
}
