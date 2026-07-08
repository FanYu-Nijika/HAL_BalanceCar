#include "experiment.h"
#include "encoder.h"
#include "motor.h"
#include "speed_pi.h"
#include <stdio.h>

#define OPEN_LOOP_STAGE_MS 3000
#define CLOSED_LOOP_STAGE_MS 8000
#define CLOSED_LOOP_TARGET_MM_S 200

/* If direction is reversed, change only these four values. */
#define LEFT_MOTOR_DIR  1
#define RIGHT_MOTOR_DIR -1
#define LEFT_ENCODER_DIR  -1
#define RIGHT_ENCODER_DIR 1

/* phase: 1=open loop, 2=closed loop, 3=manual encoder count */
static const int g_open_pwm_list[] = {0, 15, 25, 35, 50, 70};
static const int g_pi_kp_list[] = {80, 140, 220};
static const int g_pi_ki_list[] = {8, 16, 28};

static speed_pi_t g_left_pi;
static speed_pi_t g_right_pi;
static int g_phase = 1;
static int g_stage = 0;
static int g_stage_time_ms = 0;
static int g_left_speed = 0;
static int g_right_speed = 0;
static int g_left_pwm = 0;
static int g_right_pwm = 0;
static int g_left_total_count = 0;
static int g_right_total_count = 0;

static int array_count_open(void)
{
    return sizeof(g_open_pwm_list) / sizeof(g_open_pwm_list[0]);
}

static int array_count_pi(void)
{
    return sizeof(g_pi_kp_list) / sizeof(g_pi_kp_list[0]);
}

static void print_vofa_line(int target, int left_count, int right_count)
{
    printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
           g_phase,
           g_stage,
           target,
           g_left_speed,
           g_right_speed,
           g_left_pwm,
           g_right_pwm,
           left_count,
           right_count,
           g_left_total_count,
           g_right_total_count);
}

static void update_open_loop(int left_count, int right_count)
{
    int pwm;

    pwm = g_open_pwm_list[g_stage];
    g_left_pwm = pwm;
    g_right_pwm = pwm;
    motor_set_left(g_left_pwm * LEFT_MOTOR_DIR);
    motor_set_right(g_right_pwm * RIGHT_MOTOR_DIR);

    print_vofa_line(0, left_count, right_count);

    if (g_stage_time_ms >= OPEN_LOOP_STAGE_MS)
    {
        g_stage++;
        g_stage_time_ms = 0;
        if (g_stage >= array_count_open())
        {
            g_phase = 2;
            g_stage = 0;
            speed_pi_init(&g_left_pi, g_pi_kp_list[0], g_pi_ki_list[0], MOTOR_PWM_MAX);
            speed_pi_init(&g_right_pi, g_pi_kp_list[0], g_pi_ki_list[0], MOTOR_PWM_MAX);
        }
    }
}

static void update_closed_loop(int left_count, int right_count)
{
    if (g_stage_time_ms == 10)
    {
        speed_pi_init(&g_left_pi, g_pi_kp_list[g_stage], g_pi_ki_list[g_stage], MOTOR_PWM_MAX);
        speed_pi_init(&g_right_pi, g_pi_kp_list[g_stage], g_pi_ki_list[g_stage], MOTOR_PWM_MAX);
    }

    g_left_pwm = speed_pi_update(&g_left_pi, CLOSED_LOOP_TARGET_MM_S, g_left_speed);
    g_right_pwm = speed_pi_update(&g_right_pi, CLOSED_LOOP_TARGET_MM_S, g_right_speed);
    motor_set_left(g_left_pwm * LEFT_MOTOR_DIR);
    motor_set_right(g_right_pwm * RIGHT_MOTOR_DIR);

    print_vofa_line(CLOSED_LOOP_TARGET_MM_S, left_count, right_count);

    if (g_stage_time_ms >= CLOSED_LOOP_STAGE_MS)
    {
        g_stage++;
        g_stage_time_ms = 0;
        if (g_stage >= array_count_pi())
        {
            g_phase = 3;
            g_stage = 0;
            motor_stop_all();
            g_left_pwm = 0;
            g_right_pwm = 0;
            g_left_total_count = 0;
            g_right_total_count = 0;
        }
    }
}

static void update_manual_encoder_check(int left_count, int right_count)
{
    motor_stop_all();
    g_left_pwm = 0;
    g_right_pwm = 0;
    print_vofa_line(ENCODER_PULSE_PER_ROUND, left_count, right_count);
}

void experiment_init(void)
{
    motor_init();
    encoder_init();
}

void experiment_update_10ms(void)
{
    int left_count;
    int right_count;

    left_count = encoder_read_left_count() * LEFT_ENCODER_DIR;
    right_count = encoder_read_right_count() * RIGHT_ENCODER_DIR;
    g_left_total_count += left_count;
    g_right_total_count += right_count;
    g_left_speed = encoder_count_to_speed_mm_s(left_count);
    g_right_speed = encoder_count_to_speed_mm_s(right_count);
    g_stage_time_ms += 10;

    if (g_phase == 1)
    {
        update_open_loop(left_count, right_count);
    }
    else if (g_phase == 2)
    {
        update_closed_loop(left_count, right_count);
    }
    else
    {
        update_manual_encoder_check(left_count, right_count);
    }
}
