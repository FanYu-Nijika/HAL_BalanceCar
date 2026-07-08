#include "balance_control.h"
#include "kalman.h"
#include "motor.h"
#include "mpu6050.h"

#define BALANCE_DT 0.005f
#define BALANCE_FALL_ANGLE 30.0f

#define BALANCE_PWM_LIMIT 6500
#define BALANCE_PWM_DEADBAND 900
#define BALANCE_PWM_SLEW_STEP 500

/* Use these if one wheel is visibly stronger than the other. */
#define BALANCE_LEFT_PWM_GAIN  1.0f
#define BALANCE_RIGHT_PWM_GAIN 1.0f

volatile float g_balance_angle_kp = 380.0f;
volatile float g_balance_gyro_kd = 18.0f;
volatile float g_balance_target_angle_watch = 0.0f;
volatile int g_balance_output_dir = 1;
volatile int g_balance_left_motor_dir = 1;
volatile int g_balance_right_motor_dir = -1;

static kalman_t g_pitch_kalman;
static float g_target_angle = 0.0f;
static float g_pitch = 0.0f;
static int g_balance_pwm = 0;
static int g_left_pwm = 0;
static int g_right_pwm = 0;
static int g_last_output_pwm = 0;
static int g_is_fallen = 1;

static float abs_float(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }
    return value;
}

static int limit_int(int value, int limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static int scale_pwm(int pwm, float gain)
{
    float scaled;

    scaled = (float)pwm * gain;
    if (scaled > 0.0f)
    {
        return (int)(scaled + 0.5f);
    }
    return (int)(scaled - 0.5f);
}

static int apply_pwm_slew(int target_pwm)
{
    if (target_pwm > g_last_output_pwm + BALANCE_PWM_SLEW_STEP)
    {
        target_pwm = g_last_output_pwm + BALANCE_PWM_SLEW_STEP;
    }
    else if (target_pwm < g_last_output_pwm - BALANCE_PWM_SLEW_STEP)
    {
        target_pwm = g_last_output_pwm - BALANCE_PWM_SLEW_STEP;
    }

    g_last_output_pwm = target_pwm;
    return target_pwm;
}

static int float_to_pwm_with_deadband(float value)
{
    int pwm;

    if (value > -1.0f && value < 1.0f)
    {
        return apply_pwm_slew(0);
    }

    if (value > 0.0f)
    {
        pwm = (int)(value + 0.5f);
    }
    else
    {
        pwm = (int)(value - 0.5f);
    }

    if (pwm > 0 && pwm < BALANCE_PWM_DEADBAND)
    {
        pwm = BALANCE_PWM_DEADBAND;
    }
    else if (pwm < 0 && pwm > -BALANCE_PWM_DEADBAND)
    {
        pwm = -BALANCE_PWM_DEADBAND;
    }

    pwm = limit_int(pwm, BALANCE_PWM_LIMIT);
    return apply_pwm_slew(pwm);
}

static float balance_pd_output(float target_angle, float pitch, float gyro_rate)
{
    float angle_error;

    angle_error = target_angle - pitch;
    return g_balance_angle_kp * angle_error - g_balance_gyro_kd * gyro_rate;
}

static void output_balance_pwm_float(float pid_output)
{
    int pwm;

    pid_output = pid_output * (float)g_balance_output_dir;
    pwm = float_to_pwm_with_deadband(pid_output);

    g_balance_pwm = pwm;
    g_left_pwm = scale_pwm(pwm, BALANCE_LEFT_PWM_GAIN);
    g_right_pwm = scale_pwm(pwm, BALANCE_RIGHT_PWM_GAIN);

    g_left_pwm = limit_int(g_left_pwm, BALANCE_PWM_LIMIT);
    g_right_pwm = limit_int(g_right_pwm, BALANCE_PWM_LIMIT);

    motor_set_left(g_left_pwm * g_balance_left_motor_dir);
    motor_set_right(g_right_pwm * g_balance_right_motor_dir);
}

void balance_control_init(void)
{
    mpu6050_data_t data;
    float accel_angle;

    mpu6050_init();

    accel_angle = 0.0f;
    if (mpu6050_read_data(&data))
    {
        accel_angle = mpu6050_get_pitch_accel(&data);
    }

    kalman_init(&g_pitch_kalman, accel_angle);
    g_pitch = accel_angle;
    g_balance_pwm = 0;
    g_left_pwm = 0;
    g_right_pwm = 0;
    g_is_fallen = 0;
}

void balance_control_set_target_angle(float angle)
{
    g_target_angle = angle;
    g_balance_target_angle_watch = angle;
}

void balance_control_set_speed_target(float speed)
{
    (void)speed;
}

void balance_control_set_turn_target(float turn)
{
    (void)turn;
}

void balance_control_set_angle_pid(float kp, float kd)
{
    g_balance_angle_kp = kp;
    g_balance_gyro_kd = kd;
}

void balance_control_set_output_dir(int dir)
{
    if (dir < 0)
    {
        g_balance_output_dir = -1;
    }
    else
    {
        g_balance_output_dir = 1;
    }
}

void balance_control_update(void)
{
    mpu6050_data_t data;
    float accel_angle;
    float gyro_rate;

    if (!mpu6050_read_data(&data))
    {
        motor_stop_all();
        g_balance_pwm = 0;
        g_last_output_pwm = 0;
        g_is_fallen = 1;
        return;
    }

    accel_angle = mpu6050_get_pitch_accel(&data);
    gyro_rate = mpu6050_get_pitch_gyro_rate(&data);
    g_pitch = kalman_update(&g_pitch_kalman, accel_angle, gyro_rate, BALANCE_DT);
    g_target_angle = g_balance_target_angle_watch;

    if (abs_float(g_pitch) > BALANCE_FALL_ANGLE)
    {
        motor_stop_all();
        g_balance_pwm = 0;
        g_last_output_pwm = 0;
        g_is_fallen = 1;
        return;
    }

    g_is_fallen = 0;
    output_balance_pwm_float(balance_pd_output(g_target_angle, g_pitch, gyro_rate));
}

float balance_control_get_pitch(void)
{
    return g_pitch;
}

int balance_control_get_pwm(void)
{
    return g_balance_pwm;
}

int balance_control_is_fallen(void)
{
    return g_is_fallen;
}
