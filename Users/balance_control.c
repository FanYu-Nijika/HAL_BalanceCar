#include "balance_control.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050_fast.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>

/*
 * firmware_COM4_verify runs the fast balance path from MPU6050 PB9 EXTI.
 * The verified firmware exposes DMP, Kalman and C F modes. This project keeps
 * the raw-register Kalman path and mirrors the verified cascade PID output.
 */
#define BALANCE_DT 0.005f
#define BALANCE_FILTER_ALPHA 0.98f
#define BALANCE_FILTER_KALMAN 2
#define BALANCE_FILTER_COMPLEMENTARY 3

#define BALANCE_KALMAN_Q_ANGLE 0.001f
#define BALANCE_KALMAN_Q_BIAS 0.003f
#define BALANCE_KALMAN_R_MEASURE 0.02f

#define BALANCE_ZERO_SAMPLES 100
#define BALANCE_ZERO_MAX_TRIES 240
#define BALANCE_FALL_ANGLE 40.0f

#define BALANCE_PARAM_SCALE 100.0f
#define SPEED_FILTER_KEEP 0.84f
#define SPEED_FILTER_NEW 0.16f

/*
 * Keep the velocity integral below the motor saturation range.
 * With Velocity_Ki / BALANCE_PARAM_SCALE = 0.02, this gives about 1000 PWM
 * of maximum integral contribution instead of allowing the integral alone to
 * saturate the +/-6900 motor output.
 */
#define SPEED_INTEGRAL_LIMIT 50000.0f

/*
 * firmware_COM4_verify startup RAM defaults:
 * 0x20000080 = 32000.0f, 0x20000084 = 120.0f,
 * 0x20000088 = 410.0f,   0x2000008c = 2.0f.
 * The firmware divides these menu-style parameters by 100 in the PID formula.
 */
volatile float Balance_Kp = 32000.0f;
volatile float Balance_Kd = 120.0f;
volatile float Velocity_Kp = 410.0f;
volatile float Velocity_Ki = 2.0f;

/* Verified firmware default target angle is 0. Use offset for mechanical trim. */
volatile float Balance_Target_Angle = 0.0f;

/*
 * Manual fine tuning for mechanical balance point.
 * Try -1.0f, -0.5f, 0.0f, 0.5f, 1.0f if the car always tends to fall
 * toward the same direction.
 */
volatile float Balance_Target_Offset = 0.0f;
volatile int Balance_Filter_Mode = BALANCE_FILTER_KALMAN;

/*
 * Direction tuning.
 *
 * When the car body falls forward, the wheels should rotate forward
 * to chase the body.
 *
 * If the whole direction is opposite, change Balance_Output_Dir.
 * If only one wheel is opposite, change the corresponding motor dir.
 */
volatile int Balance_Output_Dir = 1;
volatile int Balance_Left_Motor_Dir = 1;
volatile int Balance_Right_Motor_Dir = 1;

static float g_balance_angle = 0.0f;
static float g_accel_angle = 0.0f;
static float g_balance_gyro_rate = 0.0f;
static float g_gyro_zero_rate = 0.0f;
static float g_kalman_bias = 0.0f;
static float g_kalman_p00 = 0.0f;
static float g_kalman_p01 = 0.0f;
static float g_kalman_p10 = 0.0f;
static float g_kalman_p11 = 0.0f;
static float g_speed_filter = 0.0f;
static float g_speed_integral = 0.0f;

static int g_balance_pwm = 0;
static int g_velocity_pwm = 0;
static int g_turn_pwm = 0;
static int g_left_pwm = 0;
static int g_right_pwm = 0;
static int g_left_encoder_count = 0;
static int g_right_encoder_count = 0;
static uint8_t g_is_fallen = 1;

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

static float limit_float(float value, float limit)
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

static void balance_speed_reset(void)
{
    g_speed_filter = 0.0f;
    g_speed_integral = 0.0f;
}

static void balance_stop(void)
{
    g_balance_pwm = 0;
    g_velocity_pwm = 0;
    g_turn_pwm = 0;
    g_left_pwm = 0;
    g_right_pwm = 0;
    balance_speed_reset();
    motor_stop_all();
}

static void balance_filter_reset(float angle)
{
    g_balance_angle = angle;
    g_accel_angle = angle;
    g_balance_gyro_rate = 0.0f;
    g_kalman_bias = 0.0f;
    g_kalman_p00 = 0.0f;
    g_kalman_p01 = 0.0f;
    g_kalman_p10 = 0.0f;
    g_kalman_p11 = 0.0f;
}

static float balance_kalman_update(float accel_angle, float gyro_rate)
{
    float rate;
    float p00_temp;
    float p01_temp;
    float s;
    float k0;
    float k1;
    float y;

    rate = gyro_rate - g_kalman_bias;
    g_balance_angle += BALANCE_DT * rate;

    g_kalman_p00 += BALANCE_DT * (BALANCE_DT * g_kalman_p11 - g_kalman_p01 - g_kalman_p10 + BALANCE_KALMAN_Q_ANGLE);
    g_kalman_p01 -= BALANCE_DT * g_kalman_p11;
    g_kalman_p10 -= BALANCE_DT * g_kalman_p11;
    g_kalman_p11 += BALANCE_KALMAN_Q_BIAS * BALANCE_DT;

    s = g_kalman_p00 + BALANCE_KALMAN_R_MEASURE;
    if (s == 0.0f)
    {
        return g_balance_angle;
    }

    k0 = g_kalman_p00 / s;
    k1 = g_kalman_p10 / s;
    y = accel_angle - g_balance_angle;

    g_balance_angle += k0 * y;
    g_kalman_bias += k1 * y;

    p00_temp = g_kalman_p00;
    p01_temp = g_kalman_p01;
    g_kalman_p00 -= k0 * p00_temp;
    g_kalman_p01 -= k0 * p01_temp;
    g_kalman_p10 -= k1 * p00_temp;
    g_kalman_p11 -= k1 * p01_temp;

    return g_balance_angle;
}

static void balance_calibrate_zero_angle(void)
{
    mpu6050_fast_data_t data;
    float angle_sum = 0.0f;
    float gyro_sum = 0.0f;
    int samples = 0;
    int tries = 0;

    while (samples < BALANCE_ZERO_SAMPLES && tries < BALANCE_ZERO_MAX_TRIES)
    {
        tries++;

        if (mpu6050_fast_read(&data))
        {
            angle_sum += mpu6050_fast_get_pitch_accel(&data);
            gyro_sum += mpu6050_fast_get_pitch_gyro_rate(&data);
            samples++;
        }

        HAL_Delay(5);
    }

    if (samples > 0)
    {
        g_gyro_zero_rate = gyro_sum / (float)samples;
        balance_filter_reset(angle_sum / (float)samples);
    }
    else
    {
        g_gyro_zero_rate = 0.0f;
        balance_filter_reset(Balance_Target_Angle);
    }
}

void balance_control_init(void)
{
    if (mpu6050_fast_init())
    {
        balance_calibrate_zero_angle();

        g_is_fallen = 0;

        printf("[BAL] MPU6050 fast init ok, target_x100=%d, gyro_zero_x100=%d\r\n",
               (int)(Balance_Target_Angle * 100.0f),
               (int)(g_gyro_zero_rate * 100.0f));
    }
    else
    {
        g_is_fallen = 1;
        printf("[BAL] MPU6050 fast init failed, motor stopped\r\n");
    }

    balance_stop();
}

void balance_control_update(void)
{
    mpu6050_fast_data_t data;
    float target_angle;
    float angle_output;
    float speed_least;
    int final_left;
    int final_right;

    /*
     * firmware_COM4_verify reads and clears TIM4 first, then TIM8 and negates it
     * at the start of the PB9 EXTI handler before attitude/control calculation.
     */
    g_left_encoder_count = encoder_read_left_count();
    g_right_encoder_count = -encoder_read_right_count();

    if (!mpu6050_fast_read(&data))
    {
        g_is_fallen = 1;
        balance_stop();

        return;
    }

    g_accel_angle = mpu6050_fast_get_pitch_accel(&data);
    g_balance_gyro_rate = mpu6050_fast_get_pitch_gyro_rate(&data) - g_gyro_zero_rate;

    if (Balance_Filter_Mode == BALANCE_FILTER_KALMAN)
    {
        g_balance_angle = balance_kalman_update(g_accel_angle, g_balance_gyro_rate);
    }
    else
    {
        g_balance_angle = BALANCE_FILTER_ALPHA
                        * (g_balance_angle + g_balance_gyro_rate * BALANCE_DT)
                        + (1.0f - BALANCE_FILTER_ALPHA) * g_accel_angle;
    }

    target_angle = Balance_Target_Angle + Balance_Target_Offset;

    if (abs_float(g_balance_angle - target_angle) > BALANCE_FALL_ANGLE)
    {
        g_is_fallen = 1;
        balance_stop();
        return;
    }

    g_is_fallen = 0;

    angle_output = (g_balance_angle - target_angle) * (Balance_Kp / BALANCE_PARAM_SCALE)
                 + g_balance_gyro_rate * (Balance_Kd / BALANCE_PARAM_SCALE);
    g_balance_pwm = (int)(angle_output * (float)Balance_Output_Dir);

    speed_least = -(float)(g_left_encoder_count + g_right_encoder_count);
    g_speed_filter = g_speed_filter * SPEED_FILTER_KEEP + speed_least * SPEED_FILTER_NEW;
    g_speed_integral += g_speed_filter;
    g_speed_integral = limit_float(g_speed_integral, SPEED_INTEGRAL_LIMIT);

    g_velocity_pwm = (int)(-g_speed_filter * (Velocity_Kp / BALANCE_PARAM_SCALE)
                         - g_speed_integral * (Velocity_Ki / BALANCE_PARAM_SCALE));
    g_turn_pwm = 0;

    final_left = g_balance_pwm + g_velocity_pwm + g_turn_pwm;
    final_right = g_balance_pwm + g_velocity_pwm - g_turn_pwm;
    g_left_pwm = limit_int(final_left, MOTOR_PWM_MAX);
    g_right_pwm = limit_int(final_right, MOTOR_PWM_MAX);

    motor_set_left(g_left_pwm * Balance_Left_Motor_Dir);
    motor_set_right(g_right_pwm * Balance_Right_Motor_Dir);
}

void balance_control_mpu_irq(void)
{
    balance_control_update();
}

float balance_control_get_angle(void)
{
    return g_balance_angle;
}

float balance_control_get_gyro_rate(void)
{
    return g_balance_gyro_rate;
}

int balance_control_get_pwm(void)
{
    return g_balance_pwm;
}

int balance_control_get_left_pwm(void)
{
    return g_left_pwm;
}

int balance_control_get_right_pwm(void)
{
    return g_right_pwm;
}

uint8_t balance_control_is_fallen(void)
{
    return g_is_fallen;
}
