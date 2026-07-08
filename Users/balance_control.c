#include "balance_control.h"
#include "motor.h"
#include "mpu6050_fast.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>

/*
 * firmware_COM4_verify runs the fast balance path from MPU6050 PB9 EXTI.
 * The verified firmware exposes DMP, Kalman and C F modes; this project keeps
 * the angle loop only and defaults to the raw-register Kalman path.
 */
#define BALANCE_DT 0.01f
#define BALANCE_FILTER_ALPHA 0.98f
#define BALANCE_FILTER_KALMAN 2
#define BALANCE_FILTER_COMPLEMENTARY 3

#define BALANCE_KALMAN_Q_ANGLE 0.001f
#define BALANCE_KALMAN_Q_BIAS 0.003f
#define BALANCE_KALMAN_R_MEASURE 0.02f

#define BALANCE_ZERO_SAMPLES 100
#define BALANCE_ZERO_MAX_TRIES 240
#define BALANCE_FALL_ANGLE 35.0f

/*
 * Your motor dead zone:
 * PWM around 55 is still static, around 60 can start moving.
 */
#define BALANCE_PWM_LIMIT 70
#define BALANCE_PWM_START 40

/*
 * This is not PWM deadband.
 * It is the zero area of the raw PD output.
 *
 * If the raw PD output is too small, treat it as noise and output 0.
 * Once it exceeds this threshold, jump over motor static friction
 * and start from BALANCE_PWM_START.
 */
#define BALANCE_OUTPUT_ZERO 4.0f
#define BALANCE_PWM_SCALE 0.8f

/*
 * Tuning entry.
 *
 * Suggested starting point:
 * - If the car is soft and falls slowly: increase Balance_Kp.
 * - If the car swings back and forth: increase Balance_Kd.
 * - If the car shakes quickly: decrease Balance_Kd.
 */
volatile float Balance_Kp = 4.0f;
volatile float Balance_Kd = 0.35f;

/*
 * Automatically calibrated during boot.
 * Hold the car at its real mechanical balance point when powering on.
 */
volatile float Balance_Target_Angle = 1.0f;

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
volatile int Balance_Output_Dir = -1;
volatile int Balance_Left_Motor_Dir = 1;
volatile int Balance_Right_Motor_Dir = -1;

static float g_balance_angle = 0.0f;
static float g_accel_angle = 0.0f;
static float g_balance_gyro_rate = 0.0f;
static float g_gyro_zero_rate = 0.0f;
static float g_kalman_bias = 0.0f;
static float g_kalman_p00 = 0.0f;
static float g_kalman_p01 = 0.0f;
static float g_kalman_p10 = 0.0f;
static float g_kalman_p11 = 0.0f;

static int g_balance_pwm = 0;
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

/*
 * Convert raw PD output to motor PWM.
 *
 * Example:
 * raw output < 4      -> PWM 0
 * raw output around 4 -> PWM 60
 * raw output larger   -> PWM gradually increases to 90
 *
 * This avoids wasting control output in the motor dead zone.
 */
static int float_to_pwm(float value)
{
    int sign;
    int pwm;
    float abs_value;

    if (value > -BALANCE_OUTPUT_ZERO && value < BALANCE_OUTPUT_ZERO)
    {
        return 0;
    }

    if (value > 0.0f)
    {
        sign = 1;
        abs_value = value;
    }
    else
    {
        sign = -1;
        abs_value = -value;
    }

    pwm = BALANCE_PWM_START
        + (int)((abs_value - BALANCE_OUTPUT_ZERO) * BALANCE_PWM_SCALE + 0.5f);

    pwm = limit_int(pwm, BALANCE_PWM_LIMIT);

    return sign * pwm;
}

static void balance_stop(void)
{
    g_balance_pwm = 0;
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
        Balance_Target_Angle = angle_sum / (float)samples;
        g_gyro_zero_rate = gyro_sum / (float)samples;
    }
    else
    {
        Balance_Target_Angle = 0.0f;
        g_gyro_zero_rate = 0.0f;
    }

    balance_filter_reset(Balance_Target_Angle);
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
    float angle_error;
    float raw_output;

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

    /*
     * Angle loop PD:
     *
     * P term:
     *   angle_error = target - actual
     *
     * D term:
     *   gyro rate is used as angular velocity feedback.
     */
    angle_error = target_angle - g_balance_angle;
    raw_output = Balance_Kp * angle_error - Balance_Kd * g_balance_gyro_rate;

    g_balance_pwm = float_to_pwm(raw_output * (float)Balance_Output_Dir);

    motor_set_left(g_balance_pwm * Balance_Left_Motor_Dir);
    motor_set_right(g_balance_pwm * Balance_Right_Motor_Dir);
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

uint8_t balance_control_is_fallen(void)
{
    return g_is_fallen;
}
