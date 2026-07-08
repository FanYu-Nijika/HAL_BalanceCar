#include "balance_control.h"
#include "motor.h"
#include "MPU6050.h"
#include "stm32f1xx_hal.h"

#define BALANCE_DT 0.01f
#define BALANCE_FILTER_ALPHA 0.98f
#define BALANCE_ZERO_SAMPLES 50
#define BALANCE_ZERO_MAX_TRIES 120
#define BALANCE_FALL_ANGLE 35.0f
#define BALANCE_PWM_LIMIT 80
#define BALANCE_PWM_DEADBAND 8

volatile float Balance_Kp = 5.0f;
volatile float Balance_Kd = 0.5f;
volatile float Balance_Target_Angle = 0.0f;
volatile int Balance_Output_Dir = -1;
volatile int Balance_Left_Motor_Dir = 1;
volatile int Balance_Right_Motor_Dir = -1;

static float g_balance_angle = 0.0f;
static float g_accel_angle = 0.0f;
static float g_balance_gyro_rate = 0.0f;
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

static int float_to_pwm(float value)
{
    int pwm;

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

    return limit_int(pwm, BALANCE_PWM_LIMIT);
}

static void balance_stop(void)
{
    g_balance_pwm = 0;
    motor_stop_all();
}

static void balance_calibrate_zero_angle(void)
{
    mpu6050_data_t data;
    float sum = 0.0f;
    int samples = 0;
    int tries = 0;

    /*
     * The car should be held at its real mechanical balance point during boot.
     * Averaging the accelerometer angle here gives the angle loop a practical
     * upright reference instead of assuming the PCB is mounted perfectly.
     */
    while (samples < BALANCE_ZERO_SAMPLES && tries < BALANCE_ZERO_MAX_TRIES)
    {
        tries++;
        if (mpu6050_read_data(&data))
        {
            sum += mpu6050_get_pitch_accel(&data);
            samples++;
        }
        HAL_Delay(10);
    }

    if (samples > 0)
    {
        Balance_Target_Angle = sum / (float)samples;
    }

    g_balance_angle = Balance_Target_Angle;
    g_accel_angle = Balance_Target_Angle;
    g_balance_gyro_rate = 0.0f;
}

void balance_control_init(void)
{
    mpu6050_init();
    balance_calibrate_zero_angle();
    balance_stop();
    g_is_fallen = 0;
}

void balance_control_update(void)
{
    mpu6050_data_t data;
    float angle_error;
    float output;

    if (!mpu6050_read_data(&data))
    {
        g_is_fallen = 1;
        balance_stop();
        return;
    }

    g_accel_angle = mpu6050_get_pitch_accel(&data);
    g_balance_gyro_rate = mpu6050_get_pitch_gyro_rate(&data);
    g_balance_angle = BALANCE_FILTER_ALPHA * (g_balance_angle + g_balance_gyro_rate * BALANCE_DT)
                    + (1.0f - BALANCE_FILTER_ALPHA) * g_accel_angle;

    if (abs_float(g_balance_angle - Balance_Target_Angle) > BALANCE_FALL_ANGLE)
    {
        g_is_fallen = 1;
        balance_stop();
        return;
    }

    g_is_fallen = 0;
    angle_error = Balance_Target_Angle - g_balance_angle;
    output = Balance_Kp * angle_error - Balance_Kd * g_balance_gyro_rate;
    g_balance_pwm = float_to_pwm(output * (float)Balance_Output_Dir);

    motor_set_left(g_balance_pwm * Balance_Left_Motor_Dir);
    motor_set_right(g_balance_pwm * Balance_Right_Motor_Dir);
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
