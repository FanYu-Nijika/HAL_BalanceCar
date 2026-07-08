#include "balance_control.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050_fast.h"
#include "stm32f1xx_hal.h"

/*
 * Verified firmware architecture from COM4/hex reverse notes:
 *
 *   MPU6050 data-ready PB9 EXTI fast path
 *       -> read encoder counts
 *       -> read raw MPU6050 attitude data
 *       -> Kalman/complementary pitch estimate
 *       -> angle PD PWM
 *       -> velocity PI PWM
 *       -> left/right PWM mix and limit
 *
 * The controller starts automatically after balance_control_init(). It does not
 * wait for a serial "init" command; the serial port is only for manual debugging.
 */
#define BALANCE_DT 0.005f
#define BALANCE_AUTO_START 1

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

/* Verified firmware clamps the velocity integral around +/-380000. */
#define SPEED_INTEGRAL_LIMIT 380000.0f

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

typedef struct
{
    float angle;
    float accel_angle;
    float gyro_rate;
    float gyro_zero_rate;

    float kalman_bias;
    float kalman_p00;
    float kalman_p01;
    float kalman_p10;
    float kalman_p11;

    float speed_filter;
    float speed_integral;

    int left_encoder_count;
    int right_encoder_count;

    int balance_pwm;
    int velocity_pwm;
    int turn_pwm;
    int left_pwm;
    int right_pwm;

    uint8_t enabled;
    uint8_t is_fallen;
} balance_runtime_t;

static balance_runtime_t g_balance;

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

static void balance_velocity_loop_reset(void)
{
    g_balance.speed_filter = 0.0f;
    g_balance.speed_integral = 0.0f;
}

static void balance_output_reset(void)
{
    g_balance.balance_pwm = 0;
    g_balance.velocity_pwm = 0;
    g_balance.turn_pwm = 0;
    g_balance.left_pwm = 0;
    g_balance.right_pwm = 0;
    motor_stop_all();
}

static void balance_fault_stop(void)
{
    balance_velocity_loop_reset();
    balance_output_reset();
}

static void balance_filter_reset(float angle)
{
    g_balance.angle = angle;
    g_balance.accel_angle = angle;
    g_balance.gyro_rate = 0.0f;
    g_balance.kalman_bias = 0.0f;
    g_balance.kalman_p00 = 0.0f;
    g_balance.kalman_p01 = 0.0f;
    g_balance.kalman_p10 = 0.0f;
    g_balance.kalman_p11 = 0.0f;
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

    rate = gyro_rate - g_balance.kalman_bias;
    g_balance.angle += BALANCE_DT * rate;

    g_balance.kalman_p00 += BALANCE_DT * (BALANCE_DT * g_balance.kalman_p11
                            - g_balance.kalman_p01 - g_balance.kalman_p10
                            + BALANCE_KALMAN_Q_ANGLE);
    g_balance.kalman_p01 -= BALANCE_DT * g_balance.kalman_p11;
    g_balance.kalman_p10 -= BALANCE_DT * g_balance.kalman_p11;
    g_balance.kalman_p11 += BALANCE_KALMAN_Q_BIAS * BALANCE_DT;

    s = g_balance.kalman_p00 + BALANCE_KALMAN_R_MEASURE;
    if (s == 0.0f)
    {
        return g_balance.angle;
    }

    k0 = g_balance.kalman_p00 / s;
    k1 = g_balance.kalman_p10 / s;
    y = accel_angle - g_balance.angle;

    g_balance.angle += k0 * y;
    g_balance.kalman_bias += k1 * y;

    p00_temp = g_balance.kalman_p00;
    p01_temp = g_balance.kalman_p01;
    g_balance.kalman_p00 -= k0 * p00_temp;
    g_balance.kalman_p01 -= k0 * p01_temp;
    g_balance.kalman_p10 -= k1 * p00_temp;
    g_balance.kalman_p11 -= k1 * p01_temp;

    return g_balance.angle;
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
        g_balance.gyro_zero_rate = gyro_sum / (float)samples;
        balance_filter_reset(angle_sum / (float)samples);
    }
    else
    {
        g_balance.gyro_zero_rate = 0.0f;
        balance_filter_reset(Balance_Target_Angle);
    }
}

static void balance_read_encoder_feedback(void)
{
    /* Verified fast path reads TIM4 first, then TIM8 and negates the right side. */
    g_balance.left_encoder_count = encoder_read_left_count();
    g_balance.right_encoder_count = -encoder_read_right_count();
}

static int balance_read_attitude_feedback(void)
{
    mpu6050_fast_data_t data;

    if (!mpu6050_fast_read(&data))
    {
        return 0;
    }

    g_balance.accel_angle = mpu6050_fast_get_pitch_accel(&data);
    g_balance.gyro_rate = mpu6050_fast_get_pitch_gyro_rate(&data)
                         - g_balance.gyro_zero_rate;

    return 1;
}

static void balance_update_attitude_filter(void)
{
    if (Balance_Filter_Mode == BALANCE_FILTER_KALMAN)
    {
        g_balance.angle = balance_kalman_update(g_balance.accel_angle,
                                                g_balance.gyro_rate);
    }
    else
    {
        g_balance.angle = BALANCE_FILTER_ALPHA
                        * (g_balance.angle + g_balance.gyro_rate * BALANCE_DT)
                        + (1.0f - BALANCE_FILTER_ALPHA) * g_balance.accel_angle;
    }
}

static float balance_get_target_angle(void)
{
    return Balance_Target_Angle + Balance_Target_Offset;
}

static int balance_is_angle_safe(float target_angle)
{
    if (abs_float(g_balance.angle - target_angle) > BALANCE_FALL_ANGLE)
    {
        return 0;
    }

    return 1;
}

static void balance_angle_loop_update(float target_angle)
{
    float angle_error;
    float angle_output;

    angle_error = g_balance.angle - target_angle;
    angle_output = angle_error * (Balance_Kp / BALANCE_PARAM_SCALE)
                 + g_balance.gyro_rate * (Balance_Kd / BALANCE_PARAM_SCALE);

    g_balance.balance_pwm = (int)(angle_output * (float)Balance_Output_Dir);
}

static void balance_velocity_loop_update(void)
{
    float speed_feedback;

    speed_feedback = -(float)(g_balance.left_encoder_count
                              + g_balance.right_encoder_count);

    g_balance.speed_filter = g_balance.speed_filter * SPEED_FILTER_KEEP
                           + speed_feedback * SPEED_FILTER_NEW;
    g_balance.speed_integral += g_balance.speed_filter;
    g_balance.speed_integral = limit_float(g_balance.speed_integral,
                                           SPEED_INTEGRAL_LIMIT);

    /* Verified firmware applies velocity feedback with negative P/I sign. */
    g_balance.velocity_pwm = (int)(-g_balance.speed_filter
                                  * (Velocity_Kp / BALANCE_PARAM_SCALE)
                                  -g_balance.speed_integral
                                  * (Velocity_Ki / BALANCE_PARAM_SCALE));
}

static void balance_turn_loop_update(void)
{
    /* Verified straight-balance fast path keeps turn output at zero. */
    g_balance.turn_pwm = 0;
}

static void balance_mix_and_output(void)
{
    int final_left;
    int final_right;

    final_left = g_balance.balance_pwm + g_balance.velocity_pwm + g_balance.turn_pwm;
    final_right = g_balance.balance_pwm + g_balance.velocity_pwm - g_balance.turn_pwm;

    g_balance.left_pwm = limit_int(final_left, MOTOR_PWM_MAX);
    g_balance.right_pwm = limit_int(final_right, MOTOR_PWM_MAX);

    motor_set_left(g_balance.left_pwm * Balance_Left_Motor_Dir);
    motor_set_right(g_balance.right_pwm * Balance_Right_Motor_Dir);
}

static void balance_fast_loop_step(void)
{
    float target_angle;

    balance_read_encoder_feedback();

    if (!balance_read_attitude_feedback())
    {
        g_balance.is_fallen = 1;
        balance_fault_stop();
        return;
    }

    balance_update_attitude_filter();
    target_angle = balance_get_target_angle();

    if (!balance_is_angle_safe(target_angle))
    {
        g_balance.is_fallen = 1;
        balance_fault_stop();
        return;
    }

    g_balance.is_fallen = 0;

    balance_angle_loop_update(target_angle);
    balance_velocity_loop_update();
    balance_turn_loop_update();
    balance_mix_and_output();
}

void balance_control_init(void)
{
    g_balance.enabled = 0;
    g_balance.is_fallen = 1;
    balance_velocity_loop_reset();
    balance_output_reset();

    if (mpu6050_fast_init())
    {
        balance_calibrate_zero_angle();
        balance_velocity_loop_reset();
        balance_output_reset();

#if BALANCE_AUTO_START
        g_balance.enabled = 1;
        g_balance.is_fallen = 0;
#endif
    }
    else
    {
        g_balance.enabled = 0;
        g_balance.is_fallen = 1;
    }
}

void balance_control_start(void)
{
    balance_velocity_loop_reset();
    balance_output_reset();
    g_balance.enabled = 1;
    g_balance.is_fallen = 0;
}

void balance_control_stop(void)
{
    g_balance.enabled = 0;
    g_balance.is_fallen = 1;
    balance_fault_stop();
}

void balance_control_update(void)
{
    if (!g_balance.enabled)
    {
        balance_output_reset();
        return;
    }

    balance_fast_loop_step();
}

void balance_control_mpu_irq(void)
{
    balance_control_update();
}

float balance_control_get_angle(void)
{
    return g_balance.angle;
}

float balance_control_get_gyro_rate(void)
{
    return g_balance.gyro_rate;
}

int balance_control_get_pwm(void)
{
    return g_balance.balance_pwm;
}

int balance_control_get_left_pwm(void)
{
    return g_balance.left_pwm;
}

int balance_control_get_right_pwm(void)
{
    return g_balance.right_pwm;
}

uint8_t balance_control_is_fallen(void)
{
    return g_balance.is_fallen;
}
