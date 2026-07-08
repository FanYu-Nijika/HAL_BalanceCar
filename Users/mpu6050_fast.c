#include "mpu6050_fast.h"
#include "STM32_I2C.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <stdio.h>

#define MPU6050_ADDR        0x68
#define MPU6050_SMPLRT_DIV  0x19
#define MPU6050_CONFIG      0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_INT_PIN_CFG  0x37
#define MPU6050_INT_ENABLE   0x38
#define MPU6050_INT_STATUS   0x3A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_PWR_MGMT_2  0x6C
#define MPU6050_WHO_AM_I    0x75

#define MPU6050_RAD_TO_DEG 57.29578f
#define MPU6050_GYRO_LSB_PER_DPS 16.4f

static void mpu6050_int_gpio_init(void)
{
    GPIO_InitTypeDef gpio;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    gpio.Pin = MPU6050_INT_Pin;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MPU6050_INT_GPIO_Port, &gpio);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

static int16_t make_i16(uint8_t high, uint8_t low)
{
    return (int16_t)((uint16_t)high << 8 | low);
}

int mpu6050_fast_init(void)
{
    uint8_t who_am_i = 0;

    i2cInit();

    if (!i2cRead(MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &who_am_i))
    {
        printf("[MPU] WHO_AM_I read failed\r\n");
        return 0;
    }
    if (who_am_i != 0x68)
    {
        printf("[MPU] WHO_AM_I=0x%02X, expected 0x68\r\n", who_am_i);
        return 0;
    }

    i2cWrite(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x80);
    HAL_Delay(50);
    i2cWrite(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01);
    i2cWrite(MPU6050_ADDR, MPU6050_PWR_MGMT_2, 0x00);

    /* The verified firmware runs the balance path from MPU data-ready EXTI at 100Hz. */
    i2cWrite(MPU6050_ADDR, MPU6050_SMPLRT_DIV, 0x09);
    i2cWrite(MPU6050_ADDR, MPU6050_CONFIG, 0x03);

    /* Gyro +/-2000dps -> 16.4 LSB/(deg/s), accel +/-4g -> 8192 LSB/g. */
    i2cWrite(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x18);
    i2cWrite(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x08);

    /*
     * firmware_COM4_verify checks PB9 low inside EXTI9_5, so configure the
     * MPU data-ready interrupt as active-low and trigger the MCU on falling edge.
     */
    i2cWrite(MPU6050_ADDR, MPU6050_INT_PIN_CFG, 0x80);
    i2cWrite(MPU6050_ADDR, MPU6050_INT_ENABLE, 0x01);
    mpu6050_int_gpio_init();

    HAL_Delay(20);
    return 1;
}

int mpu6050_fast_read(mpu6050_fast_data_t *data)
{
    uint8_t buf[14];

    if (!data)
    {
        return 0;
    }
    if (!i2cRead(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 14, buf))
    {
        return 0;
    }

    data->ax = make_i16(buf[0], buf[1]);
    data->ay = make_i16(buf[2], buf[3]);
    data->az = make_i16(buf[4], buf[5]);
    data->gx = make_i16(buf[8], buf[9]);
    data->gy = make_i16(buf[10], buf[11]);
    data->gz = make_i16(buf[12], buf[13]);

    return 1;
}

int mpu6050_fast_data_ready(void)
{
    uint8_t status = 0;

    if (!i2cRead(MPU6050_ADDR, MPU6050_INT_STATUS, 1, &status))
    {
        return 0;
    }

    return (status & 0x01) ? 1 : 0;
}

float mpu6050_fast_get_pitch_accel(const mpu6050_fast_data_t *data)
{
    float ax_f;
    float az_f;

    if (!data)
    {
        return 0.0f;
    }

    ax_f = (float)data->ax;
    az_f = (float)data->az;
    return -atan2f((float)data->ay, sqrtf(ax_f * ax_f + az_f * az_f)) * MPU6050_RAD_TO_DEG;
}

float mpu6050_fast_get_pitch_gyro_rate(const mpu6050_fast_data_t *data)
{
    if (!data)
    {
        return 0.0f;
    }

    return (float)data->gy / MPU6050_GYRO_LSB_PER_DPS;
}
