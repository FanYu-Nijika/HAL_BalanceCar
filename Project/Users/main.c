#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "speed_control.h"
#include <stdio.h>
#include <math.h>
#include "OLED.h"
#include "MPU6050.h"

#define CONTROL_PERIOD_MS      10
#define SPEED_TARGET_MM_S      200
#define MPU_READ_PERIOD_MS     20    /* DMP FIFO rate = 50Hz */

/* ---- MPU6050 sensitivity constants (FSR: ¡À2g accelerometer, ¡À2000dps gyro) ---- */
#define ACCEL_SENS_LSB_PER_G   16384.0f
#define GYRO_SENS_LSB_PER_DPS  16.4f
#define G_CONST                9.8f

static void led_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    gpio_initstruct.Pin   = GPIO_PIN_5;
    gpio_initstruct.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_initstruct.Pull  = GPIO_PULLUP;
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_initstruct);

    gpio_initstruct.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOE, &gpio_initstruct);
}

int main(void)
{
    uint32_t last_tick;
    uint32_t last_mpu_tick;
    mpu6050_data_t mpu_data;
    char oled_buf[24];

    /* ---- system init ---- */
    HAL_Init();
    sys_stm32_clock_init(RCC_PLL_MUL9);
    delay_init(72);
    usart_init(115200);
    led_init();
    OLED_Init();

    /* ---- MPU6050 init ---- */
    mpu6050_init();
    printf("MPU6050 initialized.\r\n");

    /* ---- speed control init ---- */
    speed_control_init();
    speed_control_set_target(SPEED_TARGET_MM_S, SPEED_TARGET_MM_S);

    last_tick     = HAL_GetTick();
    last_mpu_tick = last_tick;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* ---- speed control (10ms) ---- */
        if (now - last_tick >= CONTROL_PERIOD_MS)
        {
            last_tick = now;
            speed_control_update();
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
        }

        /* ---- MPU6050 read & conversion (20ms) ---- */
        if (now - last_mpu_tick >= MPU_READ_PERIOD_MS)
        {
            last_mpu_tick = now;

            if (mpu6050_read_data(&mpu_data))
            {
                float ax_g  = (float)mpu_data.ax / ACCEL_SENS_LSB_PER_G;
                float ay_g  = (float)mpu_data.ay / ACCEL_SENS_LSB_PER_G;
                float az_g  = (float)mpu_data.az / ACCEL_SENS_LSB_PER_G;

                float ax_m  = ax_g * G_CONST;
                float ay_m  = ay_g * G_CONST;
                float az_m  = az_g * G_CONST;

                float gx_d  = (float)mpu_data.gx / GYRO_SENS_LSB_PER_DPS;
                float gy_d  = (float)mpu_data.gy / GYRO_SENS_LSB_PER_DPS;
                float gz_d  = (float)mpu_data.gz / GYRO_SENS_LSB_PER_DPS;

                float pitch  = mpu6050_get_pitch_accel(&mpu_data);

                /* ---- serial output (VOFA+ / serial monitor) ---- */
                printf("%6.2f,%6.2f,%6.2f,%7.1f,%7.1f,%7.1f,%6.1f\r\n",
                       ax_g, ay_g, az_g, gx_d, gy_d, gz_d, pitch);
            }
        }

        /* ---- OLED display ---- */
        OLED_Clear();

        // OLED_Printf(0, 0, OLED_6X8, "Pitch:%5.1f",
        //             mpu6050_get_pitch_accel(&mpu_data));

       

        OLED_Update();
    }
}
