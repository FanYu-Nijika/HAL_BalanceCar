#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "encoder.h"
#include "motor.h"
#include "speed_pi.h"
#include "OLED.h"
#include "balance_control.h"
#include <stdio.h>

#define BALANCE_OLED_DEBUG 0
#define BALANCE_CONTROL_PERIOD_MS 5
#define BALANCE_DISPLAY_PERIOD_MS 200

void led_init(void);

int main(void)
{
    uint32_t now_tick;
    uint32_t last_control_tick;

#if BALANCE_OLED_DEBUG
    uint32_t last_display_tick;
#endif

    HAL_Init();
    sys_stm32_clock_init(RCC_PLL_MUL9);
    delay_init(72);
    usart_init(115200);
    led_init();

#if BALANCE_OLED_DEBUG
    OLED_Init();
#endif

    motor_init();
    encoder_init();
    balance_control_init();

    last_control_tick = HAL_GetTick();

#if BALANCE_OLED_DEBUG
    last_display_tick = HAL_GetTick();
#endif

    while (1)
    {
        now_tick = HAL_GetTick();

        /*
         * 5 ms fallback loop.
         *
         * The verified firmware runs from the MPU6050 PB9 data-ready interrupt,
         * but keeping a periodic fallback prevents the car from staying silent
         * when the INT pin/EXTI path is not firing during bring-up.
         */
        if (now_tick - last_control_tick >= BALANCE_CONTROL_PERIOD_MS)
        {
            last_control_tick = now_tick;
            balance_control_update();
        }

#if BALANCE_OLED_DEBUG
        if (now_tick - last_display_tick >= BALANCE_DISPLAY_PERIOD_MS)
        {
            last_display_tick = now_tick;
            OLED_Clear();
            OLED_Printf(0, 0, OLED_6X8, "A:%d", (int)(balance_control_get_angle() * 10.0f));
            OLED_Printf(64, 0, OLED_6X8, "G:%d", (int)(balance_control_get_gyro_rate() * 10.0f));
            OLED_Printf(0, 16, OLED_6X8, "B:%d", balance_control_get_pwm());
            OLED_Printf(64, 16, OLED_6X8, "F:%d", balance_control_is_fallen());
            OLED_Printf(0, 32, OLED_6X8, "L:%d", balance_control_get_left_pwm());
            OLED_Printf(64, 32, OLED_6X8, "R:%d", balance_control_get_right_pwm());
            OLED_Update();
        }
#endif
    }
}

void led_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    gpio_initstruct.Pin = GPIO_PIN_5;
    gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_initstruct.Pull = GPIO_PULLUP;
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_initstruct);

    gpio_initstruct.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOE, &gpio_initstruct);
}
