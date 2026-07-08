#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "encoder.h"
#include "motor.h"
#include "speed_pi.h"
#include "OLED.h"
#include "balance_control.h"
#include <stdio.h>

#define BALANCE_CONTROL_PERIOD_MS 10
#define BALANCE_DISPLAY_PERIOD_MS 100

void led_init(void);

int main(void)
{
    uint32_t now_tick;
    uint32_t last_control_tick;
    uint32_t last_display_tick;

    HAL_Init();
    sys_stm32_clock_init(RCC_PLL_MUL9);
    delay_init(72);
    usart_init(115200);
    led_init();
    OLED_Init();
    motor_init();
    encoder_init();
    balance_control_init();
    last_control_tick = HAL_GetTick();
    last_display_tick = last_control_tick;

    while (1)
    {
        now_tick = HAL_GetTick();
        if (now_tick - last_control_tick >= BALANCE_CONTROL_PERIOD_MS)
        {
            last_control_tick += BALANCE_CONTROL_PERIOD_MS;
            balance_control_update();
        }

        if (now_tick - last_display_tick >= BALANCE_DISPLAY_PERIOD_MS)
        {
            last_display_tick = now_tick;
            OLED_Clear();
            OLED_ShowFloatNum(0, 0, balance_control_get_angle(), 3, 1, OLED_8X16);
            OLED_ShowFloatNum(0, 16, balance_control_get_gyro_rate(), 3, 1, OLED_8X16);
            OLED_ShowSignedNum(0, 32, balance_control_get_pwm(), 3, OLED_8X16);
            OLED_ShowFloatNum(64, 0, Balance_Kp, 2, 1, OLED_8X16);
            OLED_ShowFloatNum(64, 16, Balance_Kd, 2, 1, OLED_8X16);
            OLED_ShowSignedNum(64, 32, balance_control_is_fallen(), 1, OLED_8X16);
            OLED_Update();
        }
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
