#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "encoder.h"
#include "motor.h"
#include "speed_pi.h"
#include "OLED.h"
#include "MPU6050.h"
#include <stdio.h>

void led_init(void);

int main(void)
{
    HAL_Init();
    OLED_Init();
    sys_stm32_clock_init(RCC_PLL_MUL9);
    delay_init(72);
    usart_init(115200);
    led_init();
    motor_init();
    encoder_init();
    MPU_init();
    while (1)
    {
        OLED_Clear();
        OLED_ShowFloatNum(0, 0, fAX, 3, 2);
        OLED_ShowFloatNum(0, 16, fAY, 3, 2);
        OLED_ShowFloatNum(0, 32, fAZ, 3, 2);
        OLED_ShowFloatNum(64, 0, fAX, 3, 2);
        OLED_ShowFloatNum(64, 16, fAY, 3, 2);
        OLED_ShowFloatNum(64, 32, fAZ, 3, 2);
        OLED_Update();
        delay_ms(10);
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
