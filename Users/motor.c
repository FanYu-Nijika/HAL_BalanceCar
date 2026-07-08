#include "motor.h"

#define MOTOR_PWM_PERIOD 100

static TIM_HandleTypeDef g_tim3_pwm_handle;

static int limit_pwm(int pwm)
{
    if (pwm > MOTOR_PWM_MAX)
    {
        return MOTOR_PWM_MAX;
    }
    if (pwm < -MOTOR_PWM_MAX)
    {
        return -MOTOR_PWM_MAX;
    }
    return pwm;
}

static uint32_t pwm_to_compare(int pwm)
{
    if (pwm < 0)
    {
        pwm = -pwm;
    }

    return (uint32_t)(pwm * MOTOR_PWM_PERIOD / MOTOR_PWM_MAX);
}

static void motor_gpio_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio_initstruct.Mode = GPIO_MODE_AF_PP;
    gpio_initstruct.Pull = GPIO_NOPULL;
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio_initstruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio_initstruct);

    gpio_initstruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gpio_initstruct);
}

static void motor_pwm_channel_init(uint32_t channel)
{
    TIM_OC_InitTypeDef pwm_config;

    pwm_config.OCMode = TIM_OCMODE_PWM1;
    pwm_config.Pulse = 0;
    pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm_config.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&g_tim3_pwm_handle, &pwm_config, channel);
    HAL_TIM_PWM_Start(&g_tim3_pwm_handle, channel);
}

void motor_init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    motor_gpio_init();

    g_tim3_pwm_handle.Instance = TIM3;
    g_tim3_pwm_handle.Init.Prescaler = 71;
    g_tim3_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_tim3_pwm_handle.Init.Period = MOTOR_PWM_PERIOD;
    g_tim3_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&g_tim3_pwm_handle);

    motor_pwm_channel_init(TIM_CHANNEL_1);
    motor_pwm_channel_init(TIM_CHANNEL_2);
    motor_pwm_channel_init(TIM_CHANNEL_3);
    motor_pwm_channel_init(TIM_CHANNEL_4);

    motor_stop_all();
}

void motor_set_left(int pwm)
{
    uint32_t compare;

    pwm = limit_pwm(pwm);
    compare = pwm_to_compare(pwm);

    if (pwm > 0)
    {
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm_handle, TIM_CHANNEL_1, compare);
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm_handle, TIM_CHANNEL_2, 0);
    }
    else if (pwm < 0)
    {
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm_handle, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm_handle, TIM_CHANNEL_2, compare);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm_handle, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm_handle, TIM_CHANNEL_2, 0);
    }
}

void motor_set_right(int pwm)
{
    uint32_t compare;

    pwm = limit_pwm(pwm);
    compare = pwm_to_compare(pwm);

    if (pwm > 0)
    {
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm_handle, TIM_CHANNEL_3, compare);
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm_handle, TIM_CHANNEL_4, 0);
    }
    else if (pwm < 0)
    {
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm_handle, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm_handle, TIM_CHANNEL_4, compare);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm_handle, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&g_tim3_pwm_handle, TIM_CHANNEL_4, 0);
    }
}

void motor_stop_all(void)
{
    motor_set_left(0);
    motor_set_right(0);
}
