#include "encoder.h"

#define SAMPLE_PERIOD_MS 10
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 31416 / 10000)

static TIM_HandleTypeDef g_tim4_encoder_handle;
static TIM_HandleTypeDef g_tim8_encoder_handle;
static unsigned short g_left_last_counter;
static unsigned short g_right_last_counter;

static void encoder_gpio_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio_initstruct.Mode = GPIO_MODE_INPUT;
    gpio_initstruct.Pull = GPIO_PULLUP;
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio_initstruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &gpio_initstruct);

    gpio_initstruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &gpio_initstruct);
}

static void encoder_timer_init(TIM_HandleTypeDef *handle, TIM_TypeDef *timer)
{
    TIM_Encoder_InitTypeDef encoder_config;

    handle->Instance = timer;
    handle->Init.Prescaler = 0;
    handle->Init.CounterMode = TIM_COUNTERMODE_UP;
    handle->Init.Period = 0xffff;
    handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    encoder_config.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder_config.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC1Filter = 6;
    encoder_config.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC2Filter = 6;

    HAL_TIM_Encoder_Init(handle, &encoder_config);
    HAL_TIM_Encoder_Start(handle, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(handle, 0);
}

void encoder_init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    encoder_gpio_init();

    /* B585/C10B uses TIM4 PB6/PB7 and TIM8 PC6/PC7 as two quadrature encoder inputs. */
    encoder_timer_init(&g_tim4_encoder_handle, TIM4);
    encoder_timer_init(&g_tim8_encoder_handle, TIM8);
    g_left_last_counter = (unsigned short)__HAL_TIM_GET_COUNTER(&g_tim4_encoder_handle);
    g_right_last_counter = (unsigned short)__HAL_TIM_GET_COUNTER(&g_tim8_encoder_handle);
}

int encoder_read_left_count(void)
{
    unsigned short now_counter;
    short count;

    now_counter = (unsigned short)__HAL_TIM_GET_COUNTER(&g_tim4_encoder_handle);
    count = (short)(now_counter - g_left_last_counter);
    g_left_last_counter = now_counter;
    return count;
}

int encoder_read_right_count(void)
{
    unsigned short now_counter;
    short count;

    now_counter = (unsigned short)__HAL_TIM_GET_COUNTER(&g_tim8_encoder_handle);
    count = (short)(now_counter - g_right_last_counter);
    g_right_last_counter = now_counter;
    return count;
}

int encoder_count_to_speed_mm_s(int count)
{
    return encoder_count_to_speed_mm_s_by_period(count, SAMPLE_PERIOD_MS);
}

int encoder_count_to_speed_mm_s_by_period(int count, int sample_period_ms)
{
    if (sample_period_ms <= 0)
    {
        return 0;
    }

    return count * WHEEL_CIRCUMFERENCE_MM * 1000 / ENCODER_PULSE_PER_ROUND / sample_period_ms;
}
