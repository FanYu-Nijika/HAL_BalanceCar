#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "encoder.h"
#include "motor.h"
#include "speed_pi.h"
#include "OLED.h"
#include <stdio.h>

#define SPEED_SAMPLE_MS 10
#define PI_STAGE_MS 8000
#define PI_STAGE_GAP_MS 1000
#define PI_STAGE_COUNT 3
#define SPEED_FILTER_OLD_WEIGHT 8
#define SPEED_FILTER_NEW_WEIGHT 2
#define LEFT_BASE_PWM 54
#define RIGHT_BASE_PWM 56
#define PI_CORRECT_LIMIT 35

#define LEFT_MOTOR_DIR 1
#define RIGHT_MOTOR_DIR -1
#define LEFT_ENCODER_DIR 1
#define RIGHT_ENCODER_DIR -1

typedef struct
{
    int kp_x100;
    int ki_x100;
} pi_param_t;

static const pi_param_t g_left_pi_params[PI_STAGE_COUNT] =
{
    {8, 1},
    {10, 1},
    {12, 1}
};

static const pi_param_t g_right_pi_params[PI_STAGE_COUNT] =
{
    {8, 1},
    {10, 1},
    {12, 1}
};

static const int g_speed_targets[PI_STAGE_COUNT] =
{
    1000,
    1000,
    1000
};

void led_init(void);

int main(void)
{
    uint32_t now_tick;
    uint32_t last_tick;
    uint32_t start_tick;
    uint32_t stage_start_tick;
    uint32_t gap_start_tick;
    uint32_t sample_period_ms;
    int stage = 0;
    int test_finished = 0;
    int stage_gap = 0;
    int filter_ready = 0;
    int left_count;
    int right_count;
    int left_speed;
    int right_speed;
    int left_speed_filter = 0;
    int right_speed_filter = 0;
    int left_total_count = 0;
    int right_total_count = 0;
    int left_pwm;
    int right_pwm;
    speed_pi_t left_pi;
    speed_pi_t right_pi;

    HAL_Init();
    sys_stm32_clock_init(RCC_PLL_MUL9);
    delay_init(72);
    usart_init(115200);
    led_init();

    motor_init();
    encoder_init();
    speed_pi_init(&left_pi, g_left_pi_params[0].kp_x100, g_left_pi_params[0].ki_x100, PI_CORRECT_LIMIT);
    speed_pi_init(&right_pi, g_right_pi_params[0].kp_x100, g_right_pi_params[0].ki_x100, PI_CORRECT_LIMIT);
    start_tick = HAL_GetTick();
    stage_start_tick = start_tick;
    last_tick = HAL_GetTick();

    printf("time_ms,stage,left_kp_x100,left_ki_x100,right_kp_x100,right_ki_x100,target,left_speed,right_speed,left_filter,right_filter,left_pwm,right_pwm,left_count,right_count,left_total,right_total\r\n");

    while (1)
    {
        now_tick = HAL_GetTick();
        if (stage_gap)
        {
            if (now_tick - gap_start_tick >= PI_STAGE_GAP_MS)
            {
                stage++;
                stage_gap = 0;
                stage_start_tick = now_tick;
                last_tick = now_tick;
                speed_pi_init(&left_pi, g_left_pi_params[stage].kp_x100, g_left_pi_params[stage].ki_x100, PI_CORRECT_LIMIT);
                speed_pi_init(&right_pi, g_right_pi_params[stage].kp_x100, g_right_pi_params[stage].ki_x100, PI_CORRECT_LIMIT);
                filter_ready = 0;
                left_speed_filter = 0;
                right_speed_filter = 0;
                left_total_count = 0;
                right_total_count = 0;
                encoder_read_left_count();
                encoder_read_right_count();
            }
            continue;
        }

        if (!test_finished && (now_tick - stage_start_tick >= PI_STAGE_MS))
        {
            if (stage < PI_STAGE_COUNT - 1)
            {
                motor_stop_all();
                stage_gap = 1;
                gap_start_tick = now_tick;
            }
            else
            {
                test_finished = 1;
                motor_stop_all();
            }
        }

        if (test_finished)
        {
            continue;
        }

        if (now_tick - last_tick >= SPEED_SAMPLE_MS)
        {
            sample_period_ms = now_tick - last_tick;
            last_tick = now_tick;

            left_count = encoder_read_left_count() * LEFT_ENCODER_DIR;
            right_count = encoder_read_right_count() * RIGHT_ENCODER_DIR;
            left_speed = encoder_count_to_speed_mm_s_by_period(left_count, (int)sample_period_ms);
            right_speed = encoder_count_to_speed_mm_s_by_period(right_count, (int)sample_period_ms);
            left_total_count += left_count;
            right_total_count += right_count;
            if (filter_ready == 0)
            {
                left_speed_filter = left_speed;
                right_speed_filter = right_speed;
                filter_ready = 1;
            }
            else
            {
                left_speed_filter = (left_speed_filter * SPEED_FILTER_OLD_WEIGHT + left_speed * SPEED_FILTER_NEW_WEIGHT) / 10;
                right_speed_filter = (right_speed_filter * SPEED_FILTER_OLD_WEIGHT + right_speed * SPEED_FILTER_NEW_WEIGHT) / 10;
            }
            left_pwm = LEFT_BASE_PWM + speed_pi_update(&left_pi, g_speed_targets[stage], left_speed_filter);
            right_pwm = RIGHT_BASE_PWM + speed_pi_update(&right_pi, g_speed_targets[stage], right_speed_filter);

            motor_set_left(left_pwm * LEFT_MOTOR_DIR);
            motor_set_right(right_pwm * RIGHT_MOTOR_DIR);

            printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                   stage + 1,
                   g_left_pi_params[stage].kp_x100,
                   g_left_pi_params[stage].ki_x100,
                   g_right_pi_params[stage].kp_x100,
                   g_right_pi_params[stage].ki_x100,
                   g_speed_targets[stage],
                   left_speed,
                   right_speed
                //    left_speed_filter,
                //    right_speed_filter,
                //    left_pwm,
                //    right_pwm,
                //    left_count,
                //    right_count,
                //    left_total_count,
                //    right_total_count
                );
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
