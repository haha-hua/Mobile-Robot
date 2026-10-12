#include "bsp_pwm.h"
#include "config.h"
#include "stm32f10x.h"

/* 把占空比限制在安全范围内 */
static uint16_t clamp_duty(uint16_t duty)
{
    if (duty >= MOTOR_PWM_PERIOD) {
        return MOTOR_PWM_PERIOD - 1;
    }
    return duty;
}

void pwm_init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;

    /* ① 开时钟:TIM3 挂在 APB1,GPIOC/复用功能挂在 APB2 */
    RCC_APB1PeriphClockCmd(MOTOR_PWM_TIM_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(MOTOR_PWM_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

    /* ② 重映射:TIM3_CH1/CH2 默认在 PA6/PA7,完全重映射后落到 PC6/PC7 */
    GPIO_PinRemapConfig(MOTOR_PWM_REMAP, ENABLE);

    /* ③ 引脚配成复用推挽输出:引脚电平交给定时器控制,程序不再直接写它 */
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_PWM_PORT, &gpio);

    /* ④ 时基:72MHz / (PSC+1) / (ARR+1) = 72M/72/1000 = 1kHz PWM */
    tb.TIM_Prescaler         = 72 - 1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = MOTOR_PWM_PERIOD - 1;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(MOTOR_PWM_TIM, &tb);

    /* ⑤ 输出比较:PWM1 模式 = 计数值 < 比较值时输出高电平。
       先把比较值设为 0,即上电占空比 0,电机不转 */
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 0;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(MOTOR_PWM_TIM, &oc);
    TIM_OC2Init(MOTOR_PWM_TIM, &oc);
    TIM_OC1PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);

    /* ⑥ 启动定时器 */
    TIM_Cmd(MOTOR_PWM_TIM, ENABLE);
}

void pwm_set_duty(pwm_ch_t ch, uint16_t duty)
{
    duty = clamp_duty(duty);

    switch (ch) {
    case PWM_CH1:
        TIM_SetCompare1(MOTOR_PWM_TIM, duty);
        break;
    case PWM_CH2:
        TIM_SetCompare2(MOTOR_PWM_TIM, duty);
        break;
    default:
        break;
    }
}
