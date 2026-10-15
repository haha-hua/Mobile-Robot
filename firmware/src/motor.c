#include "motor.h"
#include "config.h"
#include "stm32f10x.h"

#define LEFT_PWM_PIN GPIO_Pin_0
#define RIGHT_PWM_PIN GPIO_Pin_1
#define LEFT_IN1_PIN GPIO_Pin_6
#define LEFT_IN2_PIN GPIO_Pin_7
#define RIGHT_IN1_PIN GPIO_Pin_8
#define RIGHT_IN2_PIN GPIO_Pin_9
#define STBY_PIN GPIO_Pin_0

static void set_pwm(motor_id_t id, uint16_t duty)
{
    if (id == MOTOR_LEFT) TIM_SetCompare3(TIM3, duty);
    else TIM_SetCompare4(TIM3, duty);
}

void motor_init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    gpio.GPIO_Pin = LEFT_IN1_PIN | LEFT_IN2_PIN | RIGHT_IN1_PIN | RIGHT_IN2_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin = STBY_PIN;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = LEFT_PWM_PIN | RIGHT_PWM_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOC, LEFT_IN1_PIN | LEFT_IN2_PIN | RIGHT_IN1_PIN | RIGHT_IN2_PIN);
    GPIO_ResetBits(GPIOA, STBY_PIN);

    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 72u - 1u;
    timer.TIM_Period = 1000u - 1u;
    TIM_TimeBaseInit(TIM3, &timer);
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(TIM3, &oc);
    TIM_OC4Init(TIM3, &oc);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void motor_enable(uint8_t enabled)
{
    if (enabled) GPIO_SetBits(GPIOA, STBY_PIN);
    else motor_stop();
}

void motor_set(motor_id_t id, int16_t command)
{
    uint16_t duty;
    if (command > PWM_MAX) command = PWM_MAX;
    if (command < -PWM_MAX) command = -PWM_MAX;
    duty = (uint16_t)(command < 0 ? -command : command);
    if (id == MOTOR_LEFT) {
        if (command >= 0) { GPIO_ResetBits(GPIOC, LEFT_IN1_PIN); GPIO_SetBits(GPIOC, LEFT_IN2_PIN); }
        else { GPIO_SetBits(GPIOC, LEFT_IN1_PIN); GPIO_ResetBits(GPIOC, LEFT_IN2_PIN); }
    } else {
        if (command >= 0) { GPIO_SetBits(GPIOC, RIGHT_IN1_PIN); GPIO_ResetBits(GPIOC, RIGHT_IN2_PIN); }
        else { GPIO_ResetBits(GPIOC, RIGHT_IN1_PIN); GPIO_SetBits(GPIOC, RIGHT_IN2_PIN); }
    }
    set_pwm(id, duty);
}

void motor_stop(void)
{
    set_pwm(MOTOR_LEFT, 0u);
    set_pwm(MOTOR_RIGHT, 0u);
    GPIO_ResetBits(GPIOC, LEFT_IN1_PIN | LEFT_IN2_PIN | RIGHT_IN1_PIN | RIGHT_IN2_PIN);
    GPIO_ResetBits(GPIOA, STBY_PIN);
}
