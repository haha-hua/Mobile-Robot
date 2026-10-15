#include "encoder.h"
#include "config.h"
#include "stm32f10x.h"

static volatile int32_t left_count;
static volatile int32_t right_count;

void encoder_init(void)
{
    GPIO_InitTypeDef gpio;
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &gpio);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource3);
    EXTI_ClearITPendingBit(EXTI_Line1 | EXTI_Line3);
    exti.EXTI_Line = EXTI_Line1 | EXTI_Line3;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_Init(&nvic);
    nvic.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_Init(&nvic);
}

/* 第一路上升沿读取第二路电平，保持前进计数为正。 */
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        left_count += GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == Bit_SET
                      ? LEFT_ENCODER_SIGN : -LEFT_ENCODER_SIGN;
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
        right_count += GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == Bit_SET
                       ? RIGHT_ENCODER_SIGN : -RIGHT_ENCODER_SIGN;
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

int32_t encoder_left_count(void) { return left_count; }
int32_t encoder_right_count(void) { return right_count; }
