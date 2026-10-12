#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f10x.h"

/* ============================================================
 * 本文件集中存放"会变的东西":引脚、频率、参数。
 * 换引脚 / 换板子时只改这里,其他 .c 文件不用动。
 * ============================================================ */

/* ---------- 系统时基 ---------- */
#define TICK_FREQ_HZ    1000u               /* SysTick 中断频率,1000Hz 即 1ms 一跳 */

/* ---------- 板载 LED(PD7,已焊在板子上,无需外接电阻) ---------- */
#define LED_PORT        GPIOD
#define LED_PIN         GPIO_Pin_7
#define LED_CLK         RCC_APB2Periph_GPIOD

/* ---------- 电机 PWM(TIM3 完全重映射: CH1=PC6, CH2=PC7) ----------
 * PWM 频率 = 72MHz / 72 / 1000 = 1kHz,占空比范围 0~999 */
#define MOTOR_PWM_TIM           TIM3
#define MOTOR_PWM_TIM_CLK       RCC_APB1Periph_TIM3
#define MOTOR_PWM_PORT          GPIOC
#define MOTOR_PWM_GPIO_CLK      RCC_APB2Periph_GPIOC
#define MOTOR_PWM_REMAP         GPIO_FullRemap_TIM3     /* 重映射后 CH1/CH2 落在 PC6/PC7 */
#define MOTOR_PWM_PERIOD        1000u                   /* 自动重装载值,也是占空比上限 */
#define MOTOR_DUTY_MAX          700u                    /* 软件限幅:第一次测试别让电机全速 */

/* ---------- 调试串口(USART1 默认引脚: PA9=TXD, PA10=RXD) ---------- */
#define UART_USART      USART1
#define UART_USART_CLK  RCC_APB2Periph_USART1     /* USART1 挂在 APB2 */
#define UART_PORT       GPIOA
#define UART_GPIO_CLK   RCC_APB2Periph_GPIOA
#define UART_TX_PIN     GPIO_Pin_9
#define UART_RX_PIN     GPIO_Pin_10
#define UART_BAUDRATE   115200u

#endif /* CONFIG_H */