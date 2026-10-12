#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>

/* 硬件 PWM 输出层:只认"通道"和"占空比",不知道电机是什么 */

typedef enum {
    PWM_CH1 = 0,    /* PC6, TIM3_CH1 */
    PWM_CH2         /* PC7, TIM3_CH2 */
} pwm_ch_t;

/* 配置 TIM3 + 引脚重映射,启动两路 PWM。上电默认占空比 0(不输出) */
void pwm_init(void);

/* 设置某一路的占空比,duty 范围 0 ~ MOTOR_PWM_PERIOD-1(超限会被截断) */
void pwm_set_duty(pwm_ch_t ch, uint16_t duty);

#endif /* BSP_PWM_H */
