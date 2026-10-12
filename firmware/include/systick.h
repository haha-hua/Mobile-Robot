#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

/* 启动系统时基(默认 1ms 一跳)。
   必须在调用 delay_ms() 之前调用一次:
   它同时使能了 SysTick 中断,而 __WFI() 正是靠中断唤醒的。 */
void systick_init(void);

/* 毫秒级延时。等待期间 CPU 进入 __WFI() 低功耗停机状态,被中断唤醒后再检查是否到时。 */
void delay_ms(uint32_t ms);

#endif /* SYSTICK_H */
