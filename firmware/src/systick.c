#include "systick.h"
#include "stm32f10x.h"
#include "config.h"

/* 毫秒级时间戳:由 SysTick 中断每 1ms 加一。
   volatile 不能省 —— 它在中断里被改、在主流程里被读,
   不加的话编译器会以为它不会变,把它缓存进寄存器,循环就永远出不来 */
static volatile uint32_t s_ms_tick = 0;

/* SysTick 中断服务函数。
   名字必须和启动文件向量表里的名字完全一致,不能改;
   它由硬件调用、不由用户代码调用,所以不写进 systick.h */
void SysTick_Handler(void)
{
    s_ms_tick++;
}

void systick_init(void)
{
    /* SystemCoreClock = 内核主频(此处 72MHz)。
       除以 TICK_FREQ_HZ 得到重装载值,即每 1ms 触发一次中断 */
    SysTick_Config(SystemCoreClock / TICK_FREQ_HZ);
}

void delay_ms(uint32_t ms)
{
    uint32_t start = s_ms_tick;

    /* 用 (当前 - 起点) < ms,而不是 (当前 < 起点 + ms):
       无符号数减法能自动处理计数溢出回绕 */
    while ((s_ms_tick - start) < ms) {
        __WFI();    /* CPU 停机,等下一次 SysTick 中断把它叫醒 */
    }
}
