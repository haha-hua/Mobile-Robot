#include "stm32f10x.h"   /* STM32F10x 标准外设库(SPL)总头文件,GPIO/RCC 等声明都在这里 */




static volatile uint32_t g_ms_tick = 0; //毫秒级时间戳

// 系统Tick 中断服务函数:每 1ms 触发一次
void SysTick_Handler(void)
{
    g_ms_tick++;
    return ;
}

static void systick_init(void )
{
    // 初始化 SysTick 中断:每 1ms 触发一次
    //SystemCoreClock:当前32位主频(默认 72MHz)
    SysTick_Config(SystemCoreClock / 1000);
}

static void delay(uint32_t ms)
{
    uint32_t start_tick = g_ms_tick;

    while(g_ms_tick - start_tick < ms)
    {
        __WFI();
    }


    return ;
}

int main(void)
{
    GPIO_InitTypeDef gpio;   /* 结构体变量:用来装"配哪个引脚 / 什么模式 / 什么速度" */
    GPIO_InitTypeDef gpio_led;


    /* 第①步:打开 GPIOD 端口的时钟。
       STM32 为省电,外设默认没有时钟;不开时钟,后面怎么配都不生效(新手最常忘的坑) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
    systick_init();    // 初始化 SysTick 中断:每 1ms 触发一次

    /* 第②步:填写这个引脚的配置参数 */
    gpio.GPIO_Pin   = GPIO_Pin_7;        /* 选 7 号引脚,即 PD7 */
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;  /* 推挽输出:引脚可以主动输出高/低电平(用于点灯) */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;  /* 引脚电平翻转速度上限 */
    GPIO_Init(GPIOD, &gpio);             /* 第③步:把上面的配置真正写进硬件 */

    gpio_led.GPIO_Pin = GPIO_Pin_6;
    gpio_led.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio_led.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio_led);


    /* 第④步:死循环,让 LED 持续闪烁 */
    while (1) {
        GPIO_SetBits(GPIOC, GPIO_Pin_6);    /* PC6 输出高电平(灯亮还是灭,取决于硬件接法) */
        delay(500);                     /* 大约等 0.5 秒(500ms) */
        GPIO_ResetBits(GPIOC, GPIO_Pin_6);  /* PC6 输出低电平 */    
        delay(500);                     /* 再等大约 0.5 秒(500ms) */

    
    }
}
