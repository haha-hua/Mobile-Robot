#include "stm32f10x.h"
#include "systick.h"
#include "bsp_uart.h"
#include <stdio.h>

int main(void)
{
    uint32_t cnt = 0;

    systick_init();
    uart_init();

    while (1) {
        printf("hello %u\r\n", (unsigned)cnt++);
        delay_ms(1000);
    }
}
