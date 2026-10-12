#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

/* 板载 USB 转串口调试通道:USART3,引脚和波特率见 config.h
 * 提供 printf 重定向,直接 printf("...") 即可输出到电脑串口终端 */

void uart_init(void);
void uart_send_byte(uint8_t b);

#endif /* BSP_UART_H */
