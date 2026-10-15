#ifndef BSP_UART_H
#define BSP_UART_H
#include <stdint.h>
#define UART_LINE_CAPACITY 64u
void uart_init(void);
/* 非阻塞发送整条日志：复制后由中断发送，忙或长度超限返回零。 */
uint8_t uart_try_write(const char *data, uint16_t length);
/* 返回1：完整行；0：暂无行；-1：整行因过长、溢出或接收错误丢弃。 */
int8_t uart_read_line(char line[UART_LINE_CAPACITY]);
#endif
