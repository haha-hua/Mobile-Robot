#include "stm32f10x.h"
#include "config.h"
#include "bsp_uart.h"
#include <stdio.h>

/* 硬件层:调试串口。
 * 引脚/波特率在 config.h;只提供最底层的字节发送,printf 由重定向实现 */

void uart_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_APB2PeriphClockCmd(
        UART_GPIO_CLK | RCC_APB2Periph_AFIO,
        ENABLE
    );
    RCC_APB2PeriphClockCmd(UART_USART_CLK, ENABLE);

    /* TX:复用推挽输出 */
    gpio.GPIO_Pin   = UART_TX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(UART_PORT, &gpio);

    /* RX:浮空输入 */
    gpio.GPIO_Pin   = UART_RX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(UART_PORT, &gpio);

    usart.USART_BaudRate            = UART_BAUDRATE;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(UART_USART, &usart);

    USART_Cmd(UART_USART, ENABLE);
}

void uart_send_byte(uint8_t b)
{
    /* 等发送寄存器空,再写入一个字节 */
    while (USART_GetFlagStatus(UART_USART, USART_FLAG_TXE) == RESET) {
    }
    USART_SendData(UART_USART, b);
}

/* 兼容 Keil 风格的字符输出调用,转给串口。 */
int fputc(int ch, FILE *f)
{
    (void)f;
    uart_send_byte((uint8_t)ch);
    return ch;
}

/* GCC/newlib 的 printf 通过 _write() 输出到标准输出。 */
int _write(int file, char *ptr, int len)
{
    int i;

    (void)file;
    for (i = 0; i < len; i++) {
        uart_send_byte((uint8_t)ptr[i]);
    }
    return len;
}
