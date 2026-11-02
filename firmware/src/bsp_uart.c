#include "stm32f10x.h"
#include "config.h"
#include "bsp_uart.h"
#include <string.h>

static char tx_buffer[512];
static volatile uint16_t tx_length;
static uint16_t tx_position;
#define RX_CAPACITY 128u
static volatile uint8_t rx_buffer[RX_CAPACITY];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_discard, rx_fault;
static char partial[UART_LINE_CAPACITY];
static uint16_t partial_length;
static uint8_t line_discard;

void uart_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;
    RCC_APB2PeriphClockCmd(UART_GPIO_CLK | RCC_APB2Periph_AFIO | UART_USART_CLK, ENABLE);
    gpio.GPIO_Pin = UART_TX_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(UART_PORT, &gpio);
    gpio.GPIO_Pin = UART_RX_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(UART_PORT, &gpio);
    usart.USART_BaudRate = UART_BAUDRATE;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(UART_USART, &usart);
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_ITConfig(UART_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART_USART, ENABLE);
}

uint8_t uart_try_write(const char *data, uint16_t length)
{
    if (tx_length != 0u || length == 0u || length > sizeof(tx_buffer)) return 0u;
    /* 只有主循环提交日志，空闲时发送中断已禁用。 */
    memcpy(tx_buffer, data, length);
    tx_position = 0u;
    tx_length = length;
    USART_ITConfig(UART_USART, USART_IT_TXE, ENABLE);
    return 1u;
}

int8_t uart_read_line(char line[UART_LINE_CAPACITY])
{
    for (;;) {
        uint8_t byte;
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (rx_fault) {
            rx_fault = 0u;
            partial_length = 0u;
            line_discard = 0u;
            __set_PRIMASK(primask);
            return -1;
        }
        if (rx_head == rx_tail) {
            __set_PRIMASK(primask);
            return 0;
        }
        byte = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1u) % RX_CAPACITY;
        __set_PRIMASK(primask);
        if (byte == '\r' || byte == '\n') {
            if (line_discard) {
                partial_length = 0u;
                line_discard = 0u;
                return -1;
            }
            if (partial_length == 0u) continue;
            partial[partial_length] = '\0';
            memcpy(line, partial, partial_length + 1u);
            partial_length = 0u;
            return 1;
        }
        if (line_discard) continue;
        if (byte == 0u || partial_length >= UART_LINE_CAPACITY - 1u) {
            line_discard = 1u;
        } else {
            partial[partial_length++] = (char)byte;
        }
    }
}

void USART1_IRQHandler(void)
{
    uint16_t flags = UART_USART->SR;
    if (flags & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        uint8_t byte = (uint8_t)USART_ReceiveData(UART_USART);
        uint16_t next = (rx_head + 1u) % RX_CAPACITY;
        if ((flags & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) ||
            (!rx_discard && next == rx_tail)) {
            /* 环形缓冲失步时连同已缓存内容丢弃，直到完整行边界再接收。 */
            rx_tail = rx_head;
            rx_fault = 1u;
            rx_discard = 1u;
        }
        if (rx_discard) {
            if (byte == '\r' || byte == '\n') rx_discard = 0u;
        } else {
            rx_buffer[rx_head] = byte;
            rx_head = next;
        }
    }
    if (USART_GetITStatus(UART_USART, USART_IT_TXE) != RESET) {
        USART_SendData(UART_USART, (uint8_t)tx_buffer[tx_position++]);
        if (tx_position >= tx_length) {
            USART_ITConfig(UART_USART, USART_IT_TXE, DISABLE);
            tx_length = 0u;
        }
    }
}
