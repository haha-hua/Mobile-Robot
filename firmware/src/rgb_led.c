#include "rgb_led.h"
#include "stm32f10x.h"

#define RGB_PORT GPIOE
#define RGB_GPIO_CLK RCC_APB2Periph_GPIOE
#define RGB_RED_PIN GPIO_Pin_9
#define RGB_GREEN_PIN GPIO_Pin_11
#define RGB_BLUE_PIN GPIO_Pin_13
#define RGB_ALL_PINS (RGB_RED_PIN | RGB_GREEN_PIN | RGB_BLUE_PIN)

void rgb_led_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RGB_GPIO_CLK, ENABLE);
    gpio.GPIO_Pin = RGB_ALL_PINS;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(RGB_PORT, &gpio);
    rgb_led_set(RGB_LED_OFF);
}

void rgb_led_set(rgb_led_color_t color)
{
    /* 共阳灯：阴极输出低电平点亮，先全灭保证一次只亮一个颜色。 */
    GPIO_SetBits(RGB_PORT, RGB_ALL_PINS);
    if (color == RGB_LED_RED) GPIO_ResetBits(RGB_PORT, RGB_RED_PIN);
    else if (color == RGB_LED_GREEN) GPIO_ResetBits(RGB_PORT, RGB_GREEN_PIN);
    else if (color == RGB_LED_BLUE) GPIO_ResetBits(RGB_PORT, RGB_BLUE_PIN);
}
