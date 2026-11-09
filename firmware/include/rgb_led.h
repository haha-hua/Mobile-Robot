#ifndef RGB_LED_H
#define RGB_LED_H

#include <stdint.h>

typedef enum {
    RGB_LED_OFF = 0,
    RGB_LED_RED,
    RGB_LED_GREEN,
    RGB_LED_BLUE
} rgb_led_color_t;

void rgb_led_init(void);
void rgb_led_set(rgb_led_color_t color);

#endif
