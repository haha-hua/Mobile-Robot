#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <stdint.h>

typedef enum {
    COLOR_PHASE_OFF = 0,
    COLOR_PHASE_RED,
    COLOR_PHASE_GREEN,
    COLOR_PHASE_BLUE
} color_phase_t;

typedef enum {
    COLOR_UNKNOWN = 0,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_YELLOW
} color_label_t;

typedef struct {
    color_phase_t phase;
    color_label_t label;
    color_label_t candidate;
    uint8_t stable_frames;
    uint8_t frame_ready;
    uint32_t frame_count;
    uint16_t ambient;
    uint16_t raw_red, raw_green, raw_blue;
    uint32_t signal_red, signal_green, signal_blue;
    uint16_t norm_red, norm_green, norm_blue;
    uint32_t signal_sum;
    uint32_t gain_red_milli, gain_green_milli, gain_blue_milli;
} color_sensor_t;

void color_sensor_init(color_sensor_t *sensor);
void color_sensor_step(color_sensor_t *sensor, uint16_t adc_raw);
uint8_t color_sensor_calibrate_white(color_sensor_t *sensor);
void color_sensor_reset_calibration(color_sensor_t *sensor);
const char *color_sensor_phase_name(color_phase_t phase);
const char *color_sensor_label_name(color_label_t label);

#endif
