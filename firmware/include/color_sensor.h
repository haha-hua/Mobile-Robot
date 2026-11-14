#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <stdint.h>

#define COLOR_CLASS_COUNT 4u

typedef enum {
    COLOR_PHASE_OFF = 0,
    COLOR_PHASE_RED,
    COLOR_PHASE_GREEN,
    COLOR_PHASE_BLUE
} color_phase_t;

typedef enum {
    COLOR_UNKNOWN = 0,
    COLOR_WHITE = 1,
    COLOR_YELLOW = 2,
    COLOR_RED = 3,
    COLOR_BLUE = 4
} color_label_t;

typedef struct {
    uint16_t r;
    uint16_t g;
    uint16_t b;
} color_centroid_t;

typedef struct {
    color_phase_t phase;
    color_label_t label;
    color_label_t candidate;
    uint8_t stable_frames;
    uint8_t frame_ready;
    uint8_t calibrated_mask;
    uint32_t frame_count;
    uint16_t ambient;
    uint16_t raw_red, raw_green, raw_blue;
    uint16_t raw_sum;
    uint32_t signal_red, signal_green, signal_blue;
    uint16_t norm_red, norm_green, norm_blue;
    uint32_t signal_sum;
    uint32_t min_dist_sq;
    uint32_t gain_red_milli, gain_green_milli, gain_blue_milli;
    color_centroid_t centroids[COLOR_CLASS_COUNT];
} color_sensor_t;

void color_sensor_init(color_sensor_t *sensor);
void color_sensor_step(color_sensor_t *sensor, uint16_t adc_raw);
uint8_t color_sensor_calibrate_white(color_sensor_t *sensor);
uint8_t color_sensor_calibrate_color(color_sensor_t *sensor, color_label_t target);
void color_sensor_reset_calibration(color_sensor_t *sensor);
uint8_t color_sensor_save_calibration(const color_sensor_t *sensor);
void color_sensor_erase_calibration(void);
const char *color_sensor_phase_name(color_phase_t phase);
const char *color_sensor_label_name(color_label_t label);

#endif
