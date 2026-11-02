#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H
#include <stdint.h>

typedef struct {
    uint16_t adc_raw;
    uint16_t millivolts;
    uint8_t do_raw;
    uint8_t dark;
    uint8_t candidate;
    uint8_t candidate_samples;
} light_sensor_t;

void light_sensor_init(light_sensor_t *sensor);
void light_sensor_sample(light_sensor_t *sensor);
#endif
