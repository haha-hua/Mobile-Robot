#include "color_sensor.h"
#include "config.h"
#include "rgb_led.h"

static uint32_t phase_sum;
static uint8_t phase_samples;

static void start_phase(color_sensor_t *sensor, color_phase_t phase)
{
    sensor->phase = phase;
    phase_sum = 0u;
    phase_samples = 0u;
    if (phase == COLOR_PHASE_RED) rgb_led_set(RGB_LED_RED);
    else if (phase == COLOR_PHASE_GREEN) rgb_led_set(RGB_LED_GREEN);
    else if (phase == COLOR_PHASE_BLUE) rgb_led_set(RGB_LED_BLUE);
    else rgb_led_set(RGB_LED_OFF);
}

static uint32_t corrected(uint32_t signal, uint32_t gain_milli)
{
    return signal * gain_milli / 1000u;
}

static uint32_t light_signal(uint16_t ambient, uint16_t raw)
{
#if COLOR_ADC_INVERTED
    return ambient > raw ? (uint32_t)(ambient - raw) : 0u;
#else
    return raw > ambient ? (uint32_t)(raw - ambient) : 0u;
#endif
}

static uint32_t white_gain(uint32_t signal)
{
    uint32_t gain;
    if (signal == 0u) return 1000u;
    gain = (1000000u + signal / 2u) / signal;
    if (gain < 100u) gain = 100u;
    if (gain > 10000u) gain = 10000u;
    return gain;
}

static void finish_frame(color_sensor_t *sensor)
{
    color_label_t candidate = COLOR_UNKNOWN;
    sensor->signal_red = light_signal(sensor->ambient, sensor->raw_red);
    sensor->signal_green = light_signal(sensor->ambient, sensor->raw_green);
    sensor->signal_blue = light_signal(sensor->ambient, sensor->raw_blue);
    sensor->signal_red = corrected(sensor->signal_red, sensor->gain_red_milli);
    sensor->signal_green = corrected(sensor->signal_green, sensor->gain_green_milli);
    sensor->signal_blue = corrected(sensor->signal_blue, sensor->gain_blue_milli);
    sensor->signal_sum = sensor->signal_red + sensor->signal_green + sensor->signal_blue;
    sensor->norm_red = sensor->signal_sum == 0u ? 0u :
                       (uint16_t)(sensor->signal_red * 1000u / sensor->signal_sum);
    sensor->norm_green = sensor->signal_sum == 0u ? 0u :
                         (uint16_t)(sensor->signal_green * 1000u / sensor->signal_sum);
    sensor->norm_blue = sensor->signal_sum == 0u ? 0u :
                        (uint16_t)(sensor->signal_blue * 1000u / sensor->signal_sum);

    if (sensor->signal_sum >= COLOR_MIN_SIGNAL) {
        if (sensor->norm_red >= COLOR_RED_MIN_R &&
            sensor->norm_green <= COLOR_RED_MAX_G) {
            candidate = COLOR_RED;
        } else if (sensor->norm_green >= COLOR_GREEN_MIN_G &&
                   sensor->norm_blue <= COLOR_GREEN_MAX_B) {
            candidate = COLOR_GREEN;
        } else if (sensor->norm_green >= COLOR_YELLOW_MIN_G &&
                   sensor->norm_green <= COLOR_YELLOW_MAX_G &&
                   sensor->norm_blue >= COLOR_YELLOW_MIN_B) {
            candidate = COLOR_YELLOW;
        } else if (sensor->norm_red >= COLOR_BLUE_MIN_R &&
                   sensor->norm_green >= COLOR_BLUE_MIN_G &&
                   sensor->norm_green <= COLOR_BLUE_MAX_G &&
                   sensor->norm_blue <= COLOR_BLUE_MAX_B) {
            candidate = COLOR_BLUE;
        }
    }

    if (candidate == sensor->candidate) {
        if (sensor->stable_frames < 255u) sensor->stable_frames++;
    } else {
        sensor->candidate = candidate;
        sensor->stable_frames = 1u;
    }
    if (sensor->stable_frames >= COLOR_STABLE_FRAMES)
        sensor->label = candidate;
    sensor->frame_ready = 1u;
    sensor->frame_count++;
}

void color_sensor_init(color_sensor_t *sensor)
{
    *sensor = (color_sensor_t){0};
    sensor->gain_red_milli = 1000u;
    sensor->gain_green_milli = 1000u;
    sensor->gain_blue_milli = 1000u;
    start_phase(sensor, COLOR_PHASE_OFF);
}

void color_sensor_step(color_sensor_t *sensor, uint16_t adc_raw)
{
    uint8_t total_samples = COLOR_SETTLE_SAMPLES + COLOR_MEASURE_SAMPLES;
    phase_samples++;
    if (phase_samples > COLOR_SETTLE_SAMPLES) phase_sum += adc_raw;
    if (phase_samples < total_samples) return;

    if (sensor->phase == COLOR_PHASE_OFF) {
        sensor->ambient = (uint16_t)(phase_sum / COLOR_MEASURE_SAMPLES);
        start_phase(sensor, COLOR_PHASE_RED);
    } else if (sensor->phase == COLOR_PHASE_RED) {
        sensor->raw_red = (uint16_t)(phase_sum / COLOR_MEASURE_SAMPLES);
        start_phase(sensor, COLOR_PHASE_GREEN);
    } else if (sensor->phase == COLOR_PHASE_GREEN) {
        sensor->raw_green = (uint16_t)(phase_sum / COLOR_MEASURE_SAMPLES);
        start_phase(sensor, COLOR_PHASE_BLUE);
    } else {
        sensor->raw_blue = (uint16_t)(phase_sum / COLOR_MEASURE_SAMPLES);
        finish_frame(sensor);
        start_phase(sensor, COLOR_PHASE_OFF);
    }
}

uint8_t color_sensor_calibrate_white(color_sensor_t *sensor)
{
    uint32_t red, green, blue;
    if (!sensor->frame_ready) return 0u;
    red = light_signal(sensor->ambient, sensor->raw_red);
    green = light_signal(sensor->ambient, sensor->raw_green);
    blue = light_signal(sensor->ambient, sensor->raw_blue);
    if (red + green + blue < COLOR_MIN_SIGNAL) return 0u;
    /* 以未校正信号计算，重复发送命令不会把旧增益再次乘进去。 */
    sensor->gain_red_milli = white_gain(red);
    sensor->gain_green_milli = white_gain(green);
    sensor->gain_blue_milli = white_gain(blue);
    return 1u;
}

void color_sensor_reset_calibration(color_sensor_t *sensor)
{
    sensor->gain_red_milli = 1000u;
    sensor->gain_green_milli = 1000u;
    sensor->gain_blue_milli = 1000u;
}

const char *color_sensor_phase_name(color_phase_t phase)
{
    static const char *names[] = {"off", "red", "green", "blue"};
    return names[phase];
}

const char *color_sensor_label_name(color_label_t label)
{
    static const char *names[] = {"unknown", "red", "green", "blue", "yellow"};
    return names[label];
}
