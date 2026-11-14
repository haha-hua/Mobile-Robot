#include "color_sensor.h"
#include "config.h"
#include "rgb_led.h"

#if defined(USE_STDPERIPH_DRIVER)
#include "stm32f10x_flash.h"
#define COLOR_FLASH_PAGE_ADDR 0x0803F800u
#define COLOR_FLASH_MAGIC     0x434C5235u
#endif

static uint32_t phase_sum;
static uint8_t phase_samples;

static void reset_default_centroids(color_sensor_t *sensor)
{
    sensor->centroids[COLOR_WHITE - 1] = (color_centroid_t){
        COLOR_WHITE_DEFAULT_R, COLOR_WHITE_DEFAULT_G, COLOR_WHITE_DEFAULT_B
    };
    sensor->centroids[COLOR_YELLOW - 1] = (color_centroid_t){
        COLOR_YELLOW_DEFAULT_R, COLOR_YELLOW_DEFAULT_G, COLOR_YELLOW_DEFAULT_B
    };
    sensor->centroids[COLOR_RED - 1] = (color_centroid_t){
        COLOR_RED_DEFAULT_R, COLOR_RED_DEFAULT_G, COLOR_RED_DEFAULT_B
    };
    sensor->centroids[COLOR_BLUE - 1] = (color_centroid_t){
        COLOR_BLUE_DEFAULT_R, COLOR_BLUE_DEFAULT_G, COLOR_BLUE_DEFAULT_B
    };
}

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
    return (signal * gain_milli + 500u) / 1000u;
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
    /* 放宽增益上限至200倍，避免反射ADC变化量较小时增益被截断导致白平衡偏向绿光。 */
    if (gain > 200000u) gain = 200000u;
    return gain;
}

static void recompute_signals(color_sensor_t *sensor)
{
    uint32_t raw_r = light_signal(sensor->ambient, sensor->raw_red);
    uint32_t raw_g = light_signal(sensor->ambient, sensor->raw_green);
    uint32_t raw_b = light_signal(sensor->ambient, sensor->raw_blue);
    uint32_t raw_total = raw_r + raw_g + raw_b;

    sensor->raw_sum = raw_total > 65535u ? 65535u : (uint16_t)raw_total;
    sensor->signal_red = corrected(raw_r, sensor->gain_red_milli);
    sensor->signal_green = corrected(raw_g, sensor->gain_green_milli);
    sensor->signal_blue = corrected(raw_b, sensor->gain_blue_milli);
    sensor->signal_sum = sensor->signal_red + sensor->signal_green + sensor->signal_blue;

    if (sensor->signal_sum == 0u) {
        sensor->norm_red = 0u;
        sensor->norm_green = 0u;
        sensor->norm_blue = 0u;
    } else {
        sensor->norm_red = (uint16_t)((sensor->signal_red * 1000u + sensor->signal_sum / 2u) /
                                      sensor->signal_sum);
        sensor->norm_green = (uint16_t)((sensor->signal_green * 1000u + sensor->signal_sum / 2u) /
                                        sensor->signal_sum);
        sensor->norm_blue = (uint16_t)((sensor->signal_blue * 1000u + sensor->signal_sum / 2u) /
                                       sensor->signal_sum);
    }
}

static color_label_t classify_nearest(const color_sensor_t *sensor, uint32_t *min_dist_out)
{
    uint8_t i;
    uint8_t best_index = 0u;
    uint32_t best_dist = 0xFFFFFFFFu;

    if (sensor->raw_sum < COLOR_MIN_RAW_SUM || sensor->signal_sum == 0u) {
        *min_dist_out = 0u;
        return COLOR_UNKNOWN;
    }

    for (i = 0u; i < COLOR_CLASS_COUNT; i++) {
        int32_t dr = (int32_t)sensor->norm_red - (int32_t)sensor->centroids[i].r;
        int32_t dg = (int32_t)sensor->norm_green - (int32_t)sensor->centroids[i].g;
        int32_t db = (int32_t)sensor->norm_blue - (int32_t)sensor->centroids[i].b;
        uint32_t dist = (uint32_t)(dr * dr + dg * dg + db * db);
        if (dist < best_dist) {
            best_dist = dist;
            best_index = i;
        }
    }

    *min_dist_out = best_dist;
    return (color_label_t)(best_index + 1u);
}

static void finish_frame(color_sensor_t *sensor)
{
    uint32_t min_dist = 0u;
    color_label_t candidate;

    recompute_signals(sensor);
    candidate = classify_nearest(sensor, &min_dist);
    sensor->min_dist_sq = min_dist;

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

#if defined(USE_STDPERIPH_DRIVER)
typedef struct {
    uint32_t magic;
    uint32_t gain_red_milli;
    uint32_t gain_green_milli;
    uint32_t gain_blue_milli;
    uint32_t rg[COLOR_CLASS_COUNT];
    uint32_t b[COLOR_CLASS_COUNT];
    uint32_t calibrated_mask;
    uint32_t checksum;
} color_flash_cal_t;

static uint32_t calc_cal_checksum(const color_flash_cal_t *cal)
{
    const uint32_t *words = (const uint32_t *)cal;
    uint32_t hash = 0xA55A3C69u;
    uint32_t count = (uint32_t)(sizeof(color_flash_cal_t) / sizeof(uint32_t)) - 1u;
    uint32_t i;
    for (i = 0u; i < count; i++)
        hash = (hash * 33u) ^ words[i];
    return hash;
}

static void load_flash_calibration(color_sensor_t *sensor)
{
    const color_flash_cal_t *stored = (const color_flash_cal_t *)COLOR_FLASH_PAGE_ADDR;
    uint8_t i;

    if (stored->magic != COLOR_FLASH_MAGIC) return;
    if (stored->checksum != calc_cal_checksum(stored)) return;
    if (stored->gain_red_milli < 100u || stored->gain_red_milli > 200000u) return;
    if (stored->gain_green_milli < 100u || stored->gain_green_milli > 200000u) return;
    if (stored->gain_blue_milli < 100u || stored->gain_blue_milli > 200000u) return;

    for (i = 0u; i < COLOR_CLASS_COUNT; i++) {
        uint16_t r = (uint16_t)(stored->rg[i] >> 16);
        uint16_t g = (uint16_t)(stored->rg[i] & 0xFFFFu);
        uint16_t b = (uint16_t)(stored->b[i] & 0xFFFFu);
        uint32_t sum = (uint32_t)r + (uint32_t)g + (uint32_t)b;
        if (r > 1000u || g > 1000u || b > 1000u || sum < 990u || sum > 1010u)
            return;
    }

    sensor->gain_red_milli = stored->gain_red_milli;
    sensor->gain_green_milli = stored->gain_green_milli;
    sensor->gain_blue_milli = stored->gain_blue_milli;
    for (i = 0u; i < COLOR_CLASS_COUNT; i++) {
        sensor->centroids[i].r = (uint16_t)(stored->rg[i] >> 16);
        sensor->centroids[i].g = (uint16_t)(stored->rg[i] & 0xFFFFu);
        sensor->centroids[i].b = (uint16_t)(stored->b[i] & 0xFFFFu);
    }
    sensor->calibrated_mask = (uint8_t)(stored->calibrated_mask & 0x1Fu);
}
#endif

void color_sensor_init(color_sensor_t *sensor)
{
    *sensor = (color_sensor_t){0};
    sensor->gain_red_milli = COLOR_DEFAULT_GAIN_RED_MILLI;
    sensor->gain_green_milli = COLOR_DEFAULT_GAIN_GREEN_MILLI;
    sensor->gain_blue_milli = COLOR_DEFAULT_GAIN_BLUE_MILLI;
    reset_default_centroids(sensor);
#if defined(USE_STDPERIPH_DRIVER)
    load_flash_calibration(sensor);
#endif
    start_phase(sensor, COLOR_PHASE_OFF);
}

void color_sensor_step(color_sensor_t *sensor, uint16_t adc_raw)
{
    uint8_t settle = (sensor->phase == COLOR_PHASE_OFF) ?
                     COLOR_OFF_SETTLE_SAMPLES : COLOR_SETTLE_SAMPLES;
    uint8_t total_samples = (uint8_t)(settle + COLOR_MEASURE_SAMPLES);

    phase_samples++;
    if (phase_samples > settle) phase_sum += adc_raw;
    if (phase_samples < total_samples) return;

    /* 点亮顺序按光敏电阻响应从弱到强：OFF -> RED -> BLUE -> GREEN -> OFF，
     * 强绿光放在最后，紧接较长的OFF暗恢复阶段，彻底消除绿光对蓝光/红光的拖尾串扰。 */
    if (sensor->phase == COLOR_PHASE_OFF) {
        sensor->ambient = (uint16_t)((phase_sum + COLOR_MEASURE_SAMPLES / 2u) /
                                     COLOR_MEASURE_SAMPLES);
        start_phase(sensor, COLOR_PHASE_RED);
    } else if (sensor->phase == COLOR_PHASE_RED) {
        sensor->raw_red = (uint16_t)((phase_sum + COLOR_MEASURE_SAMPLES / 2u) /
                                     COLOR_MEASURE_SAMPLES);
        start_phase(sensor, COLOR_PHASE_BLUE);
    } else if (sensor->phase == COLOR_PHASE_BLUE) {
        sensor->raw_blue = (uint16_t)((phase_sum + COLOR_MEASURE_SAMPLES / 2u) /
                                      COLOR_MEASURE_SAMPLES);
        start_phase(sensor, COLOR_PHASE_GREEN);
    } else {
        sensor->raw_green = (uint16_t)((phase_sum + COLOR_MEASURE_SAMPLES / 2u) /
                                       COLOR_MEASURE_SAMPLES);
        finish_frame(sensor);
        start_phase(sensor, COLOR_PHASE_OFF);
    }
}

uint8_t color_sensor_calibrate_color(color_sensor_t *sensor, color_label_t target)
{
    uint32_t red, green, blue;
    if (!sensor->frame_ready) return 0u;
    if (target < COLOR_WHITE || target > COLOR_BLUE) return 0u;

    red = light_signal(sensor->ambient, sensor->raw_red);
    green = light_signal(sensor->ambient, sensor->raw_green);
    blue = light_signal(sensor->ambient, sensor->raw_blue);
    if (red + green + blue < COLOR_MIN_RAW_SUM) return 0u;

    if (target == COLOR_WHITE) {
        if (red < COLOR_MIN_WHITE_CHANNEL ||
            green < COLOR_MIN_WHITE_CHANNEL ||
            blue < COLOR_MIN_WHITE_CHANNEL)
            return 0u;
        /* 以未校正原始信号计算白平衡增益，并重置四色默认中心点。 */
        sensor->gain_red_milli = white_gain(red);
        sensor->gain_green_milli = white_gain(green);
        sensor->gain_blue_milli = white_gain(blue);
        reset_default_centroids(sensor);
        recompute_signals(sensor);
        sensor->centroids[COLOR_WHITE - 1] = (color_centroid_t){
            sensor->norm_red, sensor->norm_green, sensor->norm_blue
        };
        sensor->calibrated_mask = 0x03u;
    } else {
        recompute_signals(sensor);
        if (sensor->signal_sum == 0u) return 0u;
        sensor->centroids[(uint8_t)target - 1u] = (color_centroid_t){
            sensor->norm_red, sensor->norm_green, sensor->norm_blue
        };
        sensor->calibrated_mask |= (uint8_t)(1u << (uint8_t)target);
    }

    sensor->candidate = target;
    sensor->label = target;
    sensor->stable_frames = COLOR_STABLE_FRAMES;
    sensor->min_dist_sq = 0u;
    return 1u;
}

uint8_t color_sensor_calibrate_white(color_sensor_t *sensor)
{
    return color_sensor_calibrate_color(sensor, COLOR_WHITE);
}

void color_sensor_reset_calibration(color_sensor_t *sensor)
{
    uint32_t min_dist = 0u;
    sensor->gain_red_milli = COLOR_DEFAULT_GAIN_RED_MILLI;
    sensor->gain_green_milli = COLOR_DEFAULT_GAIN_GREEN_MILLI;
    sensor->gain_blue_milli = COLOR_DEFAULT_GAIN_BLUE_MILLI;
    sensor->calibrated_mask = 0u;
    reset_default_centroids(sensor);
    if (sensor->frame_ready) {
        recompute_signals(sensor);
        sensor->candidate = classify_nearest(sensor, &min_dist);
        sensor->label = sensor->candidate;
        sensor->stable_frames = COLOR_STABLE_FRAMES;
        sensor->min_dist_sq = min_dist;
    }
}

uint8_t color_sensor_save_calibration(const color_sensor_t *sensor)
{
#if defined(USE_STDPERIPH_DRIVER)
    color_flash_cal_t cal = {0};
    const uint32_t *words = (const uint32_t *)&cal;
    uint32_t count = (uint32_t)(sizeof(color_flash_cal_t) / sizeof(uint32_t));
    uint32_t i;
    FLASH_Status st;

    cal.magic = COLOR_FLASH_MAGIC;
    cal.gain_red_milli = sensor->gain_red_milli;
    cal.gain_green_milli = sensor->gain_green_milli;
    cal.gain_blue_milli = sensor->gain_blue_milli;
    for (i = 0u; i < COLOR_CLASS_COUNT; i++) {
        cal.rg[i] = ((uint32_t)sensor->centroids[i].r << 16) |
                    (uint32_t)sensor->centroids[i].g;
        cal.b[i] = (uint32_t)sensor->centroids[i].b;
    }
    cal.calibrated_mask = (uint32_t)sensor->calibrated_mask;
    cal.checksum = calc_cal_checksum(&cal);

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    st = FLASH_ErasePage(COLOR_FLASH_PAGE_ADDR);
    for (i = 0u; st == FLASH_COMPLETE && i < count; i++)
        st = FLASH_ProgramWord(COLOR_FLASH_PAGE_ADDR + i * 4u, words[i]);
    FLASH_Lock();
    return st == FLASH_COMPLETE ? 1u : 0u;
#else
    (void)sensor;
    return 1u;
#endif
}

void color_sensor_erase_calibration(void)
{
#if defined(USE_STDPERIPH_DRIVER)
    if (*(const volatile uint32_t *)COLOR_FLASH_PAGE_ADDR != 0xFFFFFFFFu) {
        FLASH_Unlock();
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        (void)FLASH_ErasePage(COLOR_FLASH_PAGE_ADDR);
        FLASH_Lock();
    }
#endif
}

const char *color_sensor_phase_name(color_phase_t phase)
{
    static const char *names[] = {"off", "red", "green", "blue"};
    return names[phase];
}

const char *color_sensor_label_name(color_label_t label)
{
    static const char *names[] = {"unknown", "white", "yellow", "red", "blue"};
    return names[label];
}
