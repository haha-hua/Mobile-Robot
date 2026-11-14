#ifndef OBSTACLE_H
#define OBSTACLE_H
#include "motion.h"

typedef enum {
    AVOID_TEST, AVOID_FORWARD, AVOID_STOPPING, AVOID_TURN_LEFT, AVOID_FAULT,
    AVOID_DONE
} avoid_stage_t;

typedef struct {
    uint8_t raw_right, raw_left;         /* PA1(右)、PA2(左)原始引脚电平 */
    uint8_t right_blocked, left_blocked; /* PA1(右)、PA2(左)消抖后遮挡状态(1=有障碍) */
    uint8_t clear_right, clear_left;
    uint8_t raw_level, blocked, clear_samples, turn_required;
    uint8_t ipu_level, ipd_level, floating;
    uint8_t seen_low, seen_high, low_streak, high_streak, fast_last_level;
    uint16_t adc_raw, millivolts, min_mv, max_mv;
    uint32_t edge_count, obstacle_count;
    uint32_t low_ms, high_ms;
    avoid_stage_t stage;
    uint32_t stage_tick, stable_ms, turn_clear_ms;
    float turn_mrad;
    float total_mm; /* 平均轮程累计的前进里程，原地转向近似为0。 */
} obstacle_t;

void obstacle_init(obstacle_t *sensor);
void obstacle_sample(obstacle_t *sensor);
void obstacle_fast_sample_1ms(obstacle_t *sensor);
void obstacle_reset_diag(obstacle_t *sensor);
const char *obstacle_diag_summary(const obstacle_t *sensor);
void obstacle_disable(obstacle_t *sensor);
void obstacle_start(obstacle_t *sensor, motion_t *motion, uint32_t tick);
void obstacle_step(obstacle_t *sensor, motion_t *motion, uint32_t tick,
                   int32_t speed_left, int32_t speed_right);
const char *obstacle_stage_name(avoid_stage_t stage);
#endif
