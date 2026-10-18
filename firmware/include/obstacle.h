#ifndef OBSTACLE_H
#define OBSTACLE_H
#include "motion.h"

typedef enum {
    AVOID_TEST, AVOID_FORWARD, AVOID_STOPPING, AVOID_TURN_LEFT, AVOID_FAULT,
    AVOID_DONE
} avoid_stage_t;

typedef struct {
    uint8_t raw_level, blocked, clear_samples, turn_required;
    avoid_stage_t stage;
    uint32_t stage_tick, stable_ms, turn_clear_ms;
    float turn_mrad;
    float total_mm; /* 平均轮程累计的前进里程，原地转向近似为0。 */
} obstacle_t;

void obstacle_init(obstacle_t *sensor);
void obstacle_sample(obstacle_t *sensor);
void obstacle_disable(obstacle_t *sensor);
void obstacle_start(obstacle_t *sensor, motion_t *motion, uint32_t tick);
void obstacle_step(obstacle_t *sensor, motion_t *motion, uint32_t tick,
                   int32_t speed_left, int32_t speed_right);
const char *obstacle_stage_name(avoid_stage_t stage);
#endif
