#ifndef LIGHT_RETURN_H
#define LIGHT_RETURN_H
#include "motion.h"
#include "odometry.h"

typedef enum {
    LIGHT_RETURN_BASELINE, LIGHT_RETURN_OUT, LIGHT_RETURN_STOPPING,
    LIGHT_RETURN_TURN,
    LIGHT_RETURN_HOME, LIGHT_RETURN_DONE, LIGHT_RETURN_CANCELLED,
    LIGHT_RETURN_TIMEOUT
} light_return_stage_t;

typedef struct {
    light_return_stage_t stage, next_stage;
    uint32_t boot_tick, stage_tick, stable_ms;
    uint32_t voltage_sum, voltage_samples;
    uint16_t voltage_min, voltage_max, baseline_mv, stable_mv;
    uint8_t triggered;
    float origin_x_m, origin_y_m, outbound_yaw, last_yaw, turn_rad;
} light_return_t;

void light_return_init(light_return_t *seq, const odometry_t *odom, uint32_t tick);
void light_return_cancel(light_return_t *seq);
void light_return_step(light_return_t *seq, motion_t *motion, const odometry_t *odom,
                       uint32_t tick, uint16_t light_mv,
                       int32_t speed_left, int32_t speed_right);
const char *light_return_stage_name(light_return_stage_t stage);
#endif
