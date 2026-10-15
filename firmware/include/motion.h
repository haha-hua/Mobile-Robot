#ifndef MOTION_H
#define MOTION_H
#include <stdint.h>
typedef enum { MOTION_IDLE, MOTION_RUN, MOTION_TIMEOUT } motion_state_t;
typedef struct {
    motion_state_t state;
    uint32_t command_tick;
    uint8_t command_active;
    int32_t requested_left, requested_right;
    int32_t ramp_left, ramp_right; /* mm/s */
    int32_t target_left, target_right; /* ticks/s, existing PI input units */
} motion_t;
void motion_init(motion_t *motion);
uint8_t motion_set_velocity(motion_t *motion, int32_t v, int32_t w, uint32_t tick);
uint8_t motion_set_wheel_speeds(motion_t *motion, int32_t left, int32_t right, uint32_t tick);
void motion_stop(motion_t *motion);
void motion_step(motion_t *motion, uint32_t tick,
                 int32_t actual_left, int32_t actual_right);
/* 累计脉冲转毫米，脉冲每秒转毫米每秒，保留原整数截断。 */
int32_t motion_pulses_to_mm(int32_t pulses, int32_t counts_10_revs);
#endif
