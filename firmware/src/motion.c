#include "motion.h"
#include "config.h"
#include "kinematics.h"

int32_t motion_pulses_to_mm(int32_t pulses, int32_t counts_10_revs)
{
    return (int32_t)((int64_t)pulses * WHEEL_CIRCUMFERENCE_UM * 10 /
                     ((int64_t)counts_10_revs * 1000));
}

static int32_t mm_to_pps(int32_t speed, int32_t counts)
{
    return (int32_t)((int64_t)speed * counts * 1000 / (WHEEL_CIRCUMFERENCE_UM * 10));
}

void motion_init(motion_t *motion)
{
    *motion = (motion_t){0};
}

void motion_stop(motion_t *motion)
{
    motion->requested_left = motion->requested_right = 0;
    motion->ramp_left = motion->ramp_right = 0;
    motion->target_left = motion->target_right = 0;
    motion->command_active = 0u;
    motion->state = MOTION_IDLE;
}

static void set_request(motion_t *motion, int32_t left, int32_t right, uint32_t tick)
{
    motion->requested_left = left;
    motion->requested_right = right;
    motion->command_tick = tick;
    motion->command_active = 1u;
    motion->state = (left || right || motion->ramp_left || motion->ramp_right) ?
                    MOTION_RUN : MOTION_IDLE;
}

uint8_t motion_set_wheel_speeds(motion_t *motion, int32_t left, int32_t right, uint32_t tick)
{
    if (left < -MAX_WHEEL_SPEED_MM_S || left > MAX_WHEEL_SPEED_MM_S ||
        right < -MAX_WHEEL_SPEED_MM_S || right > MAX_WHEEL_SPEED_MM_S) return 0u;
    set_request(motion, left, right, tick);
    return 1u;
}

uint8_t motion_set_velocity(motion_t *motion, int32_t v, int32_t w, uint32_t tick)
{
    int32_t left, right;
    if (v < -MAX_LINEAR_SPEED_MM_S || v > MAX_LINEAR_SPEED_MM_S ||
        w < -MAX_ANGULAR_SPEED_MRAD_S || w > MAX_ANGULAR_SPEED_MRAD_S) return 0u;
    kinematics_inverse(v, w, &left, &right);
    set_request(motion, left, right, tick);
    return 1u;
}

static int32_t ramp(int32_t current, int32_t requested, int32_t actual)
{
    const int32_t step = WHEEL_ACCEL_MM_S2 * SPEED_SAMPLE_MS / 1000;
    int32_t next;
    if ((current > 0 && requested < 0) || (current < 0 && requested > 0)) requested = 0;
    if (current == 0 && ((requested < 0 && actual > REVERSE_SAFE_SPEED_MM_S) ||
                        (requested > 0 && actual < -REVERSE_SAFE_SPEED_MM_S))) return 0;
    if (requested > current) {
        next = current + step;
        return next > requested ? requested : next;
    }
    next = current - step;
    return next < requested ? requested : next;
}

void motion_step(motion_t *motion, uint32_t tick,
                 int32_t actual_left, int32_t actual_right)
{
    if (motion->command_active &&
        (uint32_t)(tick - motion->command_tick) >= COMMAND_TIMEOUT_MS) {
        motion_stop(motion);
        motion->state = MOTION_TIMEOUT;
        return;
    }
    if (!motion->command_active) return;
    motion->ramp_left = ramp(motion->ramp_left, motion->requested_left,
                             motion_pulses_to_mm(actual_left, LEFT_COUNTS_10_REVS));
    motion->ramp_right = ramp(motion->ramp_right, motion->requested_right,
                              motion_pulses_to_mm(actual_right, RIGHT_COUNTS_10_REVS));
    motion->target_left = mm_to_pps(motion->ramp_left, LEFT_COUNTS_10_REVS);
    motion->target_right = mm_to_pps(motion->ramp_right, RIGHT_COUNTS_10_REVS);
    motion->state = (motion->requested_left || motion->requested_right ||
                     motion->ramp_left || motion->ramp_right) ? MOTION_RUN : MOTION_IDLE;
}
