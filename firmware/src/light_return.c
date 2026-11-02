#include "light_return.h"
#include "config.h"
#include <math.h>
#include <stdlib.h>

#define PI 3.14159265358979323846f

static float wrap_pi(float angle)
{
    while (angle > PI) angle -= 2.0f * PI;
    while (angle <= -PI) angle += 2.0f * PI;
    return angle;
}

static void begin_stage(light_return_t *seq, light_return_stage_t stage, uint32_t tick)
{
    seq->stage = stage;
    seq->stage_tick = tick;
    seq->stable_ms = 0u;
    seq->voltage_sum = seq->voltage_samples = 0u;
}

/* 连续窗口内峰峰值不超过容差，窗口均值作为稳定电压。 */
static uint8_t voltage_stable(light_return_t *seq, uint16_t mv)
{
    if (seq->voltage_samples == 0u) seq->voltage_min = seq->voltage_max = mv;
    if (mv < seq->voltage_min) seq->voltage_min = mv;
    if (mv > seq->voltage_max) seq->voltage_max = mv;
    if (seq->voltage_max - seq->voltage_min > LIGHT_RETURN_STABLE_MV) {
        seq->voltage_min = seq->voltage_max = mv;
        seq->voltage_sum = seq->voltage_samples = 0u;
    }
    seq->voltage_sum += mv;
    seq->voltage_samples++;
    if (seq->voltage_samples * SPEED_SAMPLE_MS < LIGHT_RETURN_STABLE_MS) return 0u;
    seq->stable_mv = (uint16_t)((seq->voltage_sum + seq->voltage_samples / 2u) /
                               seq->voltage_samples);
    return 1u;
}

static int32_t steering(float error, uint8_t turning)
{
    float w = error * 2000.0f;
    if (w > LIGHT_RETURN_TURN_MRAD_S) w = LIGHT_RETURN_TURN_MRAD_S;
    if (w < -LIGHT_RETURN_TURN_MRAD_S) w = -LIGHT_RETURN_TURN_MRAD_S;
    if (turning && fabsf(w) < LIGHT_RETURN_MIN_TURN_MRAD_S)
        w = error > 0.0f ? LIGHT_RETURN_MIN_TURN_MRAD_S : -LIGHT_RETURN_MIN_TURN_MRAD_S;
    return (int32_t)w;
}

static void stop_then(light_return_t *seq, motion_t *motion,
                      light_return_stage_t next, uint32_t tick)
{
    motion_stop(motion);
    seq->next_stage = next;
    begin_stage(seq, LIGHT_RETURN_STOPPING, tick);
}

void light_return_init(light_return_t *seq, const odometry_t *odom, uint32_t tick)
{
    *seq = (light_return_t){0};
    seq->boot_tick = tick;
    seq->stage_tick = tick;
    seq->origin_x_m = odom->x_m;
    seq->origin_y_m = odom->y_m;
    seq->outbound_yaw = odom->yaw_rad;
}

void light_return_cancel(light_return_t *seq)
{
    if (seq->stage < LIGHT_RETURN_DONE) seq->stage = LIGHT_RETURN_CANCELLED;
}

void light_return_step(light_return_t *seq, motion_t *motion, const odometry_t *odom,
                       uint32_t tick, uint16_t light_mv,
                       int32_t speed_left, int32_t speed_right)
{
    float dx, dy, distance, error;
    int32_t v;
    if (seq->stage >= LIGHT_RETURN_DONE) return;
    if ((uint32_t)(tick - seq->boot_tick) >= AUTO_TOTAL_TIMEOUT_MS ||
        (uint32_t)(tick - seq->stage_tick) >= AUTO_STAGE_TIMEOUT_MS) {
        motion_stop(motion);
        seq->stage = LIGHT_RETURN_TIMEOUT;
        return;
    }
    switch (seq->stage) {
    case LIGHT_RETURN_BASELINE:
        motion_stop(motion);
        if (voltage_stable(seq, light_mv)) {
            seq->baseline_mv = seq->stable_mv;
            begin_stage(seq, LIGHT_RETURN_OUT, tick);
        }
        break;
    case LIGHT_RETURN_OUT:
        /* 行驶中只在完整2秒窗口结束后，用窗口均值判断颜色变化。 */
        if (voltage_stable(seq, light_mv)) {
            if (abs((int)seq->stable_mv - (int)seq->baseline_mv) >
                LIGHT_RETURN_TRIGGER_MV) {
                seq->triggered = 1u;
                stop_then(seq, motion, LIGHT_RETURN_TURN, tick);
                break;
            }
            begin_stage(seq, LIGHT_RETURN_OUT, tick);
        }
        motion_set_velocity(motion, LIGHT_RETURN_SPEED_MM_S,
                            steering(wrap_pi(seq->outbound_yaw - odom->yaw_rad), 0u), tick);
        break;
    case LIGHT_RETURN_STOPPING:
        motion_stop(motion);
        if (abs(motion_pulses_to_mm(speed_left, LEFT_COUNTS_10_REVS)) < AUTO_STOP_SPEED_MM_S &&
            abs(motion_pulses_to_mm(speed_right, RIGHT_COUNTS_10_REVS)) < AUTO_STOP_SPEED_MM_S)
            seq->stable_ms += SPEED_SAMPLE_MS;
        else seq->stable_ms = 0u;
        if (seq->stable_ms >= AUTO_STABLE_MS &&
            (uint32_t)(tick - seq->stage_tick) >= AUTO_SETTLE_MS) {
            seq->last_yaw = odom->yaw_rad;
            seq->turn_rad = 0.0f;
            begin_stage(seq, seq->next_stage, tick);
        }
        break;
    case LIGHT_RETURN_TURN:
        seq->turn_rad += wrap_pi(odom->yaw_rad - seq->last_yaw);
        seq->last_yaw = odom->yaw_rad;
        error = PI - seq->turn_rad;
        if (fabsf(error) <= LIGHT_RETURN_YAW_TOL_MRAD * 0.001f)
            stop_then(seq, motion, LIGHT_RETURN_HOME, tick);
        else motion_set_velocity(motion, 0, steering(error, 1u), tick);
        break;
    case LIGHT_RETURN_HOME:
        dx = seq->origin_x_m - odom->x_m;
        dy = seq->origin_y_m - odom->y_m;
        distance = hypotf(dx, dy);
        if (distance <= LIGHT_RETURN_POSITION_TOL_MM * 0.001f) {
            stop_then(seq, motion, LIGHT_RETURN_DONE, tick);
            break;
        }
        error = wrap_pi(atan2f(dy, dx) - odom->yaw_rad);
        if (fabsf(error) > 0.3f) {
            motion_set_velocity(motion, 0, steering(error, 1u), tick);
            break;
        }
        v = (int32_t)(distance * 1000.0f);
        if (v > LIGHT_RETURN_SPEED_MM_S) v = LIGHT_RETURN_SPEED_MM_S;
        if (v < AUTO_DRIVE_MIN_MM_S) v = AUTO_DRIVE_MIN_MM_S;
        motion_set_velocity(motion, v, steering(error, 0u), tick);
        break;
    default:
        break;
    }
}

const char *light_return_stage_name(light_return_stage_t stage)
{
    static const char *names[] = {"baseline", "out", "stopping", "turn_180",
        "home", "done", "cancelled", "timeout"};
    return names[stage];
}
