#include "auto_sequence.h"
#include "config.h"
#include <math.h>
#include <stdlib.h>

#define PI 3.14159265358979323846f
/* 每脉冲轮程(mm)，与标定一致；浮点累计避免低速时整数截断停滞。 */
#define MM_PER_PULSE_L ((float)WHEEL_CIRCUMFERENCE_UM / \
                        ((float)LEFT_COUNTS_10_REVS * 100.0f))
#define MM_PER_PULSE_R ((float)WHEEL_CIRCUMFERENCE_UM / \
                        ((float)RIGHT_COUNTS_10_REVS * 100.0f))

static float wrap_pi(float a)
{
    while (a > PI) a -= 2.0f * PI;
    while (a <= -PI) a += 2.0f * PI;
    return a;
}

static int wheel_mm_s(int32_t pulses, int32_t counts)
{
    return motion_pulses_to_mm(pulses, counts);
}

/* 原地转向：剩余角对应轮程决定轮速，w=2v/B，符号随剩余角。 */
static int32_t turn_w(float remaining_rad)
{
    float v = 2.0f * fabsf(remaining_rad) * (float)WHEEL_TRACK_MM;
    float w;
    if (v < (float)AUTO_TURN_MIN_MM_S) v = (float)AUTO_TURN_MIN_MM_S;
    if (v > (float)AUTO_TURN_MAX_MM_S) v = (float)AUTO_TURN_MAX_MM_S;
    w = v * 2000.0f / (float)WHEEL_TRACK_MM;
    return remaining_rad > 0.0f ? (int32_t)w : -(int32_t)w;
}

static int32_t drive_v(int32_t remaining_mm, int32_t cruise)
{
    int32_t v = remaining_mm * 2;
    if (v < AUTO_DRIVE_MIN_MM_S) v = AUTO_DRIVE_MIN_MM_S;
    if (v > cruise) v = cruise;
    return v;
}

static void begin_stage(auto_sequence_t *seq, seq_stage_t stage, uint32_t tick)
{
    seq->stage = stage;
    seq->stage_tick = tick;
    seq->stable_ms = 0u;
    seq->rel_mm = 0;
    seq->rel_mrad = 0;
    seq->start_avg_mm = (seq->total_left_mm + seq->total_right_mm) / 2;
    seq->start_diff_mm = seq->total_right_mm - seq->total_left_mm;
}

/* 动作段结束：撤销驱动，进入停稳等待，再切换到下一动作段。 */
static void finish_stage(auto_sequence_t *seq, motion_t *motion,
                         seq_stage_t next, uint32_t tick)
{
    motion_stop(motion);
    seq->next_stage = next;
    seq->stage = SEQ_STOPPING;
    seq->stage_tick = tick;
    seq->stable_ms = 0u;
}

void auto_sequence_init(auto_sequence_t *seq, uint32_t tick)
{
    float sweep = (float)AUTO_ARC_SWEEP_MDEG * 0.001f * PI / 180.0f;
    /* 顺时针圆弧结束后位置：圆心(Ax,0)，终点角 PI- sweep。 */
    float end_angle = PI - sweep;
    float px = (float)(AUTO_STRAIGHT_MM + AUTO_RADIUS_MM) +
               (float)AUTO_RADIUS_MM * cosf(end_angle);
    float py = (float)AUTO_RADIUS_MM * sinf(end_angle);
    float heading_after = PI * 0.5f - sweep; /* 出发段后左转90度再画弧 */
    float turn = wrap_pi(atan2f(-py, -px) - heading_after);
    *seq = (auto_sequence_t){0};
    seq->boot_tick = tick;
    seq->stage_tick = tick;
    seq->return_mm = (int32_t)(hypotf(px, py) + 0.5f);
    seq->turn_home_mrad = (int32_t)(turn * 1000.0f + (turn < 0.0f ? -0.5f : 0.5f));
}

void auto_sequence_cancel(auto_sequence_t *seq)
{
    if (seq->stage <= SEQ_STOPPING) seq->stage = SEQ_CANCELLED;
}

void auto_sequence_step(auto_sequence_t *seq, motion_t *motion, uint32_t tick,
                        int32_t delta_left, int32_t delta_right,
                        int32_t speed_left, int32_t speed_right)
{
    float sweep_rad, end_tol_rad;
    if (seq->stage > SEQ_STOPPING) return;
    seq->total_left_mm += (float)delta_left * MM_PER_PULSE_L;
    seq->total_right_mm += (float)delta_right * MM_PER_PULSE_R;
    if ((uint32_t)(tick - seq->boot_tick) >= AUTO_TOTAL_TIMEOUT_MS ||
        (uint32_t)(tick - seq->stage_tick) >= AUTO_STAGE_TIMEOUT_MS) {
        motion_stop(motion);
        seq->stage = SEQ_TIMEOUT;
        return;
    }
    sweep_rad = (float)AUTO_ARC_SWEEP_MDEG * 0.001f * PI / 180.0f;
    end_tol_rad = (float)AUTO_END_MM / (float)AUTO_RADIUS_MM;
    switch (seq->stage) {
    case SEQ_BOOT_WAIT:
        if ((uint32_t)(tick - seq->boot_tick) >= AUTO_BOOT_MS)
            begin_stage(seq, SEQ_STRAIGHT_OUT, tick);
        return;
    case SEQ_STOPPING: {
        int32_t vl = wheel_mm_s(speed_left, LEFT_COUNTS_10_REVS);
        int32_t vr = wheel_mm_s(speed_right, RIGHT_COUNTS_10_REVS);
        if (abs(vl) < AUTO_STOP_SPEED_MM_S && abs(vr) < AUTO_STOP_SPEED_MM_S)
            seq->stable_ms += SPEED_SAMPLE_MS;
        else seq->stable_ms = 0u;
        if (seq->stable_ms >= AUTO_STABLE_MS &&
            (uint32_t)(tick - seq->stage_tick) >= AUTO_SETTLE_MS)
            begin_stage(seq, seq->next_stage, tick);
        return;
    }
    case SEQ_STRAIGHT_OUT: {
        float rel = (seq->total_left_mm + seq->total_right_mm) * 0.5f - seq->start_avg_mm;
        int32_t remaining;
        seq->rel_mm = (int32_t)(rel + (rel < 0.0f ? -0.5f : 0.5f));
        remaining = AUTO_STRAIGHT_MM - seq->rel_mm;
        if (remaining <= AUTO_END_MM) {
            finish_stage(seq, motion, SEQ_TURN_CIRCLE, tick);
            return;
        }
        motion_set_velocity(motion, drive_v(remaining, AUTO_DRIVE_MAX_MM_S), 0, tick);
        return;
    }
    case SEQ_TURN_CIRCLE: {
        /* 车体转角=左右轮程差/有效轮距，左转为正。 */
        float diff = seq->total_right_mm - seq->total_left_mm - seq->start_diff_mm;
        float progress = diff / (float)WHEEL_TRACK_MM;
        float remaining = PI * 0.5f - progress;
        seq->rel_mrad = (int32_t)(progress * 1000.0f + (progress < 0.0f ? -0.5f : 0.5f));
        if (fabsf(remaining) <= (float)AUTO_END_MM / (float)WHEEL_TRACK_MM) {
            finish_stage(seq, motion, SEQ_ARC, tick);
            return;
        }
        motion_set_velocity(motion, 0, turn_w(remaining), tick);
        return;
    }
    case SEQ_ARC: {
        float diff = seq->total_right_mm - seq->total_left_mm - seq->start_diff_mm;
        float progress = diff / (float)WHEEL_TRACK_MM;
        float remaining = -sweep_rad - progress; /* 顺时针为负 */
        seq->rel_mrad = (int32_t)(progress * 1000.0f + (progress < 0.0f ? -0.5f : 0.5f));
        if (remaining >= -end_tol_rad) {
            finish_stage(seq, motion, SEQ_TURN_HOME, tick);
            return;
        }
        /* 圆弧保持曲率：w=-v/r，随剩余弧长同步减速。 */
        {
            int32_t v = drive_v((int32_t)(-remaining * (float)AUTO_RADIUS_MM +
                                          (remaining > -end_tol_rad ? -0.5f : 0.5f)),
                                AUTO_ARC_SPEED_MM_S);
            motion_set_velocity(motion, v,
                                -(int32_t)((float)v * 1000.0f / (float)AUTO_RADIUS_MM), tick);
        }
        return;
    }
    case SEQ_TURN_HOME: {
        float diff = seq->total_right_mm - seq->total_left_mm - seq->start_diff_mm;
        float progress = diff / (float)WHEEL_TRACK_MM;
        float remaining = (float)seq->turn_home_mrad * 0.001f - progress;
        seq->rel_mrad = (int32_t)(progress * 1000.0f + (progress < 0.0f ? -0.5f : 0.5f));
        if (fabsf(remaining) <= (float)AUTO_END_MM / (float)WHEEL_TRACK_MM) {
            finish_stage(seq, motion, SEQ_RETURN, tick);
            return;
        }
        motion_set_velocity(motion, 0, turn_w(remaining), tick);
        return;
    }
    case SEQ_RETURN: {
        float rel = (seq->total_left_mm + seq->total_right_mm) * 0.5f - seq->start_avg_mm;
        int32_t remaining;
        seq->rel_mm = (int32_t)(rel + (rel < 0.0f ? -0.5f : 0.5f));
        remaining = seq->return_mm - seq->rel_mm;
        if (remaining <= AUTO_END_MM) {
            finish_stage(seq, motion, SEQ_DONE, tick);
            return;
        }
        motion_set_velocity(motion, drive_v(remaining, AUTO_DRIVE_MAX_MM_S), 0, tick);
        return;
    }
    default:
        return;
    }
}

const char *auto_sequence_stage_name(seq_stage_t stage)
{
    static const char *names[] = {"boot", "straight", "turn_circle", "arc",
        "turn_home", "return", "stopping", "done", "cancelled", "timeout"};
    return names[stage];
}
