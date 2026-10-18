#include "obstacle.h"
#include "config.h"
#include "stm32f10x.h"
#include <stdlib.h>

void obstacle_sample(obstacle_t *sensor)
{
    sensor->raw_level = GPIO_ReadInputDataBit(IR_PORT, IR_PIN) ? 1u : 0u;
    if (sensor->raw_level == IR_ACTIVE_LEVEL) {
        sensor->blocked = 1u;
        sensor->clear_samples = 0u;
    } else {
        if (sensor->clear_samples < IR_CLEAR_SAMPLES) sensor->clear_samples++;
        if (sensor->clear_samples >= IR_CLEAR_SAMPLES) sensor->blocked = 0u;
    }
}

void obstacle_init(obstacle_t *sensor)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(IR_GPIO_CLK, ENABLE);
    gpio.GPIO_Pin = IR_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(IR_PORT, &gpio);
    *sensor = (obstacle_t){0};
    sensor->blocked = 1u;
    /* 初始化只记录电平，清空必须经过三个完整控制采样。 */
    sensor->raw_level = GPIO_ReadInputDataBit(IR_PORT, IR_PIN) ? 1u : 0u;
}

void obstacle_disable(obstacle_t *sensor)
{
    sensor->stage = AVOID_TEST;
    sensor->stable_ms = 0u;
}

static void stopping(obstacle_t *sensor, motion_t *motion, uint32_t tick,
                     uint8_t turn_required)
{
    motion_stop(motion);
    sensor->turn_required = turn_required;
    sensor->stage = AVOID_STOPPING;
    sensor->stage_tick = tick;
    sensor->stable_ms = 0u;
}

void obstacle_start(obstacle_t *sensor, motion_t *motion, uint32_t tick)
{
    sensor->total_mm = 0.0f;
    stopping(sensor, motion, tick, 0u);
}

void obstacle_step(obstacle_t *sensor, motion_t *motion, uint32_t tick,
                   int32_t speed_left, int32_t speed_right)
{
    /* 输入单位是脉冲/秒，分别标定成轮程后再累计车体前进距离。 */
    if (sensor->stage != AVOID_DONE && sensor->stage != AVOID_FAULT &&
        sensor->stage != AVOID_TEST) {
        float left_mm_s = (float)speed_left * WHEEL_CIRCUMFERENCE_UM /
                          ((float)LEFT_COUNTS_10_REVS * 100.0f);
        float right_mm_s = (float)speed_right * WHEEL_CIRCUMFERENCE_UM /
                           ((float)RIGHT_COUNTS_10_REVS * 100.0f);
        if (sensor->stage == AVOID_FORWARD ||
            (sensor->stage == AVOID_STOPPING && sensor->turn_required)) {
            float forward_mm_s = (left_mm_s + right_mm_s) * 0.5f;
            if (forward_mm_s > 0.0f)
                sensor->total_mm += forward_mm_s * ((float)SPEED_SAMPLE_MS * 0.001f);
        }
        if (sensor->stage == AVOID_TURN_LEFT) {
            sensor->turn_mrad += (right_mm_s - left_mm_s)
                                 * (float)SPEED_SAMPLE_MS / (float)WHEEL_TRACK_MM;
        }
        if (sensor->total_mm >= (float)AVOID_TOTAL_MM) {
            motion_stop(motion);
            sensor->stage = AVOID_DONE;
        }
    }

    switch (sensor->stage) {
    case AVOID_FORWARD:
        if (sensor->blocked) stopping(sensor, motion, tick, 1u);
        else motion_set_velocity(motion, AVOID_FORWARD_MM_S, 0, tick);
        break;
    case AVOID_STOPPING: {
        int32_t vl = motion_pulses_to_mm(speed_left, LEFT_COUNTS_10_REVS);
        int32_t vr = motion_pulses_to_mm(speed_right, RIGHT_COUNTS_10_REVS);
        motion_stop(motion);
        if (abs(vl) < AVOID_STOP_SPEED_MM_S && abs(vr) < AVOID_STOP_SPEED_MM_S)
            sensor->stable_ms += SPEED_SAMPLE_MS;
        else sensor->stable_ms = 0u;
        if (sensor->stable_ms >= AVOID_STABLE_MS &&
            (uint32_t)(tick - sensor->stage_tick) >= AVOID_STABLE_MS) {
            sensor->stage_tick = tick;
            if ((int32_t)sensor->total_mm >= AVOID_TOTAL_MM) {
                /* 总里程达标：本次任务完成，保持停车。 */
                sensor->stage = AVOID_DONE;
            } else if (sensor->turn_required || sensor->blocked) {
                sensor->turn_required = 0u;
                sensor->stage = AVOID_TURN_LEFT;
                sensor->turn_mrad = 0.0f;
                sensor->turn_clear_ms = 0u;
                motion_set_velocity(motion, 0, AVOID_TURN_MRAD_S, tick);
            } else {
                sensor->stage = AVOID_FORWARD;
                motion_set_velocity(motion, AVOID_FORWARD_MM_S, 0, tick);
            }
        }
        break;
    }
    case AVOID_TURN_LEFT:
        if (sensor->raw_level != IR_ACTIVE_LEVEL) {
            if (sensor->turn_clear_ms < AVOID_TURN_CLEAR_MS)
                sensor->turn_clear_ms += SPEED_SAMPLE_MS;
        } else sensor->turn_clear_ms = 0u;
        if (sensor->turn_mrad >= (float)AVOID_TURN_MIN_MRAD &&
            sensor->turn_clear_ms >= AVOID_TURN_CLEAR_MS) {
            stopping(sensor, motion, tick, 0u);
        } else if ((uint32_t)(tick - sensor->stage_tick) >= AVOID_TURN_TIMEOUT_MS) {
            motion_stop(motion);
            sensor->stage = AVOID_FAULT;
        } else motion_set_velocity(motion, 0, AVOID_TURN_MRAD_S, tick);
        break;
    case AVOID_FAULT:
        motion_stop(motion);
        break;
    case AVOID_DONE:
        motion_stop(motion);
        break;
    case AVOID_TEST:
        break;
    }
}

const char *obstacle_stage_name(avoid_stage_t stage)
{
    static const char *names[] = {"test", "forward", "stopping", "left", "fault", "done"};
    return names[stage];
}
