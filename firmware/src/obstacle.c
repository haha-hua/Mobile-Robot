#include "obstacle.h"
#include "config.h"
#include "stm32f10x.h"
#include <stdlib.h>

#if IR_DIAG_ENABLE
static void delay_cycles(uint32_t loops)
{
    while (loops--) {
        __asm volatile ("nop");
    }
}

static void ir_set_mode(GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = IR_PIN;
    gpio.GPIO_Mode = mode;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(IR_PORT, &gpio);
}

static uint16_t ir_read_adc2(void)
{
    uint32_t sum = 0u;
    uint8_t i;
    for (i = 0u; i < 4u; i++) {
        ADC_ClearFlag(ADC2, ADC_FLAG_EOC);
        ADC_SoftwareStartConvCmd(ADC2, ENABLE);
        while (!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC)) { }
        sum += ADC_GetConversionValue(ADC2);
    }
    return (uint16_t)((sum + 2u) / 4u);
}
#endif

void obstacle_fast_sample_1ms(obstacle_t *sensor)
{
#if IR_DIAG_ENABLE
    uint8_t cur = GPIO_ReadInputDataBit(IR_PORT, IR_PIN) ? 1u : 0u;
    if (!sensor->floating) {
        if (cur != sensor->fast_last_level) {
            sensor->fast_last_level = cur;
            sensor->edge_count++;
        }
        if (cur == IR_ACTIVE_LEVEL) sensor->low_ms++;
        else sensor->high_ms++;
    } else {
        sensor->fast_last_level = cur;
    }
#else
    (void)sensor;
#endif
}

void obstacle_sample(obstacle_t *sensor)
{
    uint8_t prev_blocked = sensor->blocked;
#if IR_DIAG_ENABLE
    uint8_t ipu, ipd;
    uint16_t adc_raw, mv;

    /* 1) PA1平时保持IPU输入上拉，直接读取上拉态电平 */
    ipu = GPIO_ReadInputDataBit(IR_PORT, IR_PIN) ? 1u : 0u;

    /* 2) 短暂切换IPD输入下拉(~40kΩ)：若OUT信号线断路悬空会被拉成0 */
    ir_set_mode(GPIO_Mode_IPD);
    delay_cycles(400u);
    ipd = GPIO_ReadInputDataBit(IR_PORT, IR_PIN) ? 1u : 0u;

    /* 3) 切换AIN无上下拉高阻态，等待模块驱动恢复后由ADC2测真实引脚电压 */
    ir_set_mode(GPIO_Mode_AIN);
    delay_cycles(800u);
    adc_raw = ir_read_adc2();
    mv = (uint16_t)(((uint32_t)adc_raw * LIGHT_ADC_VREF_MV +
                     LIGHT_ADC_MAX_VALUE / 2u) / LIGHT_ADC_MAX_VALUE);

    /* 4) 恢复默认IPU输入上拉并等待电平稳定 */
    ir_set_mode(GPIO_Mode_IPU);
    delay_cycles(300u);

    sensor->ipu_level = ipu;
    sensor->ipd_level = ipd;
    sensor->adc_raw = adc_raw;
    sensor->millivolts = mv;
    if (mv < sensor->min_mv) sensor->min_mv = mv;
    if (mv > sensor->max_mv) sensor->max_mv = mv;

    /* 上拉为1、下拉变0且高阻态无法回升(>600mV)，判定PA1信号线悬空/未接通 */
    sensor->floating = (ipu == 1u && ipd == 0u && mv < 600u) ? 1u : 0u;
    sensor->raw_right = ipu;
    sensor->raw_left = GPIO_ReadInputDataBit(IR_PORT, IR_LEFT_PIN) ? 1u : 0u;
    sensor->raw_level = ipu;

    if (sensor->floating) {
        sensor->low_streak = 0u;
        sensor->high_streak = 0u;
    } else if (sensor->raw_level == IR_ACTIVE_LEVEL) {
        sensor->high_streak = 0u;
        if (sensor->low_streak < 10u) sensor->low_streak++;
        if (sensor->low_streak >= 2u) sensor->seen_low = 1u;
    } else {
        sensor->low_streak = 0u;
        if (sensor->high_streak < 10u) sensor->high_streak++;
        if (sensor->high_streak >= 2u) sensor->seen_high = 1u;
    }
#else
    sensor->raw_right = GPIO_ReadInputDataBit(IR_PORT, IR_RIGHT_PIN) ? 1u : 0u;
    sensor->raw_left = GPIO_ReadInputDataBit(IR_PORT, IR_LEFT_PIN) ? 1u : 0u;
    sensor->raw_level = (sensor->raw_right == IR_ACTIVE_LEVEL ||
                         sensor->raw_left == IR_ACTIVE_LEVEL) ?
                        IR_ACTIVE_LEVEL : (uint8_t)(!IR_ACTIVE_LEVEL);
    sensor->ipu_level = sensor->raw_level;
    sensor->ipd_level = sensor->raw_level;
#endif

    if (sensor->raw_right == IR_ACTIVE_LEVEL) {
        sensor->right_blocked = 1u;
        sensor->clear_right = 0u;
    } else {
        if (sensor->clear_right < IR_CLEAR_SAMPLES) sensor->clear_right++;
        if (sensor->clear_right >= IR_CLEAR_SAMPLES) sensor->right_blocked = 0u;
    }

    if (sensor->raw_left == IR_ACTIVE_LEVEL) {
        sensor->left_blocked = 1u;
        sensor->clear_left = 0u;
    } else {
        if (sensor->clear_left < IR_CLEAR_SAMPLES) sensor->clear_left++;
        if (sensor->clear_left >= IR_CLEAR_SAMPLES) sensor->left_blocked = 0u;
    }

#if IR_DIAG_ENABLE
    if (sensor->raw_level == IR_ACTIVE_LEVEL) {
        sensor->blocked = 1u;
        sensor->clear_samples = 0u;
        if (!prev_blocked && !sensor->floating) sensor->obstacle_count++;
    } else {
        if (sensor->clear_samples < IR_CLEAR_SAMPLES) sensor->clear_samples++;
        if (sensor->clear_samples >= IR_CLEAR_SAMPLES) sensor->blocked = 0u;
    }
#else
    sensor->blocked = (sensor->right_blocked || sensor->left_blocked) ? 1u : 0u;
    sensor->clear_samples = (sensor->clear_right < sensor->clear_left) ?
                            sensor->clear_right : sensor->clear_left;
    if (sensor->blocked && !prev_blocked) sensor->obstacle_count++;
#endif
}

void obstacle_reset_diag(obstacle_t *sensor)
{
    sensor->min_mv = sensor->millivolts;
    sensor->max_mv = sensor->millivolts;
    sensor->edge_count = 0u;
    sensor->obstacle_count = 0u;
    sensor->low_ms = 0u;
    sensor->high_ms = 0u;
    sensor->seen_low = 0u;
    sensor->seen_high = 0u;
    sensor->low_streak = 0u;
    sensor->high_streak = 0u;
    sensor->fast_last_level = sensor->raw_level;
}

const char *obstacle_diag_summary(const obstacle_t *sensor)
{
    if (sensor->floating)
        return "FAULT:PA1_FLOATING(check_OUT_wire)";
    if (sensor->millivolts >= 600u && sensor->millivolts <= 2600u)
        return "WARN:MID_VOLTAGE(check_VCC_GND_or_AO)";
    if (sensor->seen_low && sensor->seen_high)
        return "PASS:IR_OK(0_and_1_verified)";
    if (sensor->seen_high && !sensor->seen_low)
        return "WAIT:HIGH_ONLY(wave_hand_or_tune_pot)";
    if (sensor->seen_low && !sensor->seen_high)
        return "WARN:STUCK_LOW(clear_front_or_lower_pot)";
    return "INIT:SAMPLING";
}

void obstacle_init(obstacle_t *sensor)
{
    GPIO_InitTypeDef gpio;
#if IR_DIAG_ENABLE
    ADC_InitTypeDef adc;
#endif
    RCC_APB2PeriphClockCmd(IR_GPIO_CLK, ENABLE);
    gpio.GPIO_Pin = IR_RIGHT_PIN | IR_LEFT_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(IR_PORT, &gpio);
    *sensor = (obstacle_t){0};
    sensor->right_blocked = 1u;
    sensor->left_blocked = 1u;
    sensor->blocked = 1u;
    sensor->min_mv = 0xFFFFu;
#if IR_DIAG_ENABLE
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    ADC_DeInit(ADC2);
    ADC_StructInit(&adc);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC2, &adc);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);
    ADC_Cmd(ADC2, ENABLE);
    ADC_ResetCalibration(ADC2);
    while (ADC_GetResetCalibrationStatus(ADC2)) { }
    ADC_StartCalibration(ADC2);
    while (ADC_GetCalibrationStatus(ADC2)) { }
    delay_cycles(400u);
    obstacle_sample(sensor);
    sensor->blocked = 1u;
    sensor->clear_samples = 0u;
    sensor->obstacle_count = 0u;
    sensor->fast_last_level = sensor->raw_level;
#else
    /* 初始化只记录电平，清空必须经过三个完整控制采样。 */
    sensor->raw_right = GPIO_ReadInputDataBit(IR_PORT, IR_RIGHT_PIN) ? 1u : 0u;
    sensor->raw_left = GPIO_ReadInputDataBit(IR_PORT, IR_LEFT_PIN) ? 1u : 0u;
    sensor->raw_level = (sensor->raw_right == IR_ACTIVE_LEVEL ||
                         sensor->raw_left == IR_ACTIVE_LEVEL) ?
                        IR_ACTIVE_LEVEL : (uint8_t)(!IR_ACTIVE_LEVEL);
    sensor->fast_last_level = sensor->raw_level;
#endif
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
