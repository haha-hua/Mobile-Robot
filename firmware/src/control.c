#include "control.h"
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "speed_control.h"
#include "stm32f10x.h"
#include <string.h>

/* 旧版CMSIS的GCC分支只声明PRIMASK接口，补齐实现供控制和UART共用。 */
#if defined(__arm__)
uint32_t __get_PRIMASK(void)
{
    uint32_t mask;
    __asm volatile ("mrs %0, primask" : "=r" (mask) :: "memory");
    return mask;
}

void __set_PRIMASK(uint32_t mask)
{
    __asm volatile ("msr primask, %0" :: "r" (mask) : "memory");
}
#endif

static volatile uint32_t ms_tick;
static volatile control_snapshot_t status;
static motion_t motion;
static odometry_t odom;
static test_sequence_t test;
static int32_t last_left, last_right;
static uint8_t elapsed;

void control_init(void)
{
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef nvic;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    motion_init(&motion);
    test_sequence_init(&test, 0u);
    odometry_init(&odom);
    speed_control_init();
    motor_stop();
    ms_tick = 0u;
    elapsed = 0u;
    status = (control_snapshot_t){0};
    last_left = encoder_left_count();
    last_right = encoder_right_count();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 72u - 1u;
    timer.TIM_Period = 1000u - 1u;
    TIM_TimeBaseInit(TIM2, &timer);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    TIM_Cmd(TIM2, ENABLE);
    __set_PRIMASK(primask);
}

uint8_t control_set_velocity(int32_t v, int32_t w)
{
    uint8_t accepted;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    accepted = motion_set_velocity(&motion, v, w, ms_tick);
    if (accepted) test_sequence_cancel(&test);
    __set_PRIMASK(primask);
    return accepted;
}

uint8_t control_set_wheel_speeds(int32_t left, int32_t right)
{
    uint8_t accepted;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    accepted = motion_set_wheel_speeds(&motion, left, right, ms_tick);
    if (accepted) test_sequence_cancel(&test);
    __set_PRIMASK(primask);
    return accepted;
}

void control_stop(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    test_sequence_cancel(&test);
    motion_stop(&motion);
    speed_control_reset();
    motor_stop();
    __set_PRIMASK(primask);
}

static const char *skip_space(const char *text)
{
    while (*text == ' ' || *text == '\t') text++;
    return text;
}

static uint8_t parse_number(const char **text, int32_t *value)
{
    const char *p = skip_space(*text);
    int32_t number = 0;
    int32_t sign = 1;
    if (*p == '-' || *p == '+') { if (*p == '-') sign = -1; p++; }
    if (*p < '0' || *p > '9') return 0u;
    do {
        number = number * 10 + (*p++ - '0');
        /* 协议所有数值绝对值至多2000，也避免整数解析溢出。 */
        if (number > MAX_ANGULAR_SPEED_MRAD_S) return 0u;
    } while (*p >= '0' && *p <= '9');
    if (*p != '\0' && *p != ' ' && *p != '\t') return 0u;
    *value = sign * number;
    *text = p;
    return 1u;
}

uint8_t control_command(const char *line)
{
    int32_t first, second;
    uint8_t velocity;
    const char *text = skip_space(line);
    if (strncmp(text, "stop", 4) == 0 && *skip_space(text + 4) == '\0') {
        control_stop();
        return 1u;
    }
    if (strncmp(text, "vel", 3) == 0 && (text[3] == ' ' || text[3] == '\t')) {
        velocity = 1u;
        text += 3;
    } else if (strncmp(text, "wheel", 5) == 0 && (text[5] == ' ' || text[5] == '\t')) {
        velocity = 0u;
        text += 5;
    } else return 0u;
    if (!parse_number(&text, &first) || !parse_number(&text, &second) ||
        *skip_space(text) != '\0') return 0u;
    return velocity ? control_set_velocity(first, second) : control_set_wheel_speeds(first, second);
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        ms_tick++;
        if (++elapsed >= SPEED_SAMPLE_MS) {
            int32_t left, right, delta_left, delta_right;
            int16_t pwm_left, pwm_right;
            control_snapshot_t sample = {0};
            uint32_t primask = __get_PRIMASK();
            elapsed = 0u;
            __disable_irq();
            left = encoder_left_count();
            right = encoder_right_count();
            __set_PRIMASK(primask);
            delta_left = (int32_t)((uint32_t)left - (uint32_t)last_left);
            delta_right = (int32_t)((uint32_t)right - (uint32_t)last_right);
            last_left = left;
            last_right = right;
            sample.speed_left = delta_left * (int32_t)(1000u / SPEED_SAMPLE_MS);
            sample.speed_right = delta_right * (int32_t)(1000u / SPEED_SAMPLE_MS);
            test_sequence_step(&test, &motion, ms_tick, delta_left, delta_right,
                               sample.speed_left, sample.speed_right);
            motion_step(&motion, ms_tick, sample.speed_left, sample.speed_right);
            if (motion.state != MOTION_RUN) speed_control_reset();
            speed_control_step(motion.target_left, motion.target_right,
                               sample.speed_left, sample.speed_right, &pwm_left, &pwm_right);
            if (motion.state == MOTION_RUN) {
                motor_enable(1u);
                motor_set(MOTOR_LEFT, pwm_left);
                motor_set(MOTOR_RIGHT, pwm_right);
            } else motor_stop();
            odometry_update(&odom, delta_left, delta_right, SPEED_SAMPLE_MS);
            sample.tick = ms_tick;
            sample.state = motion.state;
            sample.target_left = motion.ramp_left;
            sample.target_right = motion.ramp_right;
            sample.pwm_left = pwm_left;
            sample.pwm_right = pwm_right;
            sample.odom = odom;
            sample.test_stage = test.stage;
            sample.test_distance_mm = test_sequence_distance_mm(&test);
            sample.test_angle_rad = test_sequence_angle_rad(&test);
            status = sample;
        }
    }
}

uint32_t control_tick(void) { return ms_tick; }

void control_snapshot(control_snapshot_t *snapshot)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = status;
    __set_PRIMASK(primask);
}
