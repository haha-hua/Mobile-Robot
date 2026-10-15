#include "speed_control.h"
#include "config.h"
#include "pid.h"

static pid_t left_pid;
static pid_t right_pid;
static int8_t left_direction, right_direction;

void speed_control_reset(void)
{
    pid_reset(&left_pid);
    pid_reset(&right_pid);
    left_direction = right_direction = 0;
}

void speed_control_init(void)
{
    const pid_config_t config = {
        PID_KP_MILLI, PID_KI_MILLI, PID_KD_MILLI, SPEED_SAMPLE_MS,
        0, PWM_MAX, PID_INTEGRAL_LIMIT_MILLI
    };
    pid_init(&left_pid, &config);
    pid_init(&right_pid, &config);
    speed_control_reset();
}

static int16_t wheel_step(pid_t *pid, int8_t *previous_direction,
                          int32_t target, int32_t actual)
{
    int8_t direction = target > 0 ? 1 : target < 0 ? -1 : 0;
    if (direction == 0 || direction != *previous_direction) pid_reset(pid);
    *previous_direction = direction;
    if (direction == 0) return 0;
    return (int16_t)(direction * pid_update(pid, direction * target,
                                          direction * actual, PWM_FEEDFORWARD));
}

void speed_control_step(int32_t target_left, int32_t target_right,
                        int32_t actual_left, int32_t actual_right,
                        int16_t *left_pwm, int16_t *right_pwm)
{
    *left_pwm = wheel_step(&left_pid, &left_direction, target_left, actual_left);
    *right_pwm = wheel_step(&right_pid, &right_direction, target_right, actual_right);
}
