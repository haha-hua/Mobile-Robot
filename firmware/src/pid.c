#include "pid.h"

static int64_t clamp64(int64_t value, int64_t low, int64_t high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void pid_init(pid_t *pid, const pid_config_t *config)
{
    pid->config = *config;
    pid_reset(pid);
}

void pid_reset(pid_t *pid)
{
    pid->integral_milli = 0;
    pid->previous_error = 0;
    pid->has_previous = 0u;
}

int32_t pid_update(pid_t *pid, int32_t target, int32_t actual, int32_t feedforward)
{
    int64_t error = (int64_t)target - actual;
    int64_t proportional = error * pid->config.kp_milli / 1000;
    int64_t derivative = 0;
    int64_t candidate;
    int64_t output;
    int64_t unsaturated;

    if (pid->has_previous && pid->config.kd_milli != 0) {
        /* 毫秒周期与千分之一增益相抵，避免中间量额外放大。 */
        derivative = (error - pid->previous_error) * pid->config.kd_milli /
                     (int64_t)pid->config.period_ms;
    }
    candidate = pid->integral_milli + error * pid->config.ki_milli *
                (int64_t)pid->config.period_ms / 1000;
    candidate = clamp64(candidate, -pid->config.integral_limit_milli,
                        pid->config.integral_limit_milli);
    unsaturated = feedforward + proportional + derivative + candidate / 1000;
    if (!((unsaturated > pid->config.output_max && error > 0) ||
          (unsaturated < pid->config.output_min && error < 0))) {
        pid->integral_milli = candidate;
    }
    output = feedforward + proportional + derivative + pid->integral_milli / 1000;
    output = clamp64(output, pid->config.output_min, pid->config.output_max);
    pid->previous_error = error;
    pid->has_previous = 1u;
    return (int32_t)output;
}
