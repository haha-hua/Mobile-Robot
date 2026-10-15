#ifndef PID_H
#define PID_H
#include <stdint.h>

/* 增益为千分之一；积分与微分增益分别按秒和秒的倒数换算。 */
typedef struct {
    int32_t kp_milli, ki_milli, kd_milli;
    uint32_t period_ms;
    int32_t output_min, output_max;
    int32_t integral_limit_milli;
} pid_config_t;

typedef struct {
    pid_config_t config;
    int64_t integral_milli;
    int64_t previous_error;
    uint8_t has_previous;
} pid_t;

/* 周期必须大于零，积分限幅非负，输出下限不大于上限。 */
void pid_init(pid_t *pid, const pid_config_t *config);
void pid_reset(pid_t *pid);
/* 首次更新不产生微分突变；前馈与反馈共同参与输出限幅和抗饱和。 */
int32_t pid_update(pid_t *pid, int32_t target, int32_t actual, int32_t feedforward);
#endif
