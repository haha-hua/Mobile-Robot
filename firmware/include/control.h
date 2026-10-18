#ifndef CONTROL_H
#define CONTROL_H
#include "motion.h"
#include "odometry.h"
#include "auto_sequence.h"
#include "obstacle.h"
typedef enum { CONTROL_SENSOR, CONTROL_MANUAL, CONTROL_AVOID, CONTROL_EXPERIMENT2 } control_mode_t;
typedef struct {
    uint32_t tick; /* Timestamp of this complete 20ms control sample. */
    motion_state_t state;
    control_mode_t mode;
    uint8_t ir_raw, ir_blocked;
    avoid_stage_t avoid_stage;
    int32_t avoid_run_mm; /* avoid任务平均轮程累计，mm。 */
    int32_t speed_left, speed_right; /* ticks/s, PI feedback */
    int32_t target_left, target_right; /* 坡度后的轮目标，mm/s。 */
    int16_t pwm_left, pwm_right;
    odometry_t odom; /* m, rad, m/s, rad/s */
    auto_sequence_t seq;
} control_snapshot_t;
void control_init(void);
/* 单位mm/s和mrad/s；成功返回1，越界返回0且不刷新超时。 */
uint8_t control_set_velocity(int32_t v, int32_t w);
uint8_t control_set_wheel_speeds(int32_t left, int32_t right);
void control_stop(void);
uint8_t control_start_avoid(void);
uint8_t control_start_experiment2(void);
/* 主循环解析完整行；成功返回1，非法命令返回0。 */
uint8_t control_command(const char *line);
uint32_t control_tick(void);
void control_snapshot(control_snapshot_t *snapshot);
#endif
