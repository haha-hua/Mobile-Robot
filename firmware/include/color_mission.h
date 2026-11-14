#ifndef COLOR_MISSION_H
#define COLOR_MISSION_H

#include "color_sensor.h"
#include "motion.h"
#include "obstacle.h"
#include "odometry.h"
#include <stdint.h>

typedef enum {
    MISSION_LEG_LEAVE_RED = 0,
    MISSION_LEG_OUTBOUND,
    MISSION_LEG_RETURN
} mission_leg_t;

typedef enum {
    MISSION_IDLE = 0,
    MISSION_BOOT,
    MISSION_CRUISE,
    MISSION_AVOID_TURN,
    MISSION_AVOID_BYPASS,
    MISSION_AVOID_REALIGN,
    MISSION_WHITE_ALIGN,
    MISSION_WHITE_PASS,
    MISSION_WHITE_RETURN,
    MISSION_BLUE_PULL_IN,
    MISSION_BLUE_TURN_180,
    MISSION_RED_PULL_IN,
    MISSION_DONE,
    MISSION_CANCELLED,
    MISSION_TIMEOUT
} mission_stage_t;

#define MISSION_MAX_MAP_OBS 8u

/* 维护机器人全局位姿参考与轻量级走廊地图：
 * 坐标系以红区上电起点为(0,0)，车头初始正前方为+X轴(outbound_yaw=0)，
 * 全局+Y轴为去程左侧(返程右侧)，全局-Y轴为去程右侧(返程左侧)。 */
typedef struct {
    mission_leg_t leg;
    mission_stage_t stage;
    uint32_t boot_tick;
    uint32_t stage_tick;
    uint32_t clear_ms;
    uint32_t last_color_frame;

    /* 航向参考：outbound_yaw为去程0度方向，return_yaw为返程180度方向 */
    float outbound_yaw;
    float return_yaw;
    float goal_yaw;
    float maneuver_yaw;

    /* 在线记录的简易全局地图信息（色区范围 + 左右白边全局坐标 + 已探测圆瓶坐标表） */
    float origin_x_m;
    float origin_y_m;
    float yellow_start_x_m;
    float blue_start_x_m;
    float last_mat_x_m;
    float last_mat_y_m;
    float mat_min_sy;
    float mat_max_sy;
    float left_bound_y_m;
    float right_bound_y_m;
    float obs_x_m;
    float obs_y_m;
    float map_obs_x[MISSION_MAX_MAP_OBS];
    float map_obs_y[MISSION_MAX_MAP_OBS];

    float seg_mm;
    uint8_t has_yellow_start;
    uint8_t has_blue_start;
    uint8_t left_blue;
    uint8_t has_left_bound;
    uint8_t has_right_bound;
    uint8_t has_last_white_side;
    uint8_t last_white_pos_y;
    uint8_t has_active_obs;
    uint8_t white_armed;
    uint8_t non_mat_frames;
    uint8_t white_side_uncertain;
    uint8_t white_retry_count;
    uint8_t obs_count;
    uint8_t map_obs_count;
    int8_t avoid_side;  /* 避障转向方向：相对于当前车头，+1向左绕，-1向右绕 */
    int8_t return_side; /* 白边内切方向：相对于当前goal_yaw，-1向右内切，+1向左内切 */
    int8_t turn_dir;    /* 180度掉头锁定转向方向，防止±pi跳变抖动 */
} color_mission_t;

void color_mission_init(color_mission_t *seq, const odometry_t *odom, uint32_t tick);
void color_mission_cancel(color_mission_t *seq);
void color_mission_step(color_mission_t *seq, motion_t *motion,
                        const odometry_t *odom, const obstacle_t *obstacle,
                        const color_sensor_t *color, uint32_t tick,
                        int32_t delta_left, int32_t delta_right,
                        int32_t speed_left, int32_t speed_right);
const char *color_mission_leg_name(mission_leg_t leg);
const char *color_mission_stage_name(mission_stage_t stage);

#endif
