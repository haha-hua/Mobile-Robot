#include "color_mission.h"
#include "config.h"
#include <math.h>

#define PI         3.14159265358979323846f
#define DEG8_RAD   0.1396263f
#define DEG10_RAD  0.1745329f
#define DEG12_RAD  0.2094395f
#define DEG15_RAD  0.2617994f
#define DEG20_RAD  0.3490659f
#define DEG30_RAD  0.5235988f
#define DEG38_RAD  0.6632251f
#define DEG42_RAD  0.7330383f
#define DEG48_RAD  0.8377580f
#define DEG50_RAD  0.8726646f
#define DEG54_RAD  0.9424778f
#define DEG56_RAD  0.9773844f
#define DEG58_RAD  1.0122910f
#define DEG60_RAD  1.0471976f
#define DEG62_RAD  1.0821041f
#define DEG65_RAD  1.1344640f
#define DEG72_RAD  1.2566371f
#define DEG80_RAD  1.3962634f
#define DEG100_RAD 1.7453293f
#define DEG135_RAD 2.3561945f
#define DEG165_RAD 2.8797933f

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

/* 向量(dx, dy)在参考航向axis正前方的投影（米） */
static float proj_along(float dx, float dy, float axis)
{
    return dx * cosf(axis) + dy * sinf(axis);
}

/* 向量(dx, dy)在参考航向axis左侧法向的投影（左正右负，米） */
static float proj_left(float dx, float dy, float axis)
{
    return -dx * sinf(axis) + dy * cosf(axis);
}

/* 全局走廊纵向坐标 sx（+X：红区指向蓝区） */
static float corridor_sx(const color_mission_t *seq, const odometry_t *odom)
{
    return proj_along(odom->x_m - seq->origin_x_m,
                      odom->y_m - seq->origin_y_m,
                      seq->outbound_yaw);
}

/* 全局走廊横向坐标 sy（+Y：去程左侧/返程右侧；-Y：去程右侧/返程左侧） */
static float corridor_sy(const color_mission_t *seq, const odometry_t *odom)
{
    return proj_left(odom->x_m - seq->origin_x_m,
                     odom->y_m - seq->origin_y_m,
                     seq->outbound_yaw);
}

static void enter_stage(color_mission_t *seq, mission_stage_t stage, uint32_t tick)
{
    seq->stage = stage;
    seq->stage_tick = tick;
    seq->clear_ms = 0u;
    seq->seg_mm = 0.0f;
}

/* 行进中平滑航向跟踪角速度(mrad/s)：带小死区防止直行摆头振荡 */
static int32_t heading_track_w(float error, int32_t max_w)
{
    float w;
    if (fabsf(error) < 0.015f) return 0;
    w = error * 2200.0f;
    if (w > (float)max_w) w = (float)max_w;
    if (w < -(float)max_w) w = -(float)max_w;
    return (int32_t)w;
}

/* 快速转向/快速回正角速度(mrad/s)：大角度满速干脆回正，接近目标8度内平滑收敛防过冲 */
static int32_t heading_turn_w(float error)
{
    float abs_e = fabsf(error);
    float w;
    if (abs_e < 0.015f) return 0;
    w = error * 2900.0f;
    if (w > (float)MISSION_TURN_MAX_MRAD_S) w = (float)MISSION_TURN_MAX_MRAD_S;
    if (w < -(float)MISSION_TURN_MAX_MRAD_S) w = -(float)MISSION_TURN_MAX_MRAD_S;
    if (abs_e > 0.14f && fabsf(w) < (float)MISSION_TURN_MIN_MRAD_S) {
        w = (error >= 0.0f) ? (float)MISSION_TURN_MIN_MRAD_S :
                             -(float)MISSION_TURN_MIN_MRAD_S;
    } else if (abs_e <= 0.14f && fabsf(w) < 250.0f) {
        w = (error >= 0.0f) ? 250.0f : -250.0f;
    }
    return (int32_t)w;
}

/* 更新在线全局地图：记录红(32cm)/黄(120cm)/蓝(32cm)交界范围、垫内横向跨度与最近垫内位置 */
static void update_global_map(color_mission_t *seq, const odometry_t *odom,
                              const color_sensor_t *color)
{
    float sx = corridor_sx(seq, odom);
    float sy = corridor_sy(seq, odom);

    if (color->candidate != COLOR_UNKNOWN && color->candidate != COLOR_WHITE) {
        seq->non_mat_frames = 0u;
    } else if (seq->non_mat_frames < 255u) {
        seq->non_mat_frames++;
    }

    if (color->label != COLOR_UNKNOWN && color->label != COLOR_WHITE &&
        seq->non_mat_frames == 0u) {
        seq->white_armed = 1u;
        seq->white_side_uncertain = 0u;
        seq->last_mat_x_m = odom->x_m;
        seq->last_mat_y_m = odom->y_m;
        if (sy < seq->mat_min_sy) seq->mat_min_sy = sy;
        if (sy > seq->mat_max_sy) seq->mat_max_sy = sy;
    }

    if (seq->leg == MISSION_LEG_LEAVE_RED) {
        if (color->label == COLOR_YELLOW ||
            (color->candidate == COLOR_YELLOW && color->stable_frames >= 1u)) {
            seq->leg = MISSION_LEG_OUTBOUND;
            if (!seq->has_yellow_start) {
                seq->yellow_start_x_m = sx;
                seq->has_yellow_start = 1u;
            }
        } else if (sx >= 0.28f) {
            seq->leg = MISSION_LEG_OUTBOUND;
            if (!seq->has_yellow_start) {
                seq->yellow_start_x_m = 0.20f;
                seq->has_yellow_start = 1u;
            }
        }
    } else if (seq->leg == MISSION_LEG_RETURN && !seq->left_blue) {
        if (color->label == COLOR_YELLOW ||
            (color->candidate == COLOR_YELLOW && color->stable_frames >= 1u)) {
            seq->left_blue = 1u;
        } else if (seq->has_blue_start &&
                   sx <= seq->blue_start_x_m - 0.25f) {
            seq->left_blue = 1u;
        }
    }
}

/* 边界触发判定：稳态2帧非彩垫(白边或白边外暗地板)，保证20cm车宽完全越过瓶身进入32cm白边中线且不越界 */
static uint8_t is_border_hit(const color_mission_t *seq,
                             const color_sensor_t *color)
{
    if (!seq->white_armed) return 0u;
    if (color->label == COLOR_WHITE) return 1u;
    if (color->label == COLOR_UNKNOWN && color->frame_count >= 4u) return 1u;
    if (seq->non_mat_frames >= 2u && color->frame_count >= 4u) return 1u;
    return 0u;
}

/* 入区停车/掉头微调航向：若正从白边大角度内切入区则保持内切角使车身完全进垫；否则靠近边缘向垫心微收15度 */
static float calc_pull_in_yaw(const color_mission_t *seq, const odometry_t *odom)
{
    float sy = corridor_sy(seq, odom);
    float center_sy = 0.5f * (seq->mat_min_sy + seq->mat_max_sy);
    float rel = wrap_pi(odom->yaw_rad - seq->goal_yaw);
    int8_t in_dir = 0;

    if (seq->stage == MISSION_WHITE_RETURN) return odom->yaw_rad;
    if (sy > center_sy + 0.36f) {
        in_dir = (seq->leg != MISSION_LEG_RETURN) ? -1 : 1;
    } else if (sy < center_sy - 0.36f) {
        in_dir = (seq->leg != MISSION_LEG_RETURN) ? 1 : -1;
    }
    if (in_dir != 0 && rel * (float)in_dir > DEG15_RAD) {
        return odom->yaw_rad;
    }
    return wrap_pi(seq->goal_yaw + (float)in_dir * DEG15_RAD);
}

/* 检查是否到达终点色区（蓝区32x120cm掉头 / 红区32x120cm停车）：
 * 纵深仅32cm，既支持2帧稳态确认，也支持预估区段内高置信度(min_dist_sq<=8000)首帧快捕，防穿出32cm端区且防跨缝误判 */
static uint8_t check_zone_arrival(color_mission_t *seq,
                                  const odometry_t *odom,
                                  const obstacle_t *obstacle,
                                  const color_sensor_t *color, uint32_t tick)
{
    float sx = corridor_sx(seq, odom);
    float blue_exp_sx = (seq->has_yellow_start ? seq->yellow_start_x_m : 0.20f) + 1.20f;
    float red_exp_sx = seq->has_blue_start ?
                       (seq->blue_start_x_m - 1.20f) : seq->yellow_start_x_m;

    if (seq->stage == MISSION_AVOID_TURN &&
        (obstacle->raw_left == IR_ACTIVE_LEVEL ||
         obstacle->raw_right == IR_ACTIVE_LEVEL)) {
        return 0u;
    }

    if ((seq->leg == MISSION_LEG_OUTBOUND ||
         (seq->leg == MISSION_LEG_LEAVE_RED && sx >= 0.40f)) &&
        (color->label == COLOR_BLUE ||
         (sx >= blue_exp_sx - 0.30f &&
          color->candidate == COLOR_BLUE &&
          color->stable_frames >= 1u &&
          color->min_dist_sq <= 8000u))) {
        uint8_t confirmed_2f = (color->label == COLOR_BLUE) ? 1u : 0u;
        seq->blue_start_x_m = sx;
        seq->has_blue_start = 1u;
        seq->has_active_obs = 0u;
        seq->maneuver_yaw = calc_pull_in_yaw(seq, odom);
        enter_stage(seq, MISSION_BLUE_PULL_IN, tick);
        if (confirmed_2f) seq->seg_mm = 40.0f;
        return 1u;
    }

    if (seq->leg == MISSION_LEG_RETURN && seq->left_blue &&
        (color->label == COLOR_RED ||
         (sx <= red_exp_sx + 0.30f &&
          color->candidate == COLOR_RED &&
          color->stable_frames >= 1u &&
          color->min_dist_sq <= 8000u))) {
        uint8_t confirmed_2f = (color->label == COLOR_RED) ? 1u : 0u;
        seq->has_active_obs = 0u;
        seq->maneuver_yaw = calc_pull_in_yaw(seq, odom);
        enter_stage(seq, MISSION_RED_PULL_IN, tick);
        if (confirmed_2f) seq->seg_mm = 40.0f;
        return 1u;
    }

    return 0u;
}

static void record_map_obstacle(color_mission_t *seq, float bx, float by)
{
    uint8_t i;
    for (i = 0u; i < seq->map_obs_count; i++) {
        float dx = bx - seq->map_obs_x[i];
        float dy = by - seq->map_obs_y[i];
        if (dx * dx + dy * dy < 0.0256f) { /* 16cm内视为同一圆瓶，融合刷新坐标 */
            seq->map_obs_x[i] = 0.5f * (seq->map_obs_x[i] + bx);
            seq->map_obs_y[i] = 0.5f * (seq->map_obs_y[i] + by);
            return;
        }
    }
    if (seq->map_obs_count < MISSION_MAX_MAP_OBS) {
        seq->map_obs_x[seq->map_obs_count] = bx;
        seq->map_obs_y[seq->map_obs_count] = by;
        seq->map_obs_count++;
    }
}

/* 当PA1/PA2同时触发正前方圆瓶时，结合已知左右白边余量选择更宽敞的绕行侧（默认左绕+1） */
static int8_t choose_center_avoid_side(const color_mission_t *seq, float sy)
{
    if (seq->leg != MISSION_LEG_RETURN) {
        /* 去程：车左为+Y(left_bound)，车右为-Y(right_bound) */
        if (seq->has_left_bound && (seq->left_bound_y_m - sy) < 0.22f &&
            (!seq->has_right_bound || (sy - seq->right_bound_y_m) > 0.26f)) {
            return -1;
        }
        return 1;
    }
    /* 返程：车左为-Y(right_bound)，车右为+Y(left_bound) */
    if (seq->has_right_bound && (sy - seq->right_bound_y_m) < 0.22f &&
        (!seq->has_left_bound || (seq->left_bound_y_m - sy) > 0.26f)) {
        return -1;
    }
    return 1;
}

/* 双红外(PA1右/PA2左, 前视18cm)发现前方8cm直径圆瓶：
 * 不停车、边前进边快速大角度绕行：PA2左挡向右绕(-1)，PA1右挡向左绕(+1)，双挡择宽侧绕 */
static void start_avoid(color_mission_t *seq, const odometry_t *odom,
                        const obstacle_t *obstacle, uint32_t tick)
{
    float obs_dist_m = (float)MISSION_OBS_OFFSET_MM * 0.001f;
    float lat_off_m = (float)MISSION_OBS_LAT_OFFSET_MM * 0.001f;
    float sx = corridor_sx(seq, odom);
    float sy = corridor_sy(seq, odom);
    float cur_rel = wrap_pi(odom->yaw_rad - seq->goal_yaw);
    float obs_lat_m = 0.0f;
    float turn_mag = DEG62_RAD;
    float min_rel = DEG60_RAD;
    float target_rel;
    int8_t side;

    if (obstacle->left_blocked && !obstacle->right_blocked) {
        obs_lat_m = lat_off_m;
        side = -1;
        turn_mag = DEG62_RAD;
        min_rel = DEG60_RAD;
    } else if (obstacle->right_blocked && !obstacle->left_blocked) {
        obs_lat_m = -lat_off_m;
        side = 1;
        turn_mag = DEG62_RAD;
        min_rel = DEG60_RAD;
    } else {
        obs_lat_m = 0.0f;
        side = choose_center_avoid_side(seq, sy);
        turn_mag = DEG65_RAD;
        min_rel = DEG62_RAD;
    }

    if (seq->leg == MISSION_LEG_LEAVE_RED && sx >= 0.10f)
        seq->leg = MISSION_LEG_OUTBOUND;

    seq->obs_x_m = odom->x_m + obs_dist_m * cosf(odom->yaw_rad)
                   - obs_lat_m * sinf(odom->yaw_rad);
    seq->obs_y_m = odom->y_m + obs_dist_m * sinf(odom->yaw_rad)
                   + obs_lat_m * cosf(odom->yaw_rad);
    record_map_obstacle(seq, seq->obs_x_m, seq->obs_y_m);
    seq->has_active_obs = 1u;
    seq->has_last_white_side = 0u;
    seq->white_retry_count = 0u;
    if (seq->obs_count < 255u) seq->obs_count++;

    /* 若当前已在该侧偏转接近极限(>72度)仍遇阻，则反向脱困，绝不向后掉头 */
    if (cur_rel * (float)side >= DEG72_RAD) {
        side = (int8_t)(-side);
        target_rel = (float)side * DEG62_RAD;
    } else {
        target_rel = cur_rel + (float)side * turn_mag;
        if (side > 0 && target_rel < min_rel) target_rel = min_rel;
        if (side < 0 && target_rel > -min_rel) target_rel = -min_rel;
        if (target_rel > DEG80_RAD) target_rel = DEG80_RAD;
        if (target_rel < -DEG80_RAD) target_rel = -DEG80_RAD;
    }

    seq->avoid_side = side;
    seq->maneuver_yaw = wrap_pi(seq->goal_yaw + target_rel);
    enter_stage(seq, MISSION_AVOID_TURN, tick);
}

/* 利用全局地图预判双探头外侧盲区障碍(20cm车宽+8cm圆瓶)：返程提前快速弧线避让，跳过刚绕过的同一瓶子 */
static uint8_t check_map_blindspot_obs(color_mission_t *seq,
                                       const odometry_t *odom,
                                       uint32_t tick)
{
    uint8_t i;
    for (i = 0u; i < seq->map_obs_count; i++) {
        float dx, dy, fwd, lat;
        if (seq->has_active_obs) {
            float ax = seq->map_obs_x[i] - seq->obs_x_m;
            float ay = seq->map_obs_y[i] - seq->obs_y_m;
            if (ax * ax + ay * ay < 0.0484f) continue;
        }
        dx = seq->map_obs_x[i] - odom->x_m;
        dy = seq->map_obs_y[i] - odom->y_m;
        fwd = proj_along(dx, dy, odom->yaw_rad);
        lat = proj_left(dx, dy, odom->yaw_rad);
        if (fwd >= 0.14f && fwd <= 0.32f &&
            fabsf(lat) >= 0.080f && fabsf(lat) <= 0.185f) {
            seq->obs_x_m = seq->map_obs_x[i];
            seq->obs_y_m = seq->map_obs_y[i];
            seq->has_active_obs = 1u;
            if (lat > 0.0f) {
                seq->avoid_side = -1;
                seq->maneuver_yaw = wrap_pi(seq->goal_yaw - DEG56_RAD);
            } else {
                seq->avoid_side = 1;
                seq->maneuver_yaw = wrap_pi(seq->goal_yaw + DEG56_RAD);
            }
            enter_stage(seq, MISSION_AVOID_TURN, tick);
            return 1u;
        }
    }
    return 0u;
}

/* 限制航向漂移修正不超过当前行程基准航向±30度，杜绝连续修正导致掉头跑反 */
static void nudge_goal_yaw(color_mission_t *seq, float delta_rad)
{
    float base = (seq->leg == MISSION_LEG_RETURN) ?
                 seq->return_yaw : seq->outbound_yaw;
    float off = wrap_pi(seq->goal_yaw + delta_rad - base);
    if (off > DEG30_RAD) off = DEG30_RAD;
    if (off < -DEG30_RAD) off = -DEG30_RAD;
    seq->goal_yaw = wrap_pi(base + off);
}

/* 触碰双侧32x120cm白色边界统一处理：
 * 1) 若由主动转向避障出边(相对goal_yaw有明显横向位移或偏角)，直接按出边侧反向切回：
 *    左白边出 -> 右转(-1)切回；右白边出 -> 左转(+1)切回（去程与返程车体视角一致，全局视角相反）；
 * 2) 若直行巡航漂移出边，由全局走廊坐标sy与已知左右白边初判全局+Y/-Y，并开启290mm自校验兜底：
 *    去程(红->蓝)：全局+Y(左白边)右转(-1)，全局-Y(右白边)左转(+1)；
 *    返程(蓝->红)：操作相反，全局+Y(车右白边)左转(+1)，全局-Y(车左白边)右转(-1)。 */
static void start_white_pass_and_return(color_mission_t *seq,
                                        const odometry_t *odom,
                                        uint32_t tick)
{
    float sy = corridor_sy(seq, odom);
    float lat_goal = proj_left(odom->x_m - seq->last_mat_x_m,
                               odom->y_m - seq->last_mat_y_m,
                               seq->goal_yaw);
    float rel_yaw = wrap_pi(odom->yaw_rad - seq->goal_yaw);
    uint8_t on_global_pos_y;
    uint8_t confident = 1u;

    seq->white_armed = 0u;
    seq->white_retry_count = 0u;

    if (seq->stage != MISSION_CRUISE &&
        (lat_goal > 0.035f || rel_yaw > DEG15_RAD)) {
        /* 主动绕障向左偏出白边 -> 去程为全局+Y，返程为全局-Y */
        on_global_pos_y = (seq->leg != MISSION_LEG_RETURN) ? 1u : 0u;
    } else if (seq->stage != MISSION_CRUISE &&
               (lat_goal < -0.035f || rel_yaw < -DEG15_RAD)) {
        /* 主动绕障向右偏出白边 -> 去程为全局-Y，返程为全局+Y */
        on_global_pos_y = (seq->leg != MISSION_LEG_RETURN) ? 0u : 1u;
    } else {
        /* 巡航漂出白边：优先参考刚切回同侧与全局地图，同时始终开启280mm自校验严防打滑误判 */
        confident = 0u;
        if (rel_yaw > DEG20_RAD || lat_goal > 0.060f) {
            on_global_pos_y = (seq->leg != MISSION_LEG_RETURN) ? 1u : 0u;
        } else if (rel_yaw < -DEG20_RAD || lat_goal < -0.060f) {
            on_global_pos_y = (seq->leg != MISSION_LEG_RETURN) ? 0u : 1u;
        } else if (seq->has_last_white_side && seq->seg_mm < 1100.0f) {
            on_global_pos_y = seq->last_white_pos_y;
        } else if (seq->has_left_bound && seq->has_right_bound) {
            on_global_pos_y = (sy >= 0.5f * (seq->left_bound_y_m + seq->right_bound_y_m)) ? 1u : 0u;
        } else if (seq->has_left_bound && fabsf(sy - seq->left_bound_y_m) < 0.25f) {
            on_global_pos_y = 1u;
        } else if (seq->has_right_bound && fabsf(sy - seq->right_bound_y_m) < 0.25f) {
            on_global_pos_y = 0u;
        } else if ((seq->mat_max_sy - seq->mat_min_sy) > 0.12f) {
            on_global_pos_y = (sy >= 0.5f * (seq->mat_min_sy + seq->mat_max_sy)) ? 1u : 0u;
        } else {
            on_global_pos_y = (lat_goal >= 0.0f) ?
                              ((seq->leg != MISSION_LEG_RETURN) ? 1u : 0u) :
                              ((seq->leg != MISSION_LEG_RETURN) ? 0u : 1u);
        }
    }

    seq->white_side_uncertain = confident ? 0u : 1u;
    seq->last_white_pos_y = on_global_pos_y;
    seq->has_last_white_side = 1u;
    if (confident) {
        if (on_global_pos_y) {
            seq->left_bound_y_m = sy;
            seq->has_left_bound = 1u;
        } else {
            seq->right_bound_y_m = sy;
            seq->has_right_bound = 1u;
        }
    }

    if (seq->leg != MISSION_LEG_RETURN) {
        seq->return_side = on_global_pos_y ? -1 : 1;
    } else {
        seq->return_side = on_global_pos_y ? 1 : -1;
    }

    /* 直行巡航且无活跃障碍时擦到白边：无需前越水瓶，直接向垫面内切；临近端头加大内切角防漏检蓝/红区 */
    if (seq->stage == MISSION_CRUISE && !seq->has_active_obs) {
        float sx = corridor_sx(seq, odom);
        float blue_exp_sx = (seq->has_yellow_start ? seq->yellow_start_x_m : 0.20f) + 1.20f;
        float red_exp_sx = seq->has_blue_start ?
                           (seq->blue_start_x_m - 1.20f) : seq->yellow_start_x_m;
        float cut_ang = DEG54_RAD;
        if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx + 0.12f) ||
            (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx - 0.12f)) {
            cut_ang = DEG100_RAD;
        } else if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx - 0.25f) ||
                   (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx + 0.25f)) {
            cut_ang = DEG72_RAD;
        }
        if (seq->seg_mm >= 280.0f) {
            nudge_goal_yaw(seq, (float)seq->return_side * DEG8_RAD);
        }
        seq->maneuver_yaw = wrap_pi(seq->goal_yaw +
                                    (float)seq->return_side * cut_ang);
        enter_stage(seq, MISSION_WHITE_RETURN, tick);
        return;
    }

    if (fabsf(rel_yaw) > DEG8_RAD) {
        enter_stage(seq, MISSION_WHITE_ALIGN, tick);
    } else {
        enter_stage(seq, MISSION_WHITE_PASS, tick);
    }
}

void color_mission_init(color_mission_t *seq, const odometry_t *odom, uint32_t tick)
{
    *seq = (color_mission_t){0};
    seq->leg = MISSION_LEG_LEAVE_RED;
    seq->stage = MISSION_BOOT;
    seq->boot_tick = tick;
    seq->stage_tick = tick;
    seq->origin_x_m = odom->x_m;
    seq->origin_y_m = odom->y_m;
    seq->last_mat_x_m = odom->x_m;
    seq->last_mat_y_m = odom->y_m;
    seq->outbound_yaw = odom->yaw_rad;
    seq->return_yaw = wrap_pi(odom->yaw_rad + PI);
    seq->goal_yaw = seq->outbound_yaw;
    seq->maneuver_yaw = seq->outbound_yaw;
    seq->yellow_start_x_m = 0.20f;
    seq->white_armed = 1u;
    seq->avoid_side = 1;
    seq->return_side = -1;
    seq->turn_dir = 1;
}

void color_mission_cancel(color_mission_t *seq)
{
    if (seq->stage != MISSION_IDLE && seq->stage < MISSION_DONE)
        seq->stage = MISSION_CANCELLED;
}

void color_mission_step(color_mission_t *seq, motion_t *motion,
                        const odometry_t *odom, const obstacle_t *obstacle,
                        const color_sensor_t *color, uint32_t tick,
                        int32_t delta_left, int32_t delta_right,
                        int32_t speed_left, int32_t speed_right)
{
    float ds_mm;
    (void)speed_left;
    (void)speed_right;

    if (seq->stage == MISSION_IDLE || seq->stage >= MISSION_DONE) return;

    if ((uint32_t)(tick - seq->boot_tick) >= MISSION_TOTAL_TIMEOUT_MS) {
        motion_stop(motion);
        seq->stage = MISSION_TIMEOUT;
        return;
    }
    if (seq->stage != MISSION_CRUISE && seq->stage != MISSION_BOOT &&
        (uint32_t)(tick - seq->stage_tick) >= MISSION_MANEUVER_TIMEOUT_MS) {
        motion_stop(motion);
        seq->stage = MISSION_TIMEOUT;
        return;
    }

    ds_mm = ((float)delta_left * MM_PER_PULSE_L +
             (float)delta_right * MM_PER_PULSE_R) * 0.5f;
    if (ds_mm > 0.0f) seq->seg_mm += ds_mm;

    if (color->frame_ready && color->frame_count != seq->last_color_frame) {
        seq->last_color_frame = color->frame_count;
        if (seq->stage != MISSION_BOOT)
            update_global_map(seq, odom, color);
    }

    switch (seq->stage) {
    case MISSION_BOOT:
        motion_stop(motion);
        if ((uint32_t)(tick - seq->boot_tick) >= MISSION_BOOT_MS) {
            seq->origin_x_m = odom->x_m;
            seq->origin_y_m = odom->y_m;
            seq->last_mat_x_m = odom->x_m;
            seq->last_mat_y_m = odom->y_m;
            seq->mat_min_sy = 0.0f;
            seq->mat_max_sy = 0.0f;
            seq->outbound_yaw = odom->yaw_rad;
            seq->return_yaw = wrap_pi(odom->yaw_rad + PI);
            seq->goal_yaw = seq->outbound_yaw;
            seq->white_armed = 1u;
            enter_stage(seq, MISSION_CRUISE, tick);
        }
        break;

    case MISSION_CRUISE: {
        float sx = corridor_sx(seq, odom);
        float blue_exp_sx = (seq->has_yellow_start ? seq->yellow_start_x_m : 0.20f) + 1.20f;
        float red_exp_sx = seq->has_blue_start ?
                           (seq->blue_start_x_m - 1.20f) : seq->yellow_start_x_m;
        int32_t v = MISSION_CRUISE_MM_S;
        float err;

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        if (obstacle->blocked) {
            start_avoid(seq, odom, obstacle, tick);
            break;
        }
        if (seq->leg == MISSION_LEG_RETURN &&
            check_map_blindspot_obs(seq, odom, tick)) {
            break;
        }
        if (is_border_hit(seq, color) &&
            (seq->leg != MISSION_LEG_LEAVE_RED || sx >= 0.06f)) {
            start_white_pass_and_return(seq, odom, tick);
            break;
        }

        if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx - 0.28f) ||
            (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx + 0.28f)) {
            v = MISSION_APPROACH_MM_S;
        }
        if (seq->seg_mm >= 180.0f) seq->has_active_obs = 0u;

        err = wrap_pi(seq->goal_yaw - odom->yaw_rad);
        motion_set_velocity(motion, v,
                            heading_track_w(err, MISSION_TRACK_MAX_MRAD_S),
                            tick);
        break;
    }

    case MISSION_AVOID_TURN: {
        float err = wrap_pi(seq->maneuver_yaw - odom->yaw_rad);
        float side_rel = wrap_pi(odom->yaw_rad - seq->goal_yaw) *
                         (float)seq->avoid_side;
        uint8_t both_clear = (obstacle->raw_right != IR_ACTIVE_LEVEL &&
                              obstacle->raw_left != IR_ACTIVE_LEVEL) ? 1u : 0u;
        uint8_t inner_blocked = (seq->avoid_side > 0) ?
                                (obstacle->raw_left == IR_ACTIVE_LEVEL) :
                                (obstacle->raw_right == IR_ACTIVE_LEVEL);
        uint8_t both_blocked = (obstacle->raw_left == IR_ACTIVE_LEVEL &&
                                obstacle->raw_right == IR_ACTIVE_LEVEL) ? 1u : 0u;
        int32_t turn_v, turn_w;

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        if (is_border_hit(seq, color)) {
            start_white_pass_and_return(seq, odom, tick);
            break;
        }

        if (both_clear) {
            if (seq->clear_ms < MISSION_AVOID_CLEAR_MS)
                seq->clear_ms += SPEED_SAMPLE_MS;
        } else {
            seq->clear_ms = 0u;
        }

        if (fabsf(err) <= (float)MISSION_YAW_TOL_MRAD * 0.001f) {
            if (seq->clear_ms >= MISSION_AVOID_CLEAR_MS) {
                seq->maneuver_yaw = odom->yaw_rad;
                enter_stage(seq, MISSION_AVOID_BYPASS, tick);
                break;
            }
            if (!both_clear) {
                /* 已转到目标角但探头仍遮挡：先递增至100度横向绕出，对侧再扩至135度/165度，绝不卡死在容差死区 */
                float limit_ang = (seq->white_retry_count >= 2u) ? DEG165_RAD :
                                  ((seq->white_retry_count == 1u) ? DEG135_RAD : DEG100_RAD);
                if (side_rel >= limit_ang - 0.08f) {
                    seq->avoid_side = (int8_t)(-seq->avoid_side);
                    if (seq->white_retry_count < 10u) seq->white_retry_count++;
                    seq->maneuver_yaw = wrap_pi(seq->goal_yaw +
                                                (float)seq->avoid_side * DEG65_RAD);
                } else {
                    float next_rel = wrap_pi(odom->yaw_rad - seq->goal_yaw) +
                                     (float)seq->avoid_side * DEG20_RAD;
                    if (next_rel > limit_ang) next_rel = limit_ang;
                    if (next_rel < -limit_ang) next_rel = -limit_ang;
                    seq->maneuver_yaw = wrap_pi(seq->goal_yaw + next_rel);
                }
                err = wrap_pi(seq->maneuver_yaw - odom->yaw_rad);
            }
        }
        /* 单瓶正常转向内侧探头畅通，保持边走边快转零停顿；若内侧受阻或近距探头持续遮挡超45mm则原地快转防擦撞 */
        if (inner_blocked || (both_blocked && side_rel > DEG20_RAD) ||
            (!both_clear && seq->seg_mm >= 45.0f) || fabsf(err) > DEG80_RAD) {
            turn_v = 0L;
        } else if (!both_clear) {
            turn_v = (side_rel < DEG15_RAD) ? 95L : 120L;
        } else {
            turn_v = MISSION_AVOID_TURN_MM_S;
        }
        turn_w = (fabsf(err) <= (float)MISSION_YAW_TOL_MRAD * 0.001f) ?
                 heading_track_w(err, MISSION_TURN_MAX_MRAD_S) :
                 heading_turn_w(err);
        motion_set_velocity(motion, turn_v, turn_w, tick);
        break;
    }

    case MISSION_AVOID_BYPASS: {
        float past_mm = proj_along(odom->x_m - seq->obs_x_m,
                                   odom->y_m - seq->obs_y_m,
                                   seq->goal_yaw) * 1000.0f;
        float err;

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        if (obstacle->blocked) {
            start_avoid(seq, odom, obstacle, tick);
            break;
        }
        if (is_border_hit(seq, color)) {
            start_white_pass_and_return(seq, odom, tick);
            break;
        }
        /* 20cm车宽+8cm圆瓶横向拉开安全距离且车头越过圆瓶前段(past_mm >= -70mm)即刻快速回正 */
        if (seq->seg_mm >= (float)MISSION_BYPASS_MAX_MM ||
            (seq->seg_mm >= (float)MISSION_BYPASS_MIN_MM && past_mm >= -70.0f)) {
            enter_stage(seq, MISSION_AVOID_REALIGN, tick);
            break;
        }

        err = wrap_pi(seq->maneuver_yaw - odom->yaw_rad);
        motion_set_velocity(motion, MISSION_BYPASS_MM_S,
                            heading_track_w(err, MISSION_TRACK_MAX_MRAD_S),
                            tick);
        break;
    }

    case MISSION_AVOID_REALIGN: {
        float err = wrap_pi(seq->goal_yaw - odom->yaw_rad);

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        if (obstacle->blocked) {
            start_avoid(seq, odom, obstacle, tick);
            break;
        }
        if (seq->leg == MISSION_LEG_RETURN &&
            check_map_blindspot_obs(seq, odom, tick)) {
            break;
        }
        if (is_border_hit(seq, color)) {
            start_white_pass_and_return(seq, odom, tick);
            break;
        }
        if (fabsf(err) <= (float)MISSION_YAW_TOL_MRAD * 0.001f) {
            if (color->label == COLOR_WHITE || color->label == COLOR_UNKNOWN ||
                seq->non_mat_frames >= 2u) {
                start_white_pass_and_return(seq, odom, tick);
            } else {
                seq->has_active_obs = 0u;
                enter_stage(seq, MISSION_CRUISE, tick);
            }
            break;
        }
        /* 使用高增益快速转向函数干脆回正至goal_yaw，彻底解决回正拖沓问题 */
        motion_set_velocity(motion, MISSION_REALIGN_MM_S,
                            heading_turn_w(err),
                            tick);
        break;
    }

    case MISSION_WHITE_ALIGN: {
        float err = wrap_pi(seq->goal_yaw - odom->yaw_rad);

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        /* 若从外偏角向白边拉直过程中白边正前方仍有实测遮挡则避让；从垫内内切拉直时不重复外拐 */
        if ((obstacle->raw_left == IR_ACTIVE_LEVEL ||
             obstacle->raw_right == IR_ACTIVE_LEVEL) &&
            wrap_pi(odom->yaw_rad - seq->goal_yaw) * (float)seq->return_side < -DEG12_RAD) {
            start_avoid(seq, odom, obstacle, tick);
            break;
        }
        if (fabsf(err) <= DEG8_RAD) {
            enter_stage(seq, MISSION_WHITE_PASS, tick);
            break;
        }
        /* 在32cm宽白边上快速回正平行于走廊方向，防止越出白边外缘 */
        motion_set_velocity(motion, MISSION_WHITE_ALIGN_MM_S,
                            heading_turn_w(err), tick);
        break;
    }

    case MISSION_WHITE_PASS: {
        float past_mm = seq->has_active_obs ?
                        proj_along(odom->x_m - seq->obs_x_m,
                                   odom->y_m - seq->obs_y_m,
                                   seq->goal_yaw) * 1000.0f : 1000.0f;
        float sx = corridor_sx(seq, odom);
        float blue_exp_sx = (seq->has_yellow_start ? seq->yellow_start_x_m : 0.20f) + 1.20f;
        float red_exp_sx = seq->has_blue_start ?
                           (seq->blue_start_x_m - 1.20f) : seq->yellow_start_x_m;
        uint8_t obs_passed = (!seq->has_active_obs ||
                              past_mm >= (float)MISSION_OBS_PASS_MARGIN_MM) ? 1u : 0u;
        uint8_t near_strip_end =
            ((seq->leg != MISSION_LEG_RETURN &&
              sx >= blue_exp_sx - 0.36f &&
              (obs_passed || sx >= blue_exp_sx - 0.08f)) ||
             (seq->leg == MISSION_LEG_RETURN &&
              sx <= red_exp_sx + 0.36f &&
              (obs_passed || sx <= red_exp_sx + 0.08f))) &&
            seq->seg_mm >= 50.0f ? 1u : 0u;
        float err;

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        if (obstacle->blocked && seq->seg_mm >= 40.0f) {
            start_avoid(seq, odom, obstacle, tick);
            break;
        }
        if (color->label == COLOR_YELLOW && seq->non_mat_frames == 0u && obs_passed) {
            seq->has_active_obs = 0u;
            enter_stage(seq, MISSION_CRUISE, tick);
            break;
        }
        /* 沿32x120cm白边前越圆瓶后（或临近蓝/红端头前）及时大角度向垫面内切回 */
        if (near_strip_end ||
            seq->seg_mm >= (float)MISSION_WHITE_PASS_MAX_MM ||
            (seq->seg_mm >= (float)MISSION_WHITE_PASS_MIN_MM && obs_passed)) {
            float cut_ang = near_strip_end ? DEG65_RAD : DEG42_RAD;
            seq->has_active_obs = 0u;
            seq->maneuver_yaw = wrap_pi(seq->goal_yaw +
                                        (float)seq->return_side * cut_ang);
            enter_stage(seq, MISSION_WHITE_RETURN, tick);
            break;
        }

        err = wrap_pi(seq->goal_yaw - odom->yaw_rad);
        motion_set_velocity(motion, MISSION_BYPASS_MM_S,
                            heading_track_w(err, MISSION_TRACK_MAX_MRAD_S),
                            tick);
        break;
    }

    case MISSION_WHITE_RETURN: {
        float err = wrap_pi(seq->maneuver_yaw - odom->yaw_rad);

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        if (obstacle->blocked) {
            if (color->candidate == COLOR_WHITE || color->candidate == COLOR_UNKNOWN) {
                /* 仍在白边外侧内切时探头扫到垫内圆瓶：回正沿当前白边继续前越该圆瓶 */
                float obs_dist_m = (float)MISSION_OBS_OFFSET_MM * 0.001f;
                seq->obs_x_m = odom->x_m + obs_dist_m * cosf(odom->yaw_rad);
                seq->obs_y_m = odom->y_m + obs_dist_m * sinf(odom->yaw_rad);
                record_map_obstacle(seq, seq->obs_x_m, seq->obs_y_m);
                seq->has_active_obs = 1u;
                if (seq->obs_count < 255u) seq->obs_count++;
                enter_stage(seq, MISSION_WHITE_ALIGN, tick);
            } else {
                start_avoid(seq, odom, obstacle, tick);
            }
            break;
        }
        if (color->label != COLOR_WHITE && color->label != COLOR_UNKNOWN &&
            seq->non_mat_frames == 0u &&
            color->stable_frames >= COLOR_STABLE_FRAMES) {
            seq->white_side_uncertain = 0u;
            enter_stage(seq, MISSION_AVOID_REALIGN, tick);
            break;
        }
        /* 巡航微偏出边自校验：54度内切走满245mm仍全非彩垫则立即反转切回，确保整车中心严格留在32cm白边内 */
        if (seq->white_side_uncertain && seq->white_retry_count == 0u &&
            seq->seg_mm >= 245.0f &&
            (color->candidate == COLOR_WHITE || color->candidate == COLOR_UNKNOWN)) {
            float sx = corridor_sx(seq, odom);
            float sy = corridor_sy(seq, odom);
            float blue_exp_sx = (seq->has_yellow_start ? seq->yellow_start_x_m : 0.20f) + 1.20f;
            float red_exp_sx = seq->has_blue_start ?
                               (seq->blue_start_x_m - 1.20f) : seq->yellow_start_x_m;
            float flip_ang = DEG58_RAD;
            if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx + 0.08f) ||
                (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx - 0.08f)) {
                flip_ang = DEG100_RAD;
            } else if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx - 0.20f) ||
                       (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx + 0.20f)) {
                flip_ang = DEG80_RAD;
            }
            seq->return_side = (int8_t)(-seq->return_side);
            seq->white_side_uncertain = 0u;
            seq->white_retry_count = 1u;
            seq->last_white_pos_y = (uint8_t)(!seq->last_white_pos_y);
            nudge_goal_yaw(seq, 2.0f * (float)seq->return_side * DEG8_RAD);
            if ((seq->leg != MISSION_LEG_RETURN && seq->return_side < 0) ||
                (seq->leg == MISSION_LEG_RETURN && seq->return_side > 0)) {
                seq->left_bound_y_m = sy;
                seq->has_left_bound = 1u;
            } else {
                seq->right_bound_y_m = sy;
                seq->has_right_bound = 1u;
            }
            seq->maneuver_yaw = wrap_pi(seq->goal_yaw +
                                        (float)seq->return_side * flip_ang);
            enter_stage(seq, MISSION_WHITE_RETURN, tick);
            break;
        }
        if (seq->seg_mm >= (float)MISSION_WHITE_RETURN_MAX_MM) {
            float sx = corridor_sx(seq, odom);
            float blue_exp_sx = (seq->has_yellow_start ? seq->yellow_start_x_m : 0.20f) + 1.20f;
            float red_exp_sx = seq->has_blue_start ?
                               (seq->blue_start_x_m - 1.20f) : seq->yellow_start_x_m;
            float ret_ang = DEG54_RAD;
            if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx + 0.12f) ||
                (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx - 0.12f)) {
                ret_ang = DEG100_RAD;
            } else if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx) ||
                       (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx)) {
                /* 若纵向已抵近蓝/红端头仍在白边外侧，加大至80~100度横向切入色区，防止越过端头漏检 */
                ret_ang = DEG80_RAD;
            }
            if (seq->white_retry_count < 10u) seq->white_retry_count++;
            nudge_goal_yaw(seq, (float)seq->return_side * DEG10_RAD);
            seq->maneuver_yaw = wrap_pi(seq->goal_yaw +
                                        (float)seq->return_side * ret_ang);
            enter_stage(seq, MISSION_WHITE_RETURN, tick);
            break;
        }

        motion_set_velocity(motion, MISSION_WHITE_RETURN_MM_S,
                            heading_turn_w(err),
                            tick);
        break;
    }

    case MISSION_BLUE_PULL_IN: {
        float err = wrap_pi(seq->maneuver_yaw - odom->yaw_rad);
        if (seq->seg_mm >= (float)MISSION_BLUE_PULL_IN_MM ||
            obstacle->blocked ||
            (seq->seg_mm >= 55.0f &&
             (color->candidate == COLOR_WHITE || color->candidate == COLOR_UNKNOWN))) {
            float sy = corridor_sy(seq, odom);
            float center_sy = 0.5f * (seq->mat_min_sy + seq->mat_max_sy);
            float turn_err;
            seq->leg = MISSION_LEG_RETURN;
            seq->left_blue = 0u;
            seq->has_last_white_side = 0u;
            seq->goal_yaw = seq->return_yaw;
            turn_err = wrap_pi(seq->goal_yaw - odom->yaw_rad);
            /* 靠近+Y左边缘则顺时针(-1)朝垫内掉头，靠近-Y右边缘则逆时针(+1)朝垫内掉头 */
            if (sy > center_sy + 0.05f) seq->turn_dir = -1;
            else if (sy < center_sy - 0.05f) seq->turn_dir = 1;
            else seq->turn_dir = (turn_err >= -0.05f) ? 1 : -1;
            enter_stage(seq, MISSION_BLUE_TURN_180, tick);
            break;
        }
        motion_set_velocity(motion, MISSION_APPROACH_MM_S,
                            heading_track_w(err, MISSION_TRACK_MAX_MRAD_S),
                            tick);
        break;
    }

    case MISSION_BLUE_TURN_180: {
        float err = wrap_pi(seq->goal_yaw - odom->yaw_rad);
        int32_t w;

        if (fabsf(err) <= (float)MISSION_TURN180_TOL_MRAD * 0.001f) {
            if (obstacle->blocked) {
                start_avoid(seq, odom, obstacle, tick);
                seq->seg_mm = 45.0f;
            } else {
                enter_stage(seq, MISSION_CRUISE, tick);
            }
            break;
        }
        if (fabsf(err) > 1.48f) {
            w = (int32_t)seq->turn_dir * MISSION_TURN_MAX_MRAD_S;
        } else {
            w = heading_turn_w(err);
        }
        motion_set_velocity(motion, 0, w, tick);
        break;
    }

    case MISSION_RED_PULL_IN: {
        float err = wrap_pi(seq->maneuver_yaw - odom->yaw_rad);
        if (seq->seg_mm >= (float)MISSION_RED_PULL_IN_MM ||
            obstacle->blocked ||
            (seq->seg_mm >= 45.0f &&
             (color->candidate == COLOR_WHITE || color->candidate == COLOR_UNKNOWN))) {
            motion_stop(motion);
            enter_stage(seq, MISSION_DONE, tick);
            break;
        }
        motion_set_velocity(motion, MISSION_APPROACH_MM_S,
                            heading_track_w(err, MISSION_TRACK_MAX_MRAD_S),
                            tick);
        break;
    }

    default:
        motion_stop(motion);
        break;
    }
}

const char *color_mission_leg_name(mission_leg_t leg)
{
    static const char *names[] = {
        "leave_red", "outbound", "return"
    };
    return names[leg];
}

const char *color_mission_stage_name(mission_stage_t stage)
{
    static const char *names[] = {
        "idle", "boot", "cruise",
        "avoid_turn", "avoid_bypass", "realign",
        "white_align", "white_pass", "white_return",
        "blue_in", "turn_180", "red_in",
        "done", "cancelled", "timeout"
    };
    return names[stage];
}
