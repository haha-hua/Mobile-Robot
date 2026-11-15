#include "color_mission.h"
#include "config.h"
#include <math.h>
#include <stdlib.h>

#define PI         3.14159265358979323846f
#define DEG8_RAD   0.1396263f
#define DEG10_RAD  0.1745329f
#define DEG12_RAD  0.2094395f
#define DEG15_RAD  0.2617994f
#define DEG18_RAD  0.3141593f
#define DEG22_RAD  0.3839724f
#define DEG38_RAD  0.6632251f
#define DEG45_RAD  0.7853982f
#define DEG48_RAD  0.8377580f
#define DEG54_RAD  0.9424778f
#define DEG56_RAD  0.9773844f
#define DEG60_RAD  1.0471976f
#define DEG65_RAD  1.1344640f
#define DEG72_RAD  1.2566371f
#define DEG75_RAD  1.3089969f
#define DEG80_RAD  1.3962634f
#define DEG95_RAD  1.6580628f
#define DEG105_RAD 1.8325957f
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

static float proj_along(float dx, float dy, float axis)
{
    return dx * cosf(axis) + dy * sinf(axis);
}

static float proj_left(float dx, float dy, float axis)
{
    return -dx * sinf(axis) + dy * cosf(axis);
}

static float corridor_sx(const color_mission_t *seq, const odometry_t *odom)
{
    return proj_along(odom->x_m - seq->origin_x_m,
                      odom->y_m - seq->origin_y_m,
                      seq->outbound_yaw);
}

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

static int32_t heading_track_w(float error, int32_t max_w)
{
    float w;
    if (fabsf(error) < 0.015f) return 0;
    w = error * 2200.0f;
    if (w > (float)max_w) w = (float)max_w;
    if (w < -(float)max_w) w = -(float)max_w;
    return (int32_t)w;
}

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

static void nudge_goal_yaw(color_mission_t *seq, float delta_rad)
{
    float base = (seq->leg == MISSION_LEG_RETURN) ?
                 seq->return_yaw : seq->outbound_yaw;
    float off = wrap_pi(seq->goal_yaw + delta_rad - base);
    if (off > DEG60_RAD) off = DEG60_RAD;
    if (off < -DEG60_RAD) off = -DEG60_RAD;
    seq->goal_yaw = wrap_pi(base + off);
}

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

static uint8_t is_border_hit(const color_mission_t *seq,
                             const color_sensor_t *color)
{
    if (!seq->white_armed) return 0u;
    if (color->label == COLOR_WHITE) return 1u;
    if (color->label == COLOR_UNKNOWN && color->frame_count >= 4u) return 1u;
    if (seq->non_mat_frames >= 2u && color->frame_count >= 4u) return 1u;
    return 0u;
}

static uint8_t is_zone_preconfirm(const color_mission_t *seq,
                                  const odometry_t *odom,
                                  const color_sensor_t *color)
{
    float sx = corridor_sx(seq, odom);
    float blue_exp_sx = (seq->has_yellow_start ? seq->yellow_start_x_m : 0.20f) + 1.20f;
    float red_exp_sx = seq->has_blue_start ?
                       (seq->blue_start_x_m - 1.20f) : seq->yellow_start_x_m;

    if ((seq->leg == MISSION_LEG_OUTBOUND ||
         (seq->leg == MISSION_LEG_LEAVE_RED && sx >= 0.40f)) &&
        sx >= blue_exp_sx - 0.45f &&
        color->candidate == COLOR_BLUE && color->stable_frames >= 1u) {
        return 1u;
    }
    if (seq->leg == MISSION_LEG_RETURN && seq->left_blue &&
        sx <= red_exp_sx + 0.45f &&
        color->candidate == COLOR_RED && color->stable_frames >= 1u) {
        return 1u;
    }
    return 0u;
}

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
        sx >= blue_exp_sx - 0.48f &&
        color->label == COLOR_BLUE) {
        seq->blue_start_x_m = sx;
        seq->has_blue_start = 1u;
        seq->has_active_obs = 0u;
        seq->scan_hold = 0u;
        seq->maneuver_yaw = calc_pull_in_yaw(seq, odom);
        enter_stage(seq, MISSION_BLUE_PULL_IN, tick);
        return 1u;
    }

    if (seq->leg == MISSION_LEG_RETURN && seq->left_blue &&
        sx <= red_exp_sx + 0.48f &&
        color->label == COLOR_RED) {
        seq->has_active_obs = 0u;
        seq->scan_hold = 0u;
        seq->maneuver_yaw = calc_pull_in_yaw(seq, odom);
        enter_stage(seq, MISSION_RED_PULL_IN, tick);
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
        if (dx * dx + dy * dy < 0.0324f) { /* 18cm内视为同一圆瓶，融合刷新坐标 */
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

/* 评估从枢轴点(px,py)沿候选航向dir_yaw行驶0~38cm与地图中已记录其他水瓶的最小中心距(m) */
static float eval_heading_map_clearance(const color_mission_t *seq,
                                        float px, float py, float dir_yaw)
{
    float ux = cosf(dir_yaw);
    float uy = sinf(dir_yaw);
    float min_dist = 9.9f;
    uint8_t i;

    for (i = 0u; i < seq->map_obs_count; i++) {
        float ax = seq->map_obs_x[i] - seq->obs_x_m;
        float ay = seq->map_obs_y[i] - seq->obs_y_m;
        float vx, vy, t, qx, qy, d;
        if (ax * ax + ay * ay < 0.0324f) continue; /* 跳过当前正前方同一圆瓶 */

        vx = seq->map_obs_x[i] - px;
        vy = seq->map_obs_y[i] - py;
        t = vx * ux + vy * uy;
        if (t < -0.05f) continue;
        if (t < 0.0f) t = 0.0f;
        if (t > 0.38f) t = 0.38f;
        qx = vx - t * ux;
        qy = vy - t * uy;
        d = hypotf(qx, qy);
        if (d < min_dist) min_dist = d;
    }
    return min_dist;
}

/* 检查当前位置原地枢轴旋转(四角旋转半径13.5cm+瓶半径4cm+3cm净距=20.5cm)是否会扫到已记录水瓶 */
static uint8_t is_pivot_safe_from_map(const color_mission_t *seq,
                                      const odometry_t *odom,
                                      float move_yaw)
{
    float ux = cosf(move_yaw);
    float uy = sinf(move_yaw);
    uint8_t i;
    for (i = 0u; i < seq->map_obs_count; i++) {
        float dx = odom->x_m - seq->map_obs_x[i];
        float dy = odom->y_m - seq->map_obs_y[i];
        float d2 = dx * dx + dy * dy;
        /* 若距离小于23.5cm且继续前进正在远离该水瓶，则再多走几厘米拉开后再原地回正 */
        if (d2 < 0.0552f && (dx * ux + dy * uy) > -0.03f) {
            return 0u;
        }
    }
    return 1u;
}

/* 检查车尾后退6cm是否安全（不退穿起止垫外缘、不退撞身后已过水瓶、不在白边外退） */
static uint8_t can_safely_backoff(const color_mission_t *seq,
                                  const odometry_t *odom,
                                  const color_sensor_t *color)
{
    float sx = corridor_sx(seq, odom);
    uint8_t i;

    if (color->candidate == COLOR_WHITE || color->candidate == COLOR_UNKNOWN ||
        seq->non_mat_frames > 0u) {
        return 0u;
    }
    if (seq->leg != MISSION_LEG_RETURN) {
        if (sx < 0.06f) return 0u;
    } else {
        if (seq->has_blue_start && sx > seq->blue_start_x_m + 0.04f) return 0u;
    }
    for (i = 0u; i < seq->map_obs_count; i++) {
        float dx = seq->map_obs_x[i] - odom->x_m;
        float dy = seq->map_obs_y[i] - odom->y_m;
        float bx = proj_along(dx, dy, odom->yaw_rad);
        float by = proj_left(dx, dy, odom->yaw_rad);
        if (bx < -0.04f && bx > -0.30f && fabsf(by) < 0.20f) {
            return 0u;
        }
    }
    return 1u;
}

/* 结合扫掠角速度前导波束边沿(+/-12.6度)与2cm双探头先后次序精确定位圆瓶并基于真实方位角规划绕行 */
static void compute_avoid_plan(color_mission_t *seq, const odometry_t *odom)
{
    float obs_dist_m = (float)MISSION_OBS_OFFSET_MM * 0.001f;
    float lat_off_m = (float)MISSION_OBS_LAT_OFFSET_MM * 0.001f;
    float back_m = seq->allow_backoff ?
                   ((float)MISSION_AVOID_BACK_MM * 0.001f) : 0.0f;
    float piv_x = seq->trigger_x_m - back_m * cosf(seq->trigger_yaw);
    float piv_y = seq->trigger_y_m - back_m * sinf(seq->trigger_yaw);
    float piv_sy = proj_left(piv_x - seq->origin_x_m,
                             piv_y - seq->origin_y_m,
                             seq->outbound_yaw);
    float ray_yaw = seq->trigger_yaw;
    float obs_lat_body = 0.0f;
    float obs_yaw, obs_rel;
    float rel_L, rel_R;
    float room_left, room_right;
    float clr_L, clr_R;
    float score_L, score_R;

    /* 动态旋转扫掠补偿：旋转中障碍物必从红外波束前导边缘(约12.6度)率先切入 */
    if (seq->trigger_rot_dir > 0) {
        ray_yaw += (seq->first_ir_side < 0) ? 0.09f : 0.22f;
    } else if (seq->trigger_rot_dir < 0) {
        ray_yaw -= (seq->first_ir_side > 0) ? 0.09f : 0.22f;
    }

    if (seq->saw_left_ir && !seq->saw_right_ir) {
        obs_lat_body = lat_off_m;
    } else if (seq->saw_right_ir && !seq->saw_left_ir) {
        obs_lat_body = -lat_off_m;
    } else {
        obs_lat_body = (float)seq->first_ir_side * 0.014f;
    }

    seq->obs_x_m = seq->trigger_x_m + obs_dist_m * cosf(ray_yaw)
                   - obs_lat_body * sinf(ray_yaw);
    seq->obs_y_m = seq->trigger_y_m + obs_dist_m * sinf(ray_yaw)
                   + obs_lat_body * cosf(ray_yaw);

    obs_yaw = atan2f(seq->obs_y_m - piv_y, seq->obs_x_m - piv_x);
    obs_rel = wrap_pi(obs_yaw - seq->goal_yaw);

    /* 以障碍物真实方位角张开±60度生成左绕(L)与右绕(R)两条候选切线，保证距瓶中心线净距>=25cm */
    rel_L = obs_rel + DEG60_RAD;
    rel_R = obs_rel - DEG60_RAD;
    if (fabsf(obs_rel) < DEG48_RAD) {
        if (rel_L < DEG56_RAD) rel_L = DEG56_RAD;
        if (rel_R > -DEG56_RAD) rel_R = -DEG56_RAD;
    }
    if (rel_L > DEG105_RAD) rel_L = DEG105_RAD;
    if (rel_R < -DEG105_RAD) rel_R = -DEG105_RAD;

    if (seq->leg != MISSION_LEG_RETURN) {
        float l_lim = seq->has_left_bound ? seq->left_bound_y_m : 0.60f;
        float r_lim = seq->has_right_bound ? seq->right_bound_y_m : -0.60f;
        room_left = l_lim - piv_sy;
        room_right = piv_sy - r_lim;
    } else {
        float l_lim = seq->has_right_bound ? seq->right_bound_y_m : -0.60f;
        float r_lim = seq->has_left_bound ? seq->left_bound_y_m : 0.60f;
        room_left = piv_sy - l_lim;
        room_right = r_lim - piv_sy;
    }

    clr_L = eval_heading_map_clearance(seq, piv_x, piv_y,
                                       wrap_pi(seq->goal_yaw + rel_L));
    clr_R = eval_heading_map_clearance(seq, piv_x, piv_y,
                                       wrap_pi(seq->goal_yaw + rel_R));

    score_L = -0.60f * fabsf(rel_L)
              - 0.18f * fabsf(wrap_pi(seq->goal_yaw + rel_L - odom->yaw_rad));
    score_R = -0.60f * fabsf(rel_R)
              - 0.18f * fabsf(wrap_pi(seq->goal_yaw + rel_R - odom->yaw_rad));

    /* 优先走宽敞黄区内侧绕行；仅当内侧存在已知水瓶阻挡(中心距<24cm)时才重罚并借道32cm白边越障 */
    if (rel_L > 0.20f && room_left < 0.25f)
        score_L -= (0.25f - room_left) * 4.8f;
    if (rel_R < -0.20f && room_right < 0.25f)
        score_R -= (0.25f - room_right) * 4.8f;
    if (rel_L > 0.30f && room_left < 0.04f) score_L -= 2.2f;
    if (rel_R < -0.30f && room_right < 0.04f) score_R -= 2.2f;

    if (clr_L < 0.242f) score_L -= (0.242f - clr_L) * 48.0f;
    if (clr_R < 0.242f) score_R -= (0.242f - clr_R) * 48.0f;

    if (score_L >= score_R) {
        seq->avoid_side = 1;
        seq->maneuver_yaw = wrap_pi(seq->goal_yaw + rel_L);
    } else {
        seq->avoid_side = -1;
        seq->maneuver_yaw = wrap_pi(seq->goal_yaw + rel_R);
    }
}

static void start_avoid(color_mission_t *seq, const odometry_t *odom,
                        const obstacle_t *obstacle, const color_sensor_t *color,
                        float dyaw, uint32_t tick)
{
    float sx = corridor_sx(seq, odom);
    uint8_t l_hit = (obstacle->left_blocked ||
                     obstacle->raw_left == IR_ACTIVE_LEVEL) ? 1u : 0u;
    uint8_t r_hit = (obstacle->right_blocked ||
                     obstacle->raw_right == IR_ACTIVE_LEVEL) ? 1u : 0u;

    if (seq->leg == MISSION_LEG_LEAVE_RED && sx >= 0.08f)
        seq->leg = MISSION_LEG_OUTBOUND;

    seq->trigger_x_m = odom->x_m;
    seq->trigger_y_m = odom->y_m;
    seq->trigger_yaw = odom->yaw_rad;
    seq->saw_left_ir = l_hit;
    seq->saw_right_ir = r_hit;
    seq->first_ir_side = (l_hit && !r_hit) ? 1 : ((!l_hit && r_hit) ? -1 : 0);
    seq->trigger_rot_dir = (dyaw > 0.005f) ? 1 : ((dyaw < -0.005f) ? -1 : 0);
    seq->avoid_settled = 0u;
    seq->back_mm = 0.0f;
    seq->allow_backoff = can_safely_backoff(seq, odom, color);
    seq->scan_hold = 0u;

    compute_avoid_plan(seq, odom);
    record_map_obstacle(seq, seq->obs_x_m, seq->obs_y_m);

    seq->has_active_obs = 1u;
    seq->has_last_white_side = 0u;
    seq->white_retry_count = 0u;
    if (seq->obs_count < 255u) seq->obs_count++;

    enter_stage(seq, MISSION_AVOID_TURN, tick);
}

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
    seq->scan_hold = 0u;

    if (seq->stage != MISSION_CRUISE &&
        (lat_goal > 0.035f || rel_yaw > DEG18_RAD)) {
        on_global_pos_y = (seq->leg != MISSION_LEG_RETURN) ? 1u : 0u;
    } else if (seq->stage != MISSION_CRUISE &&
               (lat_goal < -0.035f || rel_yaw < -DEG18_RAD)) {
        on_global_pos_y = (seq->leg != MISSION_LEG_RETURN) ? 0u : 1u;
    } else {
        confident = 0u;
        if (lat_goal > 0.075f) {
            on_global_pos_y = (seq->leg != MISSION_LEG_RETURN) ? 1u : 0u;
        } else if (lat_goal < -0.075f) {
            on_global_pos_y = (seq->leg != MISSION_LEG_RETURN) ? 0u : 1u;
        } else if (seq->has_last_white_side && seq->seg_mm < 900.0f) {
            on_global_pos_y = seq->last_white_pos_y;
        } else if (seq->has_left_bound && seq->has_right_bound) {
            on_global_pos_y = (sy >= 0.5f * (seq->left_bound_y_m + seq->right_bound_y_m)) ? 1u : 0u;
        } else if (seq->has_left_bound && fabsf(sy - seq->left_bound_y_m) < 0.25f) {
            on_global_pos_y = 1u;
        } else if (seq->has_right_bound && fabsf(sy - seq->right_bound_y_m) < 0.25f) {
            on_global_pos_y = 0u;
        } else if ((seq->mat_max_sy - seq->mat_min_sy) > 0.14f) {
            on_global_pos_y = (sy >= 0.5f * (seq->mat_min_sy + seq->mat_max_sy)) ? 1u : 0u;
        } else {
            on_global_pos_y = (sy >= 0.0f) ? 1u : 0u;
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

    if (seq->stage == MISSION_CRUISE && !seq->has_active_obs) {
        float sx = corridor_sx(seq, odom);
        float blue_exp_sx = (seq->has_yellow_start ? seq->yellow_start_x_m : 0.20f) + 1.20f;
        float red_exp_sx = seq->has_blue_start ?
                           (seq->blue_start_x_m - 1.20f) : seq->yellow_start_x_m;
        float cut_ang = DEG54_RAD;
        if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx + 0.10f) ||
            (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx - 0.10f)) {
            cut_ang = DEG95_RAD;
        } else if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx - 0.25f) ||
                   (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx + 0.25f)) {
            cut_ang = DEG72_RAD;
        }
        if (seq->seg_mm >= 260.0f) {
            nudge_goal_yaw(seq, (float)seq->return_side * DEG10_RAD);
        }
        seq->maneuver_yaw = wrap_pi(seq->goal_yaw +
                                    (float)seq->return_side * cut_ang);
        enter_stage(seq, MISSION_WHITE_RETURN, tick);
        return;
    }

    /* 绕障进入白边时微收3.5度修正斜放/打滑航向偏角，保持在32cm白边中线安全越障 */
    if (confident) {
        nudge_goal_yaw(seq, (float)seq->return_side * 0.06f);
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
    seq->prev_yaw = odom->yaw_rad;
    seq->yellow_start_x_m = 0.20f;
    seq->white_armed = 1u;
    seq->scan_dir = 1;
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
    float dyaw;
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
    else if (ds_mm < 0.0f) seq->back_mm += -ds_mm;

    dyaw = wrap_pi(odom->yaw_rad - seq->prev_yaw);
    seq->prev_yaw = odom->yaw_rad;

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
            seq->prev_yaw = odom->yaw_rad;
            seq->white_armed = 1u;
            seq->scan_dir = 1;
            enter_stage(seq, MISSION_CRUISE, tick);
        }
        break;

    case MISSION_CRUISE: {
        float sx = corridor_sx(seq, odom);
        float blue_exp_sx = (seq->has_yellow_start ? seq->yellow_start_x_m : 0.20f) + 1.20f;
        float red_exp_sx = seq->has_blue_start ?
                           (seq->blue_start_x_m - 1.20f) : seq->yellow_start_x_m;
        float rel = wrap_pi(odom->yaw_rad - seq->goal_yaw);
        int32_t v = MISSION_CRUISE_MM_S;
        int32_t w;

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        if (obstacle->blocked) {
            start_avoid(seq, odom, obstacle, color, dyaw, tick);
            break;
        }
        if (is_border_hit(seq, color) &&
            (seq->leg != MISSION_LEG_LEAVE_RED || sx >= 0.06f)) {
            start_white_pass_and_return(seq, odom, tick);
            break;
        }

        if (seq->seg_mm >= 90.0f) {
            seq->has_active_obs = 0u;
            seq->scan_hold = 0u;
        }

        if (is_zone_preconfirm(seq, odom, color)) {
            v = MISSION_ZONE_CONFIRM_MM_S;
            w = heading_track_w(wrap_pi(seq->goal_yaw - odom->yaw_rad),
                                MISSION_TRACK_MAX_MRAD_S);
        } else if ((seq->leg == MISSION_LEG_LEAVE_RED && sx < 0.06f) ||
                   seq->scan_hold) {
            v = 130L;
            w = heading_track_w(wrap_pi(seq->goal_yaw - odom->yaw_rad),
                                MISSION_TRACK_MAX_MRAD_S);
        } else {
            float flip_rad = (float)MISSION_SCAN_FLIP_MRAD * 0.001f;
            if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx - 0.25f) ||
                (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx + 0.25f)) {
                v = MISSION_APPROACH_MM_S;
            }
            if (fabsf(rel) > DEG38_RAD) {
                w = heading_track_w(wrap_pi(seq->goal_yaw - odom->yaw_rad),
                                    MISSION_TURN_MAX_MRAD_S);
            } else {
                if (seq->scan_dir > 0 && rel >= flip_rad) seq->scan_dir = -1;
                else if (seq->scan_dir < 0 && rel <= -flip_rad) seq->scan_dir = 1;
                w = (int32_t)seq->scan_dir * MISSION_SCAN_TURN_MRAD_S;
            }
        }

        motion_set_velocity(motion, v, w, tick);
        break;
    }

    case MISSION_AVOID_TURN: {
        uint32_t elapsed_ms = (uint32_t)(tick - seq->stage_tick);
        uint8_t both_clear = (obstacle->raw_right != IR_ACTIVE_LEVEL &&
                              obstacle->raw_left != IR_ACTIVE_LEVEL) ? 1u : 0u;
        float err, side_rel;
        int32_t turn_w;

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        if (is_border_hit(seq, color)) {
            start_white_pass_and_return(seq, odom, tick);
            break;
        }

        /* 阶段1：同步采集双探头 + 安全微退5.8cm拉开前轮对角旋转半径(13.5cm)，杜绝近距原地转向蹭前轮 */
        if (!seq->avoid_settled) {
            uint8_t prev_l = seq->saw_left_ir;
            uint8_t prev_r = seq->saw_right_ir;
            if (obstacle->left_blocked || obstacle->raw_left == IR_ACTIVE_LEVEL)
                seq->saw_left_ir = 1u;
            if (obstacle->right_blocked || obstacle->raw_right == IR_ACTIVE_LEVEL)
                seq->saw_right_ir = 1u;
            if (seq->saw_left_ir != prev_l || seq->saw_right_ir != prev_r) {
                compute_avoid_plan(seq, odom);
                record_map_obstacle(seq, seq->obs_x_m, seq->obs_y_m);
            }
            if (seq->allow_backoff &&
                seq->back_mm < (float)MISSION_AVOID_BACK_MM &&
                elapsed_ms < 900u) {
                motion_set_velocity(motion, -MISSION_AVOID_BACK_MM_S, 0L, tick);
                break;
            }
            if (elapsed_ms < MISSION_AVOID_SETTLE_MS ||
                abs(motion->ramp_left + motion->ramp_right) > 20L) {
                motion_set_velocity(motion, 0L, 0L, tick);
                break;
            }
            seq->avoid_settled = 1u;
            seq->clear_ms = 0u;
        }

        /* 阶段2：纯原地枢轴快转(v=0)至绕行角，同时利用旋转过程实时激光式扫掠侧方并排第2瓶 */
        side_rel = wrap_pi(odom->yaw_rad - seq->goal_yaw) * (float)seq->avoid_side;
        if (!both_clear &&
            wrap_pi(odom->yaw_rad - seq->trigger_yaw) * (float)seq->avoid_side > DEG22_RAD) {
            float obs_dist_m = (float)MISSION_OBS_OFFSET_MM * 0.001f;
            float ray_yaw = odom->yaw_rad + (float)seq->avoid_side * 0.14f;
            float req_rel = side_rel + DEG54_RAD;
            float cur_target_rel = wrap_pi(seq->maneuver_yaw - seq->goal_yaw) *
                                   (float)seq->avoid_side;
            seq->obs_x_m = odom->x_m + obs_dist_m * cosf(ray_yaw);
            seq->obs_y_m = odom->y_m + obs_dist_m * sinf(ray_yaw);
            record_map_obstacle(seq, seq->obs_x_m, seq->obs_y_m);
            if (req_rel > DEG105_RAD) req_rel = DEG105_RAD;
            if (req_rel > cur_target_rel) {
                seq->maneuver_yaw = wrap_pi(seq->goal_yaw +
                                            (float)seq->avoid_side * req_rel);
            }
        }

        err = wrap_pi(seq->maneuver_yaw - odom->yaw_rad);

        if (both_clear) {
            if (seq->clear_ms < MISSION_AVOID_CLEAR_MS)
                seq->clear_ms += SPEED_SAMPLE_MS;
        } else {
            seq->clear_ms = 0u;
        }

        if (fabsf(err) <= (float)MISSION_YAW_TOL_MRAD * 0.001f) {
            if (seq->clear_ms >= MISSION_AVOID_CLEAR_MS) {
                seq->maneuver_yaw = odom->yaw_rad;
                seq->scan_dir = seq->avoid_side;
                enter_stage(seq, MISSION_AVOID_BYPASS, tick);
                break;
            }
            if (!both_clear) {
                float obs_dist_m = (float)MISSION_OBS_OFFSET_MM * 0.001f;
                float limit_ang = (seq->white_retry_count >= 2u) ? DEG165_RAD :
                                  ((seq->white_retry_count == 1u) ? DEG135_RAD : DEG105_RAD);
                seq->obs_x_m = odom->x_m + obs_dist_m * cosf(odom->yaw_rad);
                seq->obs_y_m = odom->y_m + obs_dist_m * sinf(odom->yaw_rad);
                record_map_obstacle(seq, seq->obs_x_m, seq->obs_y_m);

                if (side_rel >= limit_ang - 0.08f) {
                    seq->avoid_side = (int8_t)(-seq->avoid_side);
                    if (seq->white_retry_count < 10u) seq->white_retry_count++;
                    seq->maneuver_yaw = wrap_pi(seq->goal_yaw +
                                                (float)seq->avoid_side * DEG60_RAD);
                } else {
                    float next_rel = wrap_pi(odom->yaw_rad - seq->goal_yaw) +
                                     (float)seq->avoid_side * DEG22_RAD;
                    if (next_rel > limit_ang) next_rel = limit_ang;
                    if (next_rel < -limit_ang) next_rel = -limit_ang;
                    seq->maneuver_yaw = wrap_pi(seq->goal_yaw + next_rel);
                }
                err = wrap_pi(seq->maneuver_yaw - odom->yaw_rad);
            }
        }

        turn_w = (fabsf(err) <= (float)MISSION_YAW_TOL_MRAD * 0.001f) ?
                 heading_track_w(err, MISSION_TURN_MAX_MRAD_S) :
                 heading_turn_w(err);
        motion_set_velocity(motion, MISSION_AVOID_TURN_MM_S, turn_w, tick);
        break;
    }

    case MISSION_AVOID_BYPASS: {
        float lat_obs_mm = proj_left(odom->x_m - seq->obs_x_m,
                                     odom->y_m - seq->obs_y_m,
                                     seq->goal_yaw) *
                           (float)seq->avoid_side * 1000.0f;
        float along_obs_mm = proj_along(odom->x_m - seq->obs_x_m,
                                        odom->y_m - seq->obs_y_m,
                                        seq->goal_yaw) * 1000.0f;
        float out_rel = wrap_pi(odom->yaw_rad - seq->maneuver_yaw) *
                        (float)seq->avoid_side;
        int32_t v = is_zone_preconfirm(seq, odom, color) ?
                    MISSION_ZONE_CONFIRM_MM_S : MISSION_BYPASS_MM_S;
        int32_t w;

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        if (obstacle->blocked) {
            start_avoid(seq, odom, obstacle, color, dyaw, tick);
            break;
        }
        if (is_border_hit(seq, color)) {
            start_white_pass_and_return(seq, odom, tick);
            break;
        }
        if (seq->seg_mm >= (float)MISSION_BYPASS_MAX_MM ||
            (seq->seg_mm >= (float)MISSION_BYPASS_MIN_MM &&
             (lat_obs_mm >= (float)MISSION_OBS_SAFE_LAT_MM ||
              along_obs_mm >= 120.0f) &&
             is_pivot_safe_from_map(seq, odom, seq->maneuver_yaw))) {
            enter_stage(seq, MISSION_AVOID_REALIGN, tick);
            break;
        }

        /* 单向外展微摆扫掠[0, +14.5度*avoid_side]：绝不向被绕水瓶内拐，同时消除外轮盲区未记录水瓶漏检 */
        if (fabsf(out_rel) > DEG38_RAD) {
            w = heading_track_w(wrap_pi(seq->maneuver_yaw - odom->yaw_rad),
                                MISSION_TURN_MAX_MRAD_S);
        } else {
            if (seq->scan_dir == seq->avoid_side && out_rel >= 0.25f) {
                seq->scan_dir = (int8_t)(-seq->avoid_side);
            } else if (seq->scan_dir != seq->avoid_side && out_rel <= 0.01f) {
                seq->scan_dir = seq->avoid_side;
            }
            w = (int32_t)seq->scan_dir * MISSION_SCAN_TURN_MRAD_S;
        }
        motion_set_velocity(motion, v, w, tick);
        break;
    }

    case MISSION_AVOID_REALIGN: {
        float err = wrap_pi(seq->goal_yaw - odom->yaw_rad);

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        if (obstacle->blocked) {
            start_avoid(seq, odom, obstacle, color, dyaw, tick);
            break;
        }
        if (is_border_hit(seq, color)) {
            start_white_pass_and_return(seq, odom, tick);
            break;
        }

        if (abs(motion->ramp_left + motion->ramp_right) > 20L) {
            motion_set_velocity(motion, 0L, 0L, tick);
            break;
        }

        if (fabsf(err) <= (float)MISSION_YAW_TOL_MRAD * 0.001f) {
            if (color->label == COLOR_WHITE || color->label == COLOR_UNKNOWN ||
                seq->non_mat_frames >= 2u) {
                start_white_pass_and_return(seq, odom, tick);
            } else {
                seq->has_active_obs = 0u;
                seq->scan_hold = 0u;
                /* 首拍向-avoid_side微扫：既补扫REALIGN未覆盖的内侧扇区，又使后轮远离刚绕过的水瓶 */
                seq->scan_dir = (int8_t)(-seq->avoid_side);
                enter_stage(seq, MISSION_CRUISE, tick);
            }
            break;
        }
        motion_set_velocity(motion, MISSION_REALIGN_MM_S,
                            heading_turn_w(err),
                            tick);
        break;
    }

    case MISSION_WHITE_ALIGN: {
        float err = wrap_pi(seq->goal_yaw - odom->yaw_rad);

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        if ((obstacle->raw_left == IR_ACTIVE_LEVEL ||
             obstacle->raw_right == IR_ACTIVE_LEVEL) &&
            wrap_pi(odom->yaw_rad - seq->goal_yaw) * (float)seq->return_side < -DEG12_RAD) {
            start_avoid(seq, odom, obstacle, color, dyaw, tick);
            break;
        }

        if (abs(motion->ramp_left + motion->ramp_right) > 20L) {
            motion_set_velocity(motion, 0L, 0L, tick);
            break;
        }

        if (fabsf(err) <= (float)MISSION_YAW_TOL_MRAD * 0.001f) {
            enter_stage(seq, MISSION_WHITE_PASS, tick);
            break;
        }
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
              sx >= blue_exp_sx - 0.34f && obs_passed) ||
             (seq->leg == MISSION_LEG_RETURN &&
              sx <= red_exp_sx + 0.34f && obs_passed)) &&
            seq->seg_mm >= 50.0f ? 1u : 0u;
        int32_t v = is_zone_preconfirm(seq, odom, color) ?
                    MISSION_ZONE_CONFIRM_MM_S : MISSION_BYPASS_MM_S;
        float err;

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        if (obstacle->blocked && seq->seg_mm >= 30.0f) {
            start_avoid(seq, odom, obstacle, color, dyaw, tick);
            break;
        }
        if (color->label == COLOR_YELLOW && seq->non_mat_frames == 0u && obs_passed) {
            seq->has_active_obs = 0u;
            seq->scan_hold = 0u;
            seq->scan_dir = seq->return_side;
            enter_stage(seq, MISSION_CRUISE, tick);
            break;
        }
        if (color->label == COLOR_UNKNOWN && seq->seg_mm >= 60.0f) {
            /* 轮径误差导致沿白边直行偏出外侧地板时，立即修正航向基准并折返场内 */
            nudge_goal_yaw(seq, (float)seq->return_side * DEG15_RAD);
            seq->has_active_obs = 0u;
            seq->maneuver_yaw = wrap_pi(seq->goal_yaw +
                                        (float)seq->return_side * DEG56_RAD);
            enter_stage(seq, MISSION_WHITE_RETURN, tick);
            break;
        }
        if (near_strip_end ||
            seq->seg_mm >= (float)MISSION_WHITE_PASS_MAX_MM ||
            (seq->seg_mm >= (float)MISSION_WHITE_PASS_MIN_MM && obs_passed)) {
            float cut_ang = near_strip_end ? DEG65_RAD : DEG45_RAD;
            seq->has_active_obs = 0u;
            seq->maneuver_yaw = wrap_pi(seq->goal_yaw +
                                        (float)seq->return_side * cut_ang);
            enter_stage(seq, MISSION_WHITE_RETURN, tick);
            break;
        }

        err = wrap_pi(seq->goal_yaw - odom->yaw_rad);
        motion_set_velocity(motion, v,
                            heading_track_w(err, MISSION_TRACK_MAX_MRAD_S),
                            tick);
        break;
    }

    case MISSION_WHITE_RETURN: {
        float base_yaw = (seq->leg == MISSION_LEG_RETURN) ?
                         seq->return_yaw : seq->outbound_yaw;
        float err = wrap_pi(seq->maneuver_yaw - odom->yaw_rad);
        int32_t v, w;

        if (check_zone_arrival(seq, odom, obstacle, color, tick)) break;

        if (obstacle->blocked) {
            if (color->candidate == COLOR_WHITE || color->candidate == COLOR_UNKNOWN) {
                float obs_dist_m = (float)MISSION_OBS_OFFSET_MM * 0.001f;
                seq->obs_x_m = odom->x_m + obs_dist_m * cosf(odom->yaw_rad);
                seq->obs_y_m = odom->y_m + obs_dist_m * sinf(odom->yaw_rad);
                record_map_obstacle(seq, seq->obs_x_m, seq->obs_y_m);
                seq->has_active_obs = 1u;
                if (seq->obs_count < 255u) seq->obs_count++;
                enter_stage(seq, MISSION_WHITE_ALIGN, tick);
            } else {
                start_avoid(seq, odom, obstacle, color, dyaw, tick);
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
        if (seq->white_side_uncertain && seq->white_retry_count == 0u &&
            seq->seg_mm >= 440.0f &&
            (color->candidate == COLOR_WHITE || color->candidate == COLOR_UNKNOWN)) {
            float sx = corridor_sx(seq, odom);
            float sy = corridor_sy(seq, odom);
            float blue_exp_sx = (seq->has_yellow_start ? seq->yellow_start_x_m : 0.20f) + 1.20f;
            float red_exp_sx = seq->has_blue_start ?
                               (seq->blue_start_x_m - 1.20f) : seq->yellow_start_x_m;
            float flip_ang = DEG65_RAD;
            if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx + 0.06f) ||
                (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx - 0.06f)) {
                flip_ang = DEG105_RAD;
            } else if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx - 0.22f) ||
                       (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx + 0.22f)) {
                flip_ang = DEG80_RAD;
            }
            seq->return_side = (int8_t)(-seq->return_side);
            seq->white_side_uncertain = 0u;
            seq->white_retry_count = 1u;
            seq->last_white_pos_y = (uint8_t)(!seq->last_white_pos_y);
            nudge_goal_yaw(seq, 2.0f * (float)seq->return_side * DEG10_RAD);
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
            {
                float rel_base = wrap_pi(seq->maneuver_yaw - base_yaw);
                if (rel_base > DEG105_RAD) rel_base = DEG105_RAD;
                if (rel_base < -DEG105_RAD) rel_base = -DEG105_RAD;
                seq->maneuver_yaw = wrap_pi(base_yaw + rel_base);
            }
            seq->seg_mm = 0.0f;
            break;
        }
        if (seq->seg_mm >= (float)MISSION_WHITE_RETURN_MAX_MM) {
            float sx = corridor_sx(seq, odom);
            float blue_exp_sx = (seq->has_yellow_start ? seq->yellow_start_x_m : 0.20f) + 1.20f;
            float red_exp_sx = seq->has_blue_start ?
                               (seq->blue_start_x_m - 1.20f) : seq->yellow_start_x_m;
            float ret_ang = DEG65_RAD;
            float rel_base;
            if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx + 0.08f) ||
                (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx - 0.08f)) {
                ret_ang = DEG105_RAD;
            } else if ((seq->leg != MISSION_LEG_RETURN && sx >= blue_exp_sx - 0.15f) ||
                       (seq->leg == MISSION_LEG_RETURN && sx <= red_exp_sx + 0.15f)) {
                ret_ang = DEG80_RAD;
            }
            if (seq->white_retry_count < 10u) seq->white_retry_count++;
            nudge_goal_yaw(seq, (float)seq->return_side * DEG15_RAD);
            seq->maneuver_yaw = wrap_pi(seq->goal_yaw +
                                        (float)seq->return_side * ret_ang);
            rel_base = wrap_pi(seq->maneuver_yaw - base_yaw);
            if (rel_base > DEG105_RAD) rel_base = DEG105_RAD;
            if (rel_base < -DEG105_RAD) rel_base = -DEG105_RAD;
            seq->maneuver_yaw = wrap_pi(base_yaw + rel_base);
            seq->seg_mm = 0.0f;
            break;
        }

        if (fabsf(err) > DEG22_RAD) {
            v = 0L;
            w = heading_turn_w(err);
        } else {
            float rel_r = -err;
            v = is_zone_preconfirm(seq, odom, color) ?
                MISSION_ZONE_CONFIRM_MM_S : MISSION_WHITE_RETURN_MM_S;
            if (seq->scan_dir > 0 && rel_r >= 0.24f) seq->scan_dir = -1;
            else if (seq->scan_dir < 0 && rel_r <= -0.24f) seq->scan_dir = 1;
            w = (int32_t)seq->scan_dir * 1450L;
        }
        motion_set_velocity(motion, v, w, tick);
        break;
    }

    case MISSION_BLUE_PULL_IN: {
        float err = wrap_pi(seq->maneuver_yaw - odom->yaw_rad);
        if (seq->seg_mm >= (float)MISSION_BLUE_PULL_IN_MM ||
            obstacle->blocked ||
            (seq->seg_mm >= 30.0f &&
             (color->candidate == COLOR_WHITE || color->candidate == COLOR_UNKNOWN))) {
            float sy = corridor_sy(seq, odom);
            float center_sy = 0.5f * (seq->mat_min_sy + seq->mat_max_sy);
            float turn_err;
            seq->leg = MISSION_LEG_RETURN;
            seq->left_blue = 0u;
            seq->has_last_white_side = 0u;
            seq->goal_yaw = seq->return_yaw;
            turn_err = wrap_pi(seq->goal_yaw - odom->yaw_rad);
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

        if (abs(motion->ramp_left + motion->ramp_right) > 20L) {
            motion_set_velocity(motion, 0L, 0L, tick);
            break;
        }

        if (fabsf(err) <= (float)MISSION_TURN180_TOL_MRAD * 0.001f) {
            if (obstacle->blocked) {
                start_avoid(seq, odom, obstacle, color, dyaw, tick);
            } else {
                seq->scan_hold = 0u;
                seq->scan_dir = seq->turn_dir;
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
            (seq->seg_mm >= 30.0f &&
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
