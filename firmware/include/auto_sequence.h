#ifndef AUTO_SEQUENCE_H
#define AUTO_SEQUENCE_H
#include "motion.h"

/* 实验二分段流程：所有段之间停车对准，使用已验证的直线与原地旋转逻辑。 */
typedef enum {
    SEQ_BOOT_WAIT,   /* 上电等待 */
    SEQ_STRAIGHT_OUT, /* 直线到圆周下端 */
    SEQ_TURN_CIRCLE,  /* 原地左转90度对准圆弧切线 */
    SEQ_ARC,          /* 顺时针圆弧 */
    SEQ_TURN_HOME,    /* 原地对准原点 */
    SEQ_RETURN,       /* 直线返回起点 */
    SEQ_STOPPING,     /* 等待停稳 */
    SEQ_DONE, SEQ_CANCELLED, SEQ_TIMEOUT
} seq_stage_t;

typedef struct {
    seq_stage_t stage, next_stage;
    uint32_t boot_tick, stage_tick, stable_ms;
    float total_left_mm, total_right_mm; /* 累计轮程，浮点避免低速截断停滞 */
    float start_avg_mm, start_diff_mm;   /* 段起点轮程均值与左右差 */
    int32_t return_mm;                   /* 返回段目标距离 */
    int32_t turn_home_mrad;              /* 对准原点的原地转角 */
    int32_t rel_mm, rel_mrad;            /* 当前段进度，供日志 */
} auto_sequence_t;

void auto_sequence_init(auto_sequence_t *seq, uint32_t tick);
void auto_sequence_cancel(auto_sequence_t *seq);
void auto_sequence_step(auto_sequence_t *seq, motion_t *motion, uint32_t tick,
                        int32_t delta_left, int32_t delta_right,
                        int32_t speed_left, int32_t speed_right);
const char *auto_sequence_stage_name(seq_stage_t stage);
#endif
