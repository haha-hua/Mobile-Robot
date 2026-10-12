#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* 电机语义层:只认"哪个电机"和"速度",不出现 TIM/GPIO 字眼 */

typedef enum {
    MOTOR_LEFT = 0,     /* 左轮,PC6,TIM3_CH1 */
    MOTOR_RIGHT         /* 右轮,PC7,TIM3_CH2 */
} motor_id_t;

/* 初始化电机(内部调用 pwm_init)。上电时电机为停止状态 */
void motor_init(void);

/* 设置电机输出 duty:0(停)~ 1000(最大)。
 * 注意:当前为开环调速,只有"快慢",还没有"方向"——
 * 左右电机为镜像安装,同占空比时转向天然相反,
 * 待确认电机模块的 PIN 针含义后,补上方向控制并统一为"正 = 车头方向" */
void motor_set(motor_id_t id, uint16_t duty);

/* 两个电机全部停止 */
void motor_stop(void);

#endif /* MOTOR_H */
