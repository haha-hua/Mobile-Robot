#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

typedef enum { MOTOR_LEFT = 0, MOTOR_RIGHT } motor_id_t;
/* 初始化保持待机禁用，PWM为零。 */
void motor_init(void);
void motor_enable(uint8_t enabled);
/* 有符号占空比：正数前进，负数后退，限制为正负PWM_MAX。 */
void motor_set(motor_id_t id, int16_t command);
/* 清零PWM和方向线；控制器状态由控制模块管理。 */
void motor_stop(void);
#endif
