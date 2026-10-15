#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H
#include <stdint.h>
void speed_control_init(void);
void speed_control_reset(void);
void speed_control_step(int32_t target_left, int32_t target_right,
                        int32_t actual_left, int32_t actual_right,
                        int16_t *left_pwm, int16_t *right_pwm);
#endif
