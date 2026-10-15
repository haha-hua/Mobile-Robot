#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <stdint.h>

typedef struct {
    float linear_m_s;
    float angular_rad_s;
} kinematics_body_t;

/* 轮速mm/s；逆解车体输入mm/s、mrad/s，正解输出m/s、rad/s。 */
void kinematics_inverse(int32_t v_mm_s, int32_t w_mrad_s,
                        int32_t *left_mm_s, int32_t *right_mm_s);
void kinematics_forward(float left_mm_s, float right_mm_s,
                        kinematics_body_t *body);
#endif
