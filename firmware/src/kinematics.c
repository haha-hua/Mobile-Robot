#include "kinematics.h"
#include "config.h"

void kinematics_inverse(int32_t v_mm_s, int32_t w_mrad_s,
                        int32_t *left_mm_s, int32_t *right_mm_s)
{
    int64_t left = (int64_t)v_mm_s * 2000 - (int64_t)w_mrad_s * WHEEL_TRACK_MM;
    int64_t right = (int64_t)v_mm_s * 2000 + (int64_t)w_mrad_s * WHEEL_TRACK_MM;
    int64_t peak = left < 0 ? -left : left;
    int64_t right_abs = right < 0 ? -right : right;
    if (right_abs > peak) peak = right_abs;
    if (peak > (int64_t)MAX_WHEEL_SPEED_MM_S * 2000) {
        left = left * MAX_WHEEL_SPEED_MM_S * 2000 / peak;
        right = right * MAX_WHEEL_SPEED_MM_S * 2000 / peak;
    }
    *left_mm_s = (int32_t)(left / 2000);
    *right_mm_s = (int32_t)(right / 2000);
}

void kinematics_forward(float left_mm_s, float right_mm_s,
                        kinematics_body_t *body)
{
    body->linear_m_s = (left_mm_s + right_mm_s) * 0.0005f;
    body->angular_rad_s = (right_mm_s - left_mm_s) / (float)WHEEL_TRACK_MM;
}
