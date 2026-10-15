#include "odometry.h"
#include "config.h"
#include "kinematics.h"
#include <math.h>

static float ticks_to_m(int32_t ticks, int32_t counts_10_revs)
{
    return (float)ticks * (float)WHEEL_CIRCUMFERENCE_UM * 10.0e-6f /
           (float)counts_10_revs;
}

static float wrap_angle(float angle)
{
    while (angle > 3.14159265358979323846f) angle -= 6.28318530717958647692f;
    while (angle <= -3.14159265358979323846f) angle += 6.28318530717958647692f;
    return angle;
}

void odometry_init(odometry_t *odom)
{
    *odom = (odometry_t){0};
}

void odometry_update(odometry_t *odom, int32_t delta_left_ticks,
                     int32_t delta_right_ticks, uint32_t period_ms)
{
    float left = ticks_to_m(delta_left_ticks, LEFT_COUNTS_10_REVS);
    float right = ticks_to_m(delta_right_ticks, RIGHT_COUNTS_10_REVS);
    float ds = (left + right) * 0.5f;
    float dtheta = (right - left) / ((float)WHEEL_TRACK_MM * 0.001f);
    float half_angle = dtheta * 0.5f;
    float heading = odom->yaw_rad + half_angle;
    float sinc;
    if (half_angle > -1.0e-5f && half_angle < 1.0e-5f) {
        sinc = 1.0f - half_angle * half_angle / 6.0f;
    } else {
        sinc = sinf(half_angle) / half_angle;
    }
    odom->x_m += ds * sinc * cosf(heading);
    odom->y_m += ds * sinc * sinf(heading);
    odom->yaw_rad = wrap_angle(odom->yaw_rad + dtheta);
    if (period_ms != 0u) {
        kinematics_body_t body;
        kinematics_forward(left * 1.0e6f / (float)period_ms,
                           right * 1.0e6f / (float)period_ms, &body);
        odom->linear_m_s = body.linear_m_s;
        odom->angular_rad_s = body.angular_rad_s;
    }
}
