#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <stdint.h>

typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;
    float linear_m_s;
    float angular_rad_s;
} odometry_t;

void odometry_init(odometry_t *odom);
void odometry_update(odometry_t *odom, int32_t delta_left_ticks,
                     int32_t delta_right_ticks, uint32_t period_ms);
#endif
