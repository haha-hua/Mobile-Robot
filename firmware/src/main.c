#include "bsp_uart.h"
#include "config.h"
#include "control.h"
#include "encoder.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>

static const char *state_name(motion_state_t state)
{
    static const char *names[] = {"idle", "run", "timeout"};
    return names[state];
}

static long fixed_milli(float value)
{
    return (long)(value * 1000.0f + (value < 0.0f ? -0.5f : 0.5f));
}

int main(void)
{
    uint32_t report_start = 0u;
    char report[384];
    char line[UART_LINE_CAPACITY];
    const char *reply = 0;
    control_snapshot_t snapshot;

    motor_init();
    encoder_init();
    uart_init();
    control_init();
    {
        int length = snprintf(report, sizeof(report),
            "READY: auto in 2s: forward 1m, left 360deg, left 90deg, stop once\r\n"
            "stop/valid vel v w (mm/s,mrad/s)/wheel L R (mm/s) cancels auto\r\n"
            "effective_track=%ldmm physical_track=%ldmm timeout=%ums accel=%ld reverse_gate=%ld\r\n"
            "counts/10rev L=%ld R=%ld\r\n",
            (long)WHEEL_TRACK_MM, (long)PHYSICAL_WHEEL_TRACK_MM, COMMAND_TIMEOUT_MS,
            (long)WHEEL_ACCEL_MM_S2, (long)REVERSE_SAFE_SPEED_MM_S,
            (long)LEFT_COUNTS_10_REVS, (long)RIGHT_COUNTS_10_REVS);
        if (length > 0 && length < (int)sizeof(report)) uart_try_write(report, (uint16_t)length);
    }
    while (1) {
        int8_t result;
        if (reply) {
            if (uart_try_write(reply, (uint16_t)strlen(reply))) reply = 0;
        } else if ((result = uart_read_line(line)) != 0) {
            if (result < 0) reply = "ERR line discarded (length/RX fault)\r\n";
            else reply = control_command(line) ? "OK\r\n" : "ERR syntax/range\r\n";
        }
        if (!reply && (uint32_t)(control_tick() - report_start) >= 200u) {
            int length;
            control_snapshot(&snapshot);
            length = snprintf(report, sizeof(report),
                "%s t=%lu auto=%s rel_mm=%+ld rel_mrad=%+ld targetL_mm_s=%+ld targetR_mm_s=%+ld actualL_mm_s=%+ld actualR_mm_s=%+ld "
                "v_mm_s=%+ld w_mrad_s=%+ld x_mm=%+ld y_mm=%+ld yaw_mrad=%+ld pwmL=%+d pwmR=%+d\r\n",
                state_name(snapshot.state), (unsigned long)snapshot.tick,
                test_sequence_stage_name(snapshot.test_stage),
                (long)snapshot.test_distance_mm, fixed_milli(snapshot.test_angle_rad),
                (long)snapshot.target_left, (long)snapshot.target_right,
                (long)motion_pulses_to_mm(snapshot.speed_left, LEFT_COUNTS_10_REVS),
                (long)motion_pulses_to_mm(snapshot.speed_right, RIGHT_COUNTS_10_REVS),
                fixed_milli(snapshot.odom.linear_m_s),
                fixed_milli(snapshot.odom.angular_rad_s),
                fixed_milli(snapshot.odom.x_m),
                fixed_milli(snapshot.odom.y_m),
                fixed_milli(snapshot.odom.yaw_rad),
                (int)snapshot.pwm_left, (int)snapshot.pwm_right);
            if (length > 0 && length < (int)sizeof(report)) uart_try_write(report, (uint16_t)length);
            report_start = snapshot.tick;
        }
    }
}
