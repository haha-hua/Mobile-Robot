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

static const char *mode_name(control_mode_t mode)
{
    static const char *names[] = {"sensor", "manual", "avoid", "exp2"};
    return names[mode];
}

static long fixed_milli(float value)
{
    return (long)(value * 1000.0f + (value < 0.0f ? -0.5f : 0.5f));
}

int main(void)
{
    uint32_t report_start = 0u;
    uint8_t auto_start_pending = MOTION_OUTPUT_ENABLE;
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
            "READY PA1 active %s (3.3V OUT)\r\n"
#if MOTION_OUTPUT_ENABLE
            "AUTO: avoid starts in %ums; stop cancels\r\n"
#else
            "STATIC TEST: motors disabled\r\n"
#endif
            "avoid / sensor / experiment2 / stop / vel v w / wheel L R"
#if !MOTION_OUTPUT_ENABLE
            " (motion commands rejected in static test)"
#endif
            "\r\n"
            "units: mm/s,mrad/s; log T/A=wheel mm/s O=x,y,yaw(mm,mrad)\r\n"
            "avoid: forward %ldmm total, stop/turn on obstacle, done stops\r\n"
            "track=%ld timeout=%ums; experiment2 waits %ums\r\n",
            IR_ACTIVE_LEVEL == 0u ? "low" : "high",
            (unsigned)AUTO_BOOT_MS,
            (long)AVOID_TOTAL_MM,
            (long)WHEEL_TRACK_MM, COMMAND_TIMEOUT_MS,
            (unsigned)AUTO_BOOT_MS);
        if (length > 0 && length < (int)sizeof(report)) uart_try_write(report, (uint16_t)length);
    }

    while (1) {
        int8_t result;
        if (reply) {
            if (uart_try_write(reply, (uint16_t)strlen(reply))) reply = 0;
        } else if ((result = uart_read_line(line)) != 0) {
            if (result < 0) reply = "ERR line discarded (length/RX fault)\r\n";
            else {
                uint8_t accepted = control_command(line);
                if (accepted) auto_start_pending = 0u;
                reply = accepted ? "OK\r\n" : "ERR syntax/range\r\n";
            }
        }
        if (auto_start_pending && control_tick() >= AUTO_BOOT_MS) {
            auto_start_pending = 0u;
            control_start_avoid();
        }
        if (!reply && (uint32_t)(control_tick() - report_start) >= 200u) {
            int length;
            control_snapshot(&snapshot);
            length = snprintf(report, sizeof(report),
                "t=%lu m=%s s=%s ir=%u obs=%u av=%s run=%ld seq=%s rel=%ld,%ld "
                "T=%ld,%ld A=%ld,%ld v=%ld w=%ld O=%ld,%ld,%ld P=%d,%d\r\n",
                (unsigned long)snapshot.tick, mode_name(snapshot.mode),
                state_name(snapshot.state), (unsigned)snapshot.ir_raw,
                (unsigned)snapshot.ir_blocked, obstacle_stage_name(snapshot.avoid_stage),
                (long)snapshot.avoid_run_mm,
                auto_sequence_stage_name(snapshot.seq.stage),
                (long)snapshot.seq.rel_mm, (long)snapshot.seq.rel_mrad,
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
