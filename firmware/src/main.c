#include "bsp_uart.h"
#include "config.h"
#include "control.h"
#include "encoder.h"
#include "motor.h"
#include "rgb_led.h"
#include <stdio.h>
#include <string.h>

static const char *state_name(motion_state_t state)
{
    static const char *names[] = {"idle", "run", "timeout"};
    return names[state];
}

int main(void)
{
    uint32_t report_start = 0u;
    uint32_t status_start = 0u;
    uint32_t last_color_frame = 0u;
    uint8_t auto_start_pending = MOTION_OUTPUT_ENABLE;
    char report[512];
    char line[UART_LINE_CAPACITY];
    const char *reply = 0;
    control_snapshot_t snapshot;

    motor_init();
    encoder_init();
    rgb_led_init();
    uart_init();
    control_init();
    {
        int length = snprintf(report, sizeof(report),
            "READY UART=115200 8N1; LIGHT AO=PA4 DO=PD11; "
            "RGB TEST PE9/PE11/PE13; MOTORS DISABLED; "
            "commands: color_calibrate / color_reset / stop\r\n");
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
        if (auto_start_pending) {
            auto_start_pending = 0u;
            control_start_light_return();
        }
        if (!reply && (uint32_t)(control_tick() - report_start) >= 100u) {
            int length = 0;
            control_snapshot(&snapshot);
            report_start = snapshot.tick;
            if (snapshot.color.frame_count != last_color_frame) {
                length = snprintf(report, sizeof(report),
                    "COLOR n=%lu t=%lu ambient=%u raw=%u,%u,%u "
                    "signal=%lu,%lu,%lu norm=%u,%u,%u sum=%lu "
                    "label=%s candidate=%s stable=%u\r\n",
                    (unsigned long)snapshot.color.frame_count,
                    (unsigned long)snapshot.tick,
                    (unsigned)snapshot.color.ambient,
                    (unsigned)snapshot.color.raw_red,
                    (unsigned)snapshot.color.raw_green,
                    (unsigned)snapshot.color.raw_blue,
                    (unsigned long)snapshot.color.signal_red,
                    (unsigned long)snapshot.color.signal_green,
                    (unsigned long)snapshot.color.signal_blue,
                    (unsigned)snapshot.color.norm_red,
                    (unsigned)snapshot.color.norm_green,
                    (unsigned)snapshot.color.norm_blue,
                    (unsigned long)snapshot.color.signal_sum,
                    color_sensor_label_name(snapshot.color.label),
                    color_sensor_label_name(snapshot.color.candidate),
                    (unsigned)snapshot.color.stable_frames);
                if (length > 0 && length < (int)sizeof(report) &&
                    uart_try_write(report, (uint16_t)length))
                    last_color_frame = snapshot.color.frame_count;
            } else if ((uint32_t)(snapshot.tick - status_start) >= 1000u) {
                length = snprintf(report, sizeof(report),
                    "STATUS t=%lu s=%s T=%ld,%ld A=%ld,%ld P=%d,%d "
                    "light_adc=%u light_mv=%u light_do=%u light_dark=%u\r\n",
                    (unsigned long)snapshot.tick, state_name(snapshot.state),
                    (long)snapshot.target_left, (long)snapshot.target_right,
                    (long)motion_pulses_to_mm(snapshot.speed_left, LEFT_COUNTS_10_REVS),
                    (long)motion_pulses_to_mm(snapshot.speed_right, RIGHT_COUNTS_10_REVS),
                    (int)snapshot.pwm_left, (int)snapshot.pwm_right,
                    (unsigned)snapshot.light_adc, (unsigned)snapshot.light_mv,
                    (unsigned)snapshot.light_do, (unsigned)snapshot.light_dark);
                if (length > 0 && length < (int)sizeof(report) &&
                    uart_try_write(report, (uint16_t)length))
                    status_start = snapshot.tick;
            }
        }
    }
}
