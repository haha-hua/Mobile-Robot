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

int main(void)
{
    uint32_t report_start = 0u;
    uint8_t auto_start_pending = MOTION_OUTPUT_ENABLE;
    char report[512];
    char line[UART_LINE_CAPACITY];
    const char *reply = 0;
    control_snapshot_t snapshot;

    motor_init();
    encoder_init();
    uart_init();
    control_init();
    {
        int length = snprintf(report, sizeof(report),
            "READY UART=115200 8N1; LIGHT AO=PA4 DO=PD11; "
            "commands: stop / vel v w / wheel L R\r\n");
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
        if (!reply && (uint32_t)(control_tick() - report_start) >= 200u) {
            int length;
            control_snapshot(&snapshot);
            length = snprintf(report, sizeof(report),
                "t=%lu s=%s T=%ld,%ld A=%ld,%ld P=%d,%d "
                "light_adc=%u light_mv=%u light_do=%u light_dark=%u "
                "lr=%s base=%u stable=%u trig=%u\r\n",
                (unsigned long)snapshot.tick, state_name(snapshot.state),
                (long)snapshot.target_left, (long)snapshot.target_right,
                (long)motion_pulses_to_mm(snapshot.speed_left, LEFT_COUNTS_10_REVS),
                (long)motion_pulses_to_mm(snapshot.speed_right, RIGHT_COUNTS_10_REVS),
                (int)snapshot.pwm_left, (int)snapshot.pwm_right,
                (unsigned)snapshot.light_adc, (unsigned)snapshot.light_mv,
                (unsigned)snapshot.light_do, (unsigned)snapshot.light_dark,
                light_return_stage_name(snapshot.light_return.stage),
                (unsigned)snapshot.light_return.baseline_mv,
                (unsigned)snapshot.light_return.stable_mv,
                (unsigned)snapshot.light_return.triggered);
            if (length > 0 && length < (int)sizeof(report)) uart_try_write(report, (uint16_t)length);
            report_start = snapshot.tick;
        }
    }
}
