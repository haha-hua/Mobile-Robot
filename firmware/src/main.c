#include "bsp_uart.h"
#include "config.h"
#include "control.h"
#include "encoder.h"
#include "motor.h"
#include "rgb_led.h"
#include <stdio.h>
#include <string.h>

#if !IR_DIAG_ENABLE
static const char *state_name(motion_state_t state)
{
    static const char *names[] = {"idle", "run", "timeout"};
    return names[state];
}
#else
static const char *ir_state_label(const obstacle_t *ir)
{
    if (ir->floating) return "FLOATING";
    return (ir->raw_level == IR_ACTIVE_LEVEL) ? "OBSTACLE" : "CLEAR   ";
}
#endif

int main(void)
{
    uint32_t report_start = 0u;
    uint32_t status_start = 0u;
#if IR_DIAG_ENABLE
    uint8_t prev_ir_raw = 0xFFu;
    uint8_t prev_ir_floating = 0xFFu;
#else
    uint32_t last_color_frame = 0u;
#endif
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
#if IR_DIAG_ENABLE
            "READY UART=115200 8N1; IR_DIAG ON PA1 (0=OBSTACLE, 1=CLEAR); MOTORS DISABLED\r\n"
            "LED: GREEN=clear, RED=obstacle, BLUE_BLINK=PA1_floating; commands: reset | stop\r\n"
#else
            "READY UART=115200 8N1; IR_R=PA1 IR_L=PA2; LIGHT AO=PA4 DO=PD11; "
            "RGB PE9/PE11/PE13; AUTO 4-COLOR MISSION(red->yellow->blue->yellow->red); "
            "commands: mission | stop | cal white | cal yellow | cal red | cal blue | color_reset\r\n"
#endif
            );
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
            control_start_mission();
        }
#if IR_DIAG_ENABLE
        if (!reply && (uint32_t)(control_tick() - report_start) >= 20u) {
            int length = 0;
            control_snapshot(&snapshot);
            report_start = snapshot.tick;
            if (snapshot.ir.raw_level != prev_ir_raw ||
                snapshot.ir.floating != prev_ir_floating) {
                length = snprintf(report, sizeof(report),
                    "[EVENT t=%lums] state=%s raw=%u blk=%u mv=%u ipu=%u ipd=%u "
                    "edges=%lu trig=%lu => %s\r\n",
                    (unsigned long)snapshot.tick,
                    ir_state_label(&snapshot.ir),
                    (unsigned)snapshot.ir.raw_level,
                    (unsigned)snapshot.ir.blocked,
                    (unsigned)snapshot.ir.millivolts,
                    (unsigned)snapshot.ir.ipu_level,
                    (unsigned)snapshot.ir.ipd_level,
                    (unsigned long)snapshot.ir.edge_count,
                    (unsigned long)snapshot.ir.obstacle_count,
                    obstacle_diag_summary(&snapshot.ir));
                if (length > 0 && length < (int)sizeof(report) &&
                    uart_try_write(report, (uint16_t)length)) {
                    prev_ir_raw = snapshot.ir.raw_level;
                    prev_ir_floating = snapshot.ir.floating;
                    status_start = snapshot.tick;
                }
            } else if ((uint32_t)(snapshot.tick - status_start) >= 400u) {
                length = snprintf(report, sizeof(report),
                    "[IR t=%lums] state=%s raw=%u blk=%u mv=%u(min=%u,max=%u) "
                    "ipu=%u ipd=%u edges=%lu trig=%lu low/high=%lu/%lums => %s\r\n",
                    (unsigned long)snapshot.tick,
                    ir_state_label(&snapshot.ir),
                    (unsigned)snapshot.ir.raw_level,
                    (unsigned)snapshot.ir.blocked,
                    (unsigned)snapshot.ir.millivolts,
                    (unsigned)snapshot.ir.min_mv,
                    (unsigned)snapshot.ir.max_mv,
                    (unsigned)snapshot.ir.ipu_level,
                    (unsigned)snapshot.ir.ipd_level,
                    (unsigned long)snapshot.ir.edge_count,
                    (unsigned long)snapshot.ir.obstacle_count,
                    (unsigned long)snapshot.ir.low_ms,
                    (unsigned long)snapshot.ir.high_ms,
                    obstacle_diag_summary(&snapshot.ir));
                if (length > 0 && length < (int)sizeof(report) &&
                    uart_try_write(report, (uint16_t)length))
                    status_start = snapshot.tick;
            }
        }
#else
        if (!reply && (uint32_t)(control_tick() - report_start) >= 100u) {
            int length = 0;
            control_snapshot(&snapshot);
            report_start = snapshot.tick;
            if (snapshot.color.frame_count != last_color_frame) {
                length = snprintf(report, sizeof(report),
                    "COLOR n=%lu t=%lu ambient=%u raw=%u,%u,%u raw_sum=%u "
                    "signal=%lu,%lu,%lu norm=%u,%u,%u sum=%lu dist=%lu cal=0x%02x "
                    "label=%s candidate=%s stable=%u\r\n",
                    (unsigned long)snapshot.color.frame_count,
                    (unsigned long)snapshot.tick,
                    (unsigned)snapshot.color.ambient,
                    (unsigned)snapshot.color.raw_red,
                    (unsigned)snapshot.color.raw_green,
                    (unsigned)snapshot.color.raw_blue,
                    (unsigned)snapshot.color.raw_sum,
                    (unsigned long)snapshot.color.signal_red,
                    (unsigned long)snapshot.color.signal_green,
                    (unsigned long)snapshot.color.signal_blue,
                    (unsigned)snapshot.color.norm_red,
                    (unsigned)snapshot.color.norm_green,
                    (unsigned)snapshot.color.norm_blue,
                    (unsigned long)snapshot.color.signal_sum,
                    (unsigned long)snapshot.color.min_dist_sq,
                    (unsigned)snapshot.color.calibrated_mask,
                    color_sensor_label_name(snapshot.color.label),
                    color_sensor_label_name(snapshot.color.candidate),
                    (unsigned)snapshot.color.stable_frames);
                if (length > 0 && length < (int)sizeof(report) &&
                    uart_try_write(report, (uint16_t)length))
                    last_color_frame = snapshot.color.frame_count;
            } else if ((uint32_t)(snapshot.tick - status_start) >= 1000u) {
                length = snprintf(report, sizeof(report),
                    "STATUS t=%lu s=%s leg=%s stage=%s col=%s ir=%u(L=%u,R=%u) T=%ld,%ld A=%ld,%ld "
                    "x=%ld y=%ld yaw=%ld gyaw=%ld obs=%u\r\n",
                    (unsigned long)snapshot.tick, state_name(snapshot.state),
                    color_mission_leg_name(snapshot.mission.leg),
                    color_mission_stage_name(snapshot.mission.stage),
                    color_sensor_label_name(snapshot.color.label),
                    (unsigned)snapshot.ir_blocked,
                    (unsigned)snapshot.ir.left_blocked,
                    (unsigned)snapshot.ir.right_blocked,
                    (long)snapshot.target_left, (long)snapshot.target_right,
                    (long)motion_pulses_to_mm(snapshot.speed_left, LEFT_COUNTS_10_REVS),
                    (long)motion_pulses_to_mm(snapshot.speed_right, RIGHT_COUNTS_10_REVS),
                    (long)(snapshot.odom.x_m * 1000.0f),
                    (long)(snapshot.odom.y_m * 1000.0f),
                    (long)(snapshot.odom.yaw_rad * 57.2957795f),
                    (long)(snapshot.mission.goal_yaw * 57.2957795f),
                    (unsigned)snapshot.mission.obs_count);
                if (length > 0 && length < (int)sizeof(report) &&
                    uart_try_write(report, (uint16_t)length))
                    status_start = snapshot.tick;
            }
        }
#endif
    }
}
