#include "test_sequence.h"
#include "config.h"
#include <math.h>

#define TEST_BOOT_MS 2000u
#define TEST_STAGE_TIMEOUT_MS 20000u
#define TEST_STABLE_MS 200u
#define TEST_DWELL_MS 1000u
#define TEST_STOP_SPEED_MM_S 5.0f
#define TEST_FORWARD_MM 1000.0f
#define TEST_PI 3.14159265358979323846f
#define TEST_FORWARD_SPEED_MM_S 120
#define TEST_TURN_SPEED_MM_S 60
#define TEST_MIN_SPEED_MM_S 15

static uint8_t active(test_stage_t stage)
{
    return stage <= TEST_WAIT_90;
}

void test_sequence_init(test_sequence_t *test, uint32_t tick)
{
    *test = (test_sequence_t){0};
    test->stage_tick = tick;
}

void test_sequence_cancel(test_sequence_t *test)
{
    if (active(test->stage)) test->stage = TEST_CANCELLED;
}

float test_sequence_distance_mm(const test_sequence_t *test)
{
    return (test->left_mm + test->right_mm) * 0.5f;
}

float test_sequence_angle_rad(const test_sequence_t *test)
{
    return (test->right_mm - test->left_mm) / (float)WHEEL_TRACK_MM;
}

static void start_stage(test_sequence_t *test, test_stage_t stage, uint32_t tick)
{
    test->stage = stage;
    test->stage_tick = tick;
    test->stable_ms = 0u;
    test->left_mm = test->right_mm = 0.0f;
}

static float ticks_to_mm(int32_t ticks, int32_t counts)
{
    return (float)ticks * (float)WHEEL_CIRCUMFERENCE_UM * 0.01f / (float)counts;
}

void test_sequence_step(test_sequence_t *test, motion_t *motion, uint32_t tick,
                        int32_t delta_left, int32_t delta_right,
                        int32_t speed_left, int32_t speed_right)
{
    float remaining;
    int32_t speed, maximum;
    uint8_t forward;
    if (!active(test->stage)) return;
    if (test->stage != TEST_BOOT_WAIT) {
        test->left_mm += ticks_to_mm(delta_left, LEFT_COUNTS_10_REVS);
        test->right_mm += ticks_to_mm(delta_right, RIGHT_COUNTS_10_REVS);
    }
    if ((uint32_t)(tick - test->stage_tick) >= TEST_STAGE_TIMEOUT_MS) {
        motion_stop(motion);
        test->stage = TEST_TIMEOUT;
        return;
    }
    if (test->stage == TEST_BOOT_WAIT) {
        if ((uint32_t)(tick - test->stage_tick) < TEST_BOOT_MS) return;
        start_stage(test, TEST_FORWARD, tick);
    }
    if (test->stage == TEST_WAIT_FORWARD || test->stage == TEST_WAIT_360 ||
        test->stage == TEST_WAIT_90) {
        if (fabsf(ticks_to_mm(speed_left, LEFT_COUNTS_10_REVS)) <= TEST_STOP_SPEED_MM_S &&
            fabsf(ticks_to_mm(speed_right, RIGHT_COUNTS_10_REVS)) <= TEST_STOP_SPEED_MM_S) {
            test->stable_ms += SPEED_SAMPLE_MS;
        } else test->stable_ms = 0u;
        if (test->stable_ms < TEST_STABLE_MS + TEST_DWELL_MS) return;
        if (test->stage == TEST_WAIT_90) {
            test->stage = TEST_DONE;
            return;
        }
        start_stage(test, test->stage == TEST_WAIT_FORWARD ? TEST_TURN_360 : TEST_TURN_90, tick);
    }
    forward = test->stage == TEST_FORWARD;
    remaining = forward ? TEST_FORWARD_MM - test_sequence_distance_mm(test) :
        ((test->stage == TEST_TURN_360 ? 2.0f * TEST_PI : 0.5f * TEST_PI) -
         test_sequence_angle_rad(test)) * (float)WHEEL_TRACK_MM * 0.5f;
    /* 末段降到15mm/s；剩余2mm轮程时撤销驱动，留出采样及滑行余量。 */
    if (remaining <= 2.0f) {
        motion_stop(motion);
        test->stage = forward ? TEST_WAIT_FORWARD :
                      test->stage == TEST_TURN_360 ? TEST_WAIT_360 : TEST_WAIT_90;
        test->stage_tick = tick;
        test->stable_ms = 0u;
        return;
    }
    maximum = forward ? TEST_FORWARD_SPEED_MM_S : TEST_TURN_SPEED_MM_S;
    /* 以剩余轮程限制速度，比原有200mm/s²斜坡更早减速。 */
    speed = (int32_t)(remaining * 2.0f);
    if (speed > maximum) speed = maximum;
    if (speed < TEST_MIN_SPEED_MM_S) speed = TEST_MIN_SPEED_MM_S;
    motion_set_wheel_speeds(motion, forward ? speed : -speed, speed, tick);
}

const char *test_sequence_stage_name(test_stage_t stage)
{
    static const char *names[] = {
        "boot", "forward", "wait_forward", "turn360", "wait360",
        "turn90", "wait90", "done", "cancelled", "timeout"
    };
    return names[stage];
}
