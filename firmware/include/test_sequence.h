#ifndef TEST_SEQUENCE_H
#define TEST_SEQUENCE_H
#include "motion.h"

typedef enum {
    TEST_BOOT_WAIT, TEST_FORWARD, TEST_WAIT_FORWARD, TEST_TURN_360,
    TEST_WAIT_360, TEST_TURN_90, TEST_WAIT_90, TEST_DONE,
    TEST_CANCELLED, TEST_TIMEOUT
} test_stage_t;

typedef struct {
    test_stage_t stage;
    uint32_t stage_tick, stable_ms;
    float left_mm, right_mm;
} test_sequence_t;

void test_sequence_init(test_sequence_t *test, uint32_t tick);
void test_sequence_cancel(test_sequence_t *test);
void test_sequence_step(test_sequence_t *test, motion_t *motion, uint32_t tick,
                        int32_t delta_left, int32_t delta_right,
                        int32_t speed_left, int32_t speed_right);
float test_sequence_distance_mm(const test_sequence_t *test);
float test_sequence_angle_rad(const test_sequence_t *test);
const char *test_sequence_stage_name(test_stage_t stage);
#endif
