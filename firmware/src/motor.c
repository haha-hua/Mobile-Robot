#include "motor.h"
#include "bsp_pwm.h"
#include "config.h"

/* 软件限幅:第一次上电别让电机全速,跑通后再把 MOTOR_DUTY_MAX 放开 */
static uint16_t limit_duty(uint16_t duty)
{
    if (duty > MOTOR_DUTY_MAX) {
        return MOTOR_DUTY_MAX;
    }
    return duty;
}

void motor_init(void)
{
    pwm_init();
}

void motor_set(motor_id_t id, uint16_t duty)
{
    duty = limit_duty(duty);

    switch (id) {
    case MOTOR_LEFT:
        pwm_set_duty(PWM_CH1, duty);
        break;
    case MOTOR_RIGHT:
        pwm_set_duty(PWM_CH2, duty);
        break;
    }
}

void motor_stop(void)
{
    motor_set(MOTOR_LEFT, 0);
    motor_set(MOTOR_RIGHT, 0);
}
