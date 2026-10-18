#ifndef CONFIG_H
#define CONFIG_H

/* 实测轮径65毫米，编码器只统计第一路上升沿。 */
#define WHEEL_CIRCUMFERENCE_UM 204204L
#define LEFT_COUNTS_10_REVS 4926L
#define RIGHT_COUNTS_10_REVS 4910L
#define LEFT_ENCODER_SIGN 1
#define RIGHT_ENCODER_SIGN (-1)

#define SPEED_SAMPLE_MS 20u
/* 正前方单个三针红外模块，OUT必须为3.3V兼容电平。PA1输入上拉；
 * PA1默认复用为TIM2_CH2，但TIM2仅作时基无输出，不冲突。 */
#define IR_PORT GPIOA
#define IR_GPIO_CLK RCC_APB2Periph_GPIOA
#define IR_PIN GPIO_Pin_1
#ifndef IR_ACTIVE_LEVEL
#define IR_ACTIVE_LEVEL 0u
#endif
/* 静态测试开关：置0禁用电机输出并拒绝运动命令，仅保留红外采样；
 * 调完电位器后改回1恢复避障和实验二。 */
#ifndef MOTION_OUTPUT_ENABLE
#define MOTION_OUTPUT_ENABLE 1
#endif
#define IR_CLEAR_SAMPLES 3u
#define AVOID_FORWARD_MM_S 80L
#define AVOID_TURN_MRAD_S 600L
#define AVOID_STOP_SPEED_MM_S 10L
#define AVOID_STABLE_MS 300u
#define AVOID_TURN_MIN_MRAD 1571L
#define AVOID_TURN_CLEAR_MS 300u
#define AVOID_TURN_TIMEOUT_MS 8000u
/* avoid任务总里程：平均轮程达到该值后停车完成。 */
#define AVOID_TOTAL_MM 5000L

/* 物理测量轮距保留；正逆解及里程计统一使用有效轮距。 */
#define PHYSICAL_WHEEL_TRACK_MM 180L
#define WHEEL_TRACK_MM 152L
#define MAX_WHEEL_SPEED_MM_S 200L
#define MAX_LINEAR_SPEED_MM_S 200L
#define MAX_ANGULAR_SPEED_MRAD_S 2000L
#define WHEEL_ACCEL_MM_S2 200L
#define REVERSE_SAFE_SPEED_MM_S 50L
#define COMMAND_TIMEOUT_MS 1000u

/* 实验二分段流程：直线->停->原地左转90度->顺时针整圈圆弧->原地左转90度->返回。
 * 出发0.5m到圆周最左点，圆心(0.7,0)，半径0.2m；整圈后回到起点，再对准原点返回。 */
#define AUTO_BOOT_MS 2000u
#define AUTO_STAGE_TIMEOUT_MS 60000u
#define AUTO_TOTAL_TIMEOUT_MS 180000u
#define AUTO_STABLE_MS 300u
#define AUTO_SETTLE_MS 800u
#define AUTO_STRAIGHT_MM 500L
#define AUTO_RADIUS_MM 200L
#define AUTO_ARC_SWEEP_MDEG 360000L
#define AUTO_ARC_SPEED_MM_S 120L
#define AUTO_DRIVE_MAX_MM_S 180L
#define AUTO_DRIVE_MIN_MM_S 15L
#define AUTO_TURN_MAX_MM_S 90L
#define AUTO_TURN_MIN_MM_S 15L
#define AUTO_STOP_SPEED_MM_S 10L
#define AUTO_END_MM 2L

/* 参数以千分之一表示；积分增益按秒定义，20毫秒步长等于原来的20。 */
#define PWM_MAX 500
#define PID_KP_MILLI 500
#define PID_KI_MILLI 1000
#define PID_KD_MILLI 0
#define PID_INTEGRAL_LIMIT_MILLI 200000L
#define PWM_FEEDFORWARD 300

/* 调试串口保持USART1、PA9/PA10、115200。 */
#define UART_USART USART1
#define UART_USART_CLK RCC_APB2Periph_USART1
#define UART_PORT GPIOA
#define UART_GPIO_CLK RCC_APB2Periph_GPIOA
#define UART_TX_PIN GPIO_Pin_9
#define UART_RX_PIN GPIO_Pin_10
#define UART_BAUDRATE 115200u

#endif
