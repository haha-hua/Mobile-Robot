#ifndef CONFIG_H
#define CONFIG_H

/* 实测轮径65毫米，编码器只统计第一路上升沿。 */
#define WHEEL_CIRCUMFERENCE_UM 204204L
#define LEFT_COUNTS_10_REVS 4926L
#define RIGHT_COUNTS_10_REVS 4910L
#define LEFT_ENCODER_SIGN 1
#define RIGHT_ENCODER_SIGN (-1)

#define SPEED_SAMPLE_MS 20u
/* 前方双路三针红外避障模块，OUT必须为3.3V兼容电平(低电平有效)：
 * PA1 = 右侧避障传感器，PA2 = 左侧避障传感器，均配置为输入上拉(IPU)；
 * PA1/PA2默认复用为TIM2_CH2/CH3，但TIM2仅作时基无输出，不冲突。 */
#define IR_PORT GPIOA
#define IR_GPIO_CLK RCC_APB2Periph_GPIOA
#define IR_RIGHT_PIN GPIO_Pin_1
#define IR_LEFT_PIN GPIO_Pin_2
#define IR_PIN IR_RIGHT_PIN
#ifndef IR_ACTIVE_LEVEL
#define IR_ACTIVE_LEVEL 0u
#endif
/* 静态测试开关：置0禁用电机输出并拒绝运动命令，仅采样传感器；
 * 置1启用上电自动执行四色地图往返避障任务，也可通过串口选择其他运动模式。 */
#ifndef MOTION_OUTPUT_ENABLE
#define MOTION_OUTPUT_ENABLE 1
#endif
/* 红外传感器专项自检开关：置1启用PA1上下拉断线检测、ADC2引脚电压测量、
 * RGB指示灯实时反馈及串口事件诊断；正常跑车保持为0。 */
#ifndef IR_DIAG_ENABLE
#define IR_DIAG_ENABLE 0
#endif
#define IR_CLEAR_SAMPLES 3u
#define LIGHT_DARK_ACTIVE_LEVEL 0u
#define LIGHT_STATE_CONFIRM_SAMPLES 3u
#define LIGHT_ADC_VREF_MV 3300u
#define LIGHT_ADC_MAX_VALUE 4095u
#define LIGHT_ADC_AVG_SAMPLES 8u
/* 光敏电阻暗恢复较慢：OFF阶段等待120ms回稳，各LED阶段按红->蓝->绿等待80ms后采样40ms（单帧520ms）。 */
#define COLOR_OFF_SETTLE_SAMPLES 6u
#define COLOR_SETTLE_SAMPLES 4u
#define COLOR_MEASURE_SAMPLES 2u
/* 当前光敏模块实测为光增强时AO下降；若换成相反输出的模块改为0。 */
#define COLOR_ADC_INVERTED 1u
/* 三路原始ADC反射总变化量下限：低于此值说明探头悬空或被完全遮死，判为unknown。 */
#define COLOR_MIN_RAW_SUM 15u
#define COLOR_MIN_WHITE_CHANNEL 3u
#define COLOR_STABLE_FRAMES 2u

/* 实测硬件默认增益：由当前小车白地板多帧实测比例直接固化（R:G:B ≈ 3.22:1.00:2.39） */
#define COLOR_DEFAULT_GAIN_RED_MILLI   3220u
#define COLOR_DEFAULT_GAIN_GREEN_MILLI 1000u
#define COLOR_DEFAULT_GAIN_BLUE_MILLI  2390u

/* 实测四色默认归一化中心点 (norm_r, norm_g, norm_b，三者和=1000)：
 * 直接由当前小车白地板与三块泡沫垫串口实测数据均值固化，烧录即用，无需串口标定。 */
#define COLOR_WHITE_DEFAULT_R  334u
#define COLOR_WHITE_DEFAULT_G  334u
#define COLOR_WHITE_DEFAULT_B  332u
#define COLOR_YELLOW_DEFAULT_R 426u
#define COLOR_YELLOW_DEFAULT_G 414u
#define COLOR_YELLOW_DEFAULT_B 160u
#define COLOR_RED_DEFAULT_R    580u
#define COLOR_RED_DEFAULT_G    152u
#define COLOR_RED_DEFAULT_B    268u
#define COLOR_BLUE_DEFAULT_R   137u
#define COLOR_BLUE_DEFAULT_G   339u
#define COLOR_BLUE_DEFAULT_B   524u
#define AVOID_FORWARD_MM_S 400L
#define AVOID_TURN_MRAD_S 900L
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
#define MAX_WHEEL_SPEED_MM_S 400L
#define MAX_LINEAR_SPEED_MM_S 400L
#define MAX_ANGULAR_SPEED_MRAD_S 2000L
#define WHEEL_ACCEL_MM_S2 950L
#define REVERSE_SAFE_SPEED_MM_S 100L
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
#define AUTO_ARC_SPEED_MM_S 240L
#define AUTO_DRIVE_MAX_MM_S 360L
#define AUTO_DRIVE_MIN_MM_S 30L
#define AUTO_TURN_MAX_MM_S 90L
#define AUTO_TURN_MIN_MM_S 15L
#define AUTO_STOP_SPEED_MM_S 10L
#define AUTO_END_MM 2L

/* 光电压返航：每2秒计算一个AO电压均值，窗口波动受限才接纳。 */
#define LIGHT_RETURN_TRIGGER_MV 150
#define LIGHT_RETURN_STABLE_MV 250u
#define LIGHT_RETURN_STABLE_MS 2000u
#define LIGHT_RETURN_SPEED_MM_S 240L
#define LIGHT_RETURN_TURN_MRAD_S 900L
#define LIGHT_RETURN_MIN_TURN_MRAD_S 200L
#define LIGHT_RETURN_YAW_TOL_MRAD 30L
#define LIGHT_RETURN_POSITION_TOL_MM 15L

/* 四色地图全局位姿往返避障任务参数（场地：红/蓝/左右白边均为32x120cm，中间黄区120x120cm；车长18cm宽20cm，圆瓶直径8cm）：
 * 双红外(PA1右/PA2左)前视18cm，遇圆形水瓶不停车、边前进边快速大角度绕行，越过水瓶即刻快速回正；
 * 双侧白边自适应：去程左白边右转回、右白边左转回；返程掉头180度后方向相反，回到红区中心停车。 */
#define MISSION_BOOT_MS              1500u
#define MISSION_MANEUVER_TIMEOUT_MS  20000u
#define MISSION_TOTAL_TIMEOUT_MS     180000u
#define MISSION_CRUISE_MM_S          160L
#define MISSION_AVOID_TURN_MM_S      138L
#define MISSION_BYPASS_MM_S          160L
#define MISSION_REALIGN_MM_S         140L
#define MISSION_WHITE_ALIGN_MM_S     95L
#define MISSION_WHITE_RETURN_MM_S    145L
#define MISSION_APPROACH_MM_S        115L
#define MISSION_TURN_MAX_MRAD_S      1800L
#define MISSION_TURN_MIN_MRAD_S      450L
#define MISSION_TRACK_MAX_MRAD_S     950L
#define MISSION_YAW_TOL_MRAD         50L
#define MISSION_TURN180_TOL_MRAD     35L
#define MISSION_AVOID_CLEAR_MS       60u
#define MISSION_OBS_OFFSET_MM        265L
#define MISSION_OBS_LAT_OFFSET_MM    60L
#define MISSION_OBS_PASS_MARGIN_MM   210L
#define MISSION_BYPASS_MIN_MM        215L
#define MISSION_BYPASS_MAX_MM        275L
#define MISSION_WHITE_PASS_MIN_MM    220L
#define MISSION_WHITE_PASS_MAX_MM    520L
#define MISSION_WHITE_RETURN_MAX_MM  450L
#define MISSION_BLUE_PULL_IN_MM      115L
#define MISSION_RED_PULL_IN_MM       120L

/* 参数以千分之一表示；积分增益按秒定义，20毫秒步长等于原来的20。 */
#define PWM_MAX 1000
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
