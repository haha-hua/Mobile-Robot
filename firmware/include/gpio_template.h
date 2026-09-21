/*
 * gpio_template.h —— GPIO 配置「套路」速查模板
 * ================================================================
 * 注意:这【不是】一个要被 #include 的头文件,只是一份给自己看的参考笔记!
 *       它的位置在 include/ 下,不会参与编译。
 *       写 GPIO 时,从这里复制代码到 src/main.c,改掉引脚号即可。
 * ================================================================
 *
 * 【万能套路】配置任何 STM32 外设都是同样这 4 步(GPIO / USART / TIM / SPI / ADC ...):
 *   ① 开时钟   RCC_xxxPeriphClockCmd(...)   ← 最容易忘!不开时钟 = 白配
 *   ② 填结构体  xxx_InitTypeDef 变量,把参数写进去
 *   ③ 调 Init   xxx_Init(...)                把配置真正写进硬件
 *   ④ 使用     Set/Reset、Read/Write、Send/Receive ...
 *
 * 【只需要记住 2 个"概念",其余 API 名字随用随查,不用背】
 *   - 别忘记开时钟
 *   - 引脚是"输出"(主动驱动外部) 还是 "输入"(读外部电平)?
 */


/* ============ 模板 1:GPIO 输出(例如点灯) ============ */
static void led_init(void)
{
    GPIO_InitTypeDef gpio;

    /* ① 开时钟:端口是哪个就开哪个 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    /* ② 填结构体 */
    gpio.GPIO_Pin   = GPIO_Pin_7;         /* 引脚号:GPIO_Pin_0 ~ GPIO_Pin_15 */
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;   /* 推挽输出 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;   /* 2MHz / 10MHz / 50MHz(输入模式下此项不影响功能) */

    /* ③ 把配置写进硬件 */
    GPIO_Init(GPIOD, &gpio);

    /* ④ 使用:三种设置电平的方式 */
    GPIO_SetBits(GPIOD, GPIO_Pin_7);            /* 置高电平 */
    GPIO_ResetBits(GPIOD, GPIO_Pin_7);          /* 置低电平 */
    GPIO_WriteBit(GPIOD, GPIO_Pin_7, Bit_SET);  /* 按变量写:Bit_SET / Bit_RESET */
}


/* ============ 模板 2:GPIO 输入(例如读按键) ============ */
static void key_init(void)
{
    GPIO_InitTypeDef gpio;

    /* ① 开时钟(改成实际端口) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* ② 填结构体 */
    gpio.GPIO_Pin   = GPIO_Pin_0;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;      /* 上拉输入:按键一端接 GND,按下读到 0 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;   /* 输入下无效,但习惯上仍填一个值 */

    /* ③ 把配置写进硬件 */
    GPIO_Init(GPIOA, &gpio);

    /* ④ 使用:读引脚电平 */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0) {
        /* 按键被按下 */
    }
}


/* ============ 速查表 ============
 *
 * 端口       : GPIOA / GPIOB / GPIOC / GPIOD / GPIOE
 * 开时钟     : RCC_APB2Periph_GPIOA ... RCC_APB2Periph_GPIOE
 * 引脚号     : GPIO_Pin_0 ... GPIO_Pin_15,GPIO_Pin_All
 *
 * 常用模式(GPIOMode_TypeDef):
 *   GPIO_Mode_Out_PP       推挽输出   —— 点灯、驱动普通模块
 *   GPIO_Mode_Out_OD       开漏输出   —— 需要外部上拉 / 总线共享(如软件 I2C)
 *   GPIO_Mode_IN_FLOATING  浮空输入   —— 外部电路已有上下拉
 *   GPIO_Mode_IPU          上拉输入   —— 按键接 GND(按下 = 低电平)
 *   GPIO_Mode_IPD          下拉输入   —— 按键接 VCC(按下 = 高电平)
 *   GPIO_Mode_AF_PP        复用推挽   —— 给 USART / SPI / TIM 等外设当引脚
 *   GPIO_Mode_AF_OD        复用开漏   —— 同上,开漏形式(如 I2C)
 *   GPIO_Mode_AIN          模拟输入   —— 给 ADC 用
 *
 * 电平操作:
 *   GPIO_SetBits(x, pin)            置高
 *   GPIO_ResetBits(x, pin)          置低
 *   GPIO_WriteBit(x, pin, Bit_SET)  按值写
 *   GPIO_ReadInputDataBit(x, pin)   读输入电平
 *
 * 忘了某个参数能填什么?把光标放到名字上按 F12"跳转到定义",
 * 头文件(lib/spl/include/stm32f10x_gpio.h)的注释里就写了所有可选值。
 */
