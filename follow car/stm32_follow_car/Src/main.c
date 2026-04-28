/*
 * main.c - STM32F407VGT6 UWB跟随小车主程序
 *
 * 功能概述:
 *   基于UWB测距的跟随小车, 后轮差速驱动, 前轮从动轮
 *   通过超声波(RS485) + 激光雷达(LD06)实现避障
 *   避障方式: 差速转弯绕行, 不停车
 *   通过JDY-31蓝牙模块接收微信小程序的参数修改和方向控制
 *
 * 硬件连接 (参考接线表v3):
 *   PB7/PB6   - USART1       - JDY-31蓝牙模块 9600bps
 *   PE9/PE11  - TIM1 CH1/CH2 - 左后轮 BTS7960 (R_PWM反转/L_PWM正转)
 *   PE13/PE14 - TIM1 CH3/CH4 - 右后轮 BTS7960 (R_PWM反转/L_PWM正转)
 *   PD5/PD6   - USART2       - UWB主站 115200bps
 *   PD8/PD9   - USART3       - RS485超声波传感器 9600bps
 *   PA0/PA1   - UART4        - LD06激光雷达 230400bps
 *   PB8/PB9   - I2C1         - 0.96寸OLED (SSD1306)
 *   PC13      - GPIO INPUT   - 急停按钮 (断开=急停)
 *   PE3       - GPIO INPUT   - 模式开关 (接通=跟随模式)
 *   PE4       - GPIO INPUT   - 上电反馈
 *   PD4       - GPIO OUTPUT  - 蓝色LED
 *   PD7       - GPIO OUTPUT  - 红色LED
 *   PE0       - GPIO OUTPUT  - 绿色LED
 *   PE1       - GPIO OUTPUT  - 黄色LED
 */

#include "main.h"
#include "bsp.h"
#include "motor.h"
#include "uwb.h"
#include "ultrasonic.h"
#include "lidar.h"
#include "oled.h"
#include "follow.h"
#include "ble.h"

/* ========== 全局可调参数 (所有模块共用) ========== */
CarParams_t g_car_params;

static void CarParams_InitDefaults(void)
{
    g_car_params.follow_distance_m      = DEFAULT_FOLLOW_DISTANCE_M;
    g_car_params.obstacle_dist_cm       = DEFAULT_OBSTACLE_DIST_CM;
    g_car_params.lidar_obstacle_dist_mm = DEFAULT_LIDAR_OBSTACLE_DIST_MM;
    g_car_params.motor_base_speed       = DEFAULT_MOTOR_BASE_SPEED;
    g_car_params.motor_turn_speed       = DEFAULT_MOTOR_TURN_SPEED;
    g_car_params.motor_slow_speed       = DEFAULT_MOTOR_SLOW_SPEED;
    g_car_params.uwb_timeout_ms         = DEFAULT_UWB_TIMEOUT_MS;
    g_car_params.ultrasonic_poll_ms     = DEFAULT_ULTRASONIC_POLL_MS;
    g_car_params.uwb_angle_tolerance_deg = DEFAULT_UWB_ANGLE_TOLERANCE_DEG;
    g_car_params.emergency_stop_dist_cm  = DEFAULT_EMERGENCY_STOP_DIST_CM;
    g_car_params.pid_dist_kp             = DEFAULT_PID_DIST_KP;
    g_car_params.pid_dist_ki             = DEFAULT_PID_DIST_KI;
    g_car_params.pid_dist_kd             = DEFAULT_PID_DIST_KD;
    g_car_params.pid_angle_kp            = DEFAULT_PID_ANGLE_KP;
    g_car_params.pid_angle_ki            = DEFAULT_PID_ANGLE_KI;
    g_car_params.pid_angle_kd            = DEFAULT_PID_ANGLE_KD;
}

/* ========== 主函数 ========== */
int main(void)
{
    /* HAL初始化 */
    HAL_Init();

    /* 配置系统时钟 168MHz */
    BSP_ClockConfig();

    /* 参数初始化 (在外设之前, 某些模块初始化时会读参数) */
    CarParams_InitDefaults();

    /* 外设初始化 */
    BSP_GPIO_Init();
    BSP_TIM1_Init();
    BSP_USART1_Init();     /* JDY-33蓝牙 */
    BSP_USART2_Init();
    BSP_USART3_Init();
    BSP_UART4_Init();
    BSP_I2C1_Init();

    /* 模块初始化 */
    Motor_Init();
    UWB_Init();
    Ultrasonic_Init();
    Lidar_Init();
    OLED_Init();
    Follow_Init();
    BLE_Init();

    /* 开机显示 */
    OLED_ShowString(0, 0, "UWB Follow Car");
    OLED_ShowString(0, 2, "BLE Ready");
    LED_Green(true);
    HAL_Delay(1000);
    OLED_Clear();
    LED_Green(false);

    /* 等待上电反馈 */
    while (!IsPowerOn()) {
        LED_Red(true);
        HAL_Delay(200);
        LED_Red(false);
        HAL_Delay(200);
    }
    LED_Red(false);

    /* ========== 主循环 ========== */
    while (1)
    {
        /* 蓝牙指令处理 (最先处理, 保证参数及时生效) */
        BLE_Poll();

        /* 轮询各传感器模块 */
        UWB_Poll();
        Ultrasonic_Poll();
        Lidar_Poll();

        /* 跟随和避障决策 (内部会检查蓝牙手动控制) */
        Follow_Update();

        /* 转向灯闪烁更新 */
        TurnSignal_Tick();

        /* 适当延时, 降低CPU占用, 控制主循环频率约50Hz */
        HAL_Delay(20);
    }
}

/* ========== 中断服务函数 ========== */

void USART1_IRQHandler(void)
{
    BLE_UART_IRQHandler();
}

void USART2_IRQHandler(void)
{
    UWB_UART_IRQHandler();
}

void USART3_IRQHandler(void)
{
    Ultrasonic_UART_IRQHandler();
}

void UART4_IRQHandler(void)
{
    Lidar_UART_IRQHandler();
}

/* ========== SysTick (HAL需要) ========== */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ========== 错误处理 ========== */
void Error_Handler(void)
{
    __disable_irq();
    /* 红灯常亮表示错误 */
    LED_AllOff();
    LED_Red(true);
    while (1) {}
}

/* ========== HAL断言回调 (调试用) ========== */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
