#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ============ 可调参数结构体 (通过蓝牙可修改) ============ */
typedef struct {
    float follow_distance_m;        /* 跟随设定距离 (米)       默认 1.0  */
    float obstacle_dist_cm;         /* 避障触发距离 (厘米)     默认 30   */
    float lidar_obstacle_dist_mm;   /* 雷达避障距离 (毫米)     默认 300  */
    float motor_base_speed;         /* 基准电机速度 0-999      默认 500  */
    float motor_turn_speed;         /* 转弯外侧轮速度          默认 700  */
    float motor_slow_speed;         /* 转弯内侧轮速度          默认 200  */
    float uwb_timeout_ms;           /* UWB数据超时 (ms)        默认 500  */
    float ultrasonic_poll_ms;       /* 超声波轮询间隔 (ms)     默认 50   */
    float uwb_angle_tolerance_deg;  /* UWB角度容许值 (度)      默认 20   */
    float emergency_stop_dist_cm;   /* 紧急停车距离 (厘米)     默认 15   */
    float pid_dist_kp;              /* 距离PID Kp             默认 300  */
    float pid_dist_ki;              /* 距离PID Ki             默认 5    */
    float pid_dist_kd;              /* 距离PID Kd             默认 80   */
    float pid_angle_kp;             /* 角度PID Kp             默认 4    */
    float pid_angle_ki;             /* 角度PID Ki             默认 0.3  */
    float pid_angle_kd;             /* 角度PID Kd             默认 1.5  */
} CarParams_t;

extern CarParams_t g_car_params;

/* 参数默认值 (用于初始化和复位) */
#define DEFAULT_FOLLOW_DISTANCE_M       1.0f
#define DEFAULT_OBSTACLE_DIST_CM        30.0f
#define DEFAULT_LIDAR_OBSTACLE_DIST_MM  300.0f
#define DEFAULT_MOTOR_BASE_SPEED        500.0f
#define DEFAULT_MOTOR_TURN_SPEED        700.0f
#define DEFAULT_MOTOR_SLOW_SPEED        200.0f
#define DEFAULT_UWB_TIMEOUT_MS          500.0f
#define DEFAULT_ULTRASONIC_POLL_MS      50.0f
#define DEFAULT_UWB_ANGLE_TOLERANCE_DEG 20.0f
#define DEFAULT_EMERGENCY_STOP_DIST_CM  15.0f

/* PID 默认增益 */
#define DEFAULT_PID_DIST_KP     300.0f   /* 距离PID比例 */
#define DEFAULT_PID_DIST_KI     5.0f     /* 距离PID积分 */
#define DEFAULT_PID_DIST_KD     80.0f    /* 距离PID微分 */
#define DEFAULT_PID_ANGLE_KP    4.0f     /* 角度PID比例 */
#define DEFAULT_PID_ANGLE_KI    0.3f     /* 角度PID积分 */
#define DEFAULT_PID_ANGLE_KD    1.5f     /* 角度PID微分 */

/* ============ LED引脚 ============ */
#define LED_BLUE_PIN            GPIO_PIN_4
#define LED_BLUE_PORT           GPIOD
#define LED_RED_PIN             GPIO_PIN_7
#define LED_RED_PORT            GPIOD
#define LED_GREEN_PIN           GPIO_PIN_0
#define LED_GREEN_PORT          GPIOE
#define LED_YELLOW_PIN          GPIO_PIN_1
#define LED_YELLOW_PORT         GPIOE

/* ============ 转向灯引脚 ============ */
#define TURN_LEFT_PIN           GPIO_PIN_0
#define TURN_LEFT_PORT          GPIOD
#define TURN_RIGHT_PIN          GPIO_PIN_1
#define TURN_RIGHT_PORT         GPIOD

/* ============ 输入引脚 ============ */
#define ESTOP_PIN               GPIO_PIN_13
#define ESTOP_PORT              GPIOC
#define MODE_SW_PIN             GPIO_PIN_3
#define MODE_SW_PORT            GPIOE
#define POWER_FB_PIN            GPIO_PIN_4
#define POWER_FB_PORT           GPIOE

/* ============ 全局句柄 ============ */
extern TIM_HandleTypeDef  htim1;
extern UART_HandleTypeDef huart1;   /* JDY-31蓝牙  */
extern UART_HandleTypeDef huart2;   /* UWB    */
extern UART_HandleTypeDef huart3;   /* RS485超声波 */
extern UART_HandleTypeDef huart4;   /* LD06雷达   */
extern I2C_HandleTypeDef  hi2c1;    /* OLED   */

void Error_Handler(void);

#endif /* __MAIN_H */
