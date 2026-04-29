#ifndef __BLE_H
#define __BLE_H

#include "main.h"

/*
 * BLE 蓝牙通信协议 — USART1 接 JDY-33 双模蓝牙模块
 *
 * 帧格式:  HEAD(0xAA) + CMD(1B) + LEN(1B) + DATA(nB) + XOR_CS(1B)
 *
 * CMD:
 *   0x01  SET_PARAM    设置参数   DATA = paramIndex(1B) + float_LE(4B)
 *   0x02  JOYSTICK     摇杆控制   DATA = x(int8) + y(int8)
 *         x: 左负右正 -100~+100, y: 后负前正 -100~+100, (0,0)=停止
 *   0x03  READ_PARAM   读取参数   DATA = paramIndex(1B)
 *   0x04  PARAM_ACK    参数应答   DATA = paramIndex(1B) + float_LE(4B)
 *
 * 参数索引:
 *   0  follow_distance_m
 *   1  obstacle_dist_cm
 *   2  lidar_obstacle_dist_mm
 *   3  motor_base_speed
 *   4  motor_turn_speed
 *   5  motor_slow_speed
 *   6  uwb_timeout_ms
 *   7  ultrasonic_poll_ms
 *   8  uwb_angle_tolerance_deg
 *   9  emergency_stop_dist_cm
 *   10 pid_dist_kp
 *   11 pid_dist_ki
 *   12 pid_dist_kd
 *   13 pid_angle_kp
 *   14 pid_angle_ki
 *   15 pid_angle_kd
 *   16 max_follow_speed
 */

#define BLE_FRAME_HEAD      0xAA
#define BLE_CMD_SET_PARAM   0x01
#define BLE_CMD_JOYSTICK    0x02
#define BLE_CMD_READ_PARAM  0x03
#define BLE_CMD_PARAM_ACK   0x04

#define BLE_PARAM_COUNT     17

#define BLE_RX_BUF_SIZE     64

/* 接收状态机 */
typedef enum {
    BLE_RX_WAIT_HEAD,
    BLE_RX_WAIT_CMD,
    BLE_RX_WAIT_LEN,
    BLE_RX_WAIT_DATA,
    BLE_RX_WAIT_CS
} BLE_RxState_e;

typedef struct {
    BLE_RxState_e state;
    uint8_t   cmd;
    uint8_t   len;
    uint8_t   idx;
    uint8_t   buf[BLE_RX_BUF_SIZE];
    volatile uint8_t ready;   /* 1 = 一帧完整可处理 */
} BLE_RxCtx_t;

extern BLE_RxCtx_t          g_ble_rx;

/* 摇杆 X/Y 值 (-100 ~ +100) */
extern volatile int8_t      g_ble_joy_x;       /* 左负右正 */
extern volatile int8_t      g_ble_joy_y;       /* 后负前正 */
extern volatile bool        g_ble_manual_active;  /* 蓝牙手动控制激活 */
extern volatile uint32_t    g_ble_joy_tick;    /* 最后收到摇杆指令的时间 */

#define BLE_MANUAL_TIMEOUT_MS  500  /* 手动控制超时(松手后自动停) */

void BLE_Init(void);
void BLE_Poll(void);               /* 主循环调用, 处理完整帧 */
void BLE_UART_IRQHandler(void);    /* USART1 中断中调用 */
void BLE_SendParamAck(uint8_t paramIndex, float value);

#endif /* __BLE_H */
