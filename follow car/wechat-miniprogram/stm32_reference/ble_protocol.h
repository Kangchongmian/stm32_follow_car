/**
 * ============================================================
 *  uwb_car 蓝牙通信协议 — STM32端参考代码 (USART1 接 JDY-31)
 * ============================================================
 *
 *  帧格式:  0xAA + CMD(1B) + LEN(1B) + DATA(nB) + XOR_CS(1B)
 *
 *  CMD:
 *    0x01  SET_PARAM    — 设置参数  DATA = index(1B) + float_LE(4B)
 *    0x02  DIRECTION    — 方向控制  DATA = dir(1B)  0停 1前 2后 3左 4右
 *    0x03  READ_PARAM   — 读取参数  DATA = index(1B)
 *    0x04  PARAM_ACK    — 参数应答  DATA = index(1B) + float_LE(4B)
 *
 *  参数索引:
 *    0  FOLLOW_DISTANCE_M
 *    1  OBSTACLE_DIST_CM
 *    2  LIDAR_OBSTACLE_DIST_MM
 *    3  MOTOR_BASE_SPEED
 *    4  MOTOR_TURN_SPEED
 *    5  MOTOR_SLOW_SPEED
 *    6  UWB_TIMEOUT_MS
 *    7  ULTRASONIC_POLL_MS
 *
 *  将以下代码集成到 uwb_car 工程中:
 *    1. 在 USART1 中断中调用 BLE_ReceiveByte()
 *    2. 在主循环中调用 BLE_ProcessCommand()
 *    3. 参数已声明为全局变量，替换原有 #define 宏
 * ============================================================
 */

#ifndef __BLE_PROTOCOL_H
#define __BLE_PROTOCOL_H

#include <stdint.h>
#include <string.h>

/* ---- 全局可调参数（替代原 #define） ---- */
typedef struct {
    float follow_distance_m;        /* 默认 1.0   */
    float obstacle_dist_cm;         /* 默认 30    */
    float lidar_obstacle_dist_mm;   /* 默认 300   */
    float motor_base_speed;         /* 默认 500   */
    float motor_turn_speed;         /* 默认 700   */
    float motor_slow_speed;         /* 默认 200   */
    float uwb_timeout_ms;           /* 默认 500   */
    float ultrasonic_poll_ms;       /* 默认 50    */
} CarParams_t;

extern CarParams_t g_car_params;

/* ---- 方向指令 ---- */
typedef enum {
    DIR_STOP    = 0,
    DIR_FORWARD = 1,
    DIR_BACK    = 2,
    DIR_LEFT    = 3,
    DIR_RIGHT   = 4
} Direction_e;

extern volatile Direction_e g_ble_direction;

/* ---- 协议常量 ---- */
#define BLE_FRAME_HEAD      0xAA
#define BLE_CMD_SET_PARAM   0x01
#define BLE_CMD_DIRECTION   0x02
#define BLE_CMD_READ_PARAM  0x03
#define BLE_CMD_PARAM_ACK   0x04

#define BLE_RX_BUF_SIZE     64

/* ---- 接收状态机 ---- */
typedef enum {
    RX_WAIT_HEAD,
    RX_WAIT_CMD,
    RX_WAIT_LEN,
    RX_WAIT_DATA,
    RX_WAIT_CS
} RxState_e;

typedef struct {
    RxState_e state;
    uint8_t   cmd;
    uint8_t   len;
    uint8_t   idx;
    uint8_t   buf[BLE_RX_BUF_SIZE];
    uint8_t   ready;   /* 1 = 一帧完整可处理 */
} BLE_RxCtx_t;

extern BLE_RxCtx_t g_ble_rx;

/* ==== 函数实现 ==== */

static inline void BLE_Init(void)
{
    g_car_params.follow_distance_m      = 1.0f;
    g_car_params.obstacle_dist_cm       = 30.0f;
    g_car_params.lidar_obstacle_dist_mm = 300.0f;
    g_car_params.motor_base_speed       = 500.0f;
    g_car_params.motor_turn_speed       = 700.0f;
    g_car_params.motor_slow_speed       = 200.0f;
    g_car_params.uwb_timeout_ms         = 500.0f;
    g_car_params.ultrasonic_poll_ms     = 50.0f;

    g_ble_direction = DIR_STOP;
    memset(&g_ble_rx, 0, sizeof(g_ble_rx));
    g_ble_rx.state = RX_WAIT_HEAD;
}

/**
 * 在 USART1 接收中断中调用，逐字节喂入
 */
static inline void BLE_ReceiveByte(uint8_t byte)
{
    switch (g_ble_rx.state) {
    case RX_WAIT_HEAD:
        if (byte == BLE_FRAME_HEAD) {
            g_ble_rx.state = RX_WAIT_CMD;
        }
        break;
    case RX_WAIT_CMD:
        g_ble_rx.cmd = byte;
        g_ble_rx.state = RX_WAIT_LEN;
        break;
    case RX_WAIT_LEN:
        g_ble_rx.len = byte;
        g_ble_rx.idx = 0;
        if (byte == 0 || byte > BLE_RX_BUF_SIZE) {
            g_ble_rx.state = RX_WAIT_HEAD; /* 非法长度，重置 */
        } else {
            g_ble_rx.state = RX_WAIT_DATA;
        }
        break;
    case RX_WAIT_DATA:
        g_ble_rx.buf[g_ble_rx.idx++] = byte;
        if (g_ble_rx.idx >= g_ble_rx.len) {
            g_ble_rx.state = RX_WAIT_CS;
        }
        break;
    case RX_WAIT_CS: {
        /* 校验: XOR(HEAD, CMD, LEN, DATA[0..n-1]) */
        uint8_t cs = BLE_FRAME_HEAD ^ g_ble_rx.cmd ^ g_ble_rx.len;
        for (uint8_t i = 0; i < g_ble_rx.len; i++) {
            cs ^= g_ble_rx.buf[i];
        }
        if (cs == byte) {
            g_ble_rx.ready = 1;
        }
        g_ble_rx.state = RX_WAIT_HEAD;
        break;
    }
    }
}

/**
 * 通过 USART1 发送一帧（需用户实现 USART1_SendByte）
 */
extern void USART1_SendByte(uint8_t byte);

static inline void BLE_SendFrame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t cs = BLE_FRAME_HEAD ^ cmd ^ len;
    USART1_SendByte(BLE_FRAME_HEAD);
    USART1_SendByte(cmd);
    USART1_SendByte(len);
    for (uint8_t i = 0; i < len; i++) {
        USART1_SendByte(data[i]);
        cs ^= data[i];
    }
    USART1_SendByte(cs);
}

/**
 * 发送参数应答
 */
static inline void BLE_SendParamAck(uint8_t paramIndex, float value)
{
    uint8_t data[5];
    data[0] = paramIndex;
    memcpy(&data[1], &value, 4);
    BLE_SendFrame(BLE_CMD_PARAM_ACK, data, 5);
}

/**
 * 设置参数（根据索引）
 */
static inline void BLE_SetParam(uint8_t index, float value)
{
    float *params = (float *)&g_car_params;
    if (index < 8) {
        params[index] = value;
    }
}

/**
 * 读取参数（根据索引）
 */
static inline float BLE_GetParam(uint8_t index)
{
    float *params = (float *)&g_car_params;
    if (index < 8) {
        return params[index];
    }
    return 0.0f;
}

/**
 * 主循环中调用，处理已接收的完整帧
 */
static inline void BLE_ProcessCommand(void)
{
    if (!g_ble_rx.ready) return;
    g_ble_rx.ready = 0;

    switch (g_ble_rx.cmd) {
    case BLE_CMD_SET_PARAM:
        if (g_ble_rx.len >= 5) {
            uint8_t idx = g_ble_rx.buf[0];
            float val;
            memcpy(&val, &g_ble_rx.buf[1], 4);
            BLE_SetParam(idx, val);
            /* 回传确认 */
            BLE_SendParamAck(idx, val);
        }
        break;

    case BLE_CMD_DIRECTION:
        if (g_ble_rx.len >= 1) {
            uint8_t dir = g_ble_rx.buf[0];
            if (dir <= DIR_RIGHT) {
                g_ble_direction = (Direction_e)dir;
            }
        }
        break;

    case BLE_CMD_READ_PARAM:
        if (g_ble_rx.len >= 1) {
            uint8_t idx = g_ble_rx.buf[0];
            float val = BLE_GetParam(idx);
            BLE_SendParamAck(idx, val);
        }
        break;

    default:
        break;
    }
}

#endif /* __BLE_PROTOCOL_H */
