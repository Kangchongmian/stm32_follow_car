/*
 * ble.c - JDY-33 蓝牙通信协议 (USART1)
 *
 * 接收微信小程序的参数修改和摇杆控制指令
 * 帧格式: 0xAA + CMD + LEN + DATA + XOR校验
 */
#include "ble.h"
#include "motor.h"

/* ========== 全局变量 ========== */
BLE_RxCtx_t              g_ble_rx;
volatile int8_t          g_ble_joy_x       = 0;
volatile int8_t          g_ble_joy_y       = 0;
volatile bool            g_ble_manual_active = false;
volatile uint32_t        g_ble_joy_tick    = 0;

/* ========== 调试变量 (Keil Watch窗口监控) ========== */
volatile uint32_t dbg_irq_cnt     = 0;  /* 中断触发总次数 */
volatile uint8_t  dbg_last_byte   = 0;  /* 最后收到的原始字节 */
volatile uint8_t  dbg_state       = 0;  /* 当前状态机状态 */
volatile uint8_t  dbg_frame_cmd   = 0;  /* 最后一帧的CMD */
volatile uint8_t  dbg_frame_len   = 0;  /* 最后一帧的LEN */
volatile uint8_t  dbg_cs_calc     = 0;  /* 计算的校验值 */
volatile uint8_t  dbg_cs_recv     = 0;  /* 收到的校验值 */
volatile uint32_t dbg_frame_ok    = 0;  /* 校验通过的帧计数 */
volatile uint32_t dbg_frame_fail  = 0;  /* 校验失败的帧计数 */
volatile int8_t   dbg_joy_x       = 0;  /* 最后解析的摇杆X */
volatile int8_t   dbg_joy_y       = 0;  /* 最后解析的摇杆Y */

/* ========== 内部函数 ========== */

/* 发送一个字节到 USART1 */
static void usart1_send_byte(uint8_t byte)
{
    while (!(huart1.Instance->SR & USART_SR_TXE)) {}
    huart1.Instance->DR = byte;
}

/* 发送一帧 */
static void ble_send_frame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t cs = BLE_FRAME_HEAD ^ cmd ^ len;
    usart1_send_byte(BLE_FRAME_HEAD);
    usart1_send_byte(cmd);
    usart1_send_byte(len);
    for (uint8_t i = 0; i < len; i++) {
        usart1_send_byte(data[i]);
        cs ^= data[i];
    }
    usart1_send_byte(cs);
}

/* 根据索引设置参数 */
static void set_param(uint8_t index, float value)
{
    if (index >= BLE_PARAM_COUNT) return;
    float *params = (float *)&g_car_params;
    params[index] = value;
}

/* 根据索引读取参数 */
static float get_param(uint8_t index)
{
    if (index >= BLE_PARAM_COUNT) return 0.0f;
    float *params = (float *)&g_car_params;
    return params[index];
}

/* ========== 公开接口 ========== */

void BLE_Init(void)
{
    memset(&g_ble_rx, 0, sizeof(g_ble_rx));
    g_ble_rx.state = BLE_RX_WAIT_HEAD;
    g_ble_joy_x = 0;
    g_ble_joy_y = 0;
    g_ble_manual_active = false;
    g_ble_joy_tick = 0;
}

void BLE_SendParamAck(uint8_t paramIndex, float value)
{
    uint8_t data[5];
    data[0] = paramIndex;
    memcpy(&data[1], &value, 4);
    ble_send_frame(BLE_CMD_PARAM_ACK, data, 5);
}

/* USART1 接收中断 — 逐字节状态机 */
void BLE_UART_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        uint8_t byte = (uint8_t)(huart1.Instance->DR & 0xFF);

        dbg_irq_cnt++;
        dbg_last_byte = byte;
        dbg_state = (uint8_t)g_ble_rx.state;

        /* 如果上一帧还没处理完，丢弃新数据等待处理 */
        if (g_ble_rx.ready) return;

        switch (g_ble_rx.state) {
        case BLE_RX_WAIT_HEAD:
            if (byte == BLE_FRAME_HEAD) {
                g_ble_rx.state = BLE_RX_WAIT_CMD;
            }
            break;

        case BLE_RX_WAIT_CMD:
            g_ble_rx.cmd = byte;
            dbg_frame_cmd = byte;
            g_ble_rx.state = BLE_RX_WAIT_LEN;
            break;

        case BLE_RX_WAIT_LEN:
            g_ble_rx.len = byte;
            g_ble_rx.idx = 0;
            dbg_frame_len = byte;
            if (byte == 0 || byte > BLE_RX_BUF_SIZE) {
                g_ble_rx.state = BLE_RX_WAIT_HEAD; /* 非法长度 */
            } else {
                g_ble_rx.state = BLE_RX_WAIT_DATA;
            }
            break;

        case BLE_RX_WAIT_DATA:
            g_ble_rx.buf[g_ble_rx.idx++] = byte;
            if (g_ble_rx.idx >= g_ble_rx.len) {
                g_ble_rx.state = BLE_RX_WAIT_CS;
            }
            break;

        case BLE_RX_WAIT_CS: {
            /* XOR 校验 */
            uint8_t cs = BLE_FRAME_HEAD ^ g_ble_rx.cmd ^ g_ble_rx.len;
            for (uint8_t i = 0; i < g_ble_rx.len; i++) {
                cs ^= g_ble_rx.buf[i];
            }
            dbg_cs_calc = cs;
            dbg_cs_recv = byte;
            if (cs == byte) {
                g_ble_rx.ready = 1;
                dbg_frame_ok++;
            } else {
                dbg_frame_fail++;
            }
            g_ble_rx.state = BLE_RX_WAIT_HEAD;
            break;
        }
        }
    }

    /* 清除溢出错误，防止卡死 */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
}

/* 主循环调用 — 处理完整帧 */
void BLE_Poll(void)
{
    if (!g_ble_rx.ready) {
        /* 手动控制超时检查 */
        if (g_ble_manual_active &&
            (HAL_GetTick() - g_ble_joy_tick > BLE_MANUAL_TIMEOUT_MS)) {
            g_ble_joy_x = 0;
            g_ble_joy_y = 0;
            g_ble_manual_active = false;
        }
        return;
    }

    /* 处理帧 */
    switch (g_ble_rx.cmd) {
    case BLE_CMD_SET_PARAM:
        if (g_ble_rx.len >= 5) {
            uint8_t idx = g_ble_rx.buf[0];
            float val;
            memcpy(&val, &g_ble_rx.buf[1], 4);
            set_param(idx, val);
            /* 回传确认 */
            BLE_SendParamAck(idx, val);
        }
        break;

    case BLE_CMD_JOYSTICK:
        if (g_ble_rx.len >= 2) {
            g_ble_joy_x = (int8_t)g_ble_rx.buf[0];
            g_ble_joy_y = (int8_t)g_ble_rx.buf[1];
            dbg_joy_x = g_ble_joy_x;
            dbg_joy_y = g_ble_joy_y;
            /* 限幅 */
            if (g_ble_joy_x > 100)  g_ble_joy_x = 100;
            if (g_ble_joy_x < -100) g_ble_joy_x = -100;
            if (g_ble_joy_y > 100)  g_ble_joy_y = 100;
            if (g_ble_joy_y < -100) g_ble_joy_y = -100;
            g_ble_joy_tick = HAL_GetTick();
            g_ble_manual_active = (g_ble_joy_x != 0 || g_ble_joy_y != 0);
        }
        break;

    case BLE_CMD_READ_PARAM:
        if (g_ble_rx.len >= 1) {
            uint8_t idx = g_ble_rx.buf[0];
            float val = get_param(idx);
            BLE_SendParamAck(idx, val);
        }
        break;

    default:
        break;
    }

    g_ble_rx.ready = 0;
}
