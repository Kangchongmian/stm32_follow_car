/*
 * uwb.c - GC-P2304-GS-2 UWB 主站通信 (USART2, PD5/PD6, 115200bps)
 *
 * 协议: 地址模式 (Address Mode)
 * 帧格式 (共10字节):
 *   [0xF0][len][id_L][id_H][dist_L][dist_H][angle_L][angle_H][RSSI][0xAA]
 *    帧头  1    2     3     4       5       6        7       8     9
 * len=6: 后面有6字节数据 (id+dist+angle+RSSI)
 * dist: uint16, 小端, 单位 cm
 * angle: int16, 小端, 单位 度 (正=左, 负=右)
 * RSSI: uint8, dBm = RSSI - 256
 * buf[9] 固定为 0xAA 作为帧尾
 */
#include "uwb.h"
#include <stdlib.h>

UWB_Data_t uwb_data;

static uint8_t uwb_rx_byte;

void UWB_Init(void)
{
    memset(&uwb_data, 0, sizeof(uwb_data));
    uwb_data.distance_m = 0.0f;
    uwb_data.valid = false;
    /* 启动接收中断 */
    HAL_UART_Receive_IT(&huart2, &uwb_rx_byte, 1);
}

/*
 * GC-P2304-GS-2 地址模式协议解析
 * 帧长固定 10 字节
 */
static bool parse_binary_frame(uint8_t *buf, uint16_t len)
{
    /* 固定帧长 10 字节 */
    if (len < 10) return false;

    /* 帧头 0xF0 */
    if (buf[0] != 0xF0) return false;

    /* len 固定为 6 */
    if (buf[1] != 0x06) return false;

    /* XOR 校验: 0xF0 ^ len ^ id_L ^ id_H ^ dist_L ^ dist_H ^ angle_L ^ angle_H ^ RSSI = CS */
    //uint8_t cs = 0xF0 ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ^ buf[6] ^ buf[7] ^ buf[8];
    //if (cs != buf[9]) return false;

    /* 帧尾 0xAA */
    if (buf[9] != 0xAA) return false;

    /* 距离: buf[4..5] 小端 uint16, 单位 cm */
    uint16_t dist_cm = (uint16_t)(buf[4] | (buf[5] << 8));
    if (dist_cm == 0 || dist_cm > 5000) return false; /* 过滤无效值, 最大50m */

    /* 角度: buf[6..7] 小端 int16, 单位 度 (正=左, 负=右) */
    int16_t angle = (int16_t)(buf[6] | (buf[7] << 8));

    /* 距离转米 */
    float dist_m = (float)dist_cm / 100.0f;

    uwb_data.distance_m = dist_m;
    uwb_data.angle_deg  = angle;
    uwb_data.last_update_tick = HAL_GetTick();
    uwb_data.valid = true;

    return true;
}

/* UART接收中断回调 */
void UWB_UART_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);

        if (uwb_data.rx_idx < UWB_RX_BUF_SIZE - 1) {
            uwb_data.rx_buf[uwb_data.rx_idx++] = ch;
        }

        /* 固定帧长 10 字节, 检测到帧尾再解析 */
        if (uwb_data.rx_idx >= 10) {
            if (parse_binary_frame(uwb_data.rx_buf, uwb_data.rx_idx)) {
                /* 解析成功, 数据已在 parse_binary_frame 中更新到 uwb_data */
            }
            uwb_data.rx_idx = 0;
        }

        /* 防止缓冲区溢出: 如果收到非 0xF0 开头的垃圾数据则清空 */
        if (uwb_data.rx_idx == 1 && uwb_data.rx_buf[0] != 0xF0) {
            uwb_data.rx_idx = 0;
        }

        if (uwb_data.rx_idx >= UWB_RX_BUF_SIZE - 1) {
            uwb_data.rx_idx = 0;
        }
    }
}

void UWB_Poll(void)
{
    /* 检查UWB数据超时 */
    if (uwb_data.valid &&
        (HAL_GetTick() - uwb_data.last_update_tick > (uint32_t)g_car_params.uwb_timeout_ms)) {
        uwb_data.valid = false;
    }
}
