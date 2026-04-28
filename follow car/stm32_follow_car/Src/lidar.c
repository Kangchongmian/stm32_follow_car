/*
 * lidar.c - LD06激光雷达驱动 (UART4, PA0/PA1, 230400bps)
 *
 * LD06数据包格式 (47字节):
 *   [0x54][PointNum][Speed_L][Speed_H][StartAngle_L][StartAngle_H]
 *   [12组 x (Dist_L, Dist_H, Confidence)]
 *   [EndAngle_L][EndAngle_H][Timestamp_L][Timestamp_H][CRC8]
 *
 * 每包12个测量点, 距离单位mm, 角度单位0.01度
 * 只提取前方扇区(约350°~10°)的最小距离用于避障
 *
 * 改进:
 *   - 增加置信度过滤 (低于阈值的点不采用)
 *   - 增加距离合理性检查 (过大过小的值丢弃)
 *   - 中断中增加溢出/帧错误清除
 *   - PointNum字段校验 (LD06固定为12)
 *   - 角度合理性校验 (start/end不超过36000)
 *   - 缓冲区写入加保护, 防止中断和主循环竞争
 *   - 连续CRC失败计数, 超限重置缓冲区
 */
#include "lidar.h"

Lidar_Data_t lidar_data;

#define LD06_PKT_SIZE       47
#define LD06_HEADER         0x54
#define LD06_POINT_NUM      0x2C    /* LD06 PointNum字段固定值 (12点, 0x2C) */
#define LIDAR_CONF_MIN      150     /* 最低置信度阈值 (0-255) */
#define LIDAR_DIST_MIN_MM   10      /* 最小有效距离 (mm), 过近视为噪声 */
#define LIDAR_DIST_MAX_MM   12000   /* 最大有效距离 (mm), LD06标称12m */
#define LIDAR_CRC_FAIL_MAX  10      /* 连续CRC失败上限, 超过则重置缓冲区 */
#define LIDAR_DATA_TIMEOUT  500     /* 数据超时 (ms), 比原来300更宽松 */
#define LIDAR_RESET_PERIOD  200     /* 周期性重置最小值的间隔 (ms) */

/* CRC8 查表 (用于LD06校验) */
static const uint8_t crc_table[256] = {
    0x00,0x4D,0x9A,0xD7,0x79,0x34,0xE3,0xAE,0xF2,0xBF,0x68,0x25,0x8B,0xC6,0x11,0x5C,
    0xA9,0xE4,0x33,0x7E,0xD0,0x9D,0x4A,0x07,0x5B,0x16,0xC1,0x8C,0x22,0x6F,0xB8,0xF5,
    0x1F,0x52,0x85,0xC8,0x66,0x2B,0xFC,0xB1,0xED,0xA0,0x77,0x3A,0x94,0xD9,0x0E,0x43,
    0xB6,0xFB,0x2C,0x61,0xCF,0x82,0x55,0x18,0x44,0x09,0xDE,0x93,0x3D,0x70,0xA7,0xEA,
    0x3E,0x73,0xA4,0xE9,0x47,0x0A,0xDD,0x90,0xCC,0x81,0x56,0x1B,0xB5,0xF8,0x2F,0x62,
    0x97,0xDA,0x0D,0x40,0xEE,0xA3,0x74,0x39,0x65,0x28,0xFF,0xB2,0x1C,0x51,0x86,0xCB,
    0x21,0x6C,0xBB,0xF6,0x58,0x15,0xC2,0x8F,0xD3,0x9E,0x49,0x04,0xAA,0xE7,0x30,0x7D,
    0x88,0xC5,0x12,0x5F,0xF1,0xBC,0x6B,0x26,0x7A,0x37,0xE0,0xAD,0x03,0x4E,0x99,0xD4,
    0x7C,0x31,0xE6,0xAB,0x05,0x48,0x9F,0xD2,0x8E,0xC3,0x14,0x59,0xF7,0xBA,0x6D,0x20,
    0xD5,0x98,0x4F,0x02,0xAC,0xE1,0x36,0x7B,0x27,0x6A,0xBD,0xF0,0x5E,0x13,0xC4,0x89,
    0x63,0x2E,0xF9,0xB4,0x1A,0x57,0x80,0xCD,0x91,0xDC,0x0B,0x46,0xE8,0xA5,0x72,0x3F,
    0xCA,0x87,0x50,0x1D,0xB3,0xFE,0x29,0x64,0x38,0x75,0xA2,0xEF,0x41,0x0C,0xDB,0x96,
    0x42,0x0F,0xD8,0x95,0x3B,0x76,0xA1,0xEC,0xB0,0xFD,0x2A,0x67,0xC9,0x84,0x53,0x1E,
    0xEB,0xA6,0x71,0x3C,0x92,0xDF,0x08,0x45,0x19,0x54,0x83,0xCE,0x60,0x2D,0xFA,0xB7,
    0x5D,0x10,0xC7,0x8A,0x24,0x69,0xBE,0xF3,0xAF,0xE2,0x35,0x78,0xD6,0x9B,0x4C,0x01,
    0xF4,0xB9,0x6E,0x23,0x8D,0xC0,0x17,0x5A,0x06,0x4B,0x9C,0xD1,0x7F,0x32,0xE5,0xA8
};

static uint8_t calc_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
        crc = crc_table[crc ^ data[i]];
    return crc;
}

/* 连续CRC失败计数 */
static uint8_t crc_fail_count = 0;

void Lidar_Init(void)
{
    memset(&lidar_data, 0, sizeof(lidar_data));
    lidar_data.front_min_dist_mm = 0xFFFF;
    lidar_data.valid = false;
    crc_fail_count = 0;
}

/* 判断角度是否在前方扇区 (330°~30°, 即 ±30°) */
static bool is_front_angle(uint16_t angle_001deg)
{
    /* 严格检查角度范围 */
    if (angle_001deg >= 36000) return false;
    uint16_t deg = angle_001deg / 100;
    return (deg >= LIDAR_FRONT_ANGLE_MIN || deg <= LIDAR_FRONT_ANGLE_MAX);
}

/* 解析一个完整的LD06数据包 */
static bool parse_packet(uint8_t *pkt)
{
    /* 1. 校验帧头 */
    if (pkt[0] != LD06_HEADER) return false;

    /* 2. 校验PointNum (LD06固定为0x2C, 即12个点/包) */
    if (pkt[1] != LD06_POINT_NUM) return false;

    /* 3. 校验CRC */
    uint8_t crc = calc_crc8(pkt, LD06_PKT_SIZE - 1);
    if (crc != pkt[LD06_PKT_SIZE - 1]) {
        crc_fail_count++;
        return false;
    }
    crc_fail_count = 0;  /* CRC通过, 重置失败计数 */

    /* 4. 提取角度, 检查合理性 */
    uint16_t start_angle = pkt[4] | (pkt[5] << 8); /* 0.01度 */
    uint16_t end_angle   = pkt[42] | (pkt[43] << 8);

    if (start_angle >= 36000 || end_angle >= 36000) return false;

    /* 5. 计算角度步进 */
    int32_t angle_diff = (int32_t)end_angle - (int32_t)start_angle;
    if (angle_diff < 0) angle_diff += 36000;

    /* 角度跨度不应过大 (LD06每包约覆盖6-20度左右) */
    if (angle_diff > 6000) return false;  /* 超过60度不合理 */

    int32_t step = (angle_diff > 0) ? (angle_diff / 11) : 0;

    /* 6. 遍历12个测量点 */
    uint16_t front_min = lidar_data.front_min_dist_mm;
    bool updated = false;

    for (int i = 0; i < 12; i++) {
        uint16_t angle = (uint16_t)((start_angle + step * i) % 36000);
        uint16_t dist  = pkt[6 + i * 3] | (pkt[7 + i * 3] << 8);
        uint8_t  conf  = pkt[8 + i * 3];

        /* 置信度过滤: 低置信度的点不可靠 */
        if (conf < LIDAR_CONF_MIN) continue;

        /* 距离合理性检查 */
        if (dist < LIDAR_DIST_MIN_MM || dist > LIDAR_DIST_MAX_MM) continue;

        if (is_front_angle(angle)) {
            if (dist < front_min) {
                front_min = dist;
                updated = true;
            }
        }
    }

    if (updated) {
        lidar_data.front_min_dist_mm = front_min;
        lidar_data.last_update_tick = HAL_GetTick();
        lidar_data.valid = true;
    }

    return true;
}

/* 在缓冲区中搜索并解析数据包 */
static void process_buffer(void)
{
    /* 如果连续CRC失败过多, 可能是同步丢失, 强制重置 */
    if (crc_fail_count >= LIDAR_CRC_FAIL_MAX) {
        lidar_data.rx_idx = 0;
        crc_fail_count = 0;
        return;
    }

    uint16_t i = 0;
    uint16_t rx_len = lidar_data.rx_idx;  /* 快照, 减少与中断竞争 */

    while (i + LD06_PKT_SIZE <= rx_len) {
        if (lidar_data.rx_buf[i] == LD06_HEADER &&
            lidar_data.rx_buf[i + 1] == LD06_POINT_NUM) {
            /* 看起来像有效帧头, 尝试解析 */
            parse_packet(&lidar_data.rx_buf[i]);
            i += LD06_PKT_SIZE;
        } else {
            i++;
        }
    }

    /* 将未处理的数据移到缓冲区头部 */
    if (i > 0) {
        __disable_irq();  /* 短暂关中断, 保护 rx_idx */
        if (i < lidar_data.rx_idx) {
            uint16_t remain = lidar_data.rx_idx - i;
            memmove(lidar_data.rx_buf, &lidar_data.rx_buf[i], remain);
            lidar_data.rx_idx = remain;
        } else {
            lidar_data.rx_idx = 0;
        }
        __enable_irq();
    }
}

/* UART接收中断 */
void Lidar_UART_IRQHandler(void)
{
    /* 清除溢出错误 (ORE) — 230400bps 高速下容易溢出 */
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&huart4);
        /* 溢出意味着丢数据, 记录但不重置, 让CRC自然过滤坏帧 */
    }

    /* 清除帧错误 (FE) */
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_FE)) {
        __HAL_UART_CLEAR_FEFLAG(&huart4);
    }

    /* 清除噪声错误 (NE) */
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_NE)) {
        __HAL_UART_CLEAR_NEFLAG(&huart4);
    }

    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE)) {
        uint8_t ch = (uint8_t)(huart4.Instance->DR & 0xFF);
        uint16_t idx = lidar_data.rx_idx;
        if (idx < LIDAR_RX_BUF_SIZE) {
            lidar_data.rx_buf[idx] = ch;
            lidar_data.rx_idx = idx + 1;
        } else {
            /* 缓冲区满, 强制重置 (不应正常发生, 说明主循环处理太慢) */
            lidar_data.rx_idx = 0;
        }
    }
}

void Lidar_Poll(void)
{
    /* 处理接收缓冲区 */
    if (lidar_data.rx_idx >= LD06_PKT_SIZE) {
        process_buffer();
    }

    /* 周期性重置前方最小距离, 保持数据新鲜 */
    static uint32_t last_reset = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_reset > LIDAR_RESET_PERIOD) {
        last_reset = now;
        if (lidar_data.valid &&
            (now - lidar_data.last_update_tick > LIDAR_DATA_TIMEOUT)) {
            /* 超时: 雷达长时间没有有效更新, 标记为无效 */
            lidar_data.valid = false;
            lidar_data.front_min_dist_mm = 0xFFFF;
        } else if (lidar_data.valid) {
            /* 正常: 重置以获取下一个周期的最小值 */
            lidar_data.front_min_dist_mm = 0xFFFF;
        }
    }
}
