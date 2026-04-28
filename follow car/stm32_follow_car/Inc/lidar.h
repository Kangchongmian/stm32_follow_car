#ifndef __LIDAR_H
#define __LIDAR_H

#include "main.h"

/*
 * LD06 激光雷达 - UART4 (PA0/PA1) 230400bps
 * 数据包: 0x54 开头, 每包12个测量点
 * 用于前方避障: 提取前方扇区最小距离
 */
#define LIDAR_RX_BUF_SIZE   256
#define LIDAR_POINTS_PER_PKT 12
#define LIDAR_FRONT_ANGLE_MIN 330  /* 前方扇区: 330°-30° (即正前方±30°) */
#define LIDAR_FRONT_ANGLE_MAX 30

typedef struct {
    uint16_t front_min_dist_mm;  /* 前方最小障碍物距离 (毫米) */
    uint32_t last_update_tick;
    bool     valid;
    uint8_t  rx_buf[LIDAR_RX_BUF_SIZE];
    uint16_t rx_idx;
} Lidar_Data_t;

extern Lidar_Data_t lidar_data;

void Lidar_Init(void);
void Lidar_Poll(void);
void Lidar_UART_IRQHandler(void);

#endif
