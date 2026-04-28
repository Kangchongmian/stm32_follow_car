#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "main.h"

/*
 * 4个超声波传感器通过RS485总线(USART3)连接
 * Modbus RTU协议, 地址1-4
 * 传感器布局:
 *   ID 1: 前方左侧
 *   ID 2: 前方右侧
 *   ID 3: 左侧
 *   ID 4: 右侧
 */
#define US_SENSOR_COUNT     4
#define US_FRONT_LEFT       0
#define US_FRONT_RIGHT      1
#define US_LEFT             2
#define US_RIGHT            3

#define US_RX_BUF_SIZE      32

typedef struct {
    uint16_t distance_cm[US_SENSOR_COUNT]; /* 每个传感器的距离 (厘米) */
    uint32_t last_update_tick[US_SENSOR_COUNT];
    bool     valid[US_SENSOR_COUNT];
    uint8_t  rx_buf[US_RX_BUF_SIZE];
    uint16_t rx_idx;
    uint8_t  current_poll_id;              /* 当前正在轮询的传感器 */
    uint32_t last_poll_tick;
} Ultrasonic_Data_t;

extern Ultrasonic_Data_t us_data;

void Ultrasonic_Init(void);
void Ultrasonic_Poll(void);        /* 主循环调用, 轮询传感器并解析 */
void Ultrasonic_UART_IRQHandler(void);

#endif
