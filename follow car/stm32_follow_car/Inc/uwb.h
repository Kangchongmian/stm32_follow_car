#ifndef __UWB_H
#define __UWB_H

#include "main.h"

#define UWB_RX_BUF_SIZE  128

typedef struct {
    float    distance_m;       /* 主站到从站距离 (米) */
    int16_t  angle_deg;        /* 标签相对小车角度 (度), 正=左, 负=右 */
    uint32_t last_update_tick; /* 最后一次有效更新的时间戳 */
    bool     valid;            /* 数据是否有效 */
    uint8_t  rx_buf[UWB_RX_BUF_SIZE];
    uint16_t rx_idx;
} UWB_Data_t;

extern UWB_Data_t uwb_data;

void UWB_Init(void);
void UWB_Poll(void);           /* 在主循环调用, 解析接收数据 */
void UWB_UART_IRQHandler(void);

#endif
