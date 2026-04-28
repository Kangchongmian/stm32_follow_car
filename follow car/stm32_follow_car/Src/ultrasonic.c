/*
 * ultrasonic.c - RS485超声波传感器阵列 (USART3, PD8/PD9, 9600bps)
 *
 * 4个超声波传感器通过RS485总线连接, Modbus RTU协议
 *   地址1: 前方左侧
 *   地址2: 前方右侧
 *   地址3: 左侧
 *   地址4: 右侧
 *
 * 读取命令 (Modbus 功能码03, 读保持寄存器):
 *   [ADDR][03][00][01][00][01][CRC_L][CRC_H]
 * 响应:
 *   [ADDR][03][02][DATA_H][DATA_L][CRC_L][CRC_H]
 */
#include "ultrasonic.h"
/* 调试变量 */
volatile uint8_t  dbg_us_tx_buf[8];   /* 最近一次发送的Modbus请求 */
volatile uint8_t  dbg_us_tx_addr = 0; /* 最近一次请求的传感器地址 */
Ultrasonic_Data_t us_data;

/* CRC16-Modbus查表法 */
static uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void Ultrasonic_Init(void)
{
    memset(&us_data, 0, sizeof(us_data));
    for (int i = 0; i < US_SENSOR_COUNT; i++) {
        us_data.distance_cm[i] = 0xFFFF;  /* 无效值 */
        us_data.valid[i] = false;
    }
    us_data.current_poll_id = 0;
    us_data.last_poll_tick = HAL_GetTick();
}

/* 发送Modbus读取请求 */
static void send_read_cmd(uint8_t addr)
{
    uint8_t cmd[8];
    cmd[0] = addr;       /* 从站地址 1-4 */
    cmd[1] = 0x03;       /* 功能码: 读保持寄存器 */
    cmd[2] = 0x01;       /* 起始地址高字节 */
    cmd[3] = 0x00;       /* 起始地址低字节 (寄存器1=距离) */
    cmd[4] = 0x00;       /* 数量高字节 */
    cmd[5] = 0x01;       /* 数量低字节 (读1个寄存器) */

    uint16_t crc = crc16_modbus(cmd, 6);
    cmd[6] = crc & 0xFF;         /* CRC低字节 */
    cmd[7] = (crc >> 8) & 0xFF;  /* CRC高字节 */
	
/* 调试: 保存到全局变量供Keil监控 */
    memcpy((void*)dbg_us_tx_buf, cmd, 8);
    dbg_us_tx_addr = addr;
	
    HAL_UART_Transmit(&huart3, cmd, 8, 10);
}

/* 解析Modbus响应 */
static void parse_response(void)
{
    if (us_data.rx_idx < 7) return;

    uint8_t *buf = us_data.rx_buf;
    uint8_t addr = buf[0];
    uint8_t func = buf[1];
    uint8_t byte_count = buf[2];

    if (func != 0x03 || byte_count != 0x02) return;
    if (addr < 1 || addr > US_SENSOR_COUNT) return;

    /* 验证CRC */
    uint16_t crc_calc = crc16_modbus(buf, 5);
    uint16_t crc_recv = buf[5] | (buf[6] << 8);
    if (crc_calc != crc_recv) return;

    /* 提取距离值 (单位: mm -> cm) */
    uint16_t dist_mm = (buf[3] << 8) | buf[4];
    uint8_t idx = addr - 1;
    us_data.distance_cm[idx] = dist_mm / 10;  /* mm转cm */
    us_data.last_update_tick[idx] = HAL_GetTick();
    us_data.valid[idx] = true;
}

/* UART接收中断 */
void Ultrasonic_UART_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE)) {
        uint8_t ch = (uint8_t)(huart3.Instance->DR & 0xFF);
        if (us_data.rx_idx < US_RX_BUF_SIZE) {
            us_data.rx_buf[us_data.rx_idx++] = ch;
        }
    }
}

/* 主循环轮询: 依次查询4个传感器 */
void Ultrasonic_Poll(void)
{
    uint32_t now = HAL_GetTick();

    if (now - us_data.last_poll_tick < (uint32_t)g_car_params.ultrasonic_poll_ms) return;
    us_data.last_poll_tick = now;

    /* 先解析上一次的响应 */
    if (us_data.rx_idx > 0) {
        parse_response();
        us_data.rx_idx = 0;
    }

    /* 发送下一个传感器的读取请求 */
    us_data.current_poll_id++;
    if (us_data.current_poll_id > US_SENSOR_COUNT)
        us_data.current_poll_id = 1;

    send_read_cmd(us_data.current_poll_id);
}
