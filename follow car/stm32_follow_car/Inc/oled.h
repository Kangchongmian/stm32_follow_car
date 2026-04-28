#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#define OLED_ADDR  0x78  /* SSD1306 7-bit addr 0x3C << 1 */

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
void OLED_ShowFloat(uint8_t x, uint8_t y, float val, uint8_t decimals);
void OLED_Update(float uwb_dist, uint16_t front_obs, uint16_t lidar_obs);

#endif
