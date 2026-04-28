#ifndef __BSP_H
#define __BSP_H

#include "main.h"

void BSP_ClockConfig(void);
void BSP_GPIO_Init(void);
void BSP_TIM1_Init(void);      /* 电机PWM */
void BSP_USART1_Init(void);    /* JDY-31蓝牙 */
void BSP_USART2_Init(void);    /* UWB */
void BSP_USART3_Init(void);    /* RS485超声波 */
void BSP_UART4_Init(void);     /* LD06雷达 */
void BSP_I2C1_Init(void);      /* OLED */

/* LED控制 */
void LED_Blue(bool on);
void LED_Red(bool on);
void LED_Green(bool on);
void LED_Yellow(bool on);
void LED_AllOff(void);

/* 转向灯控制 */
#define TURN_NONE   0
#define TURN_LEFT   1
#define TURN_RIGHT  2
void TurnSignal_Set(uint8_t dir);   /* 设置转向: TURN_NONE/LEFT/RIGHT */
void TurnSignal_Tick(void);          /* 主循环调用, 处理闪烁 */

/* 输入读取 */
bool IsEstopPressed(void);
bool IsFollowMode(void);
bool IsPowerOn(void);

#endif
