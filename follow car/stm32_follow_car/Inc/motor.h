#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/* 电机方向 */
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} MotorDir_t;

void Motor_Init(void);
void Motor_SetLeft(MotorDir_t dir, uint16_t speed);   /* speed: 0-999 */
void Motor_SetRight(MotorDir_t dir, uint16_t speed);
void Motor_Forward(uint16_t speed);
void Motor_Backward(uint16_t speed);
void Motor_TurnLeft(uint16_t outer, uint16_t inner);
void Motor_TurnRight(uint16_t outer, uint16_t inner);
void Motor_Stop(void);
void Motor_ForwardDiff(int16_t left_speed, int16_t right_speed);  /* 差速直行, speed 0-999, 可为负(慢侧轮) */

#endif
