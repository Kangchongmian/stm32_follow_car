/*
 * motor.c - 后轮差速电机控制
 *
 * BTS7960驱动器: R_PWM=反转, L_PWM=正转
 * 左后轮: TIM1_CH1(PE9)=R_PWM反转, TIM1_CH2(PE11)=L_PWM正转
 * 右后轮: TIM1_CH3(PE13)=R_PWM反转, TIM1_CH4(PE14)=L_PWM正转
 */
#include "motor.h"

void Motor_Init(void)
{
    Motor_Stop();
}

/* 设置左后轮 */
void Motor_SetLeft(MotorDir_t dir, uint16_t speed)
{
    if (speed > 666) speed = 666;
    switch (dir) {
    case MOTOR_FORWARD:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);       /* R_PWM=0 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);   /* L_PWM=正转 */
        break;
    case MOTOR_BACKWARD:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);   /* R_PWM=反转 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);       /* L_PWM=0 */
        break;
    default:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        break;
    }
}

/* 设置右后轮 */
void Motor_SetRight(MotorDir_t dir, uint16_t speed)
{
    if (speed > 666) speed = 666;
    switch (dir) {
    case MOTOR_FORWARD:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);       /* R_PWM=0 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);   /* L_PWM=正转 */
        break;
    case MOTOR_BACKWARD:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);   /* R_PWM=反转 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);       /* L_PWM=0 */
        break;
    default:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        break;
    }
}

void Motor_Forward(uint16_t speed)
{
    Motor_SetLeft(MOTOR_FORWARD, speed);
    Motor_SetRight(MOTOR_FORWARD, speed);
}

void Motor_Backward(uint16_t speed)
{
    Motor_SetLeft(MOTOR_BACKWARD, speed);
    Motor_SetRight(MOTOR_BACKWARD, speed);
}

/* 左转: 右轮快(外侧), 左轮慢(内侧) */
void Motor_TurnLeft(uint16_t outer, uint16_t inner)
{
    Motor_SetLeft(MOTOR_FORWARD, inner);
    Motor_SetRight(MOTOR_FORWARD, outer);
}

/* 右转: 左轮快(外侧), 右轮慢(内侧) */
void Motor_TurnRight(uint16_t outer, uint16_t inner)
{
    Motor_SetLeft(MOTOR_FORWARD, outer);
    Motor_SetRight(MOTOR_FORWARD, inner);
}

void Motor_Stop(void)
{
    Motor_SetLeft(MOTOR_STOP, 0);
    Motor_SetRight(MOTOR_STOP, 0);
}

/* 差速直行: left_speed/right_speed 为 0-999 的正向速度,
 * 通过减少较慢侧轮的速度实现差速转向纠偏
 * angle > 0 (标签偏左) -> 右轮减慢, 小车向左转
 * angle < 0 (标签偏右) -> 左轮减慢, 小车向右转 */
void Motor_ForwardDiff(int16_t left_speed, int16_t right_speed)
{
    if (left_speed > 666)  left_speed = 666;
    if (right_speed > 666) right_speed = 666;
    if (left_speed < 0)    left_speed = 0;
    if (right_speed < 0)  right_speed = 0;

    Motor_SetLeft(MOTOR_FORWARD,  (uint16_t)left_speed);
    Motor_SetRight(MOTOR_FORWARD, (uint16_t)right_speed);
}
