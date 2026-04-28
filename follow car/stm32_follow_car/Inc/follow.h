#ifndef __FOLLOW_H
#define __FOLLOW_H

#include "main.h"

typedef enum {
    STATE_IDLE = 0,       /* 空闲 / 手动模式 */
    STATE_FOLLOW,         /* 正常跟随 */
    STATE_AVOID_LEFT,     /* 左转避障 */
    STATE_AVOID_RIGHT,    /* 右转避障 */
    STATE_EMERGENCY_STOP, /* 紧急停车 (障碍物过近) */
    STATE_BLE_MANUAL,     /* 蓝牙手动遥控 */
    STATE_ESTOP           /* 急停 */
} FollowState_t;

extern FollowState_t follow_state;

void Follow_Init(void);
void Follow_Update(void);  /* 主循环调用, 综合传感器数据决策 */

#endif
