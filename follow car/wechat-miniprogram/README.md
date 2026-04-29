# UWB 小车蓝牙控制 — 微信小程序

## 项目结构

```
wechat-miniprogram/
├── app.js                      # 小程序入口
├── app.json                    # 全局配置
├── app.wxss                    # 全局样式
├── project.config.json         # 项目配置
├── sitemap.json
├── utils/
│   └── ble.js                  # BLE 通信协议工具
├── pages/
│   └── index/
│       ├── index.js            # 主页逻辑
│       ├── index.wxml          # 主页布局
│       ├── index.wxss          # 主页样式
│       └── index.json          # 页面配置
└── stm32_reference/
    └── ble_protocol.h          # STM32 端协议参考代码
```

## 功能说明

1. **蓝牙连接** — 自动搜索 JDY-33 设备，显示搜索进度、设备列表、连接状态
2. **摇杆控制** — 发送 `x/y` 双轴控制值（范围 -100~100），松手自动回中并停车；独立“方向控制页”避免与参数滑动冲突
3. **参数调整** — 16 个参数通过滑块实时调整，可单个发送或全部下发（与 STM32 `CarParams_t` 一一对应）

## 通信协议

帧格式: `0xAA + CMD(1B) + LEN(1B) + DATA(nB) + XOR_CS(1B)`

| CMD  | 说明     | DATA                               |
|------|----------|------------------------------------|
| 0x01 | 设置参数 | paramIndex(1B) + float_LE(4B)      |
| 0x02 | 摇杆控制 | x(int8) + y(int8)，范围 -100~+100  |
| 0x03 | 读取参数 | paramIndex(1B)                     |
| 0x04 | 参数应答 | paramIndex(1B) + float_LE(4B)      |

参数索引与 STM32 对应关系：

| index | 参数名 |
|-------|--------|
| 0 | follow_distance_m |
| 1 | obstacle_dist_cm |
| 2 | lidar_obstacle_dist_mm |
| 3 | motor_base_speed |
| 4 | motor_turn_speed |
| 5 | motor_slow_speed |
| 6 | uwb_timeout_ms |
| 7 | ultrasonic_poll_ms |
| 8 | uwb_angle_tolerance_deg |
| 9 | emergency_stop_dist_cm |
| 10 | pid_dist_kp |
| 11 | pid_dist_ki |
| 12 | pid_dist_kd |
| 13 | pid_angle_kp |
| 14 | pid_angle_ki |
| 15 | pid_angle_kd |

## 使用方法

1. 用微信开发者工具打开 `wechat-miniprogram/` 目录
2. 在 `project.config.json` 中替换 `appid` 为你自己的小程序 AppID
3. 编译运行，手机预览即可使用

## STM32 端集成

参见 `stm32_reference/ble_protocol.h`，需要：
1. 初始化时调用 `BLE_Init()`
2. 在 USART1 中断中调用 `BLE_ReceiveByte(byte)`
3. 在主循环中调用 `BLE_ProcessCommand()`
4. 实现 `USART1_SendByte()` 函数
5. 使用 `g_car_params` 结构体替代原来的 `#define` 宏
6. 使用 `g_ble_joy_x` / `g_ble_joy_y` 变量处理摇杆控制指令
