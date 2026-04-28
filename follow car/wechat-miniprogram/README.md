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

1. **蓝牙连接** — 自动搜索 JDY-31 设备，显示搜索进度、设备列表、连接状态
2. **方向控制** — 前进/后退/左转/右转/停止，按下即走、松手即停
3. **参数调整** — 8 个参数通过滑块实时调整，可单个发送或全部下发

## 通信协议

帧格式: `0xAA + CMD(1B) + LEN(1B) + DATA(nB) + XOR_CS(1B)`

| CMD  | 说明     | DATA                              |
|------|----------|-----------------------------------|
| 0x01 | 设置参数 | paramIndex(1B) + float_LE(4B)     |
| 0x02 | 方向控制 | direction(1B): 0停 1前 2后 3左 4右 |
| 0x03 | 读取参数 | paramIndex(1B)                    |
| 0x04 | 参数应答 | paramIndex(1B) + float_LE(4B)     |

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
6. 使用 `g_ble_direction` 变量处理方向控制指令
