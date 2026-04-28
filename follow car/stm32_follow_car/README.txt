STM32F407VGT6 UWB跟随小车 - 工程说明
=========================================

一、工程文件结构
  Core/Inc/
    main.h         - 全局定义, 引脚宏, 可调参数
    bsp.h          - 板级支持包接口
    motor.h        - 电机控制接口
    uwb.h          - UWB通信接口
    ultrasonic.h   - 超声波传感器接口
    lidar.h        - LD06雷达接口
    oled.h         - OLED显示接口
    follow.h       - 跟随/避障逻辑接口

  Core/Src/
    main.c         - 主程序, 初始化, 主循环, 中断入口
    bsp.c          - 时钟/GPIO/TIM/UART/I2C初始化
    motor.c        - 后轮BTS7960差速控制
    uwb.c          - UWB串口数据解析 (支持ASCII和二进制协议)
    ultrasonic.c   - RS485 Modbus RTU超声波轮询
    lidar.c        - LD06激光雷达数据解析
    oled.c         - SSD1306 OLED I2C驱动和显示
    follow.c       - 跟随距离控制 + 避障转弯决策

二、引脚分配 (与接线表v3完全对应)
  电机PWM (TIM1, APB2, 1kHz):
    PE9  - TIM1_CH1 - 左后轮 R_PWM (反转)
    PE11 - TIM1_CH2 - 左后轮 L_PWM (正转)
    PE13 - TIM1_CH3 - 右后轮 R_PWM (反转)
    PE14 - TIM1_CH4 - 右后轮 L_PWM (正转)

  UWB (USART2, 115200bps):
    PD5 - TX -> UWB主站 RX
    PD6 - RX <- UWB主站 TX

  超声波RS485 (USART3, 9600bps):
    PD8 - TX -> RS485模块 RX
    PD9 - RX <- RS485模块 TX
    4个传感器地址: 1=前左, 2=前右, 3=左侧, 4=右侧

  激光雷达LD06 (UART4, 230400bps):
    PA0 - TX
    PA1 - RX

  OLED (I2C1, 400kHz):
    PB8 - SCL
    PB9 - SDA

  LED输出:
    PD4 - 蓝灯 (跟随运行)
    PD7 - 红灯 (急停/错误)
    PE0 - 绿灯 (到位/正常)
    PE1 - 黄灯 (手动模式)

  按钮/开关输入:
    PC13 - 急停按钮 (断开=急停)
    PE3  - 模式开关 (接通=跟随模式)
    PE4  - 上电反馈 (接通=电源正常)

三、可调参数 (在 main.h 中修改)
  FOLLOW_DISTANCE_M      = 1.0   跟随设定距离 (米)
  OBSTACLE_DIST_CM       = 30    超声波避障触发距离 (厘米)
  LIDAR_OBSTACLE_DIST_MM = 300   雷达避障触发距离 (毫米)
  MOTOR_BASE_SPEED       = 500   基准前进速度 (0-999)
  MOTOR_TURN_SPEED       = 700   避障转弯外侧轮速度
  MOTOR_SLOW_SPEED       = 200   避障转弯内侧轮速度

四、工作流程
  1. 上电后等待电源反馈信号 (PE4)
  2. 检查急停按钮 (PC13) - 断开则红灯亮, 停车
  3. 检查模式开关 (PE3) - 未接通则黄灯亮, 手动模式停车
  4. 跟随模式启动:
     a. UWB实时读取主站到从站距离
     b. 距离 >= 设定值 -> 前进跟随 (蓝灯亮)
     c. 距离 < 设定值  -> 停车等待 (绿灯亮)
  5. 前进过程中实时避障:
     - 前方障碍 -> 向空侧差速转弯绕行
     - 侧面障碍 -> 微调方向偏离
     - 避障不停车, 持续绕行

五、编译方法
  1. 打开 STM32CubeIDE, 新建 STM32F407VGT6 工程
  2. 在 Project Settings 中启用 HAL 库
  3. 将 Core/Inc 和 Core/Src 下的文件复制到工程对应目录
  4. 编译下载即可

  或使用 Keil MDK:
  1. 新建 STM32F407VGT6 工程, 添加 CMSIS 和 HAL 库
  2. 添加本工程所有 .c 和 .h 文件
  3. Include Path 添加 Core/Inc
  4. 编译下载

六、注意事项
  - UWB模块协议: 程序同时支持ASCII和Nooploop二进制格式,
    如使用其他品牌UWB模块请根据其协议修改 uwb.c 中的解析函数
  - 超声波Modbus地址: 默认1-4, 需与实际传感器地址设置一致
  - LD06雷达: 230400波特率, 确保接线正确
  - BTS7960驱动器: R_PWM控制反转, L_PWM控制正转
  - 前面两个轮子(PD12-PD15)为从动轮, 程序中不控制
