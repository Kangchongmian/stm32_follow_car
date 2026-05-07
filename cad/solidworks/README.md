# FOLO-100 SolidWorks 装配体生成器

本目录提供一个 SolidWorks VBA 宏，用于在安装了 SolidWorks 的 Windows 电脑上自动生成 FOLO-100 自动跟随车的简化装配体文件。

## 文件

| 文件 | 用途 |
| --- | --- |
| `FOLO100_GenerateAssembly.bas` | SolidWorks VBA 宏源码；运行后生成 `.SLDASM` 装配体和多个 `.SLDPRT` 零件。 |

## 生成结果

运行宏后会在以下目录生成文件：

```text
%USERPROFILE%\Documents\FOLO100_SolidWorks\
├── FOLO100_Automatic_Follow_Cart.SLDASM
└── parts\
    ├── FOLO100_platform_700x540x18.SLDPRT
    ├── FOLO100_chassis_frame_760x520x40.SLDPRT
    ├── FOLO100_battery_tray_360x190x40.SLDPRT
    ├── FOLO100_control_box_260x180x90.SLDPRT
    ├── FOLO100_motor_plate_180x140x6.SLDPRT
    ├── FOLO100_drive_wheel_305x75.SLDPRT
    ├── FOLO100_aux_wheel_200x50.SLDPRT
    ├── FOLO100_front_bumper_600x30x35.SLDPRT
    ├── FOLO100_side_guard_700x20x210.SLDPRT
    ├── FOLO100_sensor_post_40x40x135.SLDPRT
    └── FOLO100_lidar_block_90x45x45.SLDPRT
```

## 使用步骤

1. 打开 SolidWorks。
2. 选择 **Tools → Macro → New**，新建一个宏文件；或选择 **Tools → Macro → Edit** 打开已有宏。
3. 将 `FOLO100_GenerateAssembly.bas` 的内容复制到宏编辑器中，或在 VBA 编辑器中导入该 `.bas` 模块。
4. 运行 `main` 过程。
5. 打开 `%USERPROFILE%\Documents\FOLO100_SolidWorks\FOLO100_Automatic_Follow_Cart.SLDASM` 查看装配体。

## 装配坐标

宏采用与机械文档一致的坐标系：

- X 轴：车头方向为正。
- Y 轴：车左方向为正。
- Z 轴：地面向上为正。
- SolidWorks API 坐标单位为米，宏内部将机械图纸中的 mm 自动换算为 m。

关键装配位置包括：

| 模块 | 装配坐标/尺寸 |
| --- | --- |
| 整车包络 | 860 × 600 × 540 mm |
| 载物平台 | 700 × 540 × 18 mm，中心 Z=405 mm |
| 底盘骨架 | 760 × 520 × 40 mm，中心 Z=235 mm |
| 电池托盘 | 360 × 190 × 40 mm，中心 Z=285 mm |
| 控制箱 | 260 × 180 × 90 mm，中心 X=-240 mm，Z=300 mm |
| 驱动轮 | Ø305 × 75 mm，中心 X=0，Y=±245 mm，Z=152.5 mm |
| 辅助轮 | Ø200 × 50 mm，中心 X=±300，Y=±210 mm，Z=100 mm |
| 传感器立柱 | 40 × 40 × 135 mm，中心 X=365 mm，Z=472.5 mm |
| 雷达块 | 90 × 45 × 45 mm，中心 X=390 mm，Z=520 mm |

## 注意事项

- 由于 `.SLDASM/.SLDPRT` 是 SolidWorks 专有二进制格式，本仓库提供的是可复现生成装配体的宏源码；需要在 SolidWorks 中运行后才会产生真正的装配体文件。
- 生成的模型是用于总体布局、空间占用和装配演示的简化模型，不包含完整钣金折弯、轴承、减速机、螺栓螺母和真实轮胎花纹。
- 批量加工前请结合 `docs/mechanical-drawing.md` 的孔位表和实物零件做二次建模与工程图出图。
