# UWB 多源传感器数据采集系统

基于 STM32H743VITX (Cortex-M7) 的嵌入式多源传感器数据采集系统，运行 FreeRTOS。集成 DW1000 UWB 测距、UM960 GNSS 定位、ASM330LHH IMU，统一时间戳后写入 SD 卡。

## 项目目标

1. **UWB 测距**: Tag 主动发起，Anchor 被动响应，支持多 Anchor 同时测距 (DS-TWR 双次交换)，目标测距频率 ~90Hz
2. **GNSS 定位与时间同步**: 接收 GNSS 解算结果，提供 UTC 时间基准，通过 PPS 同步本地时钟
3. **IMU 采样**: ~208Hz 三轴加速度 + 三轴陀螺仪数据采集
4. **多源数据融合**: 统一时间戳 (TIM16 本地连续时钟 + GNSS UTC)，按时间排序后写入 SD 卡
5. **配置管理**: 按键进入配置模式，通过串口设置 PAN ID、短地址、设备角色，持久化到 Flash

## 硬件平台

| 外设 | 接口 | 引脚 | 用途 |
|------|------|------|------|
| DW1000 | SPI2 | PB12-15, RST=PD9, IRQ=PD8 | UWB 测距 |
| UM960 GNSS | USART3+DMA | PB10/PB11, PPS=PC0 | 定位与时间基准 |
| ASM330 IMU | SPI1 | PA5-7, CS=PC4, IRQ=PA4 | 惯性测量 |
| SD 卡 | SDMMC1 4-bit | PC8-12, PD2, DET=PB5 | 数据存储 |
| 调试串口 | USART1 460800 | PA9/PA10 | 日志与配置命令 |
| 用户按键 | EXTI | PE2 | 模式选择 / 长按复位 |
| LED | GPIO | PE4,PE5,PE6,PC13 | 状态指示 |

## 已完成功能

### 系统框架

- FreeRTOS 任务体系 (9 个任务): defaultTask, gnssTask, imuTask, dataSort, sdWriter, keyTask, ledTask, usartCMD, uwbPhy/uwbLink/uwbApp
- 分层架构: APP/ 目录下按 bsp/common/device/service/task/UWB 划分
- 启动模式: 按键按下进入 CONFIG 模式 (串口命令)，否则进入 RUN 模式
- 配置服务: Flash 持久化 (pan_id, short_addr, role)，支持串口 `read` / `set` / `default` / `reboot` 命令
- 双缓冲 SD 写入: 16KB 双块缓冲，ASCII 格式日志文件 `uwb-gnss-imu-sampling-N.log`

### GNSS

- USART3 + DMA + IDLE 接收，解析 UM960 `0xAA 0x44 0xB5` 二进制协议
- 提取 UTC 时间更新 TimeService，提供 PPS 整秒对齐
- 解算结果投递 DataService (经纬度、高度、定位质量等)

### IMU

- EXTI 中断触发采样，SPI1 读取 ASM330LHH 六轴原始数据
- ~208Hz 采样率，投递 DataService

### 时间同步 (TimeService)

- TIM2 本地连续时钟 (单调递增，不受 UTC 校时影响)
- GNSS UTC 缓存 + PPS 锚点同步机制
- 同步状态机: LOCKED → HOLDOVER → LOST
- 统一接口 `TimeService_GetTimestamp()` 供所有传感器使用

### 数据流

```
GNSS (USART3 DMA) ─→ gnssTask ─→ DataService_Send()
IMU  (SPI1 EXTI)  ─→ imuTask  ─→ DataService_Send()
UWB  (DW1000 IRQ) ─→ uwbPhy → uwbLink → uwbApp → DataService_Send()
                                                      │
                                          DataService_Queue (64 deep)
                                                      │
                                    AppSdWriterTask (排序 + ASCII + SD)
```

### UWB 协议栈 (三层架构)

```
APP/UWB/
  uwb_stack.c        入口: UwbStack_StartFromConfig() 初始化三层并启动线程
  uwb_phy.c/h        PHY: DW1000 硬件控制 (SPI 收发、中断处理、时序)
  uwb_link.c/h       LINK: 帧协议、状态机、调度 (Discovery/Ranging)
  uwb_app.c/h        APP: 测距计算、数据发布 (TWR → AppDataNode → DataService)
  uwb_protocol.c/h   帧编解码 (MAC short header + payload header + body)
  uwb_buffers.c/h    静态内存池、队列声明
  uwb_timestamp.c/h  DW1000 40-bit 时间戳读写与运算
```

**已完成:**

- Discovery 帧交互: Tag 周期性发送 DISC_REQ，Anchor 按槽延迟应答 DISC_RESP
- 多槽接收 (Tag 侧): 4 个 RX 槽，每槽 2ms，Anchor 按 `(addr - 0x30) % 4` 分配时隙
- Anchor 延迟应答: 统一 DELAYED TX (>= 1ms)，消除竞态风险
- DS-TWR 测距: 双次交换 DS-TWR，`tof = ((ra*rb - da*db) / (ra+rb+da+db))`
- Tag / Anchor 双角色支持
- 零拷贝共享内存池层间通信

**尚未实现:**

- 滑窗 DS-TWR 连续测距 (plan-v3): 复用 DISC 交换时间戳，去掉 200ms 固定周期，目标 ~90Hz
- DATA 帧传输 (数据下行): 帧类型已定义，LINK 层未处理

## SD 日志格式

GNSS: `GNSS,week,week_ms,local_sec,local_ms,utc_valid,lat,lon,hgt,...`

IMU: `IMU,week,week_ms,local_sec,local_ms,utc_valid,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z`

UWB: `UWB,week,week_ms,local_sec,local_ms,utc_valid,anchor_id,tag_id,...,distance_m,...`

## 快速测试

1. 按住 `USER_KEY` 上电 → 进入 CONFIG 模式，串口 460800 出现 `config mode`
2. 配置 Anchor: `set F0F0 0034 1` → `read` 确认 → `reboot`
3. 配置 Tag: `set F0F0 0101 0` → `read` 确认 → `reboot`
4. 松开按键上电 → RUN 模式，自动开始采集、测距、SD 写入

## 构建

- 工具链: EIDE + GCC ARM (`.eide/eide.yml`)
- 硬件配置: STM32CubeMX (`UWB.ioc`)
- 编译: C11, -O0, hard float, thumb, newlib-nano
- 烧录: OpenOCD (ST-Link) 或 JLink，基地址 `0x08000000`
- 输出: `build/Debug/`

## 目录结构

```
APP/                 自研业务代码 (主要开发区域)
├── UWB/             UWB 三层协议栈 (PHY/LINK/APP)
├── app/             系统入口 app.c (App_Start, 模式选择)
├── bsp/             板级适配 (LED, 按键, SD 检测)
├── common/          公共类型 app_types.h
├── device/          设备驱动封装 (gnss_parser, imu_device)
├── service/         跨模块服务 (config, time, data, storage, log)
└── task/            任务定义 (app_tasks, app_irq)
Core/                CubeMX 生成 (不改)
Drivers/             HAL + CMSIS (不改)
Middlewares/         FreeRTOS 内核 (不改)
FATFS/               FatFs 集成层 (不改)
Thrid/               DecaDriver (DW1000), ASM330 驱动
```

## 分支

- `main`: 主分支
- `UWB-V2`: 当前活跃开发分支，UWB 协议栈重构
- `testPro`: 在板测试分支

## 详细设计文档

| 文档 | 内容 |
|------|------|
| [docs/designed/designed.md](docs/designed/designed.md) | 系统总体设计 (线程、时间同步、缓冲区) |
| [docs/designed/uwb-designed.md](docs/designed/uwb-designed.md) | UWB 协议栈设计 |
| [docs/project-overview.md](docs/project-overview.md) | 工程文件导航 |
| [docs/plan/plan-v2/](docs/plan/plan-v2/) | 多槽接收方案 (Anchor/Tag 分文档) |
| [docs/plan/plan-v3/](docs/plan/plan-v3/) | 滑窗 TWR 测距方案 |
