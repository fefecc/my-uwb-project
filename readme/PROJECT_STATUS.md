# UWB Project Current Status

本文记录当前工程已经完成的重建内容、代码结构和接线方式。当前重建范围是除 UWB 协议栈主体之外的功能单元；UWB 相关目录目前只保留兼容编译所需的旧代码和 stub，不作为本轮重构重点。

## 1. 当前已完成内容

### 1.1 APP 框架

已在 `APP/` 下重新建立应用层结构：

```text
APP
├─ app
├─ bsp
├─ common
├─ device
├─ service
├─ task
└─ UWB
```

其中：

- `APP/app`
  - 应用入口层
  - 负责启动模式判断
  - 根据 `USER_KEY` 判断进入配置模式或运行模式

- `APP/bsp`
  - 板级封装
  - 已完成 LED、按键、SD 卡检测封装
  - 对外屏蔽 LED 有效电平差异

- `APP/common`
  - 公共类型、兼容头、双缓冲
  - 包含旧接口兼容层，例如 `timestamp`、`app_log`、`app_config`

- `APP/device`
  - 设备级驱动适配
  - 已完成 GNSS 解析器
  - 已完成 ASM330 IMU SPI 适配

- `APP/service`
  - 公共服务
  - 已完成时间、配置、数据队列、日志、存储服务

- `APP/task`
  - FreeRTOS 任务入口
  - 已完成 GNSS、IMU、数据整理、SD 写入、按键、LED、串口配置任务框架

- `APP/UWB`
  - 当前从 `APP-copy/UWB` 恢复旧代码
  - 目的只是保证现有工程构建依赖完整
  - UWB 协议栈后续按 `docs/uwb-designed.md` 单独重构

## 2. 已完成的功能单元

### 2.1 启动入口

文件：

- `APP/app/app.h`
- `APP/app/app.c`

已实现：

- `App_DetectBootMode()`
  - 读取 `USER_KEY`
  - 按键按下进入 `APP_MODE_CONFIG`
  - 未按下进入 `APP_MODE_RUN`

- `App_Start()`
  - 检测启动模式
  - 调用 `AppTasks_CreateAll(mode)` 创建应用任务

当前接线位置：

- `Core/Src/freertos.c`
  - 在 `USER CODE BEGIN Includes` 中包含 `app.h`
  - 在 `MX_FREERTOS_Init()` 的 `USER CODE BEGIN RTOS_THREADS` 中调用 `App_Start()`

这样可以保证：

- HAL 外设先完成初始化
- RTOS kernel 初始化后再创建任务
- 不直接破坏 CubeMX 生成代码结构

### 2.2 时间服务

文件：

- `APP/service/time_service.h`
- `APP/service/time_service.c`

已实现：

- `TIM16` 作为本地连续时钟来源
- 本地连续时钟使用 `sec` 和 `ms` 表达，其中 `sec` 为累计整数秒，`ms` 为当前秒内浮点毫秒
- GNSS UTC 缓存使用：

```c
typedef struct {
    uint32_t week;
    uint32_t week_ms;
} TimeUtcClock;
```

- GNSS 解析线程只调用 `TimeService_WriteUtcCache()` 写缓存
- PPS 中断触发时调用 `TimeService_OnPpsIrq()`
- PPS 中断中使用 `GNSS UTC 缓存 + 1000 ms` 更新当前 UTC 整秒锚点
- `TimeService_GetTimestamp()` 返回完整时间戳
- `TimeService_GetLocalClock()` 只返回本地连续时钟

兼容入口：

- `APP/common/timestamp.h`
- `APP/common/timestamp.c`

保留旧函数名：

- `timestamp_tick_irq()`
- `PPS_IRQHandler()`
- `gettimestamp()`

这些旧函数内部转接到新的 `TimeService`。

### 2.3 配置服务

文件：

- `APP/service/config_service.h`
- `APP/service/config_service.c`

已实现：

- Flash 配置结构 `AppConfig`
- 默认配置
- 配置合法性检查
- Flash 读取
- Flash 擦除和写入
- 保存默认配置

当前配置字段包括：

- `pan_id`
- `short_addr`
- `frame_ctrl`
- `role`
- `log_level`

兼容入口：

- `APP/common/app_config.h`
- `APP/common/app_config.c`

用于兼容旧 UWB 代码中的 `AppConfig_Get()` 等旧接口。

### 2.4 LED BSP

文件：

- `APP/bsp/bsp_led.h`
- `APP/bsp/bsp_led.c`

已实现：

- `BspLed_Set()`
- `BspLed_Toggle()`
- `BspLed_AllOff()`
- `BspLed_AllOn()`

已处理 LED 有效电平差异：

- `LED0`、`LED1` 低电平点亮
- `LED2`、`LED3` 高电平点亮

### 2.5 按键 BSP

文件：

- `APP/bsp/bsp_key.h`
- `APP/bsp/bsp_key.c`

已实现：

- 按键状态读取
- 消抖
- 短按事件
- 长按事件
- 长按释放事件

当前长按阈值在任务中配置为 `200 ms`。

### 2.6 SD 检测与存储服务

文件：

- `APP/bsp/bsp_sd.h`
- `APP/bsp/bsp_sd.c`
- `APP/service/storage_service.h`
- `APP/service/storage_service.c`

已实现：

- SD 插入检测
- SDMMC 初始化调用
- FatFs 挂载
- 自动创建新日志文件
- 将数据节点转换为 ASCII 行写入 SD 文件

当前日志文件命名：

```text
sample-1.log
sample-2.log
...
```

### 2.7 GNSS 解析器

文件：

- `APP/device/gnss_parser.h`
- `APP/device/gnss_parser.c`

已实现：

- UM960/Unicore 风格二进制帧同步头解析
- 帧头读取
- payload 长度检查
- CRC32 校验
- 回调输出完整帧
- 从帧头提取 UTC：
  - `week`
  - `week_ms`

GNSS 任务中已接入：

- USART3 DMA 接收
- IDLE 中断触发块解析
- 有效 UTC 写入 `TimeService` 缓存
- BESTNAV 类数据打包成 `AppDataNode`

### 2.8 IMU 设备适配

文件：

- `APP/device/imu_device.h`
- `APP/device/imu_device.c`

已实现：

- ASM330 SPI 读写适配
- 设备 ID 检查
- 加速度计和陀螺仪配置
- 原始 IMU 数据读取

IMU 任务中已接入：

- 等待 IMU 中断通知
- 读取 IMU 原始数据
- 获取完整时间戳
- 打包成 `AppDataNode`

### 2.9 数据队列与数据整理

文件：

- `APP/service/data_service.h`
- `APP/service/data_service.c`
- `APP/task/app_tasks.c`

已实现：

- 统一数据节点类型 `AppDataNode`
- GNSS 和 IMU 共用数据队列
- 数据整理任务按本地连续时钟进行小窗口排序
- 当前排序窗口为 `10` 个节点
- 排序后投递给 SD 写入队列

### 2.10 日志服务

文件：

- `APP/service/log_service.h`
- `APP/service/log_service.c`
- `APP/common/app_log.h`

已实现：

- USART1 文本日志输出
- 日志等级
- RTOS mutex 保护
- 旧 `log_info()`、`log_warn()`、`log_error()` 宏兼容

当前日志服务是可用的基础版本，后续可以继续升级为文档中规划的双缓冲日志调度。

### 2.11 任务框架

文件：

- `APP/task/app_tasks.h`
- `APP/task/app_tasks.c`
- `APP/task/app_irq.c`

已实现任务：

- `AppDefaultTask`
- `AppGnssTask`
- `AppImuTask`
- `AppDataSortTask`
- `AppSdWriterTask`
- `AppKeyTask`
- `AppLedTask`
- `AppUsartCmdTask`

已实现中断转接：

- `PC0` PPS 中断转接到 `TimeService_OnPpsIrq()`
- `PA4` IMU DRDY 中断通知 IMU task
- `PE2` USER_KEY 中断通知 key task
- USART3 IDLE 中断通过兼容入口 `GNSSIdleHandler()` 通知 GNSS task

## 3. 当前启动流程

```text
main()
  |
  |-- HAL_Init()
  |-- SystemClock_Config()
  |-- PeriphCommonClock_Config()
  |
  |-- MX_GPIO_Init()
  |-- MX_DMA_Init()
  |-- MX_SPI1_Init()
  |-- MX_USART1_UART_Init()
  |-- MX_TIM16_Init()
  |-- MX_USART3_UART_Init()
  |-- MX_UART4_Init()
  |-- MX_SPI2_Init()
  |-- MX_I2C1_Init()
  |
  |-- osKernelInitialize()
  |-- MX_FREERTOS_Init()
        |
        |-- CubeMX defaultTask
        |-- App_Start()
              |
              |-- App_DetectBootMode()
              |-- AppTasks_CreateAll(mode)
  |
  |-- osKernelStart()
```

## 4. 当前模式行为

### 4.1 配置模式

进入条件：

- 上电时 `USER_KEY` 按下

当前行为：

- LED0 到 LED3 顺序流水闪烁
- USART1 进入配置命令模式

当前支持命令：

```text
read
default
set <pan_hex> <short_hex> <role>
reboot
```

示例：

```text
set f0f0 0034 1
```

### 4.2 运行模式

进入条件：

- 上电时 `USER_KEY` 未按下

当前行为：

- 初始化时间服务
- 加载 Flash 配置
- 启动 GNSS task
- 启动 IMU task
- 启动数据整理 task
- 启动 SD 写入 task
- LED3 周期闪烁作为运行指示

## 5. 当前保留的兼容层

为了不修改 CubeMX 生成代码，同时让旧中断入口和旧 UWB 编译依赖继续可用，当前保留了几个兼容文件：

```text
APP/common/timestamp.h
APP/common/timestamp.c
APP/common/app_config.h
APP/common/app_config.c
APP/common/app_log.h
APP/task/UM960samplingtask.h
APP/task/DW1000samplingtask.h
APP/task/DW1000samplingtask.c
```

这些文件的目的不是继续扩展旧架构，而是让当前工程在过渡阶段能够编译和运行。

## 6. UWB 当前状态

当前没有重建 UWB 协议栈。

为了保证工程现有构建配置可通过，已将旧 UWB 适配代码从 `APP-copy/UWB` 恢复到：

```text
APP/UWB
```

同时提供了 `DW1000samplingtask` 兼容 stub。

当前 UWB 状态：

- 旧 UWB 代码参与编译
- UWB task 不在 `AppTasks_CreateAll()` 中启动
- UWB 协议栈后续应按 `docs/uwb-designed.md` 单独重构

## 7. 当前构建状态

已执行 EIDE builder 全量构建：

```text
unify_builder --rebuild
build successfully
```

当前已知 warning：

```text
Thrid/decadriver/trilateration.c
```

存在两个未使用变量 warning，不影响构建。

## 8. 后续建议顺序

建议后续按以下顺序继续推进：

1. 上板验证启动模式判断
2. 验证 USART1 配置模式命令
3. 验证 TIM16 本地连续时钟
4. 验证 GNSS UTC 缓存写入和 PPS 同步
5. 验证 IMU DRDY 中断和 IMU 数据读取
6. 验证 SD 卡挂载和 ASCII 写入
7. 在上述功能稳定后，再开始 UWB 协议栈重构
