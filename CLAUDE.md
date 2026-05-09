# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Critical Caveats

- **No `%llu` / `%lld`** in `printf`/`vsnprintf`. newlib-nano does not support 64-bit format specifiers. Use split-32 hex: `0x%02lX%08lX` with `(uint32_t)(val>>32)`, `(uint32_t)val`, or cast to `(unsigned long)` and use `%lu`.
- **If a log line shows `local=lu` or `rx_ts=lu`**, all subsequent formatted fields in that line are unreliable — varargs have been misaligned by a skipped 64-bit argument.
- **Known existing `%llu` usage** in `APP/task/app_tasks.c:658` (UWB ASCII format) and `APP/service/storage_service.c:91,111` (GNSS/IMU ASCII format) — these produce garbage for the timestamp field on target.
- **FreeRTOS heap is 128KB** (`configTOTAL_HEAP_SIZE = 131072`). Every task stack and queue is allocated from this pool.
- **No unit test framework.** All testing is on hardware. Branch `testPro` exists for ad-hoc on-target tests.

## 项目概述

STM32H743VITX (Cortex-M7) 嵌入式多源传感器数据采集系统，FreeRTOS。
采集 DW1000 UWB 测距、UM960 GNSS 定位、ASM330LHH IMU 数据，统一时间戳后写入 SD 卡。

## 构建系统

- 工具链: EIDE + GCC ARM，配置 `.eide/eide.yml`
- 硬件配置: STM32CubeMX (`UWB.ioc`)
- 烧录: OpenOCD (ST-Link) 或 JLink，基地址 `0x08000000`
- 编译输出: `build/Debug/`
- 链接脚本: `STM32H743VITX_FLASH.ld`
- C11, -O0 (Debug), hard float, thumb, newlib-nano, `-u _printf_float`
- 预定义宏: `DEBUG`, `USE_PWR_LDO_SUPPLY`, `USE_HAL_DRIVER`, `STM32H743xx`

## 启动链

```
main() → MX_FREERTOS_Init() → StartDefaultTask + App_Start()
  App_Start() → App_DetectBootMode()  (key pressed = CONFIG, else RUN)
              → AppTasks_CreateAll(mode)
```

`AppTasks_CreateAll()` 按顺序初始化服务: LogService, TimeService, ConfigService (Flash), DataService, SD FIFO。CONFIG 模式跳过 UWB/GNSS/IMU 任务，仅运行 USART 配置 shell。

## 内存布局

| Region | Origin | Size | Notes |
|--------|--------|------|-------|
| FLASH | 0x08000000 | 2048K | |
| DTCMRAM | 0x20000000 | 128K | |
| RAM_D1 | 0x24000000 | 512K | Main stack (`_estack`) |
| RAM_D2 | 0x30000000 | 288K | |
| RAM_D3 | 0x38000000 | 64K | |

Linker: `_Min_Heap_Size = 4KB`, `_Min_Stack_Size = 8KB`。FreeRTOS 使用独立的 heap_4 (128KB)，与 C heap 分开。

## FreeRTOS 配置

- V10.3.1, heap_4, 抢占式, 1ms tick, 56 优先级
- `configTOTAL_HEAP_SIZE = 128KB`
- `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`
- CMSIS-RTOS v2 API (`osThreadNew`, `osDelay` 等)
- ISR 到任务: 仅 `xTaskNotifyFromISR` / `vTaskNotifyGiveFromISR`

## 任务清单

| Task | Stack (words×4) | Priority | Source | Trigger |
|------|-----------------|----------|--------|---------|
| defaultTask | 512 | Low | freertos.c | Periodic delay |
| gnssTask | 1024 | AboveNormal | app_tasks.c | DMA-idle notify |
| imuTask | 768 | AboveNormal | app_tasks.c | EXTI notify |
| dataSort | 1024 | Normal | app_tasks.c | DataService queue |
| sdWriter | 1024 | BelowNormal | app_tasks.c | SD block ready notify |
| keyTask | 512 | Low | app_tasks.c | EXTI notify |
| ledTask | 512 | Low | app_tasks.c | Periodic delay |
| usartCMD | 768 | Low | app_tasks.c | DMA-idle / log notify |
| uwbPhy | 1024 | AboveNormal | uwb_stack.c | DW1000 IRQ notify |
| uwbLink | 1536 | AboveNormal | uwb_stack.c | PHY evt + poll 50ms |
| uwbApp | 1024 | Normal | uwb_stack.c | Link app-event queue |

## 数据流

```
GNSS (USART3 DMA) ─→ gnssTask ─→ GnssParser ─→ DataService_Send()
IMU  (SPI1 EXTI)  ─→ imuTask  ─→ ImuDevice_ReadRaw ─→ DataService_Send()
UWB  (DW1000 IRQ) ─→ uwbPhy ─→ uwbLink ─→ uwbApp ─→ DataService_Send()
                                                          │
                                             DataService_Queue (64 deep)
                                                          │
                                          AppDataSortTask (window=10, time-ordered)
                                                          │
                                          AppSdWriterTask (double-buffered 16KB, FatFs)
```

所有传感器产生 `AppDataNode` (source + timestamp + union payload)，由 `TimeTimestamp` 统一时间。

## UWB 协议栈 (三层架构)

```
APP/UWB/
  uwb_stack.c        入口: UwbStack_StartFromConfig() 初始化三层并启动线程
  uwb_phy.c/h        PHY: DW1000 硬件控制 (SPI 收发、中断处理、时序)
  uwb_link.c/h       LINK: 帧协议、状态机、调度 (Discovery/Ranging)
  uwb_app.c/h        APP: 测距计算、数据发布 (TWR→AppDataNode→DataService)
  uwb_protocol.c/h   帧编解码 (MAC short header + payload header + body)
  uwb_stack_types.h  全栈共享类型 (帧类型、PHY 命令、共享数据槽等)
  uwb_buffers.c/h    静态内存池、队列声明
  uwb_timestamp.c/h  DW1000 40-bit 时间戳读写与运算
  uwb_device.c/h     DecaDriver 平台适配 (SPI, mutex, IRQ)
  deca_spi.c/h       SPI2 底层读写
  deca_mutex.c       临界区保护
  dw1000port.c/h     reset/延迟/sleep
```

**层间通信 (zero-copy 共享内存池):**
- LINK → PHY: `q_link_phy_cmd` (主命令, depth=1), `q_link_phy_rt_reply` (实时回复, depth=1)
- PHY → LINK: `q_phy_link_evt` (事件 + 共享数据槽指针, depth=PHY_EVT_POOL_NUM)
- LINK → APP: `UwbLink_AppEventQueue()` (Discovery/RangingRaw 事件)

PHY 共享内存池 (`uwb_shared_data_slot_t`): LINK 写帧到池，PHY 读取发送；PHY 收帧写池，LINK 读取处理。所有权通过 `uwb_obj_ctl_t` (owner + state) 跟踪。

**帧类型:** DISCOVERY_REQ/ACK, RANGING_REQ/RESP, DATA_PREP_REQ/ACK, DATA_PULL_REQ, DATA_FRAGMENT, DATA_DONE

**当前状态:** Discovery + Ranging (DS-TWR) 流程可用。DATA_* 帧类型已定义但 LINK 层尚未处理。

**角色:** Tag (主动发起 Discovery/Ranging) vs Anchor/Base (被动响应 RT reply)。

**测距算法 (uwb_app.c):** 双次交换 DS-TWR，收集两次 RangingRaw 后计算 `tof = ((ra*rb - da*db) / (ra+rb+da+db))`，合并窗口 25ms。

## 中断 → 任务映射

| ISR | Pin | Handler | Target Task |
|-----|-----|---------|-------------|
| EXTI0 | PC0 | TimeService_OnPpsIrq | (直接更新时间) |
| EXTI4 | PA4 | AppTasks_NotifyImuIrqFromISR | imuTask |
| USER_KEY | PE2 | AppTasks_NotifyKeyIrqFromISR | keyTask |
| EXTI8 | PD8 | UwbStack_NotifyIrqFromISR → UwbPhy_NotifyIrqFromISR | uwbPhyTask |
| USART3 IDLE | PB10/PB11 | GNSSIdleHandler | gnssTask |
| USART1 IDLE | PA9/PA10 | USART1IdleHandler | usartCMD |

定义在 `APP/task/app_irq.c` (HAL_GPIO_EXTI_Callback) 和 `APP/task/app_tasks.c` (idle handlers)。

## 服务层 (APP/service/)

| Service | Key API | Notes |
|---------|---------|-------|
| ConfigService | Load/Save/Get, GetDefaults, IsValid | Flash 持久化, AppConfig: pan_id, short_addr, role, log_level |
| DataService | Send/Receive | FreeRTOS queue (64×AppDataNode), 统一数据通道 |
| TimeService | GetTimestamp, OnPpsIrq | TIM2 本地时钟 + GNSS PPS 同步, UTC cache |
| LogService | Write/VWrite | app_log_info/warn/error 宏, 写入 USART log slot |
| StorageService | Mount/OpenNextLog/WriteBlock | FatFs on SDMMC1, 日志文件 `uwb-gnss-imu-sampling-N.log` |

## 配置命令 (CONFIG 模式, USART1 460800)

按键开机进入 CONFIG 模式。命令: `read`, `set <pan_hex> <short_hex> <role>`, `set pan|short|role|log <value>`, `save`, `help`。

## 目录规范

```
APP/                 # 自研业务代码
├── UWB/             # UWB 协议栈 (PHY/LINK/APP 三层)
├── app/             # 系统入口 app.c (App_Start)
├── bsp/             # 板级适配 (LED, 按键, SD)
├── common/          # 公共类型 app_types.h
├── device/          # 设备驱动封装 (gnss_parser, imu_device)
├── service/         # 跨模块服务 (config, time, data, storage, log)
└── task/            # 任务定义 (app_tasks, app_irq)
APP-copy/            # 重构前快照（不参与构建）
Core/                # CubeMX 生成（不改）
Drivers/             # HAL + CMSIS（不改）
FATFS/               # FatFs 集成层（不改）
Middlewares/         # FreeRTOS 内核（不改）
Thrid/               # DecaDriver (DW1000), asm330
```

**规则:** 所有自研逻辑在 `APP/`。`main.c`/`freertos.c`/`stm32h7xx_it.c` 仅入口和分发。ISR 保持简洁。中文注释，英文变量/函数名。配置两层: Flash 持久化 + 头文件静态参数。

## 硬件接口速查

| 外设 | 接口 | 引脚 | 用途 |
|------|------|------|------|
| DW1000 | SPI2 | PB12-15, RST=PD9, IRQ=PD8 | UWB 测距 |
| UM960 GNSS | USART3+DMA | PB10/PB11, PPS=PC0 | 定位与时间基准 |
| ASM330 IMU | SPI1 | PA5-7, CS=PC4, IRQ=PA4 | 惯性测量 |
| SD 卡 | SDMMC1 4-bit | PC8-12, PD2, DET=PB5 | 数据存储 |
| 调试串口 | USART1 460800 | PA9/PA10 | 日志与配置命令 |
| 用户按键 | EXTI | PE2 | 模式选择 / 长按复位 |
| LED | GPIO | PE4,PE5,PE6,PC13 | 状态指示 |

## 当前开发状态

- 已完成: GNSS, IMU, 时间同步, 配置管理, 数据排序聚合, SD 存储, 日志服务
- 进行中: UWB 协议栈重构 (Discovery + Ranging DS-TWR 可用; DATA 传输流程未实现)

## 分支说明

- `main`: 主分支
- `UWB-V2`: 当前活跃开发分支, UWB 协议栈重构
