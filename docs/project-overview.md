# UWB 工程概览

本文档用于快速定位代码入口、任务职责和运行数据路径。面向后续维护和调试，不替代协议细节设计文档。

## 1. 工程用途

本工程是基于 `STM32H743VITX` 的多源传感器数据采集固件，运行 `FreeRTOS`。系统完成以下功能：

- `DW1000` UWB：Discovery、DS-TWR 测距、DATA 会话、Anchor 邻近表传输。
- `UM960 GNSS`：BESTNAV 解析、UTC 缓存、PPS 对齐本地时间。
- `ASM330LHH IMU`：数据就绪中断触发，SPI 读取六轴原始数据。
- `SD Card`：统一 ASCII 数据日志，16KB 双缓冲满块写入。
- `USART1`：CONFIG 模式下作为命令串口，RUN 模式下默认输出正式数据行。

## 2. 根目录

```text
UWB/
├── APP/                       # 自研业务代码，主要开发区域
├── Core/                      # STM32CubeMX 生成的启动和外设初始化代码
├── Drivers/                   # STM32 HAL / CMSIS 官方驱动
├── FATFS/                     # FatFs 与 SD 卡文件系统适配
├── Middlewares/               # FreeRTOS 等中间件
├── Thrid/                     # DW1000、ASM330LHH 等第三方驱动
├── build/                     # 编译输出目录
├── docs/                      # 项目文档
├── sampling/                  # 采样数据或测试数据
├── scripts/                   # 日志分析和辅助脚本
├── UWB.ioc                    # STM32CubeMX 工程配置
├── UWB.code-workspace         # VS Code 工作区
├── STM32H743VITX_FLASH.ld     # Flash 链接脚本
└── STM32H743VITX_RAM.ld       # RAM 链接脚本
```

## 3. APP 目录

```text
APP/
├── app/                       # 应用入口和启动模式选择
├── bsp/                       # 板级封装：LED、按键、SD 检测
├── common/                    # 公共类型和 AppDataNode 定义
├── device/                    # GNSS parser、IMU device 封装
├── service/                   # 配置、时间、数据、日志、存储服务
├── task/                      # FreeRTOS 任务和中断分发
└── UWB/                       # UWB 协议栈和 DW1000 适配
```

### 3.1 应用入口

| 文件 | 职责 |
| --- | --- |
| `APP/app/app.c` | `App_Start()` 入口；上电时按键按下进入 CONFIG，否则进入 RUN |
| `APP/app/app.h` | 应用入口声明 |

### 3.2 公共类型

| 文件 | 职责 |
| --- | --- |
| `APP/common/app_types.h` | 角色、运行模式、`AppDataSource`、GNSS/IMU/UWB payload、`AppDataNode` |

当前数据源：

```c
APP_DATA_SRC_GNSS
APP_DATA_SRC_IMU
APP_DATA_SRC_UWB_TWR
APP_DATA_SRC_UWB_ANCHOR_DATA
```

### 3.3 设备层

| 文件 | 职责 |
| --- | --- |
| `APP/device/gnss_parser.c/.h` | UM960 二进制协议解析，提取 BESTNAV 和 UTC |
| `APP/device/imu_device.c/.h` | ASM330LHH 初始化和原始六轴读取 |

### 3.4 服务层

| 文件 | 职责 |
| --- | --- |
| `APP/service/config_service.c/.h` | Flash 持久化配置，默认 PAN/地址/角色/log level |
| `APP/service/time_service.c/.h` | TIM16 20 kHz 单调时间、GNSS UTC 缓存、PPS 同步、`TimeCapture` 解析 |
| `APP/service/data_service.c/.h` | `AppDataNode` 静态队列，队列深度 64，发送时补 `enqueue_seq` |
| `APP/service/storage_service.c/.h` | FatFs 挂载、创建 `gnss-imu-uwb-%04lu.log`、块写入 |
| `APP/service/log_service.c/.h` | 普通调试日志封装 |

### 3.5 任务层

| 文件 | 职责 |
| --- | --- |
| `APP/task/app_tasks.c` | GNSS、IMU、dataSort、sdWriter、key、LED、USART 命令/输出任务 |
| `APP/task/app_tasks.h` | 任务创建、ISR 通知、LED/丢包显示接口 |
| `APP/task/app_irq.c` | `HAL_GPIO_EXTI_Callback()` 中的外部中断分发 |

## 4. UWB 协议栈目录

```text
APP/UWB/
├── uwb_stack.c/.h             # UWB 协议栈启动入口
├── uwb_stack_types.h          # 全栈共享类型、命令、测距结果
├── uwb_phy.c/.h               # PHY 层：DW1000 SPI、IRQ、TX/RX 状态机
├── uwb_link.c/.h              # LINK 层：Discovery/TWR/DATA/ring 帧调度
├── uwb_app.c/.h               # APP 层：测距计算、邻近表、Tag 拉表、数据上报
├── uwb_protocol.c/.h          # 自定义帧编解码
├── uwb_buffers.c/.h           # 层间静态队列
├── uwb_slots.c/.h             # UWB 共享 slot 池
├── uwb_mac_table.c/.h         # 已发现设备表
├── uwb_timestamp.c/.h         # DW1000 40-bit 时间戳工具
├── uwb_device.c/.h            # DW1000 初始化和平台适配
├── uwb_loss_test.c/.h         # Tag 侧丢包统计和 LED 显示命令
├── deca_spi.c/.h              # DW1000 SPI2 底层读写
├── deca_mutex.c               # DecaDriver 临界区适配
├── deca_sleep.c/.h            # DecaDriver 延时适配
├── dw1000port.c/.h            # DW1000 reset / sleep / port 封装
└── bphero_uwb.c/.h            # UWB 兼容封装
```

常用入口：

| 目标 | 优先查看 |
| --- | --- |
| UWB 启动流程 | `APP/UWB/uwb_stack.c` |
| DW1000 IRQ、本地 UWB 时间戳 | `APP/UWB/uwb_phy.c` |
| Discovery/TWR/DATA 调度 | `APP/UWB/uwb_link.c` |
| 测距结果、邻近表、Tag 拉表 | `APP/UWB/uwb_app.c` |
| UWB 帧格式 | `APP/UWB/uwb_protocol.c`、`APP/UWB/uwb_stack_types.h` |
| 丢包统计 | `APP/UWB/uwb_loss_test.c` |
| 共享数据槽 | `APP/UWB/uwb_slots.c` |

## 5. 任务和数据流

### 5.1 任务

| 任务 | 来源 | 触发/周期 | 职责 |
| --- | --- | --- | --- |
| `appDefault` | `APP/task/app_tasks.c` | 1s delay | 占位任务 |
| `gnssTask` | `APP/task/app_tasks.c` | USART3 IDLE 通知 | 处理 DMA 块，解析 BESTNAV，投递 GNSS 节点 |
| `imuTask` | `APP/task/app_tasks.c` | PA4 EXTI 通知 | 读取 IMU 原始数据，投递 IMU 节点 |
| `dataSort` | `APP/task/app_tasks.c` | DataService 队列 | 按时间排序，格式化 ASCII，同时写串口和 SD FIFO |
| `sdWriter` | `APP/task/app_tasks.c` | 16KB block ready | 挂载 SD，创建日志文件，写满块 |
| `keyTask` | `APP/task/app_tasks.c` | PE2 EXTI + 10ms poll | 运行期按键事件，Anchor 短按触发邻近表构建 |
| `ledTask` | `APP/task/app_tasks.c` | 20ms | RUN/CONFIG/丢包/Anchor 初始化 LED 状态 |
| `usartCMD` | `APP/task/app_tasks.c` | 串口 RX/TX 通知 | CONFIG 命令；RUN 数据输出 DMA |
| `uwbPhy` | `APP/UWB/uwb_phy.c` | DW1000 IRQ/命令 | DW1000 收发和时间戳捕获 |
| `uwbLink` | `APP/UWB/uwb_link.c` | PHY 事件/轮询 | UWB 帧调度、DATA 会话转发 |
| `uwbApp` | `APP/UWB/uwb_app.c` | LINK app event + 1ms poll | 测距计算、邻近表、Tag 拉表、上报 DataService |
| `uwbLoss` | `APP/UWB/uwb_loss_test.c` | Tag 角色启动 | 最近 3s 收包率统计和 LED 命令 |

### 5.2 采集数据流

```text
GNSS USART3 DMA/IDLE -> gnssTask -> DataService
IMU  EXTI + SPI1     -> imuTask  -> DataService
UWB  DW1000 IRQ      -> uwbPhy -> uwbLink -> uwbApp -> DataService

DataService -> dataSort(window=10)
            -> format_data_ascii()
            -> USART1 data output
            -> SD FIFO(16KB x 2)
            -> sdWriter -> gnss-imu-uwb-xxxx.log
```

排序键是 `(time_capture.local_tick_20k, enqueue_seq)`。`enqueue_seq` 只用于同 tick 内保持稳定顺序。

## 6. 时间戳入口

| 数据 | 时间捕获点 | 后续处理 |
| --- | --- | --- |
| GNSS | `GNSSIdleHandler()` 中 `TimeService_CaptureNow()` | `gnssTask` 取缓存时间，解析 BESTNAV 后入队 |
| IMU | `AppTasks_NotifyImuIrqFromISR()` 中 `TimeService_CaptureNow()` | `imuTask` 读 SPI 并入队 |
| UWB_TWR | `UwbPhy_NotifyIrqFromISR()` 中 `TimeService_CaptureNow()` | PHY 线程读 DW1000 TX/RX timestamp，APP 计算 `frame_local_tick_20k` 后入队 |
| UWB_ANCHOR_DATA | `tag_start_session()` 中 `TimeService_CaptureNow()` | DATA 表 CRC 验证成功后用会话启动时间发布整表 |

`dataSort` 输出时调用 `TimeService_ResolveCapture()`，只把已捕获的 `TimeCapture` 转成 UTC/week 和 `mono_ms`，不会重新取当前时间。

## 7. 日志和串口

正式数据日志文件名：

```text
gnss-imu-uwb-0001.log
gnss-imu-uwb-0002.log
...
```

正式数据行格式：

```text
utc_week,utc_ms,utc_valid,mono_ms,msg_type,<payload...>
```

支持的 `msg_type`：

- `GNSS`
- `IMU`
- `UWB_TWR`
- `UWB_ANCHOR_DATA`

RUN 模式下 `APP_USART_RUN_DATA_ONLY = 1`，`USART1` 默认只输出 `dataSort` 生成的数据行。CONFIG 模式下 `USART1` 接收命令并输出命令结果。

详细字段见 [docs/plan/plan-v6/06-data-log-ascii-format.md](plan/plan-v6/06-data-log-ascii-format.md)。

## 8. CONFIG 使用

进入方式：按住 `USER_KEY` 上电或复位。

串口：`USART1`, `460800, 8N1`。

命令：

```text
read
set <pan_hex> <short_hex> <role>
set pan <hex>
set short <hex>
set role <0|1>
set log <level>
save
help
```

角色：

| role | 含义 |
| --- | --- |
| `0` | Tag |
| `1` | Anchor |

默认配置在 `APP/service/config_service.c`：

```text
pan=0xF0F0 short=0x0034 role=1 log=2
```

`save` 成功后需要手动复位或重新上电使 RUN 模式使用新配置。

## 9. 外设和中断映射

| 中断/事件 | 来源 | 应用处理 |
| --- | --- | --- |
| `EXTI0` | GNSS PPS, PC0 | `TimeService_OnPpsIrq()` |
| `EXTI4` | IMU DRDY, PA4 | `AppTasks_NotifyImuIrqFromISR()` |
| `EXTI9_5` | DW1000 IRQ, PD8 | `UwbStack_NotifyIrqFromISR()` |
| `USER_KEY` | PE2 | `AppTasks_NotifyKeyIrqFromISR()` |
| `USART3_IRQHandler` | GNSS DMA/IDLE | `GNSSIdleHandler()` |
| `USART1_IRQHandler` | 命令串口 DMA/IDLE | `USART1IdleHandler()` |
| `TIM16_IRQHandler` | 20 kHz local clock overflow | `TimeService_OnTim16Overflow()` |

中断入口在 `Core/Src/stm32h7xx_it.c`，GPIO EXTI 分发在 `APP/task/app_irq.c`。

## 10. 常见定位路径

| 想找的内容 | 优先查看 |
| --- | --- |
| 上电到任务创建 | `Core/Src/main.c`、`Core/Src/freertos.c`、`APP/app/app.c`、`APP/task/app_tasks.c` |
| CONFIG/RUN 模式 | `APP/app/app.c`、`APP/task/app_tasks.c` |
| 串口命令 | `APP/task/app_tasks.c` 的 `handle_config_line()` |
| GNSS 解析和入队 | `APP/device/gnss_parser.c`、`APP/task/app_tasks.c` |
| IMU 采样 | `APP/device/imu_device.c`、`APP/task/app_tasks.c` |
| 时间服务 | `APP/service/time_service.c` |
| 数据排序和 ASCII 输出 | `APP/task/app_tasks.c` 的 `AppDataSortTask()`、`format_node_ascii()` |
| SD 写入 | `APP/service/storage_service.c`、`APP/task/app_tasks.c` |
| UWB 测距结果 | `APP/UWB/uwb_app.c` 的 `handle_twr_exchange()`、`publish_range_result()` |
| Anchor 邻近表 | `APP/UWB/uwb_app.c` 的 prox build / table encode 相关函数 |
| Tag 拉表 DATA 会话 | `APP/UWB/uwb_app.c` 的 `tag_start_session()`、`handle_tag_frag()` |
| UWB LINK 调度 | `APP/UWB/uwb_link.c` |
| UWB PHY IRQ | `APP/UWB/uwb_phy.c` |
| 丢包统计 | `APP/UWB/uwb_loss_test.c` |

## 11. 构建约束

- 工具链：EIDE + GCC ARM。
- C 标准：C11。
- 运行库：newlib-nano。
- 不要在 `printf`/`snprintf` 中使用 `%llu`/`%lld`；需要 64-bit 输出时拆成 32-bit 或转成十进制字符串。
- FreeRTOS heap 配置见 `Core/Inc/FreeRTOSConfig.h`。
