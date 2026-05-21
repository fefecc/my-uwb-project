# UWB 多源传感器数据采集系统

基于 `STM32H743VITX` 的 FreeRTOS 多源数据采集固件。系统集成 `DW1000` UWB 测距、`UM960` GNSS 定位授时、`ASM330LHH` IMU 采样，并把 GNSS/IMU/UWB 数据统一时间戳、排序后输出到 `USART1` 和 SD 卡。

当前代码已完成运行链路：配置管理、GNSS/IMU 采集、UWB Discovery/DS-TWR、Anchor 邻近表构建、Tag 侧 DATA 拉表、丢包统计、统一 ASCII 数据日志、SD 双缓冲写入。

## 硬件接口

| 模块 | 接口 | 引脚 | 用途 |
| --- | --- | --- | --- |
| DW1000 | SPI2 | PB12-PB15, RST=PD9, IRQ=PD8 | UWB 收发和测距 |
| UM960 GNSS | USART3 + DMA/IDLE | PB10/PB11, PPS=PC0 | BESTNAV 定位和 UTC/PPS 授时 |
| ASM330LHH IMU | SPI1 | PA5-PA7, CS=PC4, IRQ=PA4 | 六轴惯性数据 |
| SD 卡 | SDMMC1 4-bit | PC8-PC12, PD2, DET=PB5 | 数据日志存储 |
| 调试/数据串口 | USART1 | PA9/PA10 | 配置命令、运行数据输出 |
| 用户按键 | EXTI | PE2 | 上电模式选择、运行期功能触发 |
| LED | GPIO | PE4, PE5, PE6, PC13 | 运行状态、初始化/丢包显示 |

`USART1` 参数固定为 `460800, 8N1`。

## 运行模式

### CONFIG 模式

按住 `USER_KEY` 上电或复位进入 CONFIG 模式。串口会输出：

```text
config mode
cmd: read | set <pan_hex> <short_hex> <role> | set pan|short|role|log <value> | save | help
```

常用命令：

```text
read
set F0F0 0021 1
set pan F0F0
set short 0101
set role 0
set log 2
save
help
```

角色约定：

| role | 含义 |
| --- | --- |
| `0` | Tag |
| `1` | Anchor |

`set` 命令只修改当前 RAM 中的配置，执行 `save` 后才写入 Flash。保存后手动复位或重新上电进入 RUN 模式。

### RUN 模式

松开 `USER_KEY` 上电或复位进入 RUN 模式。系统会启动 UWB/GNSS/IMU、数据排序线程、SD 写线程和串口输出线程。

RUN 模式下 `USART1` 默认只输出正式数据行，普通调试日志被屏蔽，避免干扰采样记录。SD 卡中生成递增文件：

```text
gnss-imu-uwb-0001.log
gnss-imu-uwb-0002.log
...
gnss-imu-uwb-9999.log
```

文件以 `FA_CREATE_NEW` 创建，不覆盖已有实验数据。当前 SD 写入按 16KB 双缓冲满块落盘，实验结束前最后不足 16KB 的尾部数据可能仍在 RAM 中。

## 快速使用

1. 将 SD 卡格式化为板端 FatFs 可识别的文件系统并插入设备。
2. 按住 `USER_KEY` 上电进入 CONFIG 模式。
3. 配置 Anchor，例如：

```text
set F0F0 0021 1
save
```

4. 配置 Tag，例如：

```text
set F0F0 0101 0
save
```

5. 所有设备重新上电，松开 `USER_KEY` 进入 RUN 模式。
6. Tag 自动进行 UWB 测距、GNSS/IMU 采集和 SD/串口数据输出。
7. Anchor 运行期短按按键会触发邻近表构建和 ring 初始化流程；Tag 会在满足拉取条件时通过 DATA 会话获取 Anchor 表并写入统一日志。

Anchor 邻近表功能要求 Anchor 短地址在 `0x0020` 到 `0x0050` 范围内。

## 数据日志格式

所有正式数据都先进入 `DataService`，再由 `dataSort` 按 `(time_capture.local_tick_20k, enqueue_seq)` 排序后输出。SD 文件和 `USART1` 使用同一套 ASCII formatter，内容一致。

通用 CSV 前缀：

```text
utc_week,utc_ms,utc_valid,mono_ms,msg_type,<payload...>\r\n
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `utc_week` | GNSS week；未同步时为 `0` |
| `utc_ms` | week 内毫秒；未同步时为 `0` |
| `utc_valid` | `1` 表示 UTC 有效，`0` 表示无效 |
| `mono_ms` | 本地单调时间，来自 20 kHz TIM16 tick，保留 3 位小数 |
| `msg_type` | `GNSS`、`IMU`、`UWB_TWR`、`UWB_ANCHOR_DATA` |

### GNSS

```text
utc_week,utc_ms,utc_valid,mono_ms,GNSS,lat,lon,hgt,datum_id,lat_std,lon_std,hgt_std,pos_status,pos_type,diff_age,sol_age,svs_tracked,svs_in_sol
```

### IMU

```text
utc_week,utc_ms,utc_valid,mono_ms,IMU,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
```

### UWB_TWR

```text
utc_week,utc_ms,utc_valid,mono_ms,UWB_TWR,anchor_id,tag_id,exchange_seq,status_flags,distance_m,rx_pacc,fp_index,fp_ampl1,fp_ampl2,fp_ampl3,std_noise,max_noise
```

### UWB_ANCHOR_DATA

一张 Anchor 邻近表进入排序线程时仍是一个事件，输出时再按表项拆成多行。所有表项共用同一个 DATA 会话启动时间。

```text
utc_week,utc_ms,utc_valid,mono_ms,UWB_ANCHOR_DATA,source_anchor_id,table_seq,entry_index,entry_count,peer_anchor_id,dist_cm,dist_std_cm,avg_pacc,avg_fp_index,avg_fp_ampl1,avg_fp_ampl2,avg_fp_ampl3,avg_std_noise,avg_max_noise,avg_rx_power_dbm_x100,avg_fp_power_dbm_x100,samples,quality,flags,rx_error_flags,lde_status,total_len,total_frags,table_crc
```

示例：

```text
2420,345678901,1,6172839.450,IMU,12,-8,16384,31,-22,4
2420,345678920,1,6172858.000,GNSS,31.230000000,121.470000000,12.3450,61,0.0120,0.0130,0.0210,0,56,0.000,0.000,28,21
2420,345678940,1,6172877.000,UWB_TWR,0x0021,0x0101,1024,0x0007,3.422,812,71,2230,2198,2251,121,311
```

## 时间戳策略

- GNSS：在 `USART3` IDLE 中断中捕获本地时间，GNSS 任务中解析 BESTNAV 并入队。
- IMU：在 IMU DRDY EXTI 中断中捕获本地时间，IMU 任务中读取 SPI 数据并入队。
- UWB_TWR：在 DW1000 IRQ 中断中捕获本地时间，PHY/LINK/APP 线程中读取 DW1000 40-bit 时间戳并计算测距结果。
- UWB_ANCHOR_DATA：在 Tag DATA 会话启动时捕获时间，CRC 验证成功后用该时间发布整张 Anchor 表。

日志输出时只解析已有 `TimeCapture`，不会使用 formatter 运行时的时间覆盖事件时间。

## UWB 功能

- Tag 主动 Discovery，Anchor 按时隙响应。
- Tag 侧 4 个 RX slot 多 Anchor 接收。
- DS-TWR 测距，支持短/长间隔双次交换分类。
- 测距质量字段输出：`rx_pacc`、`fp_index`、`fp_ampl1/2/3`、`std_noise`、`max_noise`。
- Anchor 侧维护本地邻近表，最多 8 个条目。
- Anchor ring 初始化：短按 Anchor 按键触发 3s 本机测距窗口，并向下一 Anchor 传播初始化通知。
- Tag 侧根据测距结果触发 DATA 会话，拉取 Anchor 邻近表。
- Tag 侧丢包统计线程按最近 3s 窗口计算最佳 Anchor/slot 收包率，并通过 LED 显示。

## 主要数据流

```text
GNSS USART3 DMA/IDLE -> gnssTask -> DataService
IMU  EXTI + SPI1     -> imuTask  -> DataService
UWB  DW1000 IRQ      -> uwbPhy -> uwbLink -> uwbApp -> DataService

DataService -> dataSort -> ASCII formatter -> USART1 data slot
                                      └──-> SD FIFO -> sdWriter -> gnss-imu-uwb-xxxx.log
```

## 构建和烧录

- 工具链：EIDE + GCC ARM，配置在 `.eide/eide.yml`
- CubeMX 工程：`UWB.ioc`
- 编译输出：`build/Debug/`
- 链接脚本：`STM32H743VITX_FLASH.ld`
- 烧录：OpenOCD/ST-Link 或 J-Link，Flash 基地址 `0x08000000`
- 运行库：newlib-nano，代码中不要在 `printf`/`snprintf` 中使用 `%llu`/`%lld`

## 目录结构

```text
APP/                 自研业务代码
├── UWB/             UWB 三层协议栈和 DW1000 适配
├── app/             应用入口和模式选择
├── bsp/             LED、按键、SD 检测等板级封装
├── common/          公共类型和数据节点定义
├── device/          GNSS parser、IMU device
├── service/         config、time、data、storage、log 服务
└── task/            FreeRTOS 任务和中断分发
Core/                STM32CubeMX 生成代码
Drivers/             STM32 HAL / CMSIS
FATFS/               FatFs 集成
Middlewares/         FreeRTOS
Thrid/               DW1000 DecaDriver、ASM330LHH 驱动
docs/                设计、计划和工程说明文档
scripts/             日志分析和辅助脚本
```

## 参考文档

| 文档 | 内容 |
| --- | --- |
| [docs/project-overview.md](docs/project-overview.md) | 工程导航和代码入口 |
| [docs/plan/plan-v6/06-data-log-ascii-format.md](docs/plan/plan-v6/06-data-log-ascii-format.md) | 当前 ASCII 数据日志格式 |
| [docs/designed/uwb-designed.md](docs/designed/uwb-designed.md) | UWB 协议栈设计说明 |
| [docs/designed/designed.md](docs/designed/designed.md) | 系统总体设计说明 |
