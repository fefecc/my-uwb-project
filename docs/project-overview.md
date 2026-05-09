# UWB 工程概览

本文档只描述工程用途、目录分布和主要文件职责，用于后续快速定位文件。

## 1. 工程用途

本工程是基于 `STM32H743VITX` 的多传感器数据采集系统，运行 `FreeRTOS`。系统集成以下模块：

- `DW1000`：UWB 测距，使用 `SPI2`，中断线为 `PD8`
- `UM960 GNSS`：定位和 UTC 时间输入，使用 `USART3 + DMA/IDLE`，PPS 为 `PC0`
- `ASM330LHH IMU`：惯性数据采样，使用 `SPI1`，数据就绪中断为 `PA4`
- `SD Card`：数据存储，使用 `SDMMC1`
- `USART1`：配置命令和日志输出

工程使用 `STM32CubeMX` 生成底层初始化代码，使用 `EIDE + GCC ARM` 构建。

## 2. 根目录分布

```text
UWB/
├── APP/                       # 自研应用代码，主要业务逻辑都在这里
├── APP-copy/                  # 重构前旧版应用代码，仅作参考
├── Core/                      # STM32CubeMX 生成的启动和外设初始化代码
├── Drivers/                   # STM32 HAL / CMSIS 官方驱动
├── FATFS/                     # FatFS 与 SD 卡文件系统适配
├── Middlewares/               # FreeRTOS 等中间件
├── Thrid/                     # 第三方芯片驱动和算法库
├── build/                     # 编译输出目录
├── docs/                      # 项目文档
├── readme/                    # 补充说明材料
├── sampling/                  # 采样数据或测试数据
├── UWB.ioc                    # STM32CubeMX 工程配置
├── UWB.code-workspace         # VS Code 工作区
├── STM32H743VITX_FLASH.ld     # Flash 链接脚本
└── STM32H743VITX_RAM.ld       # RAM 链接脚本
```

## 3. APP 目录

`APP/` 是主要开发区域，业务代码按模块拆分。

```text
APP/
├── app/                       # 应用入口和运行模式选择
├── bsp/                       # 板级封装：LED、按键、SD 检测
├── common/                    # 公共类型和全局应用定义
├── device/                    # 传感器和外设设备层封装
├── service/                   # 公共服务：配置、时间、数据、日志、存储
├── task/                      # FreeRTOS 任务和中断分发
└── UWB/                       # UWB 协议栈和 DW1000 适配
```

### 3.1 应用入口

| 文件 | 用途 |
|---|---|
| `APP/app/app.c` | 应用启动入口，选择配置模式或运行模式 |
| `APP/app/app.h` | 应用入口声明 |

### 3.2 板级支持

| 文件 | 用途 |
|---|---|
| `APP/bsp/bsp_led.c/.h` | LED 封装 |
| `APP/bsp/bsp_key.c/.h` | 用户按键封装 |
| `APP/bsp/bsp_sd.c/.h` | SD 卡检测和板级适配 |

### 3.3 公共定义

| 文件 | 用途 |
|---|---|
| `APP/common/app.h` | 应用公共头 |
| `APP/common/app_types.h` | 应用层通用类型，包含配置、数据节点、角色等定义 |

### 3.4 设备层

| 文件 | 用途 |
|---|---|
| `APP/device/gnss_parser.c/.h` | GNSS 数据解析 |
| `APP/device/imu_device.c/.h` | ASM330LHH IMU 设备封装 |

### 3.5 服务层

| 文件 | 用途 |
|---|---|
| `APP/service/config_service.c/.h` | 配置加载、校验、Flash 读写 |
| `APP/service/time_service.c/.h` | 本地时间、UTC 缓存、PPS 同步 |
| `APP/service/data_service.c/.h` | GNSS / IMU / UWB 数据节点队列 |
| `APP/service/storage_service.c/.h` | SD 卡挂载、日志文件创建和写入 |
| `APP/service/log_service.c/.h` | 串口日志服务 |

### 3.6 任务层

| 文件 | 用途 |
|---|---|
| `APP/task/app_tasks.c` | GNSS、IMU、数据排序、SD 写入、按键、LED、串口等任务 |
| `APP/task/app_tasks.h` | 任务创建和任务通知接口 |
| `APP/task/app_irq.c` | GPIO EXTI 和外设中断到应用任务的分发 |

## 4. UWB 协议栈目录

`APP/UWB/` 包含 DW1000 适配和 UWB 三层协议栈。

```text
APP/UWB/
├── uwb_stack.c/.h             # UWB 协议栈启动入口
├── uwb_stack_types.h          # UWB 共享类型、帧类型、队列事件、配置
├── uwb_phy.c/.h               # PHY 层，直接管理 DW1000 收发和 IRQ
├── uwb_link.c/.h              # LINK 层，负责协议调度和帧事件处理
├── uwb_app.c/.h               # UWB APP 层，处理测距结果和上报
├── uwb_protocol.c/.h          # UWB 自定义帧编解码
├── uwb_buffers.c/.h           # PHY / LINK / APP 层间队列
├── uwb_mac_table.c/.h         # 已发现设备 MAC 表
├── uwb_timestamp.c/.h         # DW1000 40-bit 时间戳工具
├── uwb_device.c/.h            # DW1000 设备初始化和适配
├── deca_spi.c/.h              # DW1000 SPI2 底层读写
├── deca_mutex.c               # DecaDriver 临界区适配
├── deca_sleep.c/.h            # DecaDriver 延时适配
├── dw1000port.c/.h            # DW1000 reset / sleep / port 封装
└── bphero_uwb.c/.h            # UWB 相关兼容封装
```

常用查找入口：

| 目标 | 文件 |
|---|---|
| 查 UWB 启动流程 | `APP/UWB/uwb_stack.c` |
| 查 PHY 收发、IRQ、DW1000 状态 | `APP/UWB/uwb_phy.c` |
| 查发现、测距、数据传输调度 | `APP/UWB/uwb_link.c` |
| 查 UWB 到应用数据上报 | `APP/UWB/uwb_app.c` |
| 查帧格式和 payload 编解码 | `APP/UWB/uwb_protocol.c`、`APP/UWB/uwb_stack_types.h` |
| 查层间队列 | `APP/UWB/uwb_buffers.c` |
| 查 SPI 读写 | `APP/UWB/deca_spi.c` |

## 5. CubeMX 生成代码

`Core/` 主要由 STM32CubeMX 生成，一般只作为外设入口和中断入口使用。

| 文件 | 用途 |
|---|---|
| `Core/Src/main.c` | MCU 初始化入口 |
| `Core/Src/freertos.c` | FreeRTOS 默认任务入口 |
| `Core/Src/stm32h7xx_it.c` | Cortex / 外设中断入口 |
| `Core/Src/gpio.c`、`Core/Inc/gpio.h` | GPIO 初始化 |
| `Core/Src/spi.c`、`Core/Inc/spi.h` | SPI 初始化 |
| `Core/Src/usart.c`、`Core/Inc/usart.h` | USART 初始化 |
| `Core/Src/sdmmc.c`、`Core/Inc/sdmmc.h` | SDMMC 初始化 |
| `Core/Src/tim.c`、`Core/Inc/tim.h` | 定时器初始化 |
| `Core/Inc/FreeRTOSConfig.h` | FreeRTOS 配置 |

## 6. FATFS 目录

| 文件 | 用途 |
|---|---|
| `FATFS/App/fatfs.c/.h` | FatFS 应用层入口 |
| `FATFS/Target/sd_diskio.c/.h` | SD 卡磁盘 I/O 适配 |
| `FATFS/Target/bsp_driver_sd.c/.h` | SD 卡 BSP 驱动 |
| `FATFS/Target/fatfs_platform.c/.h` | FatFS 平台适配 |
| `FATFS/Target/ffconf.h` | FatFS 配置 |

## 7. 第三方库

| 路径 | 用途 |
|---|---|
| `Thrid/decadriver/` | DW1000 官方驱动、寄存器定义、参数表 |
| `Thrid/decadriver/deca_device.c` | DW1000 驱动主体 |
| `Thrid/decadriver/deca_device_api.h` | DW1000 API 声明 |
| `Thrid/decadriver/deca_regs.h` | DW1000 寄存器定义 |
| `Thrid/decadriver/trilateration.c/.h` | 三角定位算法 |
| `Thrid/asm330/asm330lhh_reg.c/.h` | ASM330LHH 官方寄存器驱动 |

## 8. 文档目录

| 路径 | 用途 |
|---|---|
| `docs/project-overview.md` | 工程导航和文件分布 |
| `docs/designed/designed.md` | 系统设计说明 |
| `docs/designed/uwb-designed.md` | UWB 协议栈设计说明 |
| `docs/log/` | 工作日志 |
| `docs/User-Manual/` | 芯片手册和规格书 |

## 9. 常见定位路径

| 想找的内容 | 优先查看 |
|---|---|
| 系统从上电到创建任务 | `Core/Src/main.c`、`Core/Src/freertos.c`、`APP/app/app.c`、`APP/task/app_tasks.c` |
| 配置模式和运行模式 | `APP/app/app.c`、`APP/service/config_service.c`、`APP/task/app_tasks.c` |
| 串口命令和日志 | `APP/task/app_tasks.c`、`APP/service/log_service.c`、`Core/Src/usart.c` |
| GNSS 解析 | `APP/device/gnss_parser.c`、`APP/task/app_tasks.c` |
| IMU 采样 | `APP/device/imu_device.c`、`APP/task/app_tasks.c` |
| 时间同步 | `APP/service/time_service.c`、`APP/task/app_irq.c` |
| 数据队列和排序 | `APP/service/data_service.c`、`APP/task/app_tasks.c` |
| SD 写入 | `APP/service/storage_service.c`、`APP/bsp/bsp_sd.c`、`FATFS/` |
| UWB 协议栈 | `APP/UWB/uwb_stack.c`、`APP/UWB/uwb_phy.c`、`APP/UWB/uwb_link.c`、`APP/UWB/uwb_app.c` |
| DW1000 底层驱动 | `APP/UWB/deca_spi.c`、`APP/UWB/dw1000port.c`、`Thrid/decadriver/` |
| 外部中断分发 | `APP/task/app_irq.c`、`Core/Src/stm32h7xx_it.c` |

