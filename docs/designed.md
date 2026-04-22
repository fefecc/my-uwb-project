# UWB Project README Lite

## 1. 硬件与接口分配

### 1.1 硬件总体说明

本工程基于 `STM32H743VITX`，用于完成以下几类功能：

- `DW1000` UWB 测距
- `GNSS` 数据接收与时间基准输入
- `IMU` 传感器采样
- `SD` 卡数据存储
- 串口调试与配置

### 1.2 硬件接口框图

```text
+----------------------------------------------------------+
|                     STM32H743VITX                        |
|                                                          |
|  USART1  <--------------------> 配置 / 日志串口          |
|  USART3  <--------------------> GNSS 主数据输入          |
|  UART4   <--------------------> 预留调试串口             |
|                                                          |
|  SPI1    <--------------------> ASM330 IMU               |
|  SPI2    <--------------------> DW1000                   |
|  SDMMC1  <--------------------> SD Card                  |
|                                                          |
|  EXTI PA4 <-------------------- IMU DRDY                 |
|  EXTI PD8 <-------------------- DW1000 IRQ               |
|  EXTI PC0 <-------------------- GNSS PPS                 |
|  EXTI PE2 <-------------------- USER_KEY                 |
|  PB5      <-------------------- SD DET                   |
|                                                          |
|  PE4/PE5/PE6/PC13 -----------> LED0/LED1/LED2/LED3       |
+----------------------------------------------------------+
```

### 1.3 外设分配

- `USART1`
  - 调试与配置串口
  - 后续配置模式下的主要交互接口

- `USART3`
  - `GNSS` 主数据输入串口
  - 采用 `DMA + IDLE` 接收

- `UART4`
  - 预留调试串口

- `SPI1`
  - 连接 `ASM330`

- `SPI2`
  - 连接 `DW1000` 

- `SDMMC1`
  - 连接 `SD` 卡

### 1.4 外部中断与辅助信号

- `PA4`：IMU 数据就绪
- `PD8`：DW1000 中断
- `PC0`：GNSS `PPS`
- `PE2`：用户按键
- `PB5`：SD 卡检测

### 1.5 指示灯说明

- `LED0`：`PE4`，低电平点亮
- `LED1`：`PE5`，低电平点亮
- `LED2`：`PE6`，高电平点亮
- `LED3`：`PC13`，高电平点亮

当前工程中 4 个灯后续不仅用于运行指示，也用于 `Boot` / 配置模式状态表达。

需要特别注意的是，4 个灯的有效电平并不统一：

- `LED0`、`LED1` 为低电平点亮
- `LED2`、`LED3` 为高电平点亮

因此，后续在设计灯功能模块时，不能直接在业务层使用“拉高即亮”或“拉低即亮”的假设，建议在 `LED` 模块内部统一封装开灯、关灯和闪烁接口，对外屏蔽实际电平差异。

---

## 2. 工程框架

### 2.1 工程目录层级

```text
.
├─ APP                // 自研代码主目录
│  ├─ app            // 业务入口与角色逻辑
│  ├─ bsp            // 板级适配
│  ├─ common         // 公共定义与公共工具
│  ├─ device         // 设备适配封装
│  ├─ service        // 配置、时间、存储等公共服务
│  └─ task           // FreeRTOS 任务
├─ APP-copy          // 重构前 APP 代码备份
├─ Core              // CubeMX 生成代码
├─ Drivers           // HAL / CMSIS 官方驱动
├─ FATFS             // CubeMX 生成的 FatFs 集成层
├─ Middlewares       // FreeRTOS / FatFs 中间件
├─ Thrid             // 第三方驱动源码
├─ UWB.ioc           // CubeMX 工程文件
├─ .mxproject
└─ .eide
   └─ eide.yml
```

### 2.2 框架原则

- `Core/`、`Drivers/`、`Middlewares/`、`FATFS/`、`Thrid/` 原则上不直接修改
- `APP/` 是主要重构区域
- `APP-copy/` 用于保存重构前源码，便于后续回溯
- 业务逻辑、设备适配、公共服务、线程组织分开维护

---

## 3. 代码框架

### 3.1 代码总体划分

当前代码框架按高层逻辑分为三部分：

1. `Boot`
2. `配置模式`
3. `运行模式`

### 3.2 代码流程框图

```text
+--------------------------------------------------+
|                    上电 / 复位                   |
+--------------------------------------------------+
                        |
                        v
+--------------------------------------------------+
|                     Boot                         |
|  - 系统进入启动阶段                              |
|  - 4 个指示灯全部熄灭                            |
|  - 检测 USER_KEY 是否按下                        |
+--------------------------------------------------+
                        |
             +----------+----------+
             |                     |
             | 按键按下            | 按键未按下
             v                     v
+----------------------------------+   +----------------------------------+
|            配置模式              |   |            运行模式              |
|  - 进入 RTOS 环境                |   |  - 读取基础配置                  |
|  - LED0~LED3 顺序闪烁            |   |  - 读取静态外参                  |
|  - USART1 配置交互               |   |  - 启动各业务线程                |
|  - 写入 Flash 基础配置           |   |  - 进入正常采集与存储流程        |
+----------------------------------+   +----------------------------------+
```

### 3.3 Boot

`Boot` 是系统上电后的统一入口状态。

当前先定义以下行为：

- 上电后进入 `Boot`
- 4 个灯全部熄灭
- 检测 `USER_KEY`
- 按键按下则进入配置模式
- 按键未按下则进入运行模式

### 3.4 配置模式

配置模式用于完成基础设备配置的交互和写入。

当前流程定义为：

- 进入 `RTOS`
- 通过 `USART1` 接收配置命令
- 配置模式下 `LED0 -> LED1 -> LED2 -> LED3` 依次闪烁
- 间隔为 `200 ms`
- 串口只对基础设备配置开放
- 接收到写入命令后，将基础配置写入 `Flash`

当前配置设计拆分为两类：

- 基础设备配置
  - 写入 `Flash`
  - 包括短 `PAN`、`ID`、角色 `Anchor/Tag`
  - 运行模式下作为 UWB 模组的主要配置来源

- 静态外参
  - 保存在头文件中
  - 作为静态变量由系统直接调用
  - 不通过串口写入

同时保留一份默认静态基础配置，用于 Flash 中没有有效配置时的降级启动。

### 3.5 运行模式

运行模式是系统正常工作路径。

之后会在后面添加

- 读取 `Flash` 中的基础设备配置
- 如果 `Flash` 中没有有效基础配置，则使用工程内默认静态基础配置
- 读取工程中的静态外参
- 根据最终生效的角色进入对应运行流程
- 将最终生效的基础配置传给 UWB 模组
- 启动各业务线程
- 进入采样、处理和存储阶段

这里的“最终生效的基础配置”指：

- 优先使用 `Flash` 中校验通过的配置
- 如果 `Flash` 配置不存在、版本不匹配、长度不匹配或角色非法，则使用默认静态配置

UWB 模组使用的 `PAN ID`、短地址、角色 `Anchor/Tag` 等基础参数，都来自这份最终生效的配置。也就是说，UWB 模组不单独维护另一套基础配置来源。


---

## 4. RTOS 与线程设计

### 4.1 线程设计目标

这一章定义当前工程的 RTOS 线程组织方式。虽然这是精简版文档，但这里仍然保留较完整的设计信息，因为后续代码重构大概率会直接参考这一章落地。

当前线程设计的核心目标如下：

- 将“系统监控”“串口命令”“传感器采样”“数据整理”“实际写卡”“按键处理”“灯控制”明确分开
- 采样线程只负责采样、时间戳打包和投递，不直接处理存储
- 整理线程只负责排序、ASCII 转换和缓存管理
- 写卡线程只负责 SD 检测、挂载、文件管理和写盘
- LED 控制统一下沉到 `ledTask`
- 按键事件统一下沉到 `keyTask`
- `keyTask` 采用订阅发布方式分发按键事件
- `ledTask` 采用任务通知方式接收其他线程下发的灯状态命令

### 4.2 线程总览

当前计划中的主要线程如下：

- `defaultTask`
  - 系统监控线程
  - 负责线程状态监控
  - 负责尽早初始化时间服务和 `TIM16` 本地连续时钟
  - 初始化阶段优先级较高，注册完成后降为最低优先级

- `usartCMDTask`
  - 配置模式下负责串口命令解析
  - 运行模式下负责日志输出调度

- `GNSSTask`
  - GNSS 数据接收与解析线程
  - 保留原始数据精度，不做人为截断
  - 负责更新时间缓存

- `IMUTask`
  - IMU 采样线程
  - 由中断触发采样

- 数据接收 / 整理线程
  - 从消息队列接收数据节点
  - 执行小范围排序和缓存管理
  - 完成 ASCII 组包

- `SD` 写入线程
  - 只负责 SD 检测、挂载、文件管理和写卡

- `keyTask`
  - 按键采集线程
  - 负责长按、短按判定
  - 维护按键事件订阅表
  - 按订阅关系向其他线程分发按键事件

- `ledTask`
  - 灯控制线程
  - 可接收上述所有线程下发的灯命令
  - 通过任务通知统一执行灯效

UWB 协议栈的线程模型和重构边界不在本文展开，统一以 [uwb-designed.md](uwb-designed.md) 为准。

### 4.3 线程整体关系框图

```text
+-------------------+
|    defaultTask    |
| 监控 / 时基维护   |
+-------------------+

+-------------------+      +-------------------+
| GNSSTask          |      | IMUTask           |
| GNSS采样与解析    |      | IMU采样           |
+-------------------+      +-------------------+
          \                        |
           \                       |
            \                      |
             v                     v
                +-----------------------------------+
                |     统一消息队列 / 数据节点流      |
                +-----------------------------------+
                                   |
                                   v
                +-----------------------------------+
                |      数据接收 / 整理线程          |
                |  小范围排序 / ASCII组包 / 缓存管理 |
                +-----------------------------------+
                                   |
                                   v
                +-----------------------------------+
                |         双缓冲 / FIFO区           |
                +-----------------------------------+
                                   |
                                   v
                +-----------------------------------+
                |          SD 写入线程              |
                +-----------------------------------+

+-------------------+      +-------------------+
|   keyTask         | ---> |    usartCMDTask   |
| 长按 / 短按检测   |      | 配置命令 / 日志输出|
| 订阅表 / 事件分发 | ---> |    其他订阅线程    |
+-------------------+      +-------------------+

所有业务线程 / 系统线程
├─ defaultTask
├─ usartCMDTask
├─ GNSSTask
├─ IMUTask
├─ 数据整理线程
├─ SD写入线程
└─ keyTask
          |
          v
+-------------------+
|    ledTask        |
| 通知驱动灯效执行  |
+-------------------+
```

### 4.4 时间与时基设计

时间服务作为独立公共服务实现，不直接写死在 `defaultTask`、`GNSSTask` 或其他业务线程里。

当前时间获取的硬件基础仍然是 `TIM16 + GNSS`：

- `TIM16` 生成本地连续时钟
- `GNSS` 提供 UTC 时间，用来同步本地 UTC 时钟
- `PPS` 只作为 GNSS UTC 与本地连续时钟之间的整秒对齐边沿，不单独生成时间

当前时间结构需要明确拆成三层：

- 外部输入 UTC 时钟
  - 当前来源为 `GNSS` 解析出的 UTC
  - 它是本地 UTC 时钟的校准来源
  - GNSS 解析线程只把 UTC 写入缓存
  - 不直接作为业务线程读取的最终时间

- 本地 UTC 时钟
  - 由时间服务维护
  - 通过 GNSS UTC 进行同步
  - 通过 PPS 边沿把 GNSS UTC 锁定到本地连续时钟的具体时刻
  - 同步完成后，基于本地连续时钟继续向前推进
  - 对外带 `valid` / `sync_state` 标志，用于说明当前 UTC 是否可信

- 本地连续时钟
  - 由 `TIM16` 连续推进
  - 上电后从 `0` 或初始化基准开始单调递增
  - 不因为外部 UTC 修正而回拨或跳变
  - 始终可用，用于采样排序、耗时计算、数据连续性判断

也就是说，上层线程以后不要直接读 `TIM16`、不要直接读 GNSS 缓存，而是统一通过时间服务获取：

- 完整时间戳结构
- 或只获取本地连续时钟

#### 时间源组成

- `TIM16`
  - 作为本地连续时钟的唯一生成来源
  - 当前约定为 `1 s` 溢出一次
  - 当前计数器值按 `ARR + 1` 折算为当前秒内的浮点毫秒
  - 中断逻辑负责累加连续的 `uint64_t` 本地整数秒
  - 该本地时间是系统最底层的连续时间基准
  - UTC 同步、失锁、重锁都不能修改本地连续时钟本身

- `GNSS UTC`
  - 在每次解析到有效报文时，将 UTC 时间提取为“时间周 + 周内毫秒”
  - 解析结果写入“GNSS UTC 缓存”
  - 收到 GNSS UTC 只代表外部候选时间有效，不代表本地 UTC 时钟已经锁定
  - 写缓存时不更新当前 UTC 整秒时钟
  - GNSS 只负责同步本地 UTC 时钟，不参与本地连续时钟生成

- `PPS`
  - 用于把 GNSS UTC 对齐到本地连续时钟的某一个时刻
  - 当 `PPS` 脉冲触发时，取最近一次有效 GNSS UTC 缓存
  - 将缓存中的 `week_ms` 加 `1000 ms`，作为当前 `PPS` 对应的 UTC 整秒时钟
  - 如果 `week_ms + 1000 ms` 超过一周，则 `week` 加 `1`，`week_ms` 回绕到下一周
  - 同时记录这一刻的本地连续时钟值，形成本地 UTC 时钟的同步锚点
  - PPS 是同步边沿，不是独立时间源

#### 建议文件边界

时间模块先按公共服务落地，建议放在：

- `APP/service/time_service.h`
- `APP/service/time_service.c`

该模块只负责：

- 初始化本地连续时钟
- 维护 GNSS UTC 缓存
- 维护本地 UTC 同步状态
- 处理 `PPS` 对齐事件
- 提供统一 `get` 接口

其他模块只做输入或读取：

- `GNSSTask` 解析出 UTC 后，调用时间服务写入 GNSS UTC 缓存
- `PPS` 中断触发后，时间服务读取 GNSS UTC 缓存，将缓存 `week_ms` 加 `1000 ms` 后更新当前 UTC 整秒时钟
- `IMUTask`、`GNSSTask`、数据整理线程、日志线程只调用时间服务读取时间戳

#### 时间结构建议

```c
typedef enum {
    TIME_SYNC_NONE = 0,
    TIME_SYNC_LOCKED,
    TIME_SYNC_HOLDOVER,
    TIME_SYNC_LOST,
} TimeSyncState;

typedef struct {
    uint64_t sec;
    float ms;
} TimeLocalClock;

typedef struct {
    uint32_t week;
    uint32_t week_ms;
} TimeUtcClock;

typedef struct {
    bool valid;
    TimeUtcClock utc;
    uint32_t seq;
} TimeGnssUtcCache;

typedef struct {
    TimeLocalClock local_clock;
    TimeUtcClock local_utc;
    bool utc_valid;
    TimeSyncState sync_state;
    uint32_t sync_seq;
} TimeTimestamp;
```

字段含义如下：

- `TimeLocalClock`
  - 系统本地连续时钟
  - 由 `TIM16` 生成
  - `sec` 是累计整数秒
  - `ms` 是当前秒内的浮点毫秒，适合日志输出和小窗口排序

- `TimeUtcClock`
  - UTC 时间，采用“时间周 + 周内毫秒”表达
  - `week` 表示 UTC 时间周
  - `week_ms` 表示当前周内的毫秒数
  - `week_ms` 范围为 `0 ~ 604799999`
  - 不单独表达可信状态，可信状态由外层 `valid` 或 `utc_valid` 表达

- `TimeGnssUtcCache`
  - GNSS UTC 缓存
  - 当前由 GNSS 解析线程更新
  - 写入时只保存最近一次 GNSS UTC
  - 不在写入时更新当前 UTC 整秒时钟
  - `seq` 用于判断输入是否更新

- `TimeTimestamp`
  - 对外返回的完整时间戳结构
  - `local_clock` 始终有效
  - `local_utc` 是本地 UTC 时钟计算结果
  - `utc_valid` 表示当前本地 UTC 是否已经同步且可信
  - `sync_state` 表示 UTC 同步状态

#### 本地连续时钟规则

本地连续时钟是整个系统的时间底座，必须满足以下规则：

- 上电初始化后单调递增
- 不受 UTC 校时影响
- 不因为 UTC 失锁而停止
- 允许从 `0` 开始计数
- 需要正确处理 `TIM16` 溢出
- 所有采样数据至少必须携带本地连续时钟

本地连续时钟的典型用途：

- 多源数据排序
- 采样时间戳
- 线程运行耗时统计
- 判断数据是否连续
- UTC 无效时作为降级时间基准

#### 本地 UTC 时钟规则

本地 UTC 时钟不是直接等于 GNSS 报文中的 UTC，而是由时间服务维护的内部时钟。

它的建立过程如下：

- GNSS 线程解析出 UTC 后，先写入 GNSS UTC 缓存
- 写 GNSS UTC 缓存时，只更新缓存，不更新当前 UTC 整秒时钟
- PPS 到来时，时间服务读取最近一次 GNSS UTC 缓存
- 时间服务把缓存中的 `week_ms` 加 `1000 ms`，并记录：
  - 当前 PPS 对应的 UTC 整秒时钟
  - 当前 PPS 对应的本地连续时钟
- 之后读取 UTC 时，通过“同步锚点 UTC + 本地连续时钟增量”计算当前 UTC

当 GNSS UTC 缓存和 PPS 正常时：

- 每次 PPS 到来都会使用最近的 GNSS UTC 缓存刷新本地 UTC 整秒时钟
- `utc_valid = true`
- `sync_state = TIME_SYNC_LOCKED`

当 GNSS UTC 或 PPS 短时间中断但本地连续时钟仍正常时：

- 本地 UTC 可以继续按本地连续时钟外推
- `sync_state` 可进入 `TIME_SYNC_HOLDOVER`
- 是否保持 `utc_valid = true` 取决于允许的保持时间

当 GNSS UTC 或 PPS 超过允许时间未恢复时：

- `utc_valid = false`
- `sync_state = TIME_SYNC_LOST`
- 本地连续时钟继续有效
- 上层仍然可以获取完整时间戳，但不能把其中的 UTC 当作可信 UTC

#### UTC 锁定与失锁策略

当前建议策略如下：

- 上电时，UTC 状态默认为无效
- 收到有效 GNSS UTC 后，只写入 GNSS UTC 缓存
- 等待下一次有效 `PPS`，用 `GNSS UTC 缓存 + 1000 ms` 建立本地 UTC 整秒时钟
- 锚点建立后，进入 `TIME_SYNC_LOCKED`
- 如果后续 `PPS` 短时间丢失，可以进入 `TIME_SYNC_HOLDOVER`
- 如果连续 `3 s` 以上未收到有效 `PPS`，进入 `TIME_SYNC_LOST`
- 失锁后，等待新的 GNSS UTC 缓存和下一次 `PPS` 重新建立锚点

这里的 `utc_valid` 语义是：

- `true`：本地 UTC 时钟已经同步，并且仍处于可信窗口内
- `false`：本地 UTC 时钟未建立或已经失去可信性

#### 对外接口建议

时间服务至少提供以下接口：

```c
void TimeService_Init(void);

void TimeService_OnTim16Overflow(void);

void TimeService_WriteUtcCache(const TimeUtcClock *utc);

void TimeService_OnPpsIrq(void);

bool TimeService_GetTimestamp(TimeTimestamp *out);

bool TimeService_GetLocalClock(TimeLocalClock *out);
```

接口行为约定如下：

- `TimeService_Init`
  - 初始化时间服务内部状态
  - 清空 GNSS UTC 缓存
  - 清空本地 UTC 锚点
  - 启动本地连续时钟

- `TimeService_OnTim16Overflow`
  - 在 `TIM16` 溢出中断或等效位置调用
  - 当前 `TIM16` 配置为 `1 s` 溢出一次，因此该接口负责累加本地整数秒

- `TimeService_WriteUtcCache`
  - 由 GNSS 解析线程调用
  - 只把解析出的 GNSS UTC 写入缓存
  - 写入后设置缓存 `valid = true`
  - 写入后递增缓存 `seq`
  - 不更新当前 UTC 整秒时钟
  - 不直接修改本地连续时钟

- `TimeService_OnPpsIrq`
  - 由 `PPS` 中断或中断下半部调用
  - 读取当前本地连续时钟
  - 读取最近一次有效 GNSS UTC 缓存
  - 将缓存 UTC 的 `week_ms` 加 `1000 ms`
  - 使用加 `1000 ms` 后的 UTC 建立或刷新当前 UTC 整秒时钟
  - 记录该 UTC 整秒对应的本地连续时钟

- `TimeService_GetTimestamp`
  - 返回完整 `TimeTimestamp`
  - `local_clock` 始终有效
  - `local_utc` 根据当前同步状态计算
  - `utc_valid` 明确告诉调用者 UTC 是否可信

- `TimeService_GetLocalClock`
  - 只返回本地连续时钟
  - 用于只关心排序、耗时和连续性的场景

#### 时间同步伪代码

```c
TimeGnssUtcCache gnss_utc_cache;

struct LocalUtcAnchor {
    bool valid;
    TimeUtcClock utc_at_anchor;
    TimeLocalClock local_at_anchor;
    uint32_t sync_seq;
};

on_gnss_message_parsed(msg):
    if msg contains valid utc:
        utc = parse_utc(msg)
        TimeService_WriteUtcCache(&utc)

TimeService_WriteUtcCache(utc):
    gnss_utc_cache.utc = *utc
    gnss_utc_cache.valid = true
    gnss_utc_cache.seq++

on_pps_irq():
    TimeService_OnPpsIrq()

TimeService_OnPpsIrq():
    local_now = read_local_continuous_clock()

    if gnss_utc_cache.valid:
        local_utc_anchor.utc_at_anchor = utc_add_ms(gnss_utc_cache.utc, 1000)
        local_utc_anchor.local_at_anchor = local_now
        local_utc_anchor.valid = true
        sync_state = TIME_SYNC_LOCKED
        utc_valid = true
        sync_seq++

TimeService_GetTimestamp(out):
    out->local_clock = read_local_continuous_clock()

    if local_utc_anchor.valid:
        delta_ms = local_clock_delta_ms(out->local_clock, local_utc_anchor.local_at_anchor)
        out->local_utc = utc_add_ms(local_utc_anchor.utc_at_anchor, delta_ms)
        out->utc_valid = utc_valid
        out->sync_state = sync_state
    else:
        clear(out->local_utc)
        out->utc_valid = false
        out->sync_state = TIME_SYNC_NONE

TimeService_GetLocalClock(out):
    *out = read_local_continuous_clock()
```

这里需要注意，`utc_add_ms(gnss_utc_cache.utc, 1000)` 只需要处理 `week_ms` 的一周回绕：

- 一周为 `604800000 ms`
- 如果 `week_ms + 1000 < 604800000`，只更新 `week_ms`
- 如果 `week_ms + 1000 >= 604800000`，则 `week += 1`，`week_ms = week_ms + 1000 - 604800000`

由于 UTC 使用毫秒表达，`TimeService_GetTimestamp` 中从本地连续时钟得到的增量也需要先换算成毫秒，再叠加到 UTC 锚点上。

这样整理后，时间模块的职责会比较清晰：

- GNSS UTC 写入函数只负责更新缓存
- PPS 中断处理函数负责用缓存 UTC 加 `1000 ms` 更新当前 UTC 整秒时钟
- 本地 UTC 时钟负责对外表达 UTC 时间
- 本地连续时钟负责提供永不停顿的系统时间基准
- 所有业务线程通过统一接口读取时间



### 4.5 `defaultTask`

`defaultTask` 定义为系统监控线程，不再是空转线程。

当前职责如下：

- 尽早完成 `TimeService` 初始化
- 启动或确认 `TIM16` 本地连续时钟
- 监控其他线程是否正常运行
- 输出必要的系统状态
- 初始化阶段使用较高优先级
- 初始化完成后将自身优先级下调为最低

#### `defaultTask` 伪代码

```c
defaultTask():
    TimeService_Init()
    start_tim16_local_clock()
    register_system_monitors()
    lower_self_priority_to_lowest()

    while true:
    //这个操作先空置，因为暂时不知道怎么监控
        check_task_health() 
        check_queue_watermark()
        check_sd_status()
        sleep(period_ms)
```

### 4.6 `usartCMDTask`

`usartCMDTask` 在不同模式下承担不同职责。

#### 配置阶段职责

- 配置模式下，`usartCMDTask` 作为串口命令控制线程使用
- 串口接收由空闲中断触发，接收到的数据块交给该线程解析
- 该线程负责配置命令的读取、校验、写入和结果反馈
- 同时接收 `keyTask` 发布过来的按键事件，用于触发辅助操作

当前配置阶段逻辑可整理为：

- 短按按键
  - 由 `keyTask` 发布短按事件
  - `usartCMDTask` 收到事件后，读取当前 `Flash` 中保存的配置并通过串口发送
  - 若 `Flash` 中配置不符合预期，则返回“配置错误”或“未完成配置”

- 长按按键
  - 由 `keyTask` 发布长按事件
  - `usartCMDTask` 收到事件后触发软复位流程

- 接收到配置数据
  - 先进行合法性校验
  - 校验通过，返回 `succeed`
  - 校验失败，返回“配置不合法”

- 接收到写入命令
  - 先将配置写入 `Flash`
  - 再读回 `Flash` 中的数据进行校验
  - 校验通过，返回“写入成功”
  - 校验失败，返回“写入失败”

- 接收到读取命令
  - 直接读取当前 `Flash` 中保存的配置
  - 将读取结果原样发送到串口，由人工确认

也就是说，在配置模式下，`usartCMDTask` 统一承担“命令入口 + 配置反馈 + 配置读写校验”的职责。

#### 运行阶段职责

运行阶段中，`usartCMDTask` 作为日志输出线程存在，同时也是全局串口日志缓冲区的管理线程。

设计要求如下：

- 所有线程日志统一发送到日志缓冲区
- 每个线程独立拥有日志缓冲区
- 缓冲区采用主一备一结构
- 发送时上锁
- 写入时上锁
- 线程检测到缓冲区有数据时启动串口发送

在运行阶段，日志写入和日志发送需要分离：

- 日志数据不是在 `usartCMDTask` 中生成
- 而是由其他线程写入各自的日志缓冲区
- 其他线程在写日志时，通过统一的全局缓冲区管理接口完成注册和写入
- `usartCMDTask` 只负责维护这些已注册的缓冲区节点，并检查是否有数据可发

当前建议的缓冲区管理方式如下：

- 提供统一的日志缓冲区模板
- 其他线程在初始化时调用模板的 `init` 接口
- 初始化完成后，该线程的日志缓冲区自动挂载到 `usartCMDTask` 维护的节点表中
- `usartCMDTask` 周期性检查这些节点
- 若发现某个线程的“主缓冲区”已准备完成，则优先发送主缓冲区
- 若主缓冲区正在发送，而“备缓冲区”也已准备完成，则等待本轮主缓冲区发送结束后继续发送备缓冲区
- 若当前没有数据，则线程休眠或等待事件

关于“主一备一”缓冲区，当前补充以下规则：

- 通知发送时，必须明确当前准备发送的是主缓冲区还是备缓冲区
- 发送中的缓冲区视为锁定状态，不能再次写入
- 写入线程只能写入当前未锁定、未满的那一块缓冲区
- 如果主缓冲区和备缓冲区同时都处于满或锁定状态，则说明该线程的数据产生速度已经超过串口发送能力
- 此时应立即上报 `error`
- `error` 信息中必须带上具体线程标识，指出是哪个线程出现了数据速度不匹配问题

需要补充说明的是：日志缓冲区和 SD 数据缓冲区虽然都采用“主一备一”双缓冲机制，但两者只是在缓冲区切换、锁定、通知和释放流程上相似；如果后续尝试复用同一套缓冲区结构，必须把“数据截断策略”单独抽象出来，不能默认两者完全一致。

串口发送方式后续建议优先使用 `DMA`。

因此，在运行模式下，`usartCMDTask` 的职责可以概括为：

- 维护全局日志缓冲区节点表
- 调度各线程日志的发送顺序
- 通过串口将日志输出到外部
- 在无数据时进入休眠，降低空转开销

#### `usartCMDTask` 伪代码

```c
usartCMDTask():
    while true:
        if current_mode == CONFIG_MODE:
            event = wait_uart_or_key_event()
            handle_config_event(event)
        else:
            for each log_node in registered_log_nodes:
                if log_node.buffer.has_data():
                    lock(log_node.buffer)
                    dma_uart_send(log_node.buffer.data)
                    unlock(log_node.buffer)
                else:
                    continue

            if no_log_data_pending():
                sleep(short_period)
```

### 4.7 `GNSSTask`

`GNSSTask` 负责 GNSS 数据接收、协议解析，并把解析出的 UTC 写入时间服务的 GNSS UTC 缓存。

当前要求如下：

- 使用原有流式解析策略
- 通过 `IDLE` 中断触发解析
- 保留原始精度，不允许人为减少精度
- 在解析过程中提取 GNSS UTC，并调用 `TimeService_WriteUtcCache`
- GNSS UTC 只用于同步本地 UTC 时钟，不生成本地连续时钟
- 将“GNSS 数据 + 完整时间戳”打包后发送到消息队列

#### `GNSSTask` 伪代码

```c
GNSSTask():
    init_uart_dma_idle_receive()

    while true:
        block = wait_dma_idle_block()
        frames = gnss_stream_parse(block)

        for each frame in frames:
            if frame.contains_valid_utc():
                utc = parse_utc_from_gnss(frame)
                TimeService_WriteUtcCache(&utc)

            TimeService_GetTimestamp(&node.timestamp)
            node.payload = frame.data
            queue_send(node)
```

### 4.8 UWB 协议栈

UWB 协议栈的线程模型、层次边界、空口协议和落地顺序统一以 [uwb-designed.md](uwb-designed.md) 为准，本文不再维护旧 UWB 线程描述。

#### 与主系统配置的衔接

UWB 模组初始化时使用主系统已经解析完成的基础配置：

- `PAN ID`
- 本机短地址 `short_addr`
- 角色 `Anchor/Tag`
- 必要的帧控制字段

配置来源规则如下：

1. 运行模式启动时，配置服务先读取 `Flash` 中的基础配置。
2. 如果 `Flash` 配置校验通过，则该配置作为当前运行配置。
3. 如果 `Flash` 配置无效或尚未写入，则配置服务加载默认静态基础配置。
4. UWB 模组只接收这份当前运行配置，不直接绕过配置服务读取 `Flash`。

因此，UWB 栈内部不再判断“配置来自 Flash 还是默认值”。它只关心当前传入的 `pan_id`、`short_addr` 和 `role` 是否可用，并按角色进入对应的 Tag 或 Anchor 行为。

后续落代码时，建议主系统提供类似下面的初始化边界：

```c
ConfigService_Load();
UWB_DeviceInitFromConfig();
```

其中 `ConfigService_Load()` 内部负责完成 Flash 配置校验和默认静态配置兜底，`UWB_DeviceInitFromConfig()` 只读取配置服务当前生效的配置。

### 4.9 `IMUTask`

`IMUTask` 负责 IMU 采样，当前继续沿用现有中断触发方案。

当前要求如下：

- IMU 采样由中断触发
- 中断只负责通知线程
- 线程中完成 SPI 读取
- 本线程相关静态配置保存在自己的线程文件中
- 采样完成后，按与 GNSS 相同的方式进行“数据 + 完整时间戳”打包
- 打包后发送到消息队列

#### `IMUTask` 伪代码

```c
IMUTask():
    init_imu_config()

    while true:
        wait_imu_irq_notify()
        imu_data = spi_read_imu()
        TimeService_GetTimestamp(&node.timestamp)
        node.payload = imu_data
        queue_send(node)
```

### 4.10 数据接收 / 整理线程

该线程是数据存储链路中的核心整理线程。

主要职责如下：

- 从消息队列接收来自 `GNSS`、`IMU`、`UWB` 的数据节点
- 开辟节点挂载区作为临时排序空间
- 由于时间上可能存在微小反转，执行小范围排序
- 当前排序窗口先按 `10` 个挂载点设计
- 在该线程中完成 ASCII 文本打包
- 排序完成后，将 ASCII 数据连续写入全局 SD 写卡 FIFO
- 当 FIFO 中某个 `16 KB` 块写满后，通知 `SD` 写入线程写盘

当前 SD 数据缓冲区是一个全局写卡 FIFO，但物理上按双缓冲实现。

具体规划如下：

- 单块缓冲区大小：`16 KB`
- 缓冲区数量：2 块
- 组织方式：主缓冲区 + 备缓冲区
- 总缓存容量：`32 KB`
- 写入单位：数据整理线程按字节连续写入
- 写盘单位：`SD` 写入线程按 `16 KB` 整块写入

关于这组 SD 数据缓冲区，当前明确以下规则：

- 数据整理线程始终向当前可写缓冲区写入
- ASCII 转换在数据整理线程中完成
- `SD` 写入线程不再逐条处理 `AppDataNode`
- `SD` 写入线程不再逐条转换 ASCII
- 当主缓冲区写满 `16 KB` 后，通知 `SD` 写入线程写主缓冲区
- 数据整理线程立即切换到备缓冲区继续写入
- 当备缓冲区写满 `16 KB` 后，通知 `SD` 写入线程写备缓冲区
- 通知时必须明确当前准备写盘的是主缓冲区还是备缓冲区
- 正在写盘的缓冲区视为锁定状态，不允许继续写入
- 写入线程只能写未锁定、未满的那一块缓冲区

这里需要特别强调：

- SD 数据缓冲区与日志缓冲区在“主一备一”“缓冲切换”“通知发送”“锁定释放”这些机制上基本一致
- 但二者在“数据截断处理”上并不相同
- 因此如果后续希望复用同一套双缓冲数据结构，必须将“截断策略”设计成可配置或可重载的策略接口，而不能直接共用同一套固定处理逻辑

边界处理原则如下：

- 当一条 ASCII 数据写入时会跨越 `16 KB` 边界，不做强制补齐
- 先把能写入当前块的前半段写满当前块
- 当前块达到 `16 KB` 后，立刻标记为 ready 并通知 `SD` 写入线程
- 数据整理线程切换到另一块缓冲区
- 剩余的 ASCII 数据继续写入下一块缓冲区
- 也就是说，一条 ASCII 数据允许跨越两个 `16 KB` 块
- 当主缓冲区和备缓冲区同时都处于满或锁定状态时，直接上报 `error`
- `error` 中需要明确标识当前是哪个数据来源线程导致写入速度与存储速度不匹配

这样处理的目标是：

- 不产生空位
- 不人为丢包
- 保持数据连续性
- 在缓冲区彻底来不及消费时，能够明确指出是哪个线程导致的吞吐不匹配

#### 数据整理线程流程图

```text
消息队列取节点
      |
      v
挂载到临时排序区
      |
      v
达到排序窗口大小？
   |         |
   | 否      | 是
   v         v
继续接收   按时间排序
              |
              v
        转换为 ASCII 数据
              |
              v
        连续写入 32KB SD FIFO
              |
              v
        当前 16KB 块满？
           |        |
           | 否     | 是
           v        v
        继续写入   标记当前块 ready
                     |
                     v
              通知 SD 写入线程
                     |
                     v
                还有空闲缓冲区？
                   |         |
                   | 是      | 否
                   v         v
             切换主/备继续写  上报 error

```

#### 数据整理线程伪代码

```c
DataSortTask():
    init_sort_buffer(10)
    init_global_sd_fifo(16KB x 2)

    while true:
        node = queue_receive()
        sort_buffer.push(node)

        if sort_buffer.count >= SORT_WINDOW:
            sort_by_timestamp(sort_buffer)

            for each node in sort_buffer:
                ascii_line = convert_node_to_ascii(node)
                sd_fifo_write(ascii_line, allow_split=true)

            sort_buffer.clear()

sd_fifo_write(data, allow_split=true):
    while data.remaining > 0:
        writable = get_current_writable_block()

        if writable == NULL:
            report_error(source_thread_id)
            return

        copied = copy_to_current_block(data)

        if current_block_full():
            mark_current_block_ready()
            notify_sd_writer(current_block_id)
            switch_to_next_block()

        data.advance(copied)
```

### 4.11 `SD` 写入线程

`SD` 写入线程只负责 SD 卡相关的实际读写工作。

其职责包括：

- 检测 SD 卡是否存在
- 若不存在，则每隔 `1 s` 轮询一次
- 若检测到插入，则执行挂载
- 检查现有文件名并创建新文件
- 文件名采用类似：
  - `uwb-gnss-imu-sampling-1.log`
- 等待数据整理线程发送 `16 KB` 块 ready 通知
- 每次收到通知后，一次性写入对应的 `16 KB` 数据块

当前要求是不写二进制文件，直接写 ASCII 文本。也就是说，文本转换必须在“数据接收 / 整理线程”中完成，写卡线程只处理最终可落盘的 `16 KB` ASCII 数据块。

这里需要特别明确：

- `SD` 写入线程不从数据节点队列读取 `AppDataNode`
- `SD` 写入线程不负责排序
- `SD` 写入线程不负责 ASCII 转换
- `SD` 写入线程只等待“主缓冲区 ready”或“备缓冲区 ready”的通知
- `SD` 写入线程每次写盘单位固定为 `16 KB`

在 SD 写盘链路中，写入线程接收到通知时，也必须明确当前要写的是：

- 主缓冲区
- 或备缓冲区

写盘完成后，对应缓冲区需要被释放，重新回到可写状态，供数据整理线程继续使用。

#### `SD` 写入线程流程图

```text
启动线程
   |
   v
检测 SD 是否存在
   |----------------------+
   | 不存在               |
   v                      |
延时 1s 后重试            |
   |                      |
   +----------------------+
   |
   v
挂载 SD
   |
   v
创建新日志文件
   |
   v
等待写入通知（主/备缓冲区标识）
   |
   v
读取指定的主缓冲区或备缓冲区
   |
   v
写入 ASCII 日志
   |
   v
释放当前缓冲区
   |
   v
循环等待下一次写入
```

#### `SD` 写入线程伪代码

```c
SDWriterTask():
    while true:
        if sd_not_present():
            sleep(1s)
            continue

        if not mounted:
            mount_sd()
            open_next_log_file()

        buffer_id = wait_sd_write_notify()
        block = get_sd_ready_buffer(buffer_id)
        if block.valid:
            lock_sd_buffer(buffer_id)
            file_write(block.data, 16KB)
            file_sync_if_needed()
            release_sd_buffer(buffer_id)
```

### 4.12 `keyTask`

`keyTask` 负责统一采集按键动作，并完成长按、短按判定。它不直接把按键逻辑写死到某个具体线程里，而是作为按键事件发布者存在。

当前设计要求如下：

- 统一采集按键状态

- 按键初始化时候需要调整一下配置模式，上升沿，下降沿都触发中断，否则可能没办法实现这个目标
- 输出“短按事件”“长按进行事件”“长按松手事件”“松手事件”等标准事件
- 不在其他业务线程中重复实现长短按判断逻辑
- 维护一张订阅表，记录哪些线程需要接收哪些按键事件
- 通过统一缓存区或事件分发表，将按键事件发送给已订阅线程

- 长短按的区分阈值暂时设置为200ms 


也就是说，`keyTask` 的职责不是“自己处理所有按键功能”，而是：

- 负责识别按键动作
- 负责生成标准按键事件
- 负责按订阅关系把事件分发出去

例如：

- `usartCMDTask` 可订阅“短按读取配置”“长按软复位”相关事件
- 其他线程也可以根据需要订阅按键事件，而不必再次自己扫描按键

#### `keyTask` 流程图

```text
扫描按键状态
    |
    v
执行消抖
    |
    v
判定短按 / 长按 / 松手
    |
    v
生成标准按键事件
    |
    v
查询订阅表
    |
    v
向已订阅线程发布事件
```

#### `keyTask` 伪代码

```c
keyTask():
    init_key_driver()
    init_key_subscribe_table()

    while true:
        raw = read_key_state()
        evt = detect_key_event(raw)

        if evt.valid:
            for each subscriber in key_subscribe_table:
                if subscriber.match(evt.type):
                    publish_key_event(subscriber, evt)

        sleep(scan_period)
```

### 4.13 `ledTask`

`ledTask` 负责统一处理所有灯光命令。它是系统中唯一真正执行 LED 动作的线程。

当前设计要求如下：

- 其他线程不直接控制 LED
- 所有线程都可以向 `ledTask` 发送状态命令
- `ledTask` 直接通过任务通知接收命令
- 收到通知后，读取对应状态值并执行对应视觉特征
- 后续 `Boot`、配置模式、错误提示、角色提示都统一通过该线程实现

这里的设计重点是：

- `ledTask` 不做业务判断
- 业务线程只负责决定“应该显示什么状态”
- `ledTask` 负责把状态翻译为具体灯效

例如可以由其他线程下发如下类型的状态：

- 全灭
- 全亮
- 单灯常亮
- 单灯闪烁
- 顺序流水灯
- 错误提示灯效

#### `ledTask` 流程图

```text
等待线程通知
    |
    v
读取通知中的状态值
    |
    v
解析状态值对应的灯效类型
    |
    v
执行对应灯效
    |
    v
返回等待下一条通知
```

#### `ledTask` 伪代码

```c
ledTask():
    while true:
        state = wait_task_notification()

        switch state:
            case LED_STATE_ALL_OFF:
                led_all_off()
            case LED_STATE_ALL_ON:
                led_all_on()
            case LED_STATE_SINGLE_ON:
                led_single_on(target_led)
            case LED_STATE_BLINK:
                led_blink(target_led, period_ms)
            case LED_STATE_SEQ:
                run_led_sequence(seq, period_ms)
            case LED_STATE_ERROR:
                run_error_pattern()
```

---

## 5. UWB 协议栈

UWB 协议栈的线程模型、层次边界、空口协议、调度策略和重构落地顺序统一以 [uwb-designed.md](uwb-designed.md) 为准。

本文不再维护旧 UWB 线程描述或旧测距流程描述，避免与 UWB 专用重构文档产生冲突。
