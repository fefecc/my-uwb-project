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

- 静态外参
  - 保存在头文件中
  - 作为静态变量由系统直接调用
  - 不通过串口写入

### 3.5 运行模式

运行模式是系统正常工作路径。

之后会在后面添加

- 读取 `Flash` 中的基础设备配置
- 读取工程中的静态外参
- 根据角色进入对应运行流程
- 启动各业务线程
- 进入采样、处理和存储阶段


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
  - 负责 `TIM16` 时基相关工作
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

当前系统的时间基准由 `TIM16 + GNSS + PPS` 共同完成。这一部分应当作为底层时间服务独立实现，不直接写死在某个业务线程中。

整体设计目标是同时保留两类时间能力：

- 本地高分辨率计数时间
- 带有效性标记的 UTC 时间

也就是说，系统内部最终应同时维护：

- 一个由本地计数器连续推进的时间基准
- 一个与 `GNSS + PPS` 对齐后的 UTC 时间基准

### 时间基准组成

- `TIM16`
  - 作为本地高精度计数器使用
  - 当前约定为 `1 s` 溢出一次
  - 当前约定计数分辨率为 `20 us`
  - 该计数器始终连续运行，作为本地时间推进依据

- `GNSS`
  - 在每次解析到有效报文时，将其中的 UTC 信息提取出来
  - 提取后的 UTC 信息先放入本地缓存结构体中
  - 此时只是“收到有效候选时间”，并不代表系统已经完成 UTC 锁定

- `PPS`
  - 当 `PPS` 脉冲触发时，取最近一次有效 GNSS UTC 缓存
  - 在该缓存时间基础上加 `1 s`，作为当前 `PPS` 对应的 UTC 整秒时刻
  - 同时记录该时刻下本地 `TIM16` 的基准计数值

### UTC 锁定与失锁策略

当前 UTC 不应在收到第一帧 GNSS 时间后立即判定为可靠，而是需要一个锁定过程。

建议策略如下：

- 上电时，UTC 状态默认为无效
- 连续 `5` 帧 GNSS 时间数据稳定后，再认为 UTC 已锁定
- 锁定完成后，将 UTC 状态标记为 `valid = true`
- 如果后续 `PPS` 连续丢失，例如连续 `3 s` 以上未收到，则认为 UTC 失锁
- 失锁后，UTC 状态重新置为无效，等待重新锁定

也就是说，`valid` 的语义不是“收到过一次 GNSS 时间”，而是“当前 UTC 基准已经建立并且仍然可信”。

### 失锁后的时间推进方式

当 UTC 已锁定时：

- 系统优先使用 `PPS + GNSS` 对齐后的 UTC 基准
- 再结合本地 `TIM16` 增量计算当前时间

当 UTC 失锁时：

- UTC 结构中的 `valid` 置为无效
- 系统仍然继续使用本地计数器推进本地时间
- 也就是说，本地时间不会中断，但 UTC 精度与可靠性不再保证

这样处理的好处是：

- 时间戳始终连续
- UTC 是否可信有明确标志位
- 上层线程可以根据 `valid` 决定是否把当前时间当作“可信 UTC”使用

### 对外时间接口建议

建议提供一个统一的 `get` 函数，对外返回完整时间结构。

该结构至少包含两部分：

- 本地计数器换算出的当前时间
- 当前 UTC 时间及其 `valid` 标志

这样设计后，上层模块在取时间时可以同时拿到：

- 连续时间
- UTC 时间
- UTC 是否可信

这比只返回单一时间值更适合后续数据采样、日志记录和时序分析。

#### 时间同步伪代码

```c
struct GnssUtcCache {
    bool valid;
    UtcTime utc_from_gnss;
    uint32_t seq;
}; // 最近一次解析出的 GNSS UTC 缓存

struct SystemUtcBase {
    bool valid;
    UtcTime pps_utc_base;
    uint32_t tim16_base_count;
};

on_gnss_message_parsed(msg):
    if msg contains valid utc:
        gnss_cache.utc_from_gnss = parse_utc(msg)
        gnss_cache.valid = true
        gnss_cache.seq++
        update_stable_counter()
        if stable_counter >= 5:
            utc_lock_ready = true

on_pps_irq():
    if gnss_cache.valid and utc_lock_ready:
        system_utc_base.pps_utc_base = gnss_cache.utc_from_gnss + 1 second
        system_utc_base.tim16_base_count = TIM16.current_count()
        system_utc_base.valid = true
        pps_loss_counter = 0

on_pps_timeout():
    pps_loss_counter++
    if pps_loss_counter >= 2:
        system_utc_base.valid = false
        utc_lock_ready = false

get_system_timestamp():
    if system_utc_base.valid:
        delta = TIM16.current_count() - system_utc_base.tim16_base_count
        return system_utc_base.pps_utc_base + delta
    else:
        return local_fallback_timestamp()
```

#### 建议的时间输出结构

```c
typedef struct {
    LocalTime local_time;
    UtcTime utc_time;
    bool utc_valid;
} SystemTimestamp;
```

建议的对外行为如下：

- `local_time`
  - 始终可用
  - 由本地计数器连续推进

- `utc_time`
  - 仅在 UTC 锁定时作为可信 UTC 使用

- `utc_valid`
  - 表示当前 UTC 是否处于锁定且可信状态

这样整理后，时间模块的职责会比较清晰：

- `GNSS` 提供 UTC 候选值
- `PPS` 提供整秒对齐
- `TIM16` 提供连续高分辨率本地时间
- `valid` 表示 UTC 是否可信



### 4.5 `defaultTask`

`defaultTask` 定义为系统监控线程，不再是空转线程。

当前职责如下：

- 尽早完成 `TIM16` 相关时基初始化
- 监控其他线程是否正常运行
- 输出必要的系统状态
- 初始化阶段使用较高优先级
- 初始化完成后将自身优先级下调为最低

#### `defaultTask` 伪代码

```c
defaultTask():
    init_tim16_timebase()
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

`GNSSTask` 负责 GNSS 数据接收、协议解析和时间缓存更新。

当前要求如下：

- 使用原有流式解析策略
- 通过 `IDLE` 中断触发解析
- 保留原始精度，不允许人为减少精度
- 在解析过程中更新 GNSS UTC 缓存
- 将“GNSS 数据 + 本地时间戳”打包后发送到消息队列

#### `GNSSTask` 伪代码

```c
GNSSTask():
    init_uart_dma_idle_receive()

    while true:
        block = wait_dma_idle_block()
        frames = gnss_stream_parse(block)

        for each frame in frames:
            if frame.contains_valid_utc():
                update_gnss_utc_cache(frame)

            node.timestamp = get_system_timestamp()
            node.payload = frame.data
            queue_send(node)
```

### 4.8 UWB 协议栈

UWB 协议栈的线程模型、层次边界、空口协议和落地顺序统一以 [uwb-designed.md](uwb-designed.md) 为准，本文不再维护旧 UWB 线程描述。

### 4.9 `IMUTask`

`IMUTask` 负责 IMU 采样，当前继续沿用现有中断触发方案。

当前要求如下：

- IMU 采样由中断触发
- 中断只负责通知线程
- 线程中完成 SPI 读取
- 本线程相关静态配置保存在自己的线程文件中
- 采样完成后，按与 GNSS 相同的方式进行“数据 + 本地时间戳”打包
- 打包后发送到消息队列

#### `IMUTask` 伪代码

```c
IMUTask():
    init_imu_config()

    while true:
        wait_imu_irq_notify()
        imu_data = spi_read_imu()
        node.timestamp = get_system_timestamp()
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
- 排序完成后，将数据压入写卡 FIFO
- 在该线程中完成 ASCII 文本打包

当前 SD 数据缓冲区规划与前面的日志缓冲区保持一致，也采用“主一备一”双缓冲机制。

具体规划如下：

- 单块缓冲区大小：`16 KB`
- 缓冲区数量：2 块
- 组织方式：主缓冲区 + 备缓冲区
- 总缓存容量：`32 KB`

关于这组 SD 数据缓冲区，当前明确以下规则：

- 数据整理线程始终向当前可写缓冲区写入
- 当主缓冲区写满后，通知 `SD` 写入线程优先发送主缓冲区
- 数据整理线程切换到备缓冲区继续写入
- 当备缓冲区写满后，通知 `SD` 写入线程发送备缓冲区
- 通知时必须明确当前准备写盘的是主缓冲区还是备缓冲区
- 正在写盘的缓冲区视为锁定状态，不允许继续写入
- 写入线程只能写未锁定、未满的那一块缓冲区

这里需要特别强调：

- SD 数据缓冲区与日志缓冲区在“主一备一”“缓冲切换”“通知发送”“锁定释放”这些机制上基本一致
- 但二者在“数据截断处理”上并不相同
- 因此如果后续希望复用同一套双缓冲数据结构，必须将“截断策略”设计成可配置或可重载的策略接口，而不能直接共用同一套固定处理逻辑

边界处理原则如下：

- 当节点数据在 `16 KB` 位置被截断时，不做强制补齐
- 直接继续向后写
- SD 写入线程并行写当前块
- 剩余数据在下一次继续写入
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
        压入 16KB 主/备缓冲区
              |
              v
        当前缓冲区满？
           |        |
           | 否     | 是
           v        v
        继续收集   通知 SD 写入线程
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
    init_sd_double_buffer(16KB x 2)

    while true:
        node = queue_receive()
        sort_buffer.push(node)

        if sort_buffer.count >= SORT_WINDOW:
            sort_by_timestamp(sort_buffer)

            for each node in sort_buffer:
                ascii_line = convert_node_to_ascii(node)
                sd_buffer_write(ascii_line, allow_split=true)

            sort_buffer.clear()

        if current_sd_buffer_full():
            notify_sd_writer(which_buffer_is_ready())

            if no_free_sd_buffer():
                report_error(source_thread_id)
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
- 将整理线程已经转换好的 ASCII 数据直接写入文件

当前要求是不写二进制文件，直接写 ASCII 文本。也就是说，文本转换必须在“数据接收 / 整理线程”中完成，写卡线程只处理最终可落盘数据。

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
            file_write(block.data)
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
