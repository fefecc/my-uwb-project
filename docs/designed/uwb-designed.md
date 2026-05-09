# UWB 三线程架构：应答下沉 + 数据帧同步方案

## Context

当前三线程（PHY/LINK/APP）架构中，Anchor 侧快速应答需要 PHY→LINK→PHY 两次上下文切换，开销约 2-4ms。目标是将应答帧的打包和发送**下沉到 PHY 层直接处理**，同时需要为数据帧（主机拉取模式）设计 LINK 与 PHY 之间的实时同步机制。

**约束**：数据帧不可打断测距周期。

---

## 0. 三层架构功能约束

### 0.1 PHY 层职责

PHY 层是 DW1000 硬件的直接管理者，负责物理层的收发时序控制：

**核心功能**：
1. **帧发送**：执行 LINK 下发的发送命令，管理 TX 时序
2. **帧接收**：管理 RX 窗口，处理接收中断
3. **快速应答**：在接收完成后，直接构造并发送应答帧（DISC_ACK、RANG_RESP、DATA_* 应答帧）

**状态流转约束**：
```
发送完成 → 直接进入接收模式 (RX)
接收完成 → 进入监听模式 (LISTENING)
```

**设计原则**：
- PHY 层不解析业务语义（如测距结果、距离值），只做帧级别的快速应答
- 发送/接收/监听的状态切换由硬件事件驱动，最小化延迟
- PHY 层保持"无状态"：不维护 session、不存储测距结果

### 0.2 LINK 层职责

LINK 层是协议调度的核心，负责协调帧类型和发送时机：

**核心功能**：
1. **帧类型调度**：决定下一帧应该发送什么类型
2. **状态机管理**：维护 Discovery / Ranging / Data 的状态转换
3. **周期控制**：管理各类帧的发送间隔

**调度约束**：

| 帧类型 | 调度规则 |
|--------|----------|
| Discovery | 未发现目标：100ms 间隔；发现目标后：1s 间隔 |
| Ranging | 连续进行，TWR 两帧数据不能打断（原子性保证） |
| Data | 如果开始消息传输，穿插在 Discovery/Ranging 之间进行 |

**调度优先级**：
```
Ranging (测距帧) > Discovery (发现帧) > Data (消息帧)
```

**测距周期保护**：
- 一次 TWR 测距（REQ + RESP）必须原子完成
- Data 帧请求在测距周期内被 PHY 忽略，待测距结束后处理
- Tag 侧重试机制保证 Data 帧最终能被处理

**状态机**：
```
IDLE ──┬── Discovery (周期性) ──┬── IDLE
       │                        │
       └── Ranging (连续) ──────┘
                │
                └── Data (穿插)
```

### 0.3 APP 层职责

APP 层是业务逻辑层，负责数据处理和上层管理：

**核心功能**：
1. **测距计算**：接收 LINK 上报的 RangingRaw 事件，计算 TOF 和距离
2. **消息包创建**：构造 Data 帧的数据内容（填充到三缓冲区）
3. **MAC 表建立**：维护已发现设备的地址映射表

**数据处理**：
- 测距结果通过 DataService 发布（AppDataNode）
- MAC 表用于快速查找设备信息（short_addr → slot_index 映射）

**与 LINK 层的接口**：
- 接收 LINK 上报的事件（Discovery、RangingRaw、Data 请求）
- 响应 LINK 的数据填充请求（Anchor 侧）
- 发起数据拉取请求（Tag 侧）

### 0.4 层间交互约束

```
┌─────────────────────────────────────────────────────────┐
│  APP 层                                                  │
│  - 测距计算 (TOF → 距离)                                 │
│  - 消息包创建 (填充三缓冲区)                              │
│  - MAC 表维护 (设备地址映射)                              │
└──────────────────────────┬──────────────────────────────┘
                           │ g_app_evt_queue (事件上报)
┌──────────────────────────┴──────────────────────────────┐
│  LINK 层                                                 │
│  - 帧类型调度 (Discovery/Ranging/Data 优先级)            │
│  - 状态机管理 (IDLE/Discovery/Ranging/Data)             │
│  - 周期控制 (Discovery: 100ms/1s, Ranging: 连续)        │
│  - TWR 原子性保证 (两帧不可打断)                          │
└──────────────────────────┬──────────────────────────────┘
                           │ q_link_phy_cmd (发送命令)
                           │ q_phy_link_evt (事件上报)
┌──────────────────────────┴──────────────────────────────┐
│  PHY 层                                                  │
│  - 帧发送 → 接收模式 (TX → RX)                          │
│  - 帧接收 → 监听模式 (RX → LISTENING)                   │
│  - 快速应答 (DISC_ACK/RANG_RESP/DATA_* 应答)            │
│  - 测距周期保护 (忽略 Data 请求直到 LISTENING)           │
└──────────────────────────┬──────────────────────────────┘
                           │ SPI + IRQ
                      ┌────┴────┐
                      │ DW1000  │
                      └─────────┘
```

**关键约束总结**：

| 层 | 约束 |
|----|------|
| PHY | 发送完成必进接收；接收完成必进监听；不存储业务状态 |
| LINK | Discovery 未发现 100ms/发现后 1s；Ranging 连续不可打断；Data 穿插 |
| APP | 只处理计算和创建；不参与硬件时序；通过队列与 LINK 通信 |

## 1. 目标架构

```
APP 线程 (uwbApp, osPriorityNormal)
  | g_app_evt_queue (depth=8)
  v
LINK 线程 (uwbLink, osPriorityHigh)
  | 调度 Discovery/Ranging
  | 管理数据帧缓冲区 (双缓冲)
  | 通知 APP 准备数据
  v
PHY 线程 (uwbPhy, osPriorityAboveNormal)
  | DW1000 硬件操作
  | 中断处理
  | ★ 快速应答直接处理 (DISC_ACK, RANG_RESP, DATA_PREP_ACK, DATA_FRAGMENT)
  v
DW1000
```

### 1.1 PHY 层职责重新划分

PHY 层不再只是"硬件驱动"，而是**硬件驱动 + 快速应答处理器**：
- 收到 DISCOVERY_REQ → 直接构造 ACK 并延时发送 → 上报 LINK（事件通知）
- 收到 RANGING_REQ → 直接解析槽分配 → 构造 RESP 并延时发送 → 上报 LINK
- 收到 DATA_PREP_REQ → 直接回复 ACK/BUSY → 上报 LINK
- 收到 DATA_PULL_REQ → 从共享缓冲区读取 → 打包发送 → 上报 LINK

PHY 层**不需要**解析业务语义（如测距计算、距离结果），只做**帧级别的快速应答**。

### 1.2 LINK 层职责

LINK 层不再参与快速应答的打包，职责变为：
- 状态机管理和定时调度（Discovery/Ranging 周期）
- 数据帧缓冲区管理（双缓冲状态流转）
- 接收 PHY 上报的事件，通知 APP 层
- 发布测距发起帧（Tag 侧的主动发送仍由 LINK 调度）

---

## 2. PHY 层快速应答设计

### 2.1 应答帧分类

| 帧类型 | 是否需要 LINK 数据 | PHY 能否自行处理 | 处理方式 |
|--------|-------------------|-----------------|----------|
| DISCOVERY_ACK | 否 | 是 | 直接构造发送 |
| RANGING_RESP | 否（payload 从请求帧解析） | 是 | 直接构造发送 |
| DATA_PREP_ACK | 需要检查缓冲区+LINK确认session | 部分快速 | PHY先检查缓冲区快速回复，LINK后续确认session |
| DATA_FRAGMENT | **是**（需要共享缓冲区中的数据） | 部分 | 从共享三缓冲区读取后发送 |
| DATA_DONE_ACK | 否 | 是 | 直接回复 |

### 2.2 快速应答的帧构造

PHY 层需要直接调用 `uwb_build_frame()` 构造帧。这意味着 PHY 层需要：
- 本地配置副本：`pan_id`, `short_addr`, `frame_ctrl`（初始化时从 LINK 配置获取）
- TX 缓冲区：专用快速应答 TX 缓冲区（`g_rt_tx_buf`，静态分配）
- 时间工具：`UwbPhy_UsToDwTime()`, `make_delayed_tx_time()` 等

### 2.3 RANGING_RESP 的 PHY 层处理流程

```
PHY 收到 RANGING_REQ:
  1. dwt_readrxdata() 读帧数据
  2. uwb_parse_frame() 解析 MAC + Payload
  3. uwb_decode_ranging_req() 解析 Ranging 请求
  4. 查找本机 base_id 对应的 slot 和 reply_delay_us
  5. 计算 tx_time = rx_ts + delay_us
  6. 构造 RANGING_RESP（包含 base_id, round_id, slot_index, req_rx_ts, resp_tx_ts）
  7. dwt_setdelayedtrxtime() + dwt_starttx(DWT_START_TX_DELAYED)
  8. 上报 LINK：RX 事件（帧类型 + 时间戳 + 发送了什么 + 发送时间）
```

**关键优化**：步骤 2-7 在 PHY 线程的同一调用栈内完成，零队列开销。

---

## 3. 数据帧同步机制（核心讨论）

### 3.1 数据流时序

```
Tag (主机/读取端)                    Anchor (从机/数据源)
  |                                      |
  |-- DATA_PREP_REQ ------------------->|  PHY 直接回复 DATA_PREP_ACK
  |<-- DATA_PREP_ACK ------------------|  PHY 上报 LINK → LINK 通知 APP
  |                                      |  APP 开始填充数据到缓冲区
  |                                      |  LINK 管理: FREE → FILLING → READY
  |                                      |
  |-- DATA_PULL_REQ(seq=0) ------------>|  PHY 检查缓冲区状态
  |<-- DATA_FRAGMENT(seq=0) ------------|  如果 READY: 读取并发送
  |                                      |  如果 FILLING/FREE: 回复 BUSY
  |                                      |
  |-- DATA_PULL_REQ(seq=1) ------------>|  ...
  |<-- DATA_FRAGMENT(seq=1) ------------|
  |                                      |
  |-- DATA_DONE(last_seq) ------------->|  PHY 直接回复 ACK
  |<-- DATA_DONE_ACK -------------------|  释放缓冲区
```

### 3.2 共享三缓冲设计

**为什么需要三缓冲**：
- 当 PHY 正在发送当前片（缓冲 A 处于 HELD）时，APP 可能正在准备下一片（缓冲 B 处于 FILLING）
- 如果 Tag 请求重发上一片，PHY 需要从 HELD 缓冲区读取并发送
- 此时如果只有双缓冲，下一片的准备会被阻塞（因为缓冲 B 正在被填充，缓冲 A 正在被读取）
- 三缓冲解决了这个问题：HELD(重发) + SENDING(当前) + FILLING(下一片) 可以同时进行

**缓冲区状态机（三缓冲）**：

```
         LINK/APP 操作                    PHY 操作
    ┌─────────────────┐
    │      FREE       │←──── session 结束或下一片发送完成
    │  (空闲，可写入)   │
    └────────┬────────┘
             │ APP 被通知写入此缓冲
    ┌────────▼────────┐
    │    FILLING      │      APP 正在填充数据（片 N+1）
    │  (APP 填充中)    │      ┌──────────────────┐
    └────────┬────────┘      │  此时其他缓冲可能:  │
             │               │  A: HELD (片 N-1)  │
    ┌────────▼────────┐      │  B: SENDING (片 N) │
    │     READY       │      └──────────────────┘
    │  (等待发送)      │      三缓冲允许同时持有:
    └────────┬────────┘      HELD + SENDING + FILLING
             │ PHY 收到 PULL_REQ(seq=N+1)
    ┌────────▼────────┐
    │   SENDING       │      PHY 正在从 DW1000 发送
    │  (PHY 发送中)    │
    └────────┬────────┘
             │ TX_DONE
    ┌────────▼────────┐
    │     HELD        │      保留备份，用于重发（片 N+1）
    │  (重发备份)      │      此时 SENDING 缓冲已变为 HELD
    └────────┬────────┘      旧的 HELD (片 N) 可以释放为 FREE
             │ 下一片发送完成 或 session 超时 或 Tag 确认收到
    └─────────────────┘
         释放为 FREE
```

**三缓冲并发场景示例**：

```
时间线:
t0: PHY 发送片 0 (buf_A: SENDING)
t1: Tag 收到片 0，请求片 1
t2: PHY 发送片 1 (buf_B: SENDING), buf_A 进入 HELD (片 0 重发备份)
t3: APP 开始填充片 2 (buf_C: FILLING)
t4: Tag 请求重发片 1
t5: PHY 从 buf_B (HELD) 读取并发送片 1
    此时: buf_A=HELD(片0), buf_B=HELD→SENDING(片1), buf_C=FILLING(片2)
    三缓冲确保片 2 的准备不被阻塞！
```

**缓冲区结构**：

```c
typedef struct {
    volatile uint8_t state;     // FREE/FILLING/READY/SENDING/HELD
    uint8_t  fragment_seq;      // 当前分片序号
    uint16_t session_id;        // 会话 ID
    uint16_t data_len;          // 有效数据长度
    uint8_t  data[UWB_DATA_FRAGMENT_BODY_MAX_LEN];  // 数据内容
    uint8_t  retry_count;       // 重发次数
} uwb_data_tx_buf_t;

#define UWB_DATA_TX_BUF_NUM  3   // 三缓冲: HELD + SENDING + FILLING
```

### 3.3 同步机制分析

**核心问题**：LINK 线程（管理状态）和 PHY 线程（读取并发送）并发访问缓冲区。

**方案：volatile 状态 + 单向状态流转 + 临界区保护状态切换**

1. **状态变量**使用 `volatile`，确保两个线程看到最新值
2. **数据区**的访问通过状态机保证互斥：
   - LINK/APP 只在 `FILLING` 状态写入 `data[]`
   - PHY 只在 `READY` 状态读取 `data[]`（读取后切换为 `SENDING`）
   - 同一个缓冲区不会同时被写入和读取
3. **状态切换**使用 `taskENTER_CRITICAL()` 保护（仅保护状态变量赋值，几条指令）

```c
// LINK/APP 侧：填充完成
taskENTER_CRITICAL();
buf->state = BUF_STATE_READY;
taskEXIT_CRITICAL();

// PHY 侧：读取并发送
taskENTER_CRITICAL();
if (buf->state == BUF_STATE_READY) {
    buf->state = BUF_STATE_SENDING;
}
taskEXIT_CRITICAL();
// 此时 PHY 独占 buf->data[]，可安全 memcpy 到 DW1000 TX buffer
```

**为什么不需要 Mutex**：
- 状态流转是**单向**的，不存在 A→B 同时 B→A 的竞争
- 临界区只保护一个 `uint8_t` 状态变量，耗时 < 1us
- Cortex-M7 的 `volatile` 读写在 `uint8_t` 级别是原子的

### 3.4 测距周期保护

**规则**：数据帧请求（PULL_REQ）不可打断测距周期。

**实现**：PHY 层状态机检查（LINK 层状态不参与判断）

```c
// PHY 收到帧后的处理逻辑
static void handle_rx_ok(uint32_t status) {
    // ... 读帧、解析 ...

    switch (phdr.frame_type) {
        // 测距/发现帧：始终处理（这是测距周期内的正常帧）
        case UWB_FRAME_DISCOVERY_REQ:
        case UWB_FRAME_RANGING_REQ:
            handle_fast_reply(&mac, &phdr, payload, rx_ts);
            break;

        // 数据帧：仅在 PHY 空闲 (LISTENING) 时处理
        case UWB_FRAME_DATA_PREP_REQ:
        case UWB_FRAME_DATA_PULL_REQ:
        case UWB_FRAME_DATA_DONE:
            if (g_phy.state == UWB_PHY_STATE_LISTENING) {
                handle_data_fast_reply(&mac, &phdr, payload, rx_ts);
            } else {
                // PHY 正忙（TX_ON 或 RX_WINDOW），忽略数据帧请求
                // Tag 会重试，测距周期结束后可处理
                app_log_warn("DATA frame ignored: phy_state=%u", g_phy.state);
            }
            break;
    }
}
```

**为什么 PHY 状态检查足够**：
- PHY 在测距周期内状态为 `TX_ON` 或 `RX_WINDOW`
- PHY 在测距周期结束后回到 `LISTENING`
- 不需要查询 LINK 状态（减少跨线程通信）
- PHY 状态是实时更新的，判断准确

**为什么忽略是安全的**：
- Tag 侧有重试机制（PULL_REQ 失败会重发同一 seq）
- PHY 在测距周期结束后会回到 LISTENING，下次 PULL_REQ 可以处理
- 忽略的帧不会导致数据丢失，只会增加延迟

### 3.5 数据帧的"打断"场景分析

| 场景 | PHY 状态 | LINK 状态 | PULL_REQ 处理 |
|------|---------|----------|--------------|
| 空闲监听 | LISTENING | IDLE | 正常处理 |
| 测距发送中 | TX_ON | DISC_TX_REQ / RANG_TX | **忽略** |
| 测距接收窗口 | RX_WINDOW | DISC_RX_SLOTS / RANG_RX | **忽略**（RX 窗口内收到的是测距帧）|
| 测距 RX 窗口收到非测距帧 | RX_WINDOW | DISC_RX_SLOTS / RANG_RX | 理论上不应发生（地址过滤） |
| 数据帧发送后等待 DONE | LISTENING | IDLE | 正常处理（PULL_REQ 或 DONE）|

---

## 4. PHY → LINK 事件上报

PHY 层处理完快速应答后，仍需上报事件给 LINK/APP。

### 4.1 上报方式

使用 FreeRTOS Queue（`q_phy_link_evt`，depth=10）：

```c
typedef struct {
    uint8_t event_type;          // RX_OK, TX_DONE, RX_TIMEOUT, DATA_PREP_REQ, ...
    uint16_t window_id;
    uint16_t src_short;
    uint64_t rx_ts;
    uint64_t tx_ts;              // 如果发送了应答
    uint8_t reply_frame_type;    // 发送了什么应答帧
    UwbRxQuality quality;
} uwb_phy_link_evt_t;
```

### 4.2 上报时机

| 场景 | 上报内容 |
|------|---------|
| Anchor 收到 DISC_REQ 并发送了 ACK | RX(DISC_REQ) + TX(DISC_ACK, tx_ts) |
| Anchor 收到 RANG_REQ 并发送了 RESP | RX(RANG_REQ) + TX(RANG_RESP, tx_ts) |
| Anchor 收到 DATA_PREP_REQ 并回复 ACK | RX(DATA_PREP_REQ) + TX(DATA_PREP_ACK) |
| Anchor 收到 PULL_REQ 并发送 FRAGMENT | RX(PULL_REQ) + TX(DATA_FRAGMENT, tx_ts) |
| Anchor 收到 PULL_REQ 但数据未就绪 | RX(PULL_REQ) + TX(DATA_FRAGMENT_BUSY) |
| Tag 发送了 DISC_REQ/RANG_REQ | TX_DONE + tx_ts |

### 4.3 队列满的处理

- 使用 depth=10，足够容纳一次测距窗口内多个 Anchor 的应答事件
- 如果队列满，丢弃最旧的事件（或直接丢弃新事件并记录警告日志）
- 对快速应答无影响（应答已经在 PHY 层完成），只影响 LINK/APP 的结果通知

---

## 5. LINK 线程对 Tag 侧主动发送的处理

Tag 侧的主动发送（DISCOVERY_REQ、RANGING_REQ）仍由 LINK 层调度：

1. LINK 调度器决定发送时机
2. LINK 构造帧
3. LINK 通过 `q_link_phy_cmd`（depth=1）发送命令给 PHY
4. PHY 执行发送并开启 RX 窗口
5. PHY 上报 TX_DONE 和后续 RX 事件给 LINK

**这个路径不是快速应答路径**，不需要极致低延迟。使用命令队列是合理的。

---

## 6. 关键讨论点

### 6.1 数据帧的时序风险

**风险**：Tag 在 Anchor 数据未准备好时发送 PULL_REQ。

**缓解**：
- PHY 返回 BUSY 标志，Tag 重试同一 seq
- LINK 在 PREP_ACK 中可以包含预估准备时间（可选优化）
- PHY 上报 PULL_REQ 给 LINK 后，LINK 可以加速 APP 数据填充（提高 APP 任务优先级）

### 6.2 双缓冲的数据一致性

**风险**：APP 正在写入缓冲区时，状态还未切换为 READY，但 PHY 读到了旧的状态 FREE。

**分析**：不会发生。
- APP 写入期间状态为 FILLING
- APP 写完后才切换为 READY
- PHY 只读取 READY 状态的缓冲区
- 临界区保护状态切换，保证原子性

### 6.3 测距周期中 LINK 调度的数据准备

**问题**：如果 LINK 正在执行测距调度（发送 DISC_REQ/RANG_REQ），此时 APP 填充数据完成，LINK 来不及处理。

**缓解**：
- APP 填充完成后，直接操作缓冲区状态（`FILLING → READY`），不需要 LINK 介入
- LINK 只在收到 PHY 上报的 DATA_PREP_REQ 时通知 APP
- 后续的 FILLING → READY 由 APP 自行管理，LINK 只负责会话生命周期

### 6.4 PHY 层帧构造的复杂度

**风险**：PHY 层需要理解帧格式（MAC header、payload header），增加了复杂度。

**缓解**：
- 使用 `uwb_protocol.c` 中的公共函数（`uwb_build_frame()`、`uwb_parse_frame()`、编解码函数）
- PHY 层只需要调用这些函数，不需要理解业务语义
- 帧格式变更只修改 `uwb_protocol.c`，不影响 PHY 层

---

## 7. 架构总结

```
                         ┌──────────────────────┐
                         │   APP 线程            │
                         │  - 测距计算           │
                         │  - 数据填充           │
                         │  - DataService 发布   │
                         └──────────┬───────────┘
                                    │ g_app_evt_queue (depth=8)
                         ┌──────────┴───────────┐
                         │   LINK 线程           │
                         │  - 调度器             │
                         │  - 状态机             │
                         │  - 数据缓冲管理       │
                         │  - Tag 侧帧构造       │
                         └──────────┬───────────┘
                         ┌──────────┴───────────┐
                         │   共享三缓冲区         │
                         │  buf_A: state + data  │
                         │  buf_B: state + data  │
                         │  buf_C: state + data  │
                         │  volatile + 临界区     │
                         └──────────┬───────────┘
                         ┌──────────┴───────────┐
                         │   PHY 线程            │
                         │  - DW1000 硬件操作    │
                         │  - ★ 快速应答构造     │
                         │  - ★ 从缓冲区读取发送 │
                         │  - 中断处理           │
                         └──────────┬───────────┘
                                    │ SPI + IRQ
                              ┌─────┴──────┐
                              │   DW1000    │
                              └────────────┘
```

**PHY 层直接处理的帧**：DISCOVERY_ACK、RANGING_RESP、DATA_PREP_ACK、DATA_FRAGMENT、DATA_DONE_ACK
**LINK 层构造的帧**：Tag 侧的 DISCOVERY_REQ、RANGING_REQ、DATA_PREP_REQ、DATA_PULL_REQ、DATA_DONE

---

## 9. Tag 侧数据帧发起流程

### 9.1 Tag 侧发起流程

Tag 侧的数据帧请求由 **APP 层驱动**，LINK 层负责帧构造和调度：

```
APP 层:
  - 决定何时拉取数据
  - 管理 session 状态（请求 ID、片序号、重试计数）
  - 处理接收到的数据分片并重组

LINK 层:
  - 接收 APP 命令（UWB_APP_CMD_DATA_PREP_REQ, UWB_APP_CMD_DATA_PULL_REQ）
  - 构造 DATA_PREP_REQ / DATA_PULL_REQ 帧
  - 通过 q_link_phy_cmd 发送命令给 PHY
  - 处理 PHY 上报的 RX 事件（DATA_FRAGMENT）
  - 上报 APP: 收到的数据分片
```

### 9.2 Tag 侧时序

```
APP                          LINK                     PHY
 │                            │                        │
 ├─ UWB_APP_CMD_DATA_PREP_REQ ┤                        │
 │                            ├─ 构造帧 ──────────────→│
 │                            │                        ├─ 发送 DATA_PREP_REQ
 │                            │                        ├─ 开启 RX 窗口
 │                            │                        │
 │                            │←─ RX(DATA_PREP_ACK) ───┤
 │←─ UWB_LINK_EVT_DATA_PREP_ACK ┤                     │
 │                            │                        │
 ├─ UWB_APP_CMD_DATA_PULL_REQ(seq=0) ┤                │
 │                            ├─ 构造帧 ──────────────→│
 │                            │                        ├─ 发送 PULL_REQ
 │                            │                        ├─ 开启 RX 窗口
 │                            │                        │
 │                            │←─ RX(DATA_FRAGMENT) ───┤
 │←─ UWB_LINK_EVT_DATA_FRAGMENT(seq=0, data) ┤       │
 │                            │                        │
 ├─ APP 重组数据...           │                        │
 │                            │                        │
 ├─ UWB_APP_CMD_DATA_PULL_REQ(seq=1) ┤                │
 │                            │ ...                    │
 │                            │                        │
 ├─ UWB_APP_CMD_DATA_DONE ───┤                        │
 │                            ├─ 构造帧 ──────────────→│
 │                            │                        ├─ 发送 DATA_DONE
 │                            │                        │
 │                            │←─ RX(DATA_DONE_ACK) ───┤
 │←─ UWB_LINK_EVT_DATA_SESSION_DONE ┤                 │
```

### 9.3 Tag 侧重试机制

```c
// APP 侧状态管理
typedef struct {
    uint16_t session_id;
    uint16_t request_id;
    uint16_t source_addr;      // Anchor 的 short_addr
    uint8_t  expected_seq;     // 期望的下一个片序号
    uint8_t  retry_count;      // 当前片的重试次数
    uint32_t last_request_ms;  // 上次请求时间
    uint8_t  *reassemble_buf;  // 重组缓冲区
    uint16_t reassemble_pos;   // 重组位置
} uwb_tag_data_session_t;
```

**重试规则**：
- 单片最大重试次数: `UWB_DATA_MAX_RETRY_PER_FRAGMENT = 3`
- 重试间隔: `UWB_DATA_RETRY_INTERVAL_MS = 100`
- 超时放弃: `UWB_DATA_SESSION_TIMEOUT_MS = 5000`

---

## 10. 日志记录策略

### 10.1 PHY 层日志

PHY 层快速应答需要记录日志，直接调用 `app_log_*` 宏：

**约束**：
- PHY 线程优先级高（AboveNormal），日志服务线程优先级低
- 日志队列可能满，PHY 层日志调用需要非阻塞

**实现**：
- PHY 层日志使用 `xQueueSend(queue, &item, 0)` 非阻塞发送
- 如果队列满，丢弃日志并增加计数器（不阻塞 PHY 线程）
- 定期上报计数器给 APP（统计日志丢失情况）

### 10.2 关键日志点

| 场景 | PHY 层日志 | LINK 层日志 |
|------|-----------|------------|
| 收到 DISC_REQ 并发送 ACK | `[PHY] [RX_DISC_REQ] src=0x%04X → TX_ACK slot=%u delay=%u tx_ts=...` | `[LINK] [ANCHOR_DISC_DONE] src=0x%04X` |
| 收到 RANG_REQ 并发送 RESP | `[PHY] [RX_RANG_REQ] src=0x%04X → TX_RESP slot=%u tx_ts=...` | `[LINK] [ANCHOR_RANG_DONE] src=0x%04X` |
| 收到 DATA_PREP_REQ | `[PHY] [RX_DATA_PREP] src=0x%04X free_buf=%u → TX_ACK status=%u` | `[LINK] [SESSION_CREATE] id=%u src=0x%04X` |
| 收到 PULL_REQ 且缓冲 READY | `[PHY] [RX_PULL] seq=%u → TX_FRAG len=%u tx_ts=...` | `[LINK] [DATA_SENT] seq=%u` |
| 收到 PULL_REQ 且缓冲 FILLING | `[PHY] [RX_PULL] seq=%u → TX_FRAG status=BUSY` | `[LINK] [DATA_BUSY] seq=%u` |
| PHY 忽略数据帧（测距周期） | `[PHY] [DATA_IGNORED] type=%u phy_state=%u` | - |

---

## 11. 三缓冲释放时机

**决策：等待下一片确认**

```
时间线:
t0: 发送片 N   → buf_A: SENDING → HELD
t1: 发送片 N+1 → buf_B: SENDING → HELD, buf_A 仍为 HELD
t2: 发送片 N+2 → buf_C: SENDING → HELD, buf_A 释放为 FREE
                 ↑
                 此时 buf_A 可以用于准备片 N+3
```

**为什么需要等待下一片确认**：
- 确保 Tag 有足够时间请求重发上一片（HELD 保留两片备份）
- 如果片 N 丢失，Tag 可以重试，Anchor 从 buf_A（HELD）重新发送
- 片 N+1 发送成功，说明 Tag 已经收到片 N，可以安全释放 buf_A

**释放逻辑**：
```c
// PHY TX_DONE 处理
void on_tx_done(uint8_t buf_index) {
    g_tx_buf[buf_index].state = BUF_STATE_HELD;
    
    // 查找上一片的 HELD 缓冲区（fragment_seq == current_seq - 1）
    uint8_t prev_seq = g_tx_buf[buf_index].fragment_seq;
    if (prev_seq == 0) return;  // 第一片没有前一片
    
    for (int i = 0; i < UWB_DATA_TX_BUF_NUM; i++) {
        if (g_tx_buf[i].state == BUF_STATE_HELD &&
            g_tx_buf[i].fragment_seq == prev_seq - 1 &&
            g_tx_buf[i].session_id == g_tx_buf[buf_index].session_id) {
            g_tx_buf[i].state = BUF_STATE_FREE;
            notify_link_buf_released(i);
            break;
        }
    }
}
```

---

## 12. Session 管理策略

**决策：单 Session**

- Tag 同时只能从一个 Anchor 拉取数据
- 简化了缓冲区管理和 session 状态机
- 如果需要从另一个 Anchor 拉取数据，必须先完成当前 session（发送 DATA_DONE）

**Session 生命周期**：
```
IDLE ── DATA_PREP_REQ ─→ ACTIVE ── DATA_DONE/TIMEOUT ─→ IDLE
```

**单 Session 的缓冲区占用**：
- 一个 session 最多占用 3 个缓冲区（HELD + SENDING + FILLING）
- 系统只需要 3 个数据缓冲区（不需要更多）
- 如果未来支持多 session，需要增加缓冲区数量

---

## 13. 实现要点总结

### 13.1 PHY 层需要新增的能力

1. 帧构造能力：调用 `uwb_build_frame()` 构造应答帧
2. 本地配置副本：`pan_id`, `short_addr`（初始化时从 LINK 获取）
3. 快速应答路径：识别 DISC_REQ / RANG_REQ / DATA_PREP_REQ / DATA_PULL_REQ / DATA_DONE
4. 缓冲区状态检查：检查三缓冲是否 FREE
5. 上报机制：通过 Queue 上报事件给 LINK

### 13.2 LINK 层需要新增的能力

1. Session 管理：创建、超时检测、释放
2. 缓冲区生命周期管理：通知 APP 填充、标记 READY
3. 数据帧事件处理：处理 PHY 上报的 DATA_PREP_REQ / DATA_FRAGMENT 事件
4. Tag 侧帧构造：构造 DATA_PREP_REQ / DATA_PULL_REQ / DATA_DONE 帧

### 13.3 APP 层需要新增的能力

1. 数据填充回调：响应 LINK 的填充请求，写入数据到指定缓冲区
2. 数据重组：Tag 侧接收 DATA_FRAGMENT 并重组
3. Session 状态管理：Tag 侧管理请求 ID、片序号、重试计数

---

## 14. 最终架构图

```
                         ┌──────────────────────┐
                         │   APP 线程            │
                         │  - 测距计算           │
                         │  - 数据填充/重组      │
                         │  - DataService 发布   │
                         └──────────┬───────────┘
                                    │ g_app_evt_queue (depth=8)
                         ┌──────────┴───────────┐
                         │   LINK 线程           │
                         │  - 调度器             │
                         │  - 状态机             │
                         │  - Session 管理      │
                         │  - 三缓冲生命周期     │
                         │  - Tag 侧帧构造       │
                         └──────────┬───────────┘
                         ┌──────────┴───────────┐
                         │   共享三缓冲区         │
                         │  buf_A: state + data  │
                         │  buf_B: state + data  │
                         │  buf_C: state + data  │
                         │  volatile + 临界区     │
                         └──────────┬───────────┘
                         ┌──────────┴───────────┐
                         │   PHY 线程            │
                         │  - DW1000 硬件操作    │
                         │  - ★ 快速应答构造     │
                         │  - ★ 三缓冲读取发送   │
                         │  - ★ 测距周期保护     │
                         │  - 中断处理           │
                         └──────────┬───────────┘
                                    │ SPI + IRQ
                              ┌─────┴──────┐
                              │   DW1000    │
                              └────────────┘
```

**数据流**：
- **Tag 侧**：APP 发起 → LINK 构造帧 → PHY 发送 → PHY 收到应答 → PHY 上报 → LINK 处理 → APP 接收
- **Anchor 侧**：PHY 收到请求 → PHY 直接应答 → PHY 上报 → LINK 管理 session/缓冲 → APP 填充数据

### 8.1 两阶段确认机制

PHY 收到 DATA_PREP_REQ 后，采用"快速初步响应 + LINK 确认"的协同机制：

```
PHY 收到 DATA_PREP_REQ:
  │
  ├─① 检查缓冲区状态（FREE 缓冲数量）
  │     - >= 1 个 FREE → 可以接收，初步回复 ACK(status=ok_pending)
  │     - 无 FREE → 暂时忙碌，回复 ACK(status=busy)
  │
  ├─② 快速发送 ACK（不超过 3ms CPU deadline）
  │     PHY 使用快速应答路径，构造并发送 ACK
  │
  ├─③ 上报 LINK：DATA_PREP_REQ 收到
  │     包含: request_id, session_id, max_fragment_len
  │
  └─④ LINK 确认 session 创建
        LINK 检查:
        - 是否有空闲 session slot？
        - 是否有足够内存？
        - 设备是否允许接收数据？
        
        确认结果通过 notify 或 queue 告知 PHY（可选）:
        - 成功: PHY 无需额外动作（ACK 已发送）
        - 失败: PHY 需发送修正帧（DATA_ABORT）
```

### 8.2 为什么需要两阶段确认

**问题**：PHY 层只看到缓冲区状态（FREE/FILLING），不知道全局 session 限制。

**解决**：
- PHY 快速检查并回复，满足 3ms deadline
- LINK 后续确认 session 创建，如果失败则发送修正帧（DATA_ABORT）

**风险**：如果 PHY 回复 ok 但 LINK 确认失败，Tag 侧会认为 session 已建立。

**缓解**：
- PHY 回复的 ACK 状态包含 `pending` 标志，表示"初步确认，待最终确认"
- Tag 侧收到 `pending` 后，等待第一个 `DATA_FRAGMENT` 才认为 session 真正建立
- LINK 确认失败时，后续 PULL_REQ 会收到 BUSY，Tag 侧自然放弃

### 8.3 session 状态管理

LINK 层管理 session 列表：

```c
typedef struct {
    uint16_t session_id;
    uint16_t request_id;
    uint16_t requester_addr;    // Tag 的 short_addr
    uint32_t start_time_ms;
    uint32_t last_activity_ms;
    uint8_t  state;             // ACTIVE / TIMEOUT / DONE
    uint8_t  buf_index[3];      // 关联的三缓冲索引
} uwb_data_session_t;

#define UWB_DATA_SESSION_MAX  2   // 最大并发 session 数
```

**session 创建时机**：
- PHY 上报 DATA_PREP_REQ 后，LINK 检查并创建 session
- session 关联三缓冲区，标记为"该片准备中"

**session 超时**：
- `UWB_DATA_SESSION_TIMEOUT_MS = 5000`
- 超时后 LINK 释放缓冲区，通知 APP session 结束
