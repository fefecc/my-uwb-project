# UWB 三层协议栈设计

## 实现状态总览

| 功能 | 状态 | 说明 |
|------|------|------|
| Discovery 帧交互 | ✅ 已实现 | Tag 发送 DISC_REQ，Anchor 按槽延迟应答 |
| 多槽接收 (Tag) | ✅ 已实现 | 4 个 RX 槽，每槽 2ms |
| Anchor 延迟应答 | ✅ 已实现 | 统一 DELAYED TX (>= 1ms) |
| DS-TWR 测距计算 | ✅ 已实现 | 双次交换 DS-TWR |
| DISC_RESP 携带时间戳 | ⏳ 计划中 | plan-v3 Phase 1 |
| 连续测距 (~90Hz) | ⏳ 计划中 | plan-v3 Phase 2 |
| 滑窗 DS-TWR | ⏳ 计划中 | plan-v3 Phase 3 |
| DATA 帧传输 | ⏳ 计划中 | 帧类型已定义，LINK 层未处理 |

---

## 1. 三层架构

```
APP 线程 (uwbApp, osPriorityNormal)
  │ g_app_evt_queue (depth=8)
  ▼
LINK 线程 (uwbLink, osPriorityAboveNormal)
  │ q_link_phy_cmd (发送命令, depth=1)
  │ q_phy_link_evt (事件上报, depth=PHY_EVT_POOL_NUM)
  ▼
PHY 线程 (uwbPhy, osPriorityAboveNormal)
  │ SPI + IRQ
  ▼
DW1000
```

### 1.1 PHY 层职责

PHY 层是 DW1000 硬件的直接管理者：

**核心功能**：
1. **帧发送**：执行 LINK 下发的发送命令，管理 TX 时序
2. **帧接收**：管理 RX 窗口，处理接收中断
3. **快速应答**：Anchor 收到 DISC_REQ 后直接构造并发送 DISC_RESP（零拷贝共享池）

**状态流转**：
```
发送完成 → 进入接收模式 (RX_SLOT)
接收完成 → 回到监听模式 (LISTENING)
```

**设计原则**：
- PHY 层不解析业务语义（如测距结果、距离值），只做帧级别的快速应答
- 发送/接收/监听的状态切换由硬件事件驱动，最小化延迟
- PHY 层保持"无状态"：不维护 session、不存储测距结果

### 1.2 LINK 层职责

LINK 层是协议调度的核心：

**核心功能**：
1. **帧类型调度**：决定下一帧应该发送什么类型
2. **状态机管理**：维护 Discovery / Ranging 的状态转换
3. **周期控制**：当前 Discovery 周期 200ms (`LINK_DISC_PERIOD_MS`)

**调度优先级**：
```
Ranging (测距帧) > Discovery (发现帧) > Data (消息帧)
```

**当前实现**：
- Tag 周期性发送 DISC_REQ，等待多槽接收
- 收到 `RX_WINDOW_END` 后输出汇总日志
- Anchor 在 PHY 层直接应答，LINK 层只做事件记录

### 1.3 APP 层职责

**核心功能**：
1. **测距计算**：接收 LINK 上报的 RangingRaw 事件，计算 TOF 和距离
2. **MAC 表维护**：维护已发现设备的地址映射表
3. **数据发布**：测距结果通过 DataService 发布

**DS-TWR 算法** (`uwb_app.c:compute_range()`):
```c
// 双次交换 DS-TWR
// Exchange 1: t1, t2, t3, t4
// Exchange 2: t5, t6, t7, t8
ra = t4 - t3;  // Anchor 第1次回复延迟
da = t6 - t5;  // Tag 第2次请求延迟
rb = t8 - t7;  // Anchor 第2次回复延迟
db = t4 - t1;  // Tag 第1次往返

tof = (ra * rb - da * db) / (ra + rb + da + db);
distance = tof * DW_TIME_UNIT * SPEED_OF_LIGHT - ANT_DELAY_COMP;
```

---

## 2. 已实现功能

### 2.1 Discovery 多槽接收 (plan-v2)

**Tag 侧**：

| 参数 | 值 |
|------|-----|
| RX 槽数量 | 4 |
| 每槽超时 | 2ms (`UWB_PHY_RX_SLOT_TIMEOUT_US = 2000U`) |
| 汇总日志 | LINK 层在 `RX_WINDOW_END` 后输出 |

**PHY 层事件流**：
```
TX_DONE → RX_SLOT_START
  ├─ Slot 0: RX_OK → RX_SLOT_DONE (slot_index=N, rx_seq=0)
  ├─ Slot 1: TIMEOUT → RX_SLOT_DONE (slot_index=-1, rx_seq=1)
  ├─ Slot 2: RX_OK → RX_SLOT_DONE (slot_index=M, rx_seq=2)
  └─ Slot 3: TIMEOUT → RX_SLOT_DONE (slot_index=-1, rx_seq=3)
→ RX_WINDOW_END (rx_seq=收帧总数)
```

**LINK 层处理**：
```c
// uwb_link.c:102-130
case PHY_EVT_RX_SLOT_DONE:
    g_link.rx_slot_results[evt.rx_seq] = evt.slot_index;
    if (evt.slot_index >= 0) {
        uwb_slot_t *s = UwbSlots_Get(evt.slot_index);
        // 处理帧数据
        UwbSlots_Free(evt.slot_index);  // 立即回收
    }
    break;

case PHY_EVT_RX_WINDOW_END:
    app_log_info("[LINK] RX_WIN ok=%u/%u r=[%d,%d,%d,%d]", ...);
    break;
```

### 2.2 Anchor 延迟应答 (plan-v2)

**槽分配公式**：
```c
#define ANCHOR_ADDR_BASE       0x30U
#define DISC_RX_SLOT_COUNT     4U
#define ANCHOR_REPLY_GUARD_US  1000U   /* 最小处理时间保护 */
#define DISC_SLOT_WIDTH_US     2000U   /* 每槽 2ms */

uint8_t assigned_slot = (g_phy.short_addr - ANCHOR_ADDR_BASE) % DISC_RX_SLOT_COUNT;
```

**延迟 TX 实现** (`uwb_phy.c:236-268`):
```c
// 收到 DISC_REQ 后
uint32_t delay_us = ANCHOR_REPLY_GUARD_US + assigned_slot * DISC_SLOT_WIDTH_US;
uint64_t tx_time = rx_ts + UwbPhy_UsToDwTime(delay_us);
dwt_setdelayedtrxtime((uint32_t)(tx_time >> 8));
dwt_starttx(DWT_START_TX_DELAYED);
```

**时序对齐** (假设 Tag 槽间处理间隙 Δ ≈ 0.05~0.15ms):
```
Tag 时间线 (相对 TX_STARTED):
  t≈0.1ms  : Slot 0 RX_ON  [0.1, 2.1]
  t≈2.1+Δ  : Slot 1 RX_ON  [2.1+Δ, 4.1+Δ]
  t≈4.1+2Δ : Slot 2 RX_ON  [4.1+2Δ, 6.1+2Δ]
  t≈6.1+3Δ : Slot 3 RX_ON  [6.1+3Δ, 8.1+3Δ]

Anchor 回复 (DELAYED TX):
  slot=0: tx_time = rx_ts + 1ms  → 到达 Tag ≈ 1.0ms ∈ Slot 0 ✓
  slot=1: tx_time = rx_ts + 3ms  → 到达 Tag ≈ 3.0ms ∈ Slot 1 ✓
  slot=2: tx_time = rx_ts + 5ms  → 到达 Tag ≈ 5.0ms ∈ Slot 2 ✓
  slot=3: tx_time = rx_ts + 7ms  → 到达 Tag ≈ 7.0ms ∈ Slot 3 ✓
```

### 2.3 层间通信 (零拷贝共享池)

```
LINK → PHY: q_link_phy_cmd (主命令, depth=1)
PHY → LINK: q_phy_link_evt (事件 + slot 指针, depth=PHY_EVT_POOL_NUM)
LINK → APP: UwbLink_AppEventQueue()
```

**共享内存池** (`uwb_shared_data_slot_t`):
- LINK 写帧到池，PHY 读取发送
- PHY 收帧写池，LINK 读取处理
- 所有权通过 `uwb_obj_ctl_t` (owner + state) 跟踪

---

## 3. 计划功能 (plan-v3)

> 详见 [docs/plan/plan-v3/滑窗TWR测距方案-v3.md](../plan/plan-v3/滑窗TWR测距方案-v3.md)

### 3.1 Phase 1: DISC_RESP 携带时间戳

**目标**：Anchor 在 DISC_RESP 中嵌入 `rx_ts` 和 `tx_ts`，供 Tag 侧 TWR 计算。

**当前问题**：
```c
// uwb_phy.c:164-165
frame.common.ext_header_len = 0;   // ← 没有扩展头
frame.common.payload_len    = 0;   // ← 没有负载
```

**修改方案**：
```c
// 新帧结构: MAC Header (9B) + Common Header (6B) + Ext Header (10B)
// Ext Header: rx_ts[5] + tx_ts[5]

static uint16_t build_fast_reply_ack(uint16_t dst_short, uint8_t seq,
                                     uint64_t rx_ts, uint64_t tx_ts)
{
    frame.common.ext_header_len = 10;
    frame.common.payload_len    = 0;

    // rx_ts: Anchor 收到 DISC_REQ 的时间 (5 bytes LE)
    UwbProtocol_WriteLe32(&frame.ext_header[0], (uint32_t)(rx_ts & 0xFFFFFFFF));
    frame.ext_header[4] = (uint8_t)(rx_ts >> 32);

    // tx_ts: Anchor 延迟发送的预计算时间 (5 bytes LE)
    UwbProtocol_WriteLe32(&frame.ext_header[5], (uint32_t)(tx_ts & 0xFFFFFFFF));
    frame.ext_header[9] = (uint8_t)(tx_ts >> 32);
    // ...
}
```

**调用顺序调整**：先计算 tx_time，再构建帧，最后写入 DW1000。

### 3.2 Phase 2: LINK 层连续测距

**目标**：去掉 200ms 固定周期，改为事件驱动，目标 ~90Hz。

**修改方案**：
- 收到 `RX_WINDOW_END` 后立即触发下一次 `DISC_REQ`
- 数据帧排队等待当前 DISC 交换完成

### 3.3 Phase 3: APP 层滑窗 DS-TWR

**目标**：复用相邻两次 DISC 交换的时间戳，实现滑窗 DS-TWR。

```
Exchange N:   t1_N → t2_N → t3_N → t4_N
Exchange N+1: t1_{N+1} → t2_{N+1} → t3_{N+1} → t4_{N+1}

DS-TWR (滑窗):
  ToF_N   = ((t4_N - t1_N) - (t3_N - t2_N)) / 2
  ToF_N+1 = ((t4_{N+1} - t1_{N+1}) - (t3_{N+1} - t2_{N+1})) / 2
  ToF_DS  = (ToF_N + ToF_N+1) / 2
```

**丢包退化**：丢包或数据帧间隔时自动退化 SS-TWR。

---

## 4. 数据帧设计 (未实现)

> 以下设计已完成规划，但尚未实现。帧类型已在 `uwb_protocol.h` 中定义。

### 4.1 帧类型

| 帧类型 | 值 | 用途 |
|--------|-----|------|
| DISCOVERY_REQ | 0x20 | Tag 发起发现 |
| DISCOVERY_RESP | 0x21 | Anchor 应答发现 |
| TWR_TAG_START | 0x40 | Tag 发起测距 |
| TWR_ANCHOR_RESP | 0x41 | Anchor 应答测距 |
| APP_DATA_CFG | 0x70 | 数据配置帧 |
| APP_DATA_FRAG | 0x71 | 数据分片帧 |
| APP_DATA_CTRL | 0x72 | 数据控制帧 |

### 4.2 数据传输流程 (规划)

```
Tag (主机)                          Anchor (从机)
  │                                      │
  ├─ DATA_PREP_REQ ─────────────────────→│
  │←─ DATA_PREP_ACK ────────────────────┤ PHY 直接回复
  │                                      │ LINK 通知 APP 填充数据
  │                                      │
  ├─ DATA_PULL_REQ(seq=0) ──────────────→│
  │←─ DATA_FRAGMENT(seq=0) ─────────────┤ 从共享三缓冲读取发送
  │                                      │
  ├─ DATA_PULL_REQ(seq=1) ──────────────→│
  │←─ DATA_FRAGMENT(seq=1) ─────────────┤
  │                                      │
  ├─ DATA_DONE ─────────────────────────→│
  │←─ DATA_DONE_ACK ────────────────────┤ 释放缓冲区
```

### 4.3 三缓冲设计 (规划)

```
状态流转: FREE → FILLING → READY → SENDING → HELD → FREE

三缓冲并发:
  t0: PHY 发送片 N   (buf_A: SENDING)
  t1: PHY 发送片 N+1 (buf_B: SENDING), buf_A → HELD (重发备份)
  t2: APP 填充片 N+2 (buf_C: FILLING)
  t3: buf_A 释放为 FREE (片 N 已确认)
```

**缓冲区结构**：
```c
typedef struct {
    volatile uint8_t state;     // FREE/FILLING/READY/SENDING/HELD
    uint8_t  fragment_seq;
    uint16_t session_id;
    uint16_t data_len;
    uint8_t  data[UWB_DATA_FRAGMENT_BODY_MAX_LEN];
} uwb_data_tx_buf_t;

#define UWB_DATA_TX_BUF_NUM  3   // 三缓冲
```

---

## 5. 关键约束

### 5.1 PHY 层

- 发送完成必进接收
- 接收完成必进监听
- 不存储业务状态
- 快速应答路径不调用阻塞 API

### 5.2 LINK 层

- Discovery 周期当前 200ms (plan-v3 将改为事件驱动)
- Ranging 连续不可打断
- Data 帧穿插在 Discovery/Ranging 之间

### 5.3 时间戳精度

- DW1000 40-bit 时间戳，精度 ~15ps
- 延迟 TX 精度 < 1ns
- 预计算 tx_time 可直接作为精确 TX 时间戳

---

## 6. 文件索引

| 文件 | 用途 |
|------|------|
| `APP/UWB/uwb_stack.c` | 协议栈入口，初始化三层 |
| `APP/UWB/uwb_phy.c` | PHY 层实现 |
| `APP/UWB/uwb_link.c` | LINK 层实现 |
| `APP/UWB/uwb_app.c` | APP 层实现，测距计算 |
| `APP/UWB/uwb_protocol.c` | 帧编解码 |
| `APP/UWB/uwb_buffers.c` | 静态内存池、队列 |
| `APP/UWB/uwb_stack_types.h` | 共享类型定义 |
| `APP/UWB/uwb_timestamp.c` | DW1000 时间戳工具 |

---

## 7. 参考文档

- [多槽接收方案-v2-Anchor.md](../plan/plan-v2/多槽接收方案-v2-Anchor.md)
- [多槽接收方案-v2-Tag.md](../plan/plan-v2/多槽接收方案-v2-Tag.md)
- [滑窗TWR测距方案-v3.md](../plan/plan-v3/滑窗TWR测距方案-v3.md)
