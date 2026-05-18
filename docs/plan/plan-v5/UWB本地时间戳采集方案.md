# UWB 本地时间戳采集方案

## 目标

在 PHY 层每次收发帧时，采集本地单调时钟 (`local_ms`) 写入 slot，作为该帧的精确时间。
APP 层根据这个时间输出带时间戳的测距/数据记录，取代当前 `HAL_GetTick()` 日志打印时间。

## 现状问题

**当前时间来源混乱：**

| 层 | 时间来源 | 问题 |
|----|----------|------|
| PHY 收帧 | `s->rx_ts = UwbPhy_ReadRxTimestamp()` | DW1000 40-bit 时间戳，仅用于 TWR 计算 |
| PHY 发帧 | `s->tx_ts = UwbPhy_ReadTxTimestamp()` | 同上 |
| APP 层 `publish_range_result` | `TimeService_GetTimestamp()` | 取的是函数调用时刻，非帧时刻 |
| 日志 `log_twr_frame` | `HAL_GetTick()` 前缀 | 打印时刻，滞后于实际收发 |

**问题本质：** 没有一个统一的"帧时刻"概念。测距结果带的是 `HAL_GetTick()` 打印时间，DATA 事件带的是 `HAL_GetTick()` 打印时间，都不是帧在空中的精确时刻。

## 新方案

### 核心思路

PHY 层在每次 **接收完成** 和 **发送完成** 时，除了 DW1000 40-bit 时间戳外，额外读一次 `read_local_ms()` 写入 slot。

所有上层（LINK → APP → DataService）统一使用这个 `local_ms` 作为帧时间。

### slot 结构扩展

```c
typedef struct {
    volatile uwb_slot_owner_t owner;

    uint8_t  data[UWB_SLOT_DATA_SIZE];
    uint16_t data_len;

    uint16_t window_id;
    uint8_t  frame_type;
    uint16_t src_short;
    uint16_t session_id;
    uint8_t  frag_id;
    uint8_t  total_frags;
    uint8_t  data_flags;

    /* DW1000 精确时间戳 (40-bit, 用于 TWR 飞行时间计算) */
    uint64_t rx_ts;          /* 不变 */
    uint64_t tx_ts;          /* 不变 */

    /* ★ 新增: 本地单调时间 ms (用于输出记录和数据分析) */
    uint64_t rx_local_ms;    /* 接收完成时刻的 local_ms */
    uint64_t tx_local_ms;    /* 发送完成时刻的 local_ms */

    /* 快速应答 */
    uint8_t  reply_type;
    uint64_t reply_tx_ts;

    UwbRxQuality quality;
} uwb_slot_t;
```

### PHY 层改动

#### 1. 接收完成时记录 `rx_local_ms`

位置：`uwb_phy.c` 收帧处理（当前第 478 行附近）

```c
// 现有
s->rx_ts = UwbPhy_ReadRxTimestamp();

// 新增
s->rx_local_ms = read_local_ms_phy();   // 封装一下，不直接暴露 TimeService 内部
```

`read_local_ms_phy()` 是一个 inline 函数，读取 `g_local_sec_count` + `CNT` 拼成 `uint64_t` ms。不依赖 `TimeService_GetLocalClock()`（那个有关中断开销），直接读全局变量 + 寄存器。

#### 2. 发送完成时记录 `tx_local_ms`

位置：`uwb_phy.c` TX_DONE 处理（当前第 945 行附近）

```c
// 现有
uint64_t tx_ts = UwbPhy_ReadTxTimestamp();
s->tx_ts = tx_ts;

// 新增
s->tx_local_ms = read_local_ms_phy();
```

快速应答的 TX_DONE 同理（当前第 917 行附近）。

### LINK 层改动

LINK 层在解析 slot 上报事件时，把 `rx_local_ms` 传递到 APP 事件中。

#### `UwbTwrExchange` 扩展

```c
typedef struct {
    uint16_t anchor_id;
    uint16_t exchange_seq;
    uint16_t response_slot_id;
    uint16_t status_flags;

    /* DW1000 时间戳 (TWR 飞行时间计算用) */
    uint64_t tag_tx_ts;
    uint64_t anchor_rx_ts;
    uint64_t anchor_tx_ts;
    uint64_t tag_rx_ts;

    /* ★ 新增: 本地单调时间 */
    uint64_t tag_tx_local_ms;   /* Tag 发送时刻 */
    uint64_t tag_rx_local_ms;   /* Tag 接收应答时刻 */

    UwbRxQuality quality;
    uint8_t retry_count;
} UwbTwrExchange;
```

填充方式（`handle_disc_rx_slot_done`）：

```c
twr.tag_tx_local_ms = (tx_slot != NULL) ? tx_slot->tx_local_ms : 0;
twr.tag_rx_local_ms = s->rx_local_ms;
```

### APP 层改动

#### TWR 测距时间戳规则

| 模式 | 帧时间戳取值 | 说明 |
|------|-------------|------|
| DS-TWR | `(prev.tag_rx_local_ms + cur.tag_rx_local_ms) / 2` | 两次交换的接收时刻平均值 |
| SS-TWR | `cur.tag_rx_local_ms` | 单次交换直接用接收时刻 |

这个帧时间戳写入 `AppDataNode.timestamp`，不再调用 `TimeService_GetTimestamp()`。

#### 数据帧时间戳规则

- DATA 交互帧的时间戳 = 收到应答（ACK/FRAG）时刻的 `rx_local_ms`
- 与 TWR 相同，从 slot 的 `rx_local_ms` 传入

#### `UwbRangeResult` 扩展

```c
typedef struct {
    // ... 现有字段 ...

    /* ★ 新增: 帧时间戳 (单调本地 ms) */
    uint64_t frame_local_ms;   /* 该测距结果对应的精确帧时刻 */
} UwbRangeResult;
```

填充：

```c
// DS-TWR (compute_range)
out->frame_local_ms = (prev->tag_rx_local_ms + cur->tag_rx_local_ms) / 2;

// SS-TWR (compute_range_ss)
out->frame_local_ms = cur->tag_rx_local_ms;
```

`publish_range_result` 中：

```c
// 之前
(void)TimeService_GetTimestamp(&node.timestamp);

// 之后
node.timestamp.local_clock.sec  = result->frame_local_ms / 1000;
node.timestamp.local_clock.ms   = (float)(result->frame_local_ms % 1000);
node.timestamp.utc_valid        = false;  // UTC 由下游按 offset 换算
```

### Anchor 侧（应答方）

Anchor 在收到帧时同样记录 `rx_local_ms` 到 slot。如果 Anchor 需要记录接收时间（例如日志分析），直接使用 slot 中的 `rx_local_ms`，不依赖 `HAL_GetTick()`。

快速应答场景中，Anchor 的 `rx_local_ms` 记录的是 DISC_REQ/DATA_CFG_REQ 的到达时刻，用于 Anchor 侧事件上报。

### `read_local_ms_phy()` 实现

PHY 线程运行在 AboveNormal 优先级，读 `g_local_sec_count` 和 TIM16 CNT 是安全的。
不需要关中断（最坏情况：秒数偏差 1，和现有 `read_local_clock_unlocked` 的竞态处理一样）。

```c
static uint64_t read_local_ms_phy(void)
{
    extern volatile uint64_t g_local_sec_count;  // 声明在 time_service.c
    uint64_t sec = g_local_sec_count;
    uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim16);

    if (__HAL_TIM_GET_FLAG(&htim16, TIM_FLAG_UPDATE) != RESET &&
        cnt < 10000U) {
        sec++;
    }

    return sec * 1000U + (uint64_t)cnt * 1000U / 20000U;
}
```

或者把 `read_local_ms` 暴露为 `TimeService_GetLocalMs()` 公开 API，避免 extern 全局变量。

## 改动范围汇总

| 文件 | 改动 |
|------|------|
| `uwb_slots.h` | `uwb_slot_t` 增加 `rx_local_ms`, `tx_local_ms` |
| `uwb_stack_types.h` | `UwbTwrExchange` 增加 `tag_tx_local_ms`, `tag_rx_local_ms`; `UwbRangeResult` 增加 `frame_local_ms` |
| `uwb_phy.c` | 收帧/发帧完成时写 `rx_local_ms` / `tx_local_ms` |
| `uwb_link.c` | 填充 `UwbTwrExchange` 中的 local_ms 字段 |
| `uwb_app.c` | `compute_range` / `compute_range_ss` 计算 `frame_local_ms`; `publish_range_result` 使用它替代 `TimeService_GetTimestamp` |
| `time_service.h` | 新增 `TimeService_GetLocalMs()` 公开 API（可选） |

## 数据流图

```
Tag 侧:
  PHY TX_DONE  → tx_local_ms 写入 tx_slot
  PHY RX_DONE  → rx_local_ms 写入 rx_slot
       ↓
  LINK 从 tx_slot / rx_slot 读取 local_ms → 填入 UwbTwrExchange
       ↓
  APP compute_range:
    DS-TWR: frame_local_ms = (prev.rx + cur.rx) / 2
    SS-TWR: frame_local_ms = cur.rx
       ↓
  publish_range_result:
    node.timestamp ← frame_local_ms  (不再用 TimeService_GetTimestamp)
       ↓
  DataService → SD (精确帧时刻)

Anchor 侧:
  PHY RX_DONE → rx_local_ms 写入 slot
       ↓
  上报事件携带 rx_local_ms (Anchor APP 可用于日志/记录)
```

## 与偏移量方案的配合

本文档的 `frame_local_ms` 是本地单调时间（ms），与 [本地时钟偏移量方案](本地时钟偏移量方案.md) 的 `utc_offset` 配合：

```
utc_of_frame = frame_local_ms + utc_offset_ms
```

SD 日志输出时，可以同时写入 `frame_local_ms`（单调）和换算后的 UTC，分析脚本只需用 `frame_local_ms` 即可精确排序和对齐。
