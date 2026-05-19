# UWB 本地时间戳采集方案

## 目标

在 `PHY` 层为每一帧采集高精度本地单调时间，用于：

- 标定 `TWR` 交换对应的真实帧时刻
- 让 `UWB` 数据节点进入全局统一时间链路
- 避免继续使用 `HAL_GetTick()` 或“函数调用时刻”近似代替帧时刻

本方案与 [本地时钟偏移量方案](本地时钟偏移量方案.md) 配套：`UWB` 侧只负责把高精度本地帧时刻传上来，真正的 `UTC` 解析放到排序线程统一完成。

## 现状问题

当前时间来源不统一：

| 层 | 时间来源 | 问题 |
|----|----------|------|
| PHY 收帧 | `s->rx_ts = UwbPhy_ReadRxTimestamp()` | 仅用于 `DW1000` 飞行时间计算 |
| PHY 发帧 | `s->tx_ts = UwbPhy_ReadTxTimestamp()` | 同上 |
| APP `publish_range_result` | `TimeService_GetTimestamp()` | 取的是发布时刻，不是帧时刻 |
| 串口日志 `TWR_FRAME` | 日志前缀本地单调时钟 | 是打印时刻，不是帧在空中的精确时刻 |

问题本质：

- 现有 `UWB` 数据节点拿到的是“业务发布时刻”
- 不是“该结果对应的帧时刻”
- 下游即使再统一排序，也是在错误时刻基础上排序

## 新方案

### 核心思路

`PHY` 层在 **RX 完成** 和 **TX 完成** 时，除了保留 `DW1000 40-bit timestamp` 以外，再采集同一时刻对应的高精度本地 tick。

内部统一使用 `20kHz tick` 语义：

- 不再使用 `rx_local_ms`
- 不再使用 `tx_local_ms`
- 不再使用 `frame_local_ms`

统一改成：

- `rx_local_tick_20k`
- `tx_local_tick_20k`
- `frame_local_tick_20k`

这些字段只表达“本地单调帧时刻”，不直接表达 `UTC`。

### 与全局时间系统的分工

`UWB` 链路只负责把帧的本地高精度时间传到 `AppDataNode` 生成点。

全局时间链路分工如下：

1. `PHY`：采样帧级 `local_tick_20k`
2. `LINK / APP`：传递和组合 `UWB` 帧时刻
3. 生成 `AppDataNode` 时：同时采样 `TimeCapture`
4. `AppDataSortTask`：按本地 tick 排序，统一解 `UTC`，统一格式化文本
5. `AppSdWriterTask`：只写最终文本块

## 数据结构调整

### slot 结构

`uwb_slot_t` 中新增高精度本地 tick 字段：

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

    uint64_t rx_ts;
    uint64_t tx_ts;

    uint64_t rx_local_tick_20k;
    uint64_t tx_local_tick_20k;

    uint8_t  reply_type;
    uint64_t reply_tx_ts;

    UwbRxQuality quality;
} uwb_slot_t;
```

约束：

- `rx_ts / tx_ts` 继续用于 `DW1000` 飞行时间计算
- `rx_local_tick_20k / tx_local_tick_20k` 用于全局时间对齐和数据排序

### LINK 层交换结构

`UwbTwrExchange` 改为传递本地高精度 tick：

```c
typedef struct {
    uint16_t anchor_id;
    uint16_t exchange_seq;
    uint16_t response_slot_id;
    uint16_t status_flags;

    uint64_t tag_tx_ts;
    uint64_t anchor_rx_ts;
    uint64_t anchor_tx_ts;
    uint64_t tag_rx_ts;

    uint64_t tag_tx_local_tick_20k;
    uint64_t tag_rx_local_tick_20k;

    UwbRxQuality quality;
    uint8_t retry_count;
} UwbTwrExchange;
```

### 测距结果结构

`UwbRangeResult` 中保存帧对应的本地高精度时刻：

```c
typedef struct {
    // ... existing fields ...

    uint64_t frame_local_tick_20k;
} UwbRangeResult;
```

## PHY 层采样规则

### RX 完成

`uwb_phy.c` 在收帧完成后：

```c
s->rx_ts = UwbPhy_ReadRxTimestamp();
s->rx_local_tick_20k = /* 读取当前本地 tick */;
```

要求：

- 使用时间服务提供的本地 tick 读取能力
- 不直接 `extern g_local_sec_count`
- `PHY` 不跨模块读取时间服务内部静态变量

### TX 完成

`uwb_phy.c` 在发帧完成后：

```c
s->tx_ts = UwbPhy_ReadTxTimestamp();
s->tx_local_tick_20k = /* 读取当前本地 tick */;
```

快速应答路径同样记录 `tx_local_tick_20k`。

## LINK / APP 传递规则

### LINK 层

`handle_disc_rx_slot_done()` 中填充：

```c
twr.tag_tx_local_tick_20k = (tx_slot != NULL) ? tx_slot->tx_local_tick_20k : 0;
twr.tag_rx_local_tick_20k = s->rx_local_tick_20k;
```

### APP 层

`TWR` 结果对应的帧时刻规则：

| 模式 | 帧时刻 |
|------|--------|
| DS-TWR | `(prev.tag_rx_local_tick_20k + cur.tag_rx_local_tick_20k) / 2` |
| SS-TWR | `cur.tag_rx_local_tick_20k` |

结果写入：

```c
out->frame_local_tick_20k = ...;
```

注意：这里仍然只得到“本地单调帧时刻”，不是最终 `UTC`。

## AppDataNode 入队规则

`UWB` 生成 `AppDataNode` 时，不再手工填写最终 `timestamp`，而是：

1. 业务字段写入 `payload.uwb`
2. 调用 `TimeService_CaptureNow()` 采样一个 `TimeCapture`
3. 将 `TimeCapture` 随 `AppDataNode` 一起入队

统一接口口径如下：

```c
typedef struct {
    uint64_t local_tick_20k;
    int64_t  utc_offset_tick_20k;
    bool     utc_valid;
    TimeSyncState sync_state;
    uint32_t sync_seq;
} TimeCapture;
```

```c
bool TimeService_CaptureNow(TimeCapture *out);
bool TimeService_ResolveCapture(const TimeCapture *cap, TimeTimestamp *out);
```

并明确：

- `AppDataNode` 入队时携带 `TimeCapture`
- 不是最终 `TimeTimestamp`

## 排序与写盘

### 排序线程

`AppDataSortTask` 负责：

1. 按 `AppDataNode.time_capture.local_tick_20k` 排序
2. 调用 `TimeService_ResolveCapture()` 统一解析 `UTC`
3. 生成标准文本格式：
   - 本地时钟
   - `UTC` 时钟
   - 消息来源
   - 数据内容
4. 写入 `SD FIFO`

### SD writer 线程

`AppSdWriterTask` 只做最终文本块写盘：

- 不排序
- 不换算 `UTC`
- 不修改字段
- 不重写文本

写入缓存区后的内容即为最终 `SD` 输出内容。

## 串口日志

串口 `TWR_FRAME` 仍按本地单调时钟记时，不输出 `UTC`。

说明：

- 串口日志适合观察实时行为和事件先后
- 不适合直接当作精确帧时刻来源
- 如果后续需要分析帧级时刻，应显式输出 `frame_local_tick_20k`
- 不应继续依赖日志前缀时间近似代替帧时刻

## 数据流图

```text
Tag 侧:
  PHY TX_DONE  -> tx_local_tick_20k 写入 tx_slot
  PHY RX_DONE  -> rx_local_tick_20k 写入 rx_slot
       |
       v
  LINK 从 tx_slot / rx_slot 读取本地 tick
       |
       v
  APP compute_range:
    DS-TWR -> frame_local_tick_20k = average(prev_rx, cur_rx)
    SS-TWR -> frame_local_tick_20k = cur_rx
       |
       v
  publish_range_result:
    填业务 payload
    同时调用 TimeService_CaptureNow()
       |
       v
  UWB -> DataService queue
    携带原始 TimeCapture
       |
       v
  AppDataSortTask:
    按 local_tick_20k 排序
    统一解析 UTC
    统一拼装文本
       |
       v
  AppSdWriterTask:
    只写最终文本块
```

## 不再采用的做法

以下旧做法不再成立：

- `extern g_local_sec_count` 直接跨模块读取时间服务内部状态
- `publish_range_result` 手工填写 `node.timestamp.local_clock`
- `node.timestamp.utc_valid = false; // UTC 由下游按 offset 换算`
- 使用 `local_ms` 作为 `UWB` 帧时刻统一语义

本方案只保留两类时间：

1. `DW1000` 的飞行时间戳
2. 本地高精度单调帧时刻 `local_tick_20k`

最终 `UTC` 始终由排序线程根据 `TimeCapture` 统一解析。
