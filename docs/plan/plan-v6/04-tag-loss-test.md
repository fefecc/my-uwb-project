# Tag 按键丢包检测

## 目标

工作阶段短按按键后，Tag 进入持续丢包检测模式。进入后一直保持该模式，不再只检测前 3s；长按复位可以退出当前运行状态。

测试允许同时看到多个 Anchor。丢包检测按 `anchor_id + response_slot_id` 分开统计每个 Anchor/slot 的收包率，LED 显示当前最近 3s 内收包率最高的一组。

Tag 正常工作时允许 APP DATA 拉取。按键进入丢包检测时，Key 线程向 `lossTestTask` 发送启动命令，不需要关闭 DATA 拉取。丢包检测只接收 LINK 层测距 TX/RX 事件，DATA 帧不会进入丢包统计。

## 数据来源

测距丢包检测只使用 LINK 层事件，不从 APP 层测距结果统计。

测距开始后，LINK 层向丢包检测队列投递 TX 事件。该 TX 事件表示本轮测距请求已经进入 LINK 测距流程，用作收包率统计的总数：

```text
RANGE_TX {
  seq
}
```

LINK 层收到有效测距响应帧后，向丢包检测队列投递 RX 事件。TX 和 RX 使用同一套测距 seq；同一个 seq 可能来自多个 Anchor/slot 的响应，因此 RX 需要按 `anchor_id + response_slot_id` 分开计数，不做全局 seq 去重。

```text
RANGE_RX {
  seq,
  anchor_id,
  response_slot_id
}
```

DATA 帧不投递为 `RANGE_TX/RANGE_RX`，因此 DATA 会话和丢包统计天然隔离。

LINK 投递点：

- `RANGE_TX`：LINK 层发起测距请求时投递。
- `RANGE_RX`：LINK 层解出有效测距响应帧后投递。

这两个事件都发生在 APP 层测距计算、发布和过滤之前，因此不会被 APP 层结果处理影响。

## 线程与队列

`lossTestTask` 常驻运行，默认处于空闲状态。Key 短按进入丢包检测后，线程清空统计桶并进入预热期。

建议队列：

| 队列 | 方向 | 内容 | 满队列处理 |
|------|------|------|------------|
| `lossTestQueue` | `uwbLinkTask -> lossTestTask` | `RANGE_TX/RANGE_RX` | LINK 层非阻塞投递，满队列直接丢弃本次事件 |
| `lossCtrlQueue` | `keyTask -> lossTestTask` | 启动丢包检测命令 | 非阻塞或短等待投递 |
| `ledCmdQueue` | `lossTestTask -> ledTask` | 丢包显示模式和收包率 | `lossTestTask` 非阻塞投递，队列满直接放弃本次 LED 更新 |

`lossTestTask` 不直接操作 LED GPIO，只通过 `ledCmdQueue` 输出显示结果。LED 没有进入丢包显示模式或队列已满时，本次显示更新可以丢弃，不影响统计。

建议 LED 命令结构：

```c
typedef enum {
    LED_LOSS_MODE_OFF = 0,
    LED_LOSS_MODE_CONFIRM,
    LED_LOSS_MODE_RATE,
} LedLossMode;

typedef struct {
    LedLossMode mode;
    uint8_t valid;
    uint8_t rate;
} LedLossCmd;
```

`valid = 0` 表示窗口无效或无显示数据，`ledTask` 熄灭 LED0/LED1/LED2。`rate` 只在 `LED_LOSS_MODE_RATE` 且 `valid != 0` 时有效。

## 统计策略

1. 按键进入检测后，前 1s 为预热期：只接收 LINK 层事件，但不统计 `tx/rx`。
2. 预热结束后，使用分桶滑窗统计最近 3s 的测距 TX 和测距 RX，不保存逐事件环形缓冲。
3. `RANGE_TX` 进入当前桶的全局 TX 计数，用作每个 Anchor/slot 收包率的分母。
4. `RANGE_RX` 按 `anchor_id + response_slot_id` 找到对应统计项，累加当前桶的 RX 计数。
5. 同一个 seq 如果来自不同 Anchor/slot，要分别进入各自统计项；当前不做 seq 去重。
6. 每 500ms 桶到期后，向前滑动一个桶：最旧桶清零作为新桶；对 6 个桶求和计算收包率并更新 LED。
7. LED 显示最近 3s 内收包率最高的 Anchor/slot；关闭基站后所有桶过期，`tx_count == 0` 时窗口无效，LED0/LED1/LED2 自动熄灭。

时间桶参数：

```text
LOSS_WINDOW_MS  = 3000
LOSS_BUCKET_MS  = 500
LOSS_BUCKET_COUNT = 6
LOSS_PEER_MAX   = 8
```

桶结构（环形缓冲，6 个桶）：

```text
bucket[0..5] {
  tx_count
  peer[0..LOSS_PEER_MAX-1] {
    valid, anchor_id, response_slot_id, rx_count
  }
}
```

当前写入桶 = `bucket[cur]`，由 `HAL_GetTick()` 定位。每次处理事件时检查是否需要推进桶：

```text
if now - bucket[cur].start_ms >= LOSS_BUCKET_MS:
    cur = (cur + 1) % LOSS_BUCKET_COUNT
    清零 bucket[cur]
    bucket[cur].start_ms = now
    计算 LED 显示
```

计算 LED 显示时，对 6 个桶求和：

```text
total_tx = sum(bucket[i].tx_count)
total_rx[peer] = sum(bucket[i].peer[j].rx_count)
```

收包率使用总和比（而非每桶独立算率再取平均），避免 tx 较少的桶产生极端值：

```text
peer_rx_rate = total_rx[peer] * 100 / total_tx
display_rate = max(peer_rx_rate)
```

如果 `total_tx == 0`，窗口无效；如果 `total_tx > 0` 但没有任何 peer RX，则 `display_rate = 0`。

超过 `LOSS_PEER_MAX` 的新 `anchor_id + response_slot_id` 直接忽略，不影响已有 peer 统计。

## LED 规则

LED 使用 `display_rate`，按进度条样式累计点亮：

| 收包率范围 | LED 状态 |
|-----------|----------|
| `display_rate >= 95%` | LED80 常亮、LED90 常亮、LED95 常亮 |
| `90% <= display_rate < 95%` | LED80 常亮、LED90 常亮、LED95 熄灭 |
| `80% <= display_rate < 90%` | LED80 常亮、LED90 熄灭、LED95 熄灭 |
| `display_rate < 80%` | LED0/LED1/LED2 熄灭 |

当前映射：LED0 = LED80，LED1 = LED90，LED2 = LED95。LED3 继续作为系统工作闪烁灯。

按键进入丢包检测后，LED0/LED1/LED2 会先同时点亮作为模式启动确认；预热完成并产生 3s 窗口统计后，再按 80/90/95 阈值显示。

## SD 日志

丢包检测不写 SD 日志，也不使用 UWB SD FIFO。

因此不输出以下日志：

- `LOSS_TX`
- `LOSS_FRAME`
- `LOSS_SUMMARY`

丢包检测结果只通过 LED 展示。后续如果需要恢复离线分析，再单独增加编译开关和日志通道，不能作为当前测试链路的默认行为。
