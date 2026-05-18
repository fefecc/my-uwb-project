# LINK 层架构描述

> 本文是 plan-v4 的主入口。方案原则是: APP 决定目标和会话, LINK 只执行传输, PHY 只作为收发工具层。

## 当前基线

当前代码已经具备以下基础:

- `uwb_protocol.h` 已定义 DATA 帧功能码: `UWB_FUNC_APP_DATA_CFG`、`UWB_FUNC_APP_DATA_FRAG`、`UWB_FUNC_APP_DATA_CTRL`、`UWB_FUNC_APP_DATA_CTRL_RESP`。
- `uwb_buffers.h/c` 已有 APP->LINK 命令队列, 当前深度为 1。
- `uwb_slots.h/c` 已扩展到 16 个 slot, 并包含 `UWB_SLOT_APP_OWN`。
- `uwb_phy.c` 已有 Anchor 侧 `DATA_CFG_REQ` 快速 ACK、`DATA_CTRL` 处理、`PHY_CMD_LOAD_PENDING` 和 `PHY_EVT_DATA_RETRY` 的工具能力。
- `uwb_link.c` 仍是测距主线实现, 数据帧路径处于预留状态: `UwbLink_SendCmd()` 直接返回失败, `PHY_EVT_RX_FRAME` 和 `PHY_EVT_DATA_RETRY` 尚未接入 DATA 处理。
- `uwb_app.c` 仍只处理测距事件, DATA 会话状态机尚未实现。

因此后续实施必须以当前代码为基线, 不能假设旧实施文档中的完整数据帧链路已经存在。

## 分层职责

### APP 层

APP 是业务和会话的拥有者:

- 选择数据帧目标 `target_id`, LINK 不维护 MAC 轮询策略。
- 生成并维护 `session_id`。
- 维护 Tag 侧和 Anchor 侧数据会话状态。
- Anchor APP 负责准备 meta、切片 payload、CRC16。
- Tag APP 负责分片拼装、CRC16 校验、失败复位策略。
- 通过 `UwbLink_SendCmd()` 下发具体传输命令。

### LINK 层

LINK 是传输执行者:

- 消费 APP->LINK 命令队列。
- 根据 APP 提供的 `target_id/session_id` 封装 DATA 帧。
- 解析 PHY 上报的 DATA 帧, 转换成 LINK->APP 事件。
- 管理 DATA 相关 slot 生命周期、重试备份、pending 重载和 `data_sm_reset()`。
- 执行测距优先的数据插入调度, 保证 DISC/TWR 帧内容不变。
- 不生成业务数据, 不选择数据目标, 不决定 APP 会话阶段。

### PHY 层

PHY 是收发工具层:

- 执行 LINK 下发的 `PHY_CMD_TX_FRAME`、`PHY_CMD_LOAD_PENDING`、`PHY_CMD_RESET` 等命令。
- 保留 Anchor 侧必要的 delayed TX 快速应答能力:
  - `DATA_CFG_REQ` -> 快速 `DATA_CTRL_RESP(ACK)`。
  - `DATA_CTRL(PULL)` -> 如果 pending frag 已加载, 直接 delayed TX 发出。
  - pending 为空时发送 WAIT 并上报 `PHY_EVT_DATA_RETRY`。
- 不维护 APP 会话状态, 不做业务决策。

## 线程与队列

```
uwbAppTask
    |
    | APP->LINK: UwbLinkCmd queue, depth=1
    v
uwbLinkTask
    |
    | LINK->PHY: phy_cmd_t queue, 当前代码 depth=2
    v
uwbPhyTask
    |
    | PHY->LINK: phy_evt_t queue, depth=12
    v
uwbLinkTask
    |
    | LINK->APP: UwbLinkAppEvent queue, depth=8
    v
uwbAppTask
```

队列深度以后以代码为准。若调度需要稳定容纳 2 个 DISC 加 1 个 DATA 命令, 再单独评估是否将 LINK->PHY 命令队列从 2 调整为 3 或 4。

## APP->LINK 接口

`UwbLinkCmd` 需要补齐 `target_id`, 由 APP 填写:

```c
typedef struct {
    UwbLinkCmdType type;
    int8_t   slot_index;   /* -1 = no slot */
    uint16_t target_id;    /* APP selected peer, LINK writes dst16 from this */
    uint16_t session_id;
    uint8_t  ctrl_type;
    uint8_t  frag_id;
    uint8_t  resp_type;
} UwbLinkCmd;
```

命令语义固定如下:

| 命令 | 发送方 | LINK 行为 |
|------|--------|-----------|
| `LINK_CMD_SEND_CFG_REQ` | Tag APP | 构建 `DATA_CFG_REQ`, `dst16=target_id`, 携带 `session_id` |
| `LINK_CMD_SEND_CTRL` | Tag APP | 构建 `DATA_CTRL`, 携带 `session_id + ctrl_type + frag_id` |
| `LINK_CMD_SEND_FRAG` | Anchor APP | 将 APP slot 中的 payload 封装为 `DATA_FRAG`, 预载到 PHY pending |
| `LINK_CMD_SEND_ACK` | Anchor APP | 构建 `DATA_CTRL_RESP`, 发送 ACK/WAIT/ERROR/STOP |
| `LINK_CMD_SESSION_RESET` | APP | 清理 DATA 资源, 不影响 DISC/TWR |

## LINK->APP 事件

| 事件 | 含义 |
|------|------|
| `UWB_LINK_APP_EVT_TWR_EXCHANGE` | 测距结果, 现有路径保持不变 |
| `UWB_LINK_APP_EVT_NEIGHBOR_SEEN` | 邻居发现 |
| `UWB_LINK_APP_EVT_DATA_CFG` | Anchor 收到 Tag 的 CFG 请求 |
| `UWB_LINK_APP_EVT_DATA_CTRL` | Anchor 收到 Tag 的 CTRL 请求 |
| `UWB_LINK_APP_EVT_DATA_FRAG` | Tag 收到 Anchor 的 DATA_FRAG, APP 处理后释放 slot |
| `UWB_LINK_APP_EVT_DATA_ACK` | Tag 收到 ACK/STOP |
| `UWB_LINK_APP_EVT_DATA_WAIT` | Tag 收到 WAIT, APP 决定延迟重试 |
| `UWB_LINK_APP_EVT_DATA_ERROR` | Tag 收到 ERROR, APP 决定复位 |
| `UWB_LINK_APP_EVT_DATA_COMPLETE` | LINK 确认 DATA 传输完成 |
| `UWB_LINK_APP_EVT_DATA_FAIL` | LINK 传输失败或重试耗尽 |

## 帧族与边界

DISC/TWR 帧内容不改:

- `UWB_FUNC_DISCOVERY_REQ`
- `UWB_FUNC_DISCOVERY_RESP`

DATA 帧由 LINK 封装:

- `UWB_FUNC_APP_DATA_CFG`: Tag->Anchor, ext header 携带 `session_id`。
- `UWB_FUNC_APP_DATA_CTRL`: Tag->Anchor, ext header 携带 `session_id + ctrl_type + frag_id`。
- `UWB_FUNC_APP_DATA_FRAG`: Anchor->Tag, ext header 携带 `session_id + frag_id + total_frags + flags`, payload 为 APP 准备的数据。
- `UWB_FUNC_APP_DATA_CTRL_RESP`: Anchor->Tag, fast response, 携带 `resp_type + extra`。此响应绑定当前 TX/RX 上下文, LINK 依据活跃 session 归属处理。

## 事件路由

LINK 统一从 PHY 事件队列取事件后分发:

```
PHY_EVT_RX_SLOT_DONE:
  frame_type == DISC_RESP       -> DISC/TWR 原路径, 解析时间戳并释放 RX slot
  frame_type == DATA_CTRL_RESP  -> Tag data transport, 释放响应 slot
  frame_type == DATA_FRAG       -> Tag data transport, 转交 APP, slot 由 APP 处理后释放
  other                         -> 释放 slot

PHY_EVT_RX_FRAME:
  frame_type == DATA_CFG        -> Anchor LINK 上报 DATA_CFG 给 APP
  frame_type == DATA_CTRL       -> Anchor LINK 上报 DATA_CTRL 给 APP
  other                         -> 按现有空闲帧策略处理并释放

PHY_EVT_TX_DONE:
  DISC TX                       -> 保持现有 TWR 时间戳处理和释放策略
  DATA TX                       -> data_sm 标记 TX 完成, 进入等待 RX/确认

PHY_EVT_RX_WINDOW_END:
  DISC                          -> outstanding--, disc_count++
  DATA                          -> 若在 DATA RX 等待中, 触发重试或失败

PHY_EVT_DATA_RETRY:
  Anchor DATA                   -> 从 LINK 备份 slot 重新加载到 PHY pending
```

## 调度策略

调度目标是测距不中断、数据不独占链路。

Tag 主循环按以下优先级执行:

1. 每轮先 `link_event_monitor()`: drain PHY events, drain APP commands, check timeouts。
2. 如果 DATA transport 正在等待 TX/RX 完成, 不提交新的 DATA 命令。
3. 如果 DISC pipeline 未满且没有 DATA 正在占用 PHY, 优先补 DISC。
4. 当至少完成 2 次 DISC 且 APP 命令队列有 DATA 命令时, 插入 1 次 DATA 交换。
5. 每次 DATA 交换完成后回到调度器, 允许再次补 DISC, 再消费下一条 APP DATA 命令。

这不是让 LINK 接管业务流程。APP 仍然逐条发布 CFG/CTRL/FRAG/ACK 命令; LINK 只决定这条命令何时安全地插入到 PHY 命令流。

## Slot 与复位

DATA slot 由 LINK 统一登记和回收:

- Tag TX slot: LINK 构建 DATA_CFG/DATA_CTRL 后持有, 完成或失败后释放。
- Anchor FRAG slot: APP 写 payload 后交给 LINK, LINK 加帧头并作为 pending/backup 管理。
- Tag RX FRAG slot: LINK 解析头后转交 APP, APP 读取 payload 后释放。
- PHY pending slot: 由 `PHY_CMD_LOAD_PENDING` 交给 PHY, `data_sm_reset()` 必须能清空。

`data_sm_reset()` 必须只清理 DATA 资源:

1. 回收 LINK 数据挂起表中的 slot。
2. 回收 DATA transport 当前 TX/RX/retry slot。
3. 下发 `PHY_CMD_LOAD_PENDING` with `slot_index=-1` 清 PHY pending。
4. 必要时下发 `PHY_CMD_RESET` 恢复 PHY 监听。
5. 丢弃残留 DATA 事件, 不破坏 DISC/TWR 正常路径。
6. 上报 `UWB_LINK_APP_EVT_DATA_FAIL` 或完成复位确认。

## 超时与错误

| 项目 | 建议值 | 处理 |
|------|--------|------|
| DISC command timeout | 100ms | 保持当前 LINK DISC 保护 |
| DATA single exchange timeout | 50ms | 重试当前 DATA 命令, 最多 3 次 |
| WAIT retry interval | 10ms | APP 决定何时重新下发同一 CTRL |
| DATA session timeout | 5s | APP 触发 `LINK_CMD_SESSION_RESET` |

ERROR 是硬错误: APP 收到 `UWB_LINK_APP_EVT_DATA_ERROR` 后应复位本次会话。WAIT 是软等待: APP 可延迟后重发当前命令。

