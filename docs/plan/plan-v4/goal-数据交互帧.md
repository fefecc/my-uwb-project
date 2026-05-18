# 数据交互帧目标

## 目标

在现有 Discovery + TWR 测距持续运行的前提下, 增加 Tag 与 Anchor 之间的可靠数据交互帧。

核心目标:

- 数据帧目标由 APP 选择并通过 `target_id` 下发。
- APP 拥有业务会话, LINK 只负责执行传输。
- 测距帧内容不变, TWR 计算逻辑不变。
- 数据会话异常时只复位 DATA 资源, 不破坏 DISC/TWR 主链路。

## 设计原则

| 原则 | 说明 |
|------|------|
| APP 决定目标 | `target_id` 来自 APP, LINK 不维护 MAC 轮询和业务选择 |
| APP 决定会话 | `session_id` 由 APP 生成和维护 |
| LINK 只执行 | LINK 负责封帧、解析、调度、重试、slot 管理和事件上报 |
| PHY 只做工具 | PHY 负责 TX/RX、delayed TX、pending frag, 不做业务状态机 |
| 测距不改帧 | DISC_REQ/DISC_RESP 帧格式和测距计算路径保持现状 |
| 数据不独占 | 每次数据交换后回到调度器, 允许测距帧继续插入 |

## 交互阶段

### 1. 在线确认

Tag APP 选择 Anchor, 生成 `session_id`, 下发:

```
LINK_CMD_SEND_CFG_REQ(target_id, session_id)
```

LINK 构建 `DATA_CFG_REQ` 并发送。Anchor PHY 快速回复 ACK, 同时把收到的 CFG 帧上报 LINK。Anchor LINK 转发 `UWB_LINK_APP_EVT_DATA_CFG` 给 APP, 由 Anchor APP 建立会话并准备 meta。

### 2. 元信息获取

Tag APP 收到 ACK 后下发:

```
LINK_CMD_SEND_CTRL(target_id, session_id, GET_INFO, frag_id=0)
```

Anchor APP 收到 `DATA_CTRL(GET_INFO)` 事件后准备 meta payload slot, 下发 `LINK_CMD_SEND_FRAG`。LINK 将该 slot 封装为 `DATA_FRAG(meta)` 并预载到 PHY pending。Tag 收到后由 APP 解析总长度、分片数和整包 CRC。

### 3. 逐片拉取

Tag APP 按 frag_id 逐片下发:

```
LINK_CMD_SEND_CTRL(target_id, session_id, PULL, frag_id=N)
```

Anchor APP 根据 `DATA_CTRL(PULL)` 准备对应分片, 交给 LINK 封装为 `DATA_FRAG(data)`。Tag APP 收到分片后做单片 CRC 和整包拼装。CRC 失败时不递增 frag_id, 重新下发同一 PULL。

### 4. 结束

Tag APP 收齐并校验通过后下发:

```
LINK_CMD_SEND_CTRL(target_id, session_id, DONE, frag_id=0)
```

Anchor 回复 ACK 并清理会话。双方 DATA 状态回到空闲, 测距链路继续运行。

## 帧类型

| 功能码 | 方向 | 用途 |
|--------|------|------|
| `UWB_FUNC_APP_DATA_CFG` | Tag->Anchor | 发起数据会话 |
| `UWB_FUNC_APP_DATA_CTRL` | Tag->Anchor | GET_INFO / PULL / DONE / STOP |
| `UWB_FUNC_APP_DATA_FRAG` | Anchor->Tag | meta 或数据分片 |
| `UWB_FUNC_APP_DATA_CTRL_RESP` | Anchor->Tag | ACK / WAIT / STOP / ERROR |

## 约束

| 项目 | 值 |
|------|----|
| 单次最大数据 | 512 bytes |
| 分片净载荷 | 62 bytes |
| 最大分片数 | 9 |
| APP->LINK 命令队列 | depth=1 |
| LINK 数据挂起表 | 4 entries |
| PHY pending frag | 1 entry |
| 单次 DATA 交换重试 | 3 |
| DATA 单交换超时 | 50ms |
| DATA 会话超时 | APP 管理, 建议 5s |
| WAIT 延迟重试 | APP 管理, 建议 10ms |

## 成功标准

- Tag 持续收到 `UWB_LINK_APP_EVT_TWR_EXCHANGE`。
- Tag APP 能选择 Anchor 并发起数据会话。
- Anchor 收到 CFG/CTRL 后由 APP 决定准备 meta/frag/ack。
- LINK 不选择目标、不生成业务 payload。
- PHY 不维护 APP 状态, 只提供快速 ACK 和 pending frag 工具能力。
- DATA 失败复位后 DISC/TWR 仍继续。

## 非目标

- 本阶段不改 DW1000 底层驱动。
- 本阶段不改 DISC/TWR 帧内容。
- 本阶段不让 LINK 层维护 MAC 轮询策略。
- 本阶段不把 PHY 变成业务状态机。

