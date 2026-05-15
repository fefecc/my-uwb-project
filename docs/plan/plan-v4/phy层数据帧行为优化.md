# PHY 层数据帧行为优化

> 创建时间: 2026-05-13
> 所属: plan-v4

## 核心变更：职责回缩

原方案中 PHY 层承担了挂起分片队列（3 slot pending）、查表匹配发送、NACK 重发备份等复杂逻辑。现将这些职责全部上移到 LINK 层，PHY 层在数据帧交互中只做三件事：**发请求**、**收请求后快速应答**、**把收到的帧位置交给 LINK**。

## 发送（Tag 侧请求帧）

与 Discovery 发送流程完全一致：

```
LINK 构建帧 → Alloc slot → 填帧数据
→ CMD(slot_index, has_pending_rx=true, rx_timeout=标准槽超时, rx_slot_count=1)
→ PHY 发送帧 → TX_DONE → 进入 RX_SLOT 模式（1 个槽，超时同 Discovery）
→ RX 完成 → 放回 IDLE 监听
```

- **不新增 CMD 类型**，继续使用 `PHY_CMD_TX_FRAME`
- `rx_slot_count = 1`，只需接收一个应答帧
- `rx_timeout_us` 使用现有的 `UWB_PHY_RX_SLOT_TIMEOUT_US`

## 应答（Anchor 侧收帧后）

PHY 在 IDLE 监听状态下收帧时，解析功能码决定应答行为。现有机制为 `build_fast_reply_ack()` 构建时间应答包（DISC_RESP / TWR_ANCHOR_RESP）。数据帧交互新增一种快速应答：

### ACK

- **触发**: 收到 `DATA_CFG_REQ`
- **帧内容**: 仅 ACK 标志位，无扩展头、无负载
- **发送方式**: delayed TX（与 Discovery RESP 相同机制）
- **后续**: 发送完成后回退 IDLE 监听，同时将收到的原始帧 slot 位置上报 LINK

### 其他数据帧

收到 `DATA_CTRL` (GET_INFO / PULL / NACK / DONE) 时，PHY **不做快速应答**，直接通过 `PHY_EVT_RX_SLOT_DONE` 将收到的帧 slot 位置上报 LINK。LINK 通知 APP，APP 准备分片后通过 `CMD(SEND_FRAG)` → LINK `PHY_CMD_TX_FRAME` → PHY 发送 `DATA_FRAG` 响应。这是一个 APP 驱动的同步流程，每片一次往返。

### 应答行为汇总

| 收到帧类型 | PHY 应答 | PHY 后续动作 |
|-----------|---------|-------------|
| DATA_CFG_REQ | ACK (fast-reply, delayed TX) | 上报收帧位置给 LINK |
| DATA_CTRL (任意) | 无 | 上报收帧位置给 LINK，由 APP→LINK→PHY 链回复 |
| 未知功能码 | 无快速应答 | 上报收帧位置给 LINK，LINK 判断后回复 ERROR 或丢弃 |

## Slot 内存管理：全部由 LINK 层回收

所有 slot 的释放统一由 LINK 层负责，PHY 层不再调用 `UwbSlots_Free()`。PHY 在上报事件时携带 `slot_index`，由 LINK 决定何时释放。

**当前机制（需改动）**: PHY 在 `TX_DONE` 时直接 `UwbSlots_Free(g_phy.tx_slot)`，然后上报 `PHY_EVT_TX_DONE` 且 `slot_index = -1`，LINK 无法知道是哪个 slot。

**新机制**: PHY 在所有事件中原样传递 `slot_index`，LINK 根据业务逻辑决定释放时机。

### 具体改动

**PHY 层（删除所有 `UwbSlots_Free()` 调用，改为事件携带 slot_index）：**

| 位置 | 当前行为 | 新行为 |
|------|---------|-------|
| TX_DONE FINISH 分支 | `UwbSlots_Free(tx_slot)` → `slot_index = -1` | 不 free，`slot_index = g_phy.tx_slot` |
| RX_SLOT 收帧 | 已是 `slot_index = g_phy.rx_slot` | 不变，LINK 回收 |
| fast_reply_active 分支 | `tx_slot = -1`（用 tx_buf，无 slot） | 不变 |
| watchdog / error | `UwbSlots_Free(tx_slot/rx_slot)` | 不 free，事件携带 slot_index |
| 硬件错误 (TX 失败等) | free slot | 不 free，事件携带 slot_index |

**LINK 层（新增回收逻辑）：**

| 事件 | LINK 处理 |
|------|---------|
| `PHY_EVT_TX_DONE` | 从 `evt.slot_index` 取 slot，`UwbSlots_Free()` |
| `PHY_EVT_RX_SLOT_DONE` | 已有回收逻辑，不变 |
| `PHY_EVT_RX_WINDOW_END` | 已有回收逻辑，不变 |
| `PHY_EVT_DATA_RX` | LINK 处理完毕后释放 |
| `PHY_EVT_DATA_ERROR` | 释放相关 slot |

### 数据帧 slot 管理（LINK 层独有逻辑）

| 职责 | 说明 |
|------|------|
| 数据帧 slot 分配 | **LINK 层 Alloc** |
| 数据帧 slot 释放 | **LINK 层在确认不需要后 Free**（可能保留为备份） |
| 重发备份 | **LINK 保留已发送的 slot**，NACK 时重新下发 CMD |
| 逐片流水线 | **LINK 在收到 APP 命令后立即下发 PHY_CMD_TX_FRAME** |

## Tag 侧 ERROR 处理

Tag 连续收到 2 个 ERROR 帧 → APP 下发 `DATA_RESET` 给 LINK（LINK 批量回收全部数据 slot），APP 自身会话复位，下次 1s 周期重启。详细流程见 [app层数据交互帧方案.md](app层数据交互帧方案.md)。

```c
// uwb_app.c Tag 侧逻辑 — APP 层维护 ERROR 计数
uint8_t data_error_count = 0;

// 收到 UWB_LINK_APP_EVT_DATA_ERROR 时:
data_error_count++;
if (data_error_count >= 2) {
    // 1. 通知 LINK 复位数据槽
    UwbLink_SendCmd(&(UwbLinkCmd){ .type = UWB_LINK_CMD_DATA_RESET });
    // 2. 复位自身会话
    session_reset();
}
// 收到正常应答帧时:
data_error_count = 0;  // 任何正常帧清零
```

## 改动范围

### 需要修改的文件

| 文件 | 改动 |
|------|------|
| `uwb_buffers.h` | 无需新增 CMD/事件类型；`phy_evt_t` 扩展 `slot_index` 承载（已在 slot 管理改动中覆盖） |
| `uwb_phy.c` | IDLE 监听收帧处理：新增 DATA_CFG_REQ 快速 ACK 应答逻辑（复用现有 fast-reply 机制，不新增 CMD）；所有 `UwbSlots_Free()` 移除，事件携带 slot_index |
| `uwb_phy.h` | 新增 `build_fast_reply_data_ack()` 声明 |
| `uwb_link.c` | 数据帧状态机；slot 生命周期管理；备份逻辑；逐片下发策略 |
| `uwb_link.h` | 新增 LINK DATA 状态和事件 |
| `uwb_app.c` | MAC 表；1s 轮询；会话状态机；数据切片/拼装/CRC；ERROR 计数和会话复位 (详见 [app层数据交互帧方案.md](app层数据交互帧方案.md)) |
| `uwb_protocol.h` | 新增 DATA 帧常量（CTRL 类型、RESP 类型、flags） |
| `uwb_stack_types.h` | MAC 表结构体；DATA 事件结构体 |

### 不需要修改的部分

- DW1000 SPI 收发、中断处理、时间戳读写（底层不变）
- Discovery 和 Ranging 的 FSM 逻辑（独立运行）
- Slot 池本身（只调大小，结构不变）
- TX/RX 状态机骨架（步骤步骤不变，扩展新分支）