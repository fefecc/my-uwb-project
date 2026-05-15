# 数据交互帧 — 中目标

> 创建时间: 2026-05-13
> 所属: plan-v4

## 目标

在已完成 Discovery + Ranging 的基础上，实现 Tag 与 Anchor 之间的可靠数据传输流程。

## 三个阶段

数据传输由三个阶段组成，逐次握手、逐层确认：

### 阶段 1: 在线确认

Tag 向 Anchor 发送 `DATA_CFG_REQ`，Anchor PHY 层立即回复 ACK。这一帧的作用是确认对方设备在线、信道畅通，并在 Anchor APP 层建立数据会话（准备发送缓冲区、通知 LINK 层进入 DATA 传输模式）。此阶段不携带业务数据，仅握手信号。

### 阶段 2: 元信息获取

Tag 在收到 ACK 后发送 `DATA_CTRL(GET_INFO)`，Anchor 返回 `DATA_FRAG(meta)` 帧，携带本次传输的控制参数：数据类型、总长度、分片数量、整包 CRC16。Tag 依靠这些参数完成后续分片的拼装与校验。元信息帧与数据分片帧使用相同帧类型 (`DATA_FRAG`)，通过 `flags` 字段中的 `meta` 位区分。

### 阶段 3: 逐片数据拉取 + 结束信号

Tag 循环发送 `DATA_CTRL(PULL, frag_id)` 逐片拉取数据，Anchor LINK 上报 CTRL 事件给 APP，APP 准备对应分片写入 slot，通过 CMD(SEND_FRAG) 交付 LINK，LINK 封装帧头后通过 `PHY_CMD_TX_FRAME` 发送 `DATA_FRAG`。每片数据附带 CRC16 校验，Tag 校验失败时发送 `DATA_CTRL(NACK, frag_id)` 请求重发，Anchor APP 重新准备该分片发送。

最后一个分片拉取成功后，Tag 发送 `DATA_CTRL(DONE)`，Anchor 回复 ACK，双方释放资源，会话结束。DONE 帧即为结束信号，ACK 为其应答。

## PHY 层数据帧行为（详细设计见 [phy层数据帧行为优化.md](phy层数据帧行为优化.md)）

核心原则：**PHY 层职责回缩，所有 slot 释放统一由 LINK 层负责**。PHY 在上报事件时携带 slot_index，LINK 根据业务逻辑决定释放时机。数据帧的 slot 管理、备份重发、逐片流水线全部由 LINK 负责，PHY 只做收发和快速应答。

### 请求帧发送（Tag 侧）

与 Discovery 发送流程一致：LINK 构建 CMD → PHY 发送帧 → TX_DONE 后进入 RX_SLOT（1 槽，超时同现有槽式接收）→ 收到应答后回退 IDLE。不新增 CMD 类型。

### 应答帧（Anchor 侧）

PHY 在 IDLE 监听状态下收到数据帧时，解析功能码：

| 应答类型 | 触发条件 | 帧内容 | PHY 后续 |
|---------|---------|-------|---------|
| **ACK** | 收到 DATA_CFG_REQ | 仅 ACK 标志 | delayed TX 发送，上报收帧位置给 LINK |
| **透传** | 收到 DATA_CTRL (GET_INFO/PULL/NACK/DONE) | 无 | 直接上报收帧位置给 LINK，由 APP→LINK→PHY 链回复 |

**关键变更**: 数据帧发送不再使用 PHY 挂起匹配机制。APP 收到 LINK 事件后，准备分片写入 slot，通过 CMD(SEND_FRAG) 交付 LINK，LINK 立即通过 `PHY_CMD_TX_FRAME` 发送。每片一次 APP→LINK→PHY 往返，逻辑更简单。

### Tag 侧 ERROR 处理

连续收到 **2 个 ERROR 帧** → 关闭当前会话，回到 Discovery 重新拉取。

## 可靠性机制

- **重发备份**: LINK 层管理，发送完成后 LINK 决定保留（重发用）/释放 slot
- **逐片下发**: LINK 每次收到 APP CMD(SEND_FRAG) 后通过 PHY_CMD_TX_FRAME 立即发送，APP 驱动每片发送
- **slot 释放统一到 LINK**: 所有 `UwbSlots_Free()` 调用从 PHY 移除，改为事件携带 slot_index，由 LINK 回收（Discovery TX slot 同理）
- **超时重试**: Tag 单分片超时重发最多 3 次，3 次失败后通知 APP DATA_FAIL
- **ERROR 安全退出**: 功能码未知或 frag_id 异常时立即 ERROR，Tag APP 连续收到 2 个 ERROR → 下发 DATA_RESET 给 LINK (LINK 批量回收数据 slot 并发回 DATA_RESET_ACK) + APP 收到确认后复位自身，下次 1s 周期重启
- **超时回收**: Anchor 侧 1s 无新请求自动释放挂起 slot

## 关键约束

| 约束 | 值 | 说明 |
|------|----|------|
| 最大数据量 | 512 字节 | 单次传输上限 |
| 分片净载荷 | 62 字节/片 | 64B payload - 2B CRC16 |
| 最大分片数 | ~9 片 | 512 ÷ 62 向上取整 |
| 单片重试 | 3 次 | 超过则放弃 |
| ERROR 容忍 | 连续 2 次 | Tag 连续收到 2 个 ERROR 关闭会话 |
| Anchor 超时 | 1s | 无请求则回收 |
| Tag 轮询间隔 | 1s | MAC 表中 Anchor 依次拉取，每次 1 个 |
| MAC 表大小 | 8 条 | 3 分钟无通信则剔除 |

## 与现有模块的关系

- **PHY**: 职责回缩——只做请求帧发送、DATA_CFG_REQ 快速 ACK 应答（复用现有 fast-reply 机制），不再调用 `UwbSlots_Free()`；所有 slot 通过事件 slot_index 交由 LINK 管理和回收。数据帧的发送由 APP 驱动（APP→LINK CMD→PHY_CMD_TX_FRAME），不新增 PHY CMD 类型
- **LINK**: 承担数据帧 slot 生命周期管理、备份重发策略、逐片下发控制。新增数据挂起表 (8 条, 跟踪所有数据相关 slot)，支持 APP 下发 DATA_RESET 时批量回收。DATA 传输状态机改为 APP 驱动（LINK 被动响应命令、通知事件），与 Discovery 2+1 交织。
- **APP**: Tag 侧：1s 轮询、MAC 表、三阶段会话状态机 (APP_SESS_IDLE → WAIT_CFG_ACK → WAIT_INFO → PULLING → WAIT_DONE_ACK)、数据拼装校验、连续 ERROR 计数 (2 次 → 通知 LINK 复位 + 自身复位)。Anchor 侧：会话状态机 (IDLE → ACTIVE → LAST_SENT)、请求到达时检查会话状态（未结束则先下发再重启）、数据预打包切片、等待 DONE ACK 后结束会话。
- **Protocol**: 帧编解码扩展 DATA_CFG_REQ / DATA_CTRL / DATA_CTRL_RESP / DATA_FRAG 四种帧类型

## 里程碑

1. 帧结构定义 + 协议常量
2. PHY 层数据帧行为：ACK/WAIT/ERROR 快速应答、单 pending 匹配发送、新增 CMD 和事件
3. LINK DATA 传输（命令队列 + 挂起表 + 收发透传 + 2+1 交织 + DATA_RESET 复位）
4. APP 层会话管理（Tag 三阶段状态机 + Anchor 会话状态机 + 先下发再重启 + MAC 表 + 1s 轮询 + CRC 校验 + ERROR 计数/复位）
5. 端到端联调测试