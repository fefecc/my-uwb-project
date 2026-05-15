# LINK 层数据帧方案

> 创建时间: 2026-05-13
> 所属: plan-v4
> 前置: plan-v3 滑窗TWR + PHY层数据帧行为优化

## 设计原则

| 原则 | 说明 |
|------|------|
| APP 层切片 | APP 负责数据切片、CRC 计算、拼装校验；交付给 LINK 的是已切好的分片 |
| APP 驱动会话，LINK 透传 | APP 维护数据会话状态机，LINK 只负责帧收发、穿插调度、超时重发、slot 挂起表管理 |
| 零拷贝 | APP 把分片数据写入 slot 后交付给 LINK，LINK 只加帧头，不再拷贝 payload |
| PHY 层最小改动 | 不新增 PHY CMD 类型（复用 `PHY_CMD_TX_FRAME`）；PHY 仅在 `irq_rx_ok()` 中对 DATA_CFG_REQ 增加快速 ACK 应答（复用现有 fast-reply 机制），数据帧收发全部由 LINK 通过现有 CMD 驱动 |
| 重发简单 | 超时重发（最多 3 次），CRC 错误发 NACK 重发同一片，失败则放弃 |
| 数据帧间隔 ≥ 15ms | 无论是否有 Discovery 帧，相邻两次数据帧发送之间必须间隔至少 15ms |

## 职责划分

```
┌─────────────────────────────────────────────────────────────────┐
│ APP 层                                                          │
│   • 数据切片 (62B/片 + CRC16)                                   │
│   • 元信息构建 (data_type, data_len, frag_count, total_crc)     │
│   • 接收端拼装 + CRC 校验                                       │
│   • 会话状态机 (Tag: IDLE→WAIT_ACK→WAIT_INFO→PULLING→DONE)      │
│   • MAC 表管理 + 1s 轮询                                        │
│   • ERROR 连续计数 → 会话复位 (通知 LINK 复位数据槽)             │
│   • 直接写 slot data[]，交付 slot_index 给 LINK                 │
├─────────────────────────────────────────────────────────────────┤
│ LINK 层                                                         │
│   • 穿插调度: 2 Discovery + 1 Data 交替                        │
│   • 帧头封装: 从 slot data[] 读取 APP 写好的 payload，加帧头   │
│   • 超时重发: 重发同一个 slot（不重建）                         │
│   • 事件转发: 收到 RX 数据帧时，解析帧头后通知 APP              │
│   • 数据挂起表: 跟踪所有数据相关 slot，支持批量回收              │
│   • DATA_RESET 复位: 收到 APP 命令后释放全部数据 slot           │
├─────────────────────────────────────────────────────────────────┤
│ PHY 层                                                          │
│   • 不变，复用 PHY_CMD_TX_FRAME                                 │
│   • 新增: DATA_CFG_REQ 快速应答 ACK (delayed TX)               │
└─────────────────────────────────────────────────────────────────┘
```

## 层间接口

### APP → LINK 命令队列

APP 切好片后，把分片数据写入 slot，然后把 slot_index 和元信息通过命令队列交给 LINK：

```c
typedef enum {
    UWB_LINK_CMD_NONE = 0,
    UWB_LINK_CMD_SEND_DATA_CFG,     /* Tag: 发起数据传输请求 (无 slot) */
    UWB_LINK_CMD_SEND_DATA_CTRL,    /* Tag: 发送 CTRL 帧 (slot 中已有 payload) */
    UWB_LINK_CMD_SEND_FRAG,         /* Anchor: 发送一个分片 (slot 中已有 payload) */
    UWB_LINK_CMD_SEND_ACK,          /* Anchor: 发送 ACK/DONE 帧回复 */
    UWB_LINK_CMD_DATA_ABORT,        /* 任意: 中止数据传输 */
    UWB_LINK_CMD_DATA_RESET,        /* APP → LINK: 复位全部数据 slot + 数据状态机 */
} UwbLinkCmdType;

typedef struct {
    UwbLinkCmdType type;
    int8_t   slot_index;         /* APP 已写好 payload 的 slot, -1=无 (如 CFG_REQ) */
    uint16_t target_id;          /* 目标短地址 */
    uint8_t  ctrl_type;          /* SEND_DATA_CTRL 时: GET_INFO/PULL/DONE/NACK/STOP */
    uint8_t  frag_id;            /* SEND_FRAG 时: 分片序号 */
} UwbLinkCmd;
```

**零拷贝关键**：APP 调用 `UwbSlots_Alloc(UWB_SLOT_APP_OWN)` 分配 slot，将分片 payload 写入 `slot->data[]`，然后将 slot_index 传给 LINK。LINK 收到命令后：
1. 读取 slot 中的 payload
2. 在 slot data 前面加上 MAC + Common + Ext Header（或重新 Encode 到同一 slot）
3. 发送 `PHY_CMD_TX_FRAME` 给 PHY
4. 发送完成后 LINK 释放 slot

**重发零拷贝**：超时重发时，LINK 不重建帧，直接重新发送同一个 slot_index 的内容（slot 尚未释放）。

### LINK → APP 事件（扩展现有队列）

```
UWB_LINK_APP_EVT_DATA_CFG       Anchor: 收到 CFG_REQ (src_id)
UWB_LINK_APP_EVT_DATA_CTRL      Anchor: 收到 CTRL 帧 (src_id, ctrl_type, frag_id)
UWB_LINK_APP_EVT_DATA_FRAG      Tag: 收到分片 (src_id, frag_id, slot_index)
UWB_LINK_APP_EVT_DATA_ACK       Tag: 收到 ACK (src_id)
UWB_LINK_APP_EVT_DATA_ERROR     Tag: 收到 ERROR 帧
UWB_LINK_APP_EVT_DATA_COMPLETE  Tag: 全部数据接收完成
UWB_LINK_APP_EVT_DATA_FAIL      Tag: 传输失败
UWB_LINK_APP_EVT_DATA_RESET_ACK LINK→APP: DATA_RESET 确认 (slot 已全部回收)
```

事件携带 slot_index，APP 直接从 slot 读取 payload，处理完毕后由 APP 释放 slot。

### Slot 所有权流转

```
发送方向 (Tag→Anchor 请求帧 / Anchor→Tag 分片帧):
  APP Alloc(slot, APP_OWN) → 写 payload → CmdQueue(slot_index) → LINK
  → LINK 加帧头 Encode → PHY_CMD_TX_FRAME(slot_index) → PHY
  → PHY 发送 → TX_DONE → LINK 释放 slot (或保留用于重发)

接收方向 (PHY 收帧):
  PHY Alloc(slot, PHY_OWN) → 读 DW1000 → RX 事件(slot_index) → LINK
  → LINK 解析帧头 → AppEvent(slot_index) → APP
  → APP 读 payload → APP 释放 slot
```

新增 `UWB_SLOT_APP_OWN` 所有权状态，使所有权流转清晰。

## 数据挂起表

### 目的

LINK 层维护一个数据 slot 挂起表，跟踪当前线程控制的所有数据相关 slot（不包括 Discovery/Ranging 的 slot）。用于：
- APP 下发 `DATA_RESET` 时批量回收
- 超时扫描释放过期备份 slot

### 结构

```c
// uwb_link.c 内部

typedef enum {
    LINK_SLOT_PURPOSE_TX_FRAME = 0,  // 已发 CMD_TX_FRAME，PHY 正在/已发送
    LINK_SLOT_PURPOSE_RX_FRAME,      // 收到的数据帧，已通知 APP，等 APP 释放
    LINK_SLOT_PURPOSE_BACKUP,        // 发送完成保留为重发备份
} link_slot_purpose_t;

typedef struct {
    int8_t  slot_index;          // -1 = 空
    link_slot_purpose_t purpose;
    uint8_t frag_id;
    uint16_t target_id;
    uint32_t registered_ms;
} link_data_slot_entry_t;

#define LINK_DATA_SLOT_MAX  8
link_data_slot_entry_t g_data_slots[LINK_DATA_SLOT_MAX];
```

### 操作

```c
// 注册: LINK 发 TX_FRAME 或收到 RX 数据帧时
static bool link_data_slot_register(int8_t idx, link_slot_purpose_t purpose,
                                     uint8_t frag_id, uint16_t target_id);

// 移除: 正常释放 slot 时
static void link_data_slot_unregister(int8_t idx);

// 批量回收: 收到 DATA_RESET 或数据会话结束时
static void link_data_slot_free_all(void) {
    for (int i = 0; i < LINK_DATA_SLOT_MAX; i++) {
        if (g_data_slots[i].slot_index != -1) {
            UwbSlots_Free(g_data_slots[i].slot_index);
            g_data_slots[i].slot_index = -1;
        }
    }
}
```

### 使用时机

| 操作 | 挂起表动作 |
|------|----------|
| LINK 发 CMD_TX_FRAME (数据帧) → PHY | register(slot, TX_FRAME) |
| PHY 上报 RX_SLOT_DONE (数据帧) → LINK | register(slot, RX_FRAME) |
| TX_DONE，LINK 保留为重发备份 | purpose 改为 BACKUP |
| 正常释放 slot (ACK 确认 / APP 读完) | unregister(slot) + UwbSlots_Free |
| 收到 DATA_RESET 命令 | free_all() + 复位数据状态机 |
| 超时扫描 (1s 周期) | free 超期 BACKUP slot + unregister |

### 与 PHY 单 pending 的关系

| 层 | 结构 | 用途 |
|----|------|------|
| PHY `pending_entry_t` | 1 条 | Anchor 侧 PHY 持有的待发送 slot（快速匹配发送），仅指向当前挂起帧 |
| LINK `g_data_slots[]` | 8 条 | LINK 层跟踪本线程所有数据 slot（TX/RX/BACKUP），用于批量回收 |

两者不冲突：PHY 的 pending 指向当前待发送的 slot_index，LINK 的挂起表记录所有数据 slot；PHY pending 是 LINK 挂起表的一个子集。

## 复位机制

### 触发路径

```
Tag APP 连续收到 2 个 ERROR 帧:
  1. APP 调用 session_request_reset():
     a. UwbLink_SendCmd({type=DATA_RESET})  → 通知 LINK 复位
     b. 进入 RESETTING 状态，等待 LINK 确认

  2. LINK drain_app_cmds() 处理 DATA_RESET:
     a. link_data_slot_free_all()          → 释放全部数据 slot
     b. data_xfer_state = IDLE             → 复位数据传输状态机
     c. disc_count = 0                     → 重置交织计数
     d. 清空 retry_slot                    → 丢弃重发备份
     e. 回复 APP: EVT(DATA_RESET_ACK)      → 确认复位完成

  3. APP 收到 DATA_RESET_ACK:
     → session_reset_confirm()             → 复位自身会话
     → sess.state = IDLE                   → 可接受下次 1s 轮询
```

### 重试机制

APP 在 RESETTING 状态下每 100ms 重发一次 DATA_RESET，直到收到 LINK 的 DATA_RESET_ACK 确认。超过 10 次重试仍无确认 → APP 强制复位（LINK 可能卡死，slot 由 LINK 超时扫描兜底回收）。

### 复位语义

- **APP 复位**: 会话状态归零，下次 1s 定时器触发时重新发起
- **LINK 复位**: 数据 slot 全部回收，数据状态机回到 IDLE，可立即接受新的数据命令
- **不影响 Discovery/Ranging**: 复位只涉及数据帧相关 slot，Discovery/TWR 的 slot 不受影响

### 防止 slot 泄漏

挂起表的关键作用是防止 slot 泄漏：
- 正常路径: slot 在 ACK 确认或 APP 读完后通过 unregister + free 释放
- 异常路径: DATA_RESET 或超时扫描通过 free_all 批量回收
- 每个 slot 注册时记录 `registered_ms`，超时扫描兜底

## 三阶段传输时序

### 阶段 1: 在线确认

```
Tag                                         Anchor
 │ APP→LINK: CMD(CFG_REQ)                      │
 │──── DATA_CFG_REQ ────────────────────────→│  LINK构建帧(无payload)发送
 │←─── ACK (PHY快速应答, delayed TX) ───────│  Anchor PHY微秒级应答
 │                                           │  PHY上报收帧位置给LINK
 │                                           │  LINK→APP: DATA_CFG事件
 │ LINK→APP: DATA_ACK 事件                    │  APP建立会话, 预打包数据
```

### 阶段 2: 元信息获取

```
Tag                                         Anchor
 │ APP→LINK: CMD(CTRL, GET_INFO)               │
 │──── DATA_CTRL(GET_INFO) ────────────────→│  LINK构建帧(CTRL帧无payload)
 │                                           │  LINK收到→通知APP: DATA_CTRL事件
 │                                           │  APP写meta到slot→CMD(SEND_FRAG)
 │←─── DATA_FRAG(meta) ────────────────────│  LINK加帧头发送
 │                                           │
 │ LINK→APP: DATA_FRAG(meta, slot_index)      │
 │ APP读payload得到meta                       │
```

### 阶段 3: 逐片拉取

```
Tag                                         Anchor
 │──── DATA_CTRL(PULL, frag=0) ─────────────→│  LINK构建帧
 │                                           │  LINK→APP: DATA_CTRL(PULL,0)
 │                                           │  APP写frag0到slot, CmdQueue(SEND_FRAG,0)
 │←─── DATA_FRAG(frag=0) ──────────────────│  LINK加帧头发送
 │──── DATA_CTRL(PULL, frag=1) ─────────────→│
 │←─── DATA_FRAG(frag=1) ──────────────────│
 │  ...                                      │
 │──── DATA_CTRL(DONE) ─────────────────────→│  Tag校验通过
 │←─── ACK ─────────────────────────────────│  APP写ACK到slot, LINK发送
```

### NACK 重传

```
Tag                                         Anchor
 │──── DATA_CTRL(PULL, frag=2) ─────────────→│
 │←─── DATA_FRAG(frag=2) ──────────────────│
 │    (APP CRC校验失败)                       │
 │──── DATA_CTRL(NACK, frag=2) ─────────────→│  LINK→APP: DATA_CTRL(NACK,2)
 │←─── DATA_FRAG(frag=2) ──────────────────│  APP重写frag2到slot, 重发
```

### 超时重发

```
Tag                                         Anchor
 │──── DATA_CTRL(PULL, frag=3) ─────────────→│
 │    (超时, 无应答)                          │
 │──── DATA_CTRL(PULL, frag=3) ─────────────→│  LINK重发同一slot
 │    (仍超时, 3次)                           │
 │  → 通知APP DATA_FAIL                     │  Anchor 1s无请求超时回收
```

### Tag 侧 ERROR 处理

连续收到 2 个 ERROR 帧 → 关闭数据会话，回到 Discovery。

## 2+1 交织逻辑

```
Discovery #1  ──→  Discovery #2  ──→  Data Transfer  ──→  Discovery #3  ──→  ...
                  disc_count=1        disc_count=2          disc_count=0 (重置)
                                       data_xfer_active
```

每次 `RX_WINDOW_END` 事件后 `disc_count++`。当 `disc_count >= 2` 且 APP 命令队列有待发送数据时：
- 重置 `disc_count = 0`
- 进入数据传输状态机 (`data_xfer_active = true`)
- 数据传输完成/失败后，恢复 Discovery

```c
/* LINK 主循环 */
for (;;) {
    drain_phy_events();
    drain_app_cmds();   /* 处理 APP 命令队列 */

    if (role == TAG) {
        if (data_xfer_active) {
            link_tag_handle_data_xfer();  /* 数据帧状态机 */
        } else if (!disc_exchange_active) {
            if (disc_count >= 2 && UwbLink_HasDataCmd()) {
                disc_count = 0;
                start_data_xfer();
            } else {
                link_tag_send_disc();
            }
        }
    } else { /* ANCHOR */
        /* drain_phy_events() + drain_app_cmds() 已处理 RX 帧和 APP 命令 */
    }
    osDelay(1);
}
```

## LINK 层数据状态机

### Tag 侧

LINK 作为传输层，被动响应 APP 命令，不做业务决策。

```
LINK 主循环 (drain_app_cmds 处理队列中的命令):

  收到 CMD(CFG_REQ):
    构建 CFG_REQ 帧(无payload) → PHY_CMD_TX_FRAME(pending_rx=true, rx_slot_count=1)
    → 进入 data_xfer_active=true, 等待 RX 应答

  收到 CMD(CTRL, type, frag_id):
    构建 CTRL 帧(ext_header: ctrl_type + frag_id) → PHY_CMD_TX_FRAME(pending_rx=true)
    → 等待 RX 应答

  drain_phy_events 处理 RX:
    RX_SLOT_DONE + func_code=0x73 (CTRL_RESP, ACK):
      → 通知 APP: DATA_ACK 事件
      → 如果之前发的是 DONE → data_xfer_active=false, 释放相关 slot
    RX_SLOT_DONE + func_code=0x71 (DATA_FRAG):
      → 解析帧头得 frag_id, total_frags, flags
      → 通知 APP: DATA_FRAG(slot_index, frag_id, ...)
    RX_SLOT_DONE + func_code=0x73 resp_type=ERROR:
      → 通知 APP: DATA_ERROR 事件
      → APP 负责 ERROR 计数

  超时 (TX_DONE 后 50ms 无 RX):
    retry_count++ → 重发同一 slot
    retry_count >= 3 → 通知 APP: DATA_FAIL → data_xfer_active=false
```

### Anchor 侧

Anchor LINK 同样是传输层，收帧后通知 APP，收到 APP 命令后发帧。

```
LINK 主循环:

  drain_phy_events 处理 RX:
    IDLE 监听收到 DATA_CFG_REQ:
      → PHY 已快速应答 ACK (微秒级)
      → LINK 收到 RX_SLOT_DONE → 通知 APP: DATA_CFG(src_id) 事件
      → data_xfer_active=true

    IDLE 监听收到 DATA_CTRL:
      → 解析 ext_header: ctrl_type, frag_id
      → 通知 APP: DATA_CTRL(src_id, ctrl_type, frag_id) 事件

  drain_app_cmds 处理 APP 命令:
    收到 CMD(SEND_FRAG, slot_index, frag_id):
      → 在 slot data 上加帧头 Encode (DATA_FRAG, frag_id)
      → PHY_CMD_TX_FRAME(slot_index, pending_rx=true, rx_slot_count=1)
      → 等待 TX_DONE + RX (Tag 的下一条 CTRL)

    收到 CMD(SEND_ACK):
      → 构建 ACK 帧 (DATA_CTRL_RESP, resp_type=ACK)
      → PHY_CMD_TX_FRAME
      → data_xfer_active=false, 释放资源

    收到 CMD(DATA_RESET):
      → link_data_slot_free_all()
      → data_xfer_active=false
      → 回复 APP: EVT(DATA_RESET_ACK)    // 确认复位完成

  超时 1s 无新请求:
    → link_data_slot_free_all()
    → data_xfer_active=false
    → 不通知 APP（APP 自己也有 1s 超时）
```

## 帧结构定义

### DATA_CFG_REQ (0x70) — Tag→Anchor

```
MAC Header (9B) | Common Header (6B) | Ext Header (0B) | Payload (0B)
```

### DATA_CTRL (0x72) — Tag→Anchor

```
MAC Header (9B) | Common Header (6B) | Ext Header (4B) | Payload (0B)

Ext Header:
  ctrl_type    (1B)  — 0x01=GET_INFO, 0x02=PULL, 0x03=DONE, 0x04=NACK, 0x05=STOP
  frag_id       (1B)  — PULL/NACK 时有效
  reserved     (2B)
```

### DATA_FRAG (0x71) — Anchor→Tag

```
MAC Header (9B) | Common Header (6B) | Ext Header (4B) | Payload (N bytes)

Ext Header:
  frag_id       (1B)  — 分片序号 (0-based)
  total_frags   (1B)  — 总分片数
  flags         (1B)  — bit0: meta帧(meta=1, 数据帧=0)
  reserved      (1B)

Payload:
  meta帧 (flags.bit0=1): data_type(1B)+data_length(2B)+frag_count(1B)+total_crc16(2B) = 6B
  数据帧 (flags.bit0=0): data[N] + crc16(2B)
  每片净数据: 64B - 2B(CRC) = 62B
```

### DATA_CTRL_RESP (0x73) — Anchor→Tag (ACK)

```
MAC Header (9B) | Common Header (6B) | Ext Header (2B) | Payload (0B)

Ext Header:
  resp_type     (1B)  — 0x00=ACK, 0x01=WAIT, 0x02=STOP
  reserved      (1B)
```

## 零拷贝设计

### 核心流程：APP 写 payload 到 slot → LINK 读出加帧头写回同一个 slot → 重发复用同一个 slot

```
┌─────────────────── APP 层 ───────────────────┐
│                                               │
│  1. UwbSlots_Alloc(UWB_SLOT_APP_OWN) → idx=5 │
│  2. memcpy(slot->data, frag_payload, N)        │
│  3. slot->data_len = N                        │
│  4. UwbLink_SendCmd({SEND_FRAG, idx=5, ...})  │
│     (交付 slot 所有权给 LINK)                   │
│                                               │
└─────────────────────┬─────────────────────────┘
                      │ CmdQueue
                      v
┌─────────────────── LINK 层 ──────────────────┐
│  5. drain_app_cmds():                          │
│     a. 读 slot->data[0..N-1] → Frame.payload  │
│     b. UwbProtocol_InitFrame() 设置帧头        │
│     c. Frame.ext_header = {frag_id, ...}       │
│     d. UwbProtocol_Encode(&Frame,              │
│           slot->data, 127, &tx_len)            │
│     e. slot->data_len = tx_len                 │
│     f. PHY_CMD_TX_FRAME(slot=5, ...)           │
│                                                │
│  重发时: 直接重发 slot=5, 不重新 Encode        │
│     PHY_CMD_TX_FRAME(slot=5, ...)               │
│                                                │
└─────────────────────┬──────────────────────────┘
                      │ cmd_queue
                      v
┌─────────────────── PHY 层 ────────────────────┐
│  6. dwt_writetxdata(slot->data, slot->data_len)│
│  7. TX 完成 → TX_DONE 事件                     │
│  8. LINK 收到 TX_DONE → 释放或保留 slot        │
└────────────────────────────────────────────────┘
```

步骤 5d 中 `UwbProtocol_Encode()` 将帧头+payload 一次性写回 `slot->data[]`，
payload 经历一次 64B 拷贝（Frame 结构体 → slot 缓冲区），
复用现有 Discovery 帧的 Encode 路径，无额外开销。

### 接收方向

```
PHY 收帧 → slot(PHY_OWN) → LINK Decode 解析帧头
          → 通知 APP(slot_index)
          → APP 从 slot->data 读 payload, CRC 校验
          → APP UwbSlots_Free(slot_index)
```

接收方向不做额外拷贝，APP 直接从 slot payload 区读取。

### Slot 生命周期（同一个 slot 复用）

```
APP Alloc(APP_OWN) → 写 payload → CmdQueue 交付
→ LINK 读 payload, 加帧头 Encode 写回同一 slot → PHY_CMD_TX_FRAME
→ PHY 发送 → TX_DONE
→ LINK:
    成功确认 → UwbSlots_Free(slot)
    需要重发 → 保留 slot, 重新 PHY_CMD_TX_FRAME(slot=5)
    超过重试上限 → UwbSlots_Free(slot), 通知 APP FAIL
```

## 重发机制

| 场景 | 策略 | 最大次数 |
|------|------|----------|
| Tag 发送后 RX 超时 | 重发同一 slot（不重建） | 3 次 |
| Tag 收到 FRAG 但 APP CRC 错误 | 发送 CTRL(NACK, frag_id) | 3 次后放弃 |
| Anchor 1s 无新请求 | 超时回收，回到 IDLE | — |
| Tag 连续收到 2 个 ERROR | 关闭会话，回到 Discovery | 2 次 |

## PHY 层改动（最小化）

### 只改一个地方：`irq_rx_ok()` 增加数据帧快速应答

```c
/* irq_rx_ok() 中, 在 DISC_REQ 快速应答之后增加: */

if (g_phy.role == APP_ROLE_ANCHOR &&
    frame.common.func_code == (uint8_t)UWB_FUNC_APP_DATA_CFG) {
    g_phy.tx_buf_len = build_fast_reply_data_ack(frame.mac.src16, frame.mac.seq);
    if (g_phy.tx_buf_len > 0) {
        /* 与 DISC_RESP 完全相同的 delayed TX 机制 */
        uint64_t rx_ts = s->rx_ts;
        uint32_t delay_us = ANCHOR_REPLY_GUARD_US;
        uint64_t tx_time = (rx_ts + UwbPhy_UsToDwTime(delay_us)) & ~0x1FFULL;
        /* ... 写入 DW1000, dwt_starttx(DWT_START_TX_DELAYED) ... */
        /* 上报 RX 事件给 LINK (slot_index 传递) */
        /* 进入 TX/WAIT */
    }
}
```

### 不新增 CMD 类型

所有数据帧发送复用 `PHY_CMD_TX_FRAME`，`has_pending_rx=true`，`rx_slot_count=1`。

## 文件修改清单

### 1. `uwb_protocol.h` — 新增帧类型和常量

| 改动 | 说明 |
|------|------|
| `UWB_FUNC_APP_DATA_CTRL_RESP = 0x73` | 新增帧功能码 |
| `UWB_DATA_CTRL_GET_INFO = 0x01` 等 | CTRL 子类型常量 |
| `UWB_DATA_RESP_ACK = 0x00` 等 | RESP 类型常量 |
| `UWB_DATA_FRAG_FLAG_META = 0x01` | FRAG flags 位掩码 |

### 2. `uwb_stack_types.h` — 新增事件结构体

| 改动 | 说明 |
|------|------|
| `UwbDataCtrlEvent` | CTRL 帧事件 (src_id, ctrl_type, frag_id) |
| `UwbDataFragEvent` | 分片事件 (src_id, frag_id, total_frags, slot_index) |
| `mac_entry_t` | MAC 表条目 (valid, anchor_id, last_seen_ms) |

### 3. `uwb_slots.h` — 扩展所有权 + 池容量

| 改动 | 说明 |
|------|------|
| `UWB_SLOT_APP_OWN` | 新增所有权状态，APP 持有（写 payload） |
| `UWB_SLOT_NUM` 从 8 改为 16 | 数据帧传输需要更多并发 slot |

### 4. `uwb_link.h` — 新增状态和接口

| 改动 | 说明 |
|------|------|
| `UwbLinkDataState` 枚举 | DATA_IDLE/WAIT_ACK/WAIT_RESP/ANCHOR_WAIT_CTRL |
| `UWB_LINK_APP_EVT_DATA_*` 事件类型 | 6 种数据事件 |
| `UwbLinkAppEvent.data` 联合体扩展 | 新增 data_cfg/data_ctrl/data_frag/data_ack 成员 |
| `UwbLinkCmd` / `UwbLinkCmdType` | APP→LINK 命令结构 |
| `UwbLink_SendCmd()` / `UwbLink_HasDataCmd()` | 命令队列 API |

### 5. `uwb_link.c` — 数据传输状态机 + 挂起表 + 复位

| 改动 | 说明 |
|------|------|
| `link_data_slot_entry_t` + `g_data_slots[]` | 数据挂起表 (8 条) |
| APP→LINK 命令队列 | 静态队列 depth=4 |
| `disc_count` 字段 | 2+1 交织计数器 |
| `drain_phy_events()` 扩展 | 解析数据帧 func_code，分发到数据处理 |
| `drain_app_cmds()` | 处理 APP 命令：加帧头 Encode 后发给 PHY；DATA_RESET 处理 |
| `link_tag_handle_data_xfer()` | Tag 侧数据帧收发（CMD→TX→RX→通知APP） |
| `link_anchor_handle_data_rx()` | Anchor 侧数据帧处理（收帧→通知APP→等CmdQueue） |
| `link_data_slot_*()` | 挂起表注册/移除/批量回收 |
| 帧构建函数 | `build_data_cfg_req()`, `build_data_ctrl()`, `build_data_resp_ack()` |
| 重发逻辑 | 保留 retry_slot，超时重发同一 slot |
| 超时扫描 | 1s 周期扫描挂起表，释放过期 BACKUP slot |

### 6. `uwb_phy.c` — 快速应答扩展

| 改动 | 说明 |
|------|------|
| `build_fast_reply_data_ack()` | 构建 DATA_CFG_ACK 帧 |
| `irq_rx_ok()` 增加分支 | DATA_CFG_REQ 快速应答 |

### 7. `uwb_app.c` — 切片+拼装+MAC 表+轮询

| 改动 | 说明 |
|------|------|
| `mac_entry_t g_mac_table[8]` | MAC 表 |
| 1s 轮询 | Tag 每 1s 轮流从 MAC 表取 Anchor 发起数据会话 |
| `g_anchor_tx_buf[512]` | Anchor 发送数据缓冲区 |
| `g_tag_rx_buf[544]` | Tag 接收拼装缓冲区 |
| 切片函数 | `app_slice_frag()` — 从 tx_buf 切片写 slot，含 CRC16 |
| 拼装函数 | `app_assemble_frag()` — 从 slot 读 payload 拼入 rx_buf，CRC 校验 |
| 事件处理扩展 | DATA_CFG/CTRL/FRAG/ACK/COMPLETE/FAIL/RESET_ACK |
| CmdQueue 交互 | APP 收到 LINK 事件后，切片写 slot，通过 UwbLink_SendCmd 交付 |

### 8. `uwb_app.h` — 新增接口

| 改动 | 说明 |
|------|------|
| 声明 `UwbApp_PollData()` | Tag 轮询入口 |

## 关键约束

| 约束 | 值 | 说明 |
|------|----|------|
| 最大数据量 | 512 字节 | 单次传输上限 |
| 分片净载荷 | 62 字节/片 | 64B payload - 2B CRC16 |
| 最大分片数 | ~9 片 | 512 ÷ 62 向上取整 |
| 数据帧最小间隔 | 15ms | 相邻两次数据帧发送之间最小间隔 |
| 单片重试 | 3 次 | 超时或 CRC 错误 |
| Anchor 超时 | 1s | 无请求则回收 |
| Tag 轮询间隔 | 1s | 每次会话间隔 1s |
| MAC 表大小 | 8 条 | 3 分钟无通信剔除 |
| Slot 池 | 16 个 | 扩容保证并发 |

## 实现顺序

1. **帧结构定义**: `uwb_protocol.h` 常量, `uwb_stack_types.h` 事件结构体
2. **Slot 池扩容 + 所有权扩展**: `uwb_slots.h` UWB_SLOT_NUM=16, UWB_SLOT_APP_OWN
3. **PHY 快速应答**: `uwb_phy.c` DATA_CFG_REQ ACK 处理
4. **LINK 穿插调度 + 命令队列**: `uwb_link.c` + `uwb_link.h` 核心逻辑
5. **APP 切片/拼装 + MAC 表 + 轮询**: `uwb_app.c` 完整数据处理
6. **端到端联调**: 硬件测试日志验证

## 验证方式

1. **编译检查**: 所有修改文件无编译错误
2. **日志验证**: 三阶段每个帧的内容和状态转换日志
3. **硬件测试**: Tag→Anchor CFG_REQ→ACK, GET_INFO→meta, 逐片 PULL→FRAG
4. **超时测试**: Anchor 无应答, Tag 重试 3 次后放弃
5. **CRC 测试**: 模拟单分片 CRC 错误, NACK 重发
6. **交织测试**: 2 次 Discovery + 1 次数据帧交替执行
7. **零拷贝验证**: 日志确认重发时无重新 Encode，slot_index 不变