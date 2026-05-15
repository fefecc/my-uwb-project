# APP 层数据交互帧方案

> 创建时间: 2026-05-14
> 所属: plan-v4
> 前置: [phy层数据帧行为优化.md](phy层数据帧行为优化.md) + [link层数据帧方案.md](link层数据帧方案.md)

## 设计原则

| 原则 | 说明 |
|------|------|
| APP 驱动会话 | APP 层维护数据会话状态机，LINK 层提供传输服务 |
| 命令-应答模式 | APP 发命令给 LINK，LINK 执行后回事件；APP 等 LINK 消费完上一条命令再发下一条 |
| 零拷贝 | APP 写 payload 到 slot，LINK 加帧头 Encode 写回同一 slot |
| 错误快速收敛 | 连续 2 个 ERROR → APP 复位自身 + 通知 LINK 复位槽 |
| 1s 周期 | Tag 每 1s 对一个 Anchor 发起一次数据会话 |

## 总体架构

```
                         ┌── APP 命令队列 ──→ LINK ──→ PHY
APP (会话状态机) ──┤
                         └── LINK 事件队列 ──→ APP (会话状态机)
```

APP 通过两个队列与 LINK 交互：
- **APP → LINK**: `g_link_cmd_queue` — APP 下发传输指令
- **LINK → APP**: `g_app_evt_queue` — LINK 上报接收事件

APP 下发命令前检查 LINK 是否已消费上一条（命令队列有空位），保证不堆积。

---

## 一、Tag 侧 APP 设计

### 1.1 会话结构

```c
typedef enum {
    APP_SESS_IDLE = 0,
    APP_SESS_WAIT_CFG_ACK,   // 已发 CFG_REQ，等 ACK
    APP_SESS_WAIT_INFO,      // 已发 GET_INFO，等 meta 分片
    APP_SESS_PULLING,        // 正在逐片拉取
    APP_SESS_WAIT_DONE_ACK,  // 已发 DONE，等 ACK
    APP_SESS_RESETTING,      // 已发 DATA_RESET，等 LINK 确认
} app_session_state_t;

typedef struct {
    app_session_state_t state;
    uint16_t anchor_id;          // 目标 Anchor
    uint32_t session_start_ms;   // 会话启动时间
    uint32_t last_cmd_ms;        // 上一条命令发送时间

    uint8_t  error_count;        // 连续 ERROR 计数
    uint8_t  retry_count;        // 当前操作重试次数

    /* 待发送命令（app_send_cmd 失败时暂存，下周期重试） */
    bool     has_pending_cmd;
    UwbLinkCmd pending_cmd;

    /* 复位重试 */
    uint32_t last_reset_ms;      // 上次发送 DATA_RESET 的时间
    uint8_t  reset_retry;        // DATA_RESET 重试次数

    /* 元信息（阶段2获取） */
    uint8_t  total_frags;
    uint16_t total_data_len;
    uint16_t total_crc;

    /* 接收拼装 */
    uint8_t  rx_buf[512];
    uint8_t  frag_bitmap[2];     // 已收到的分片位图 (max 16 frags)

    /* 当前拉取进度 */
    uint8_t  next_frag;          // 下一个要拉的 frag_id
} app_tag_session_t;
```

### 1.2 会话状态机

```
                    ┌─────────────────────────────────────────┐
                    │                                           │
                    v                                           │
  IDLE ──(1s定时器)──→ WAIT_CFG_ACK ──(ACK)──→ WAIT_INFO ──(meta)──→ PULLING
   ^                      │  │                    │  │                  │  │
   │                      │  │ ERROR(x2)          │  │ ERROR(x2)        │  │
   │                      │  v                    │  v                  │  │
   │                      │  RESETTING            │  RESETTING          │  │
   │                      │  │                    │  │                  │  │
   │                      │  │ LINK确认            │  │ LINK确认          │  │
   │                      │  v                    │  v                  │  │
   │                      │  IDLE                 │  IDLE               │  │
   │                      │                       │                     │  │
   │                      │ 超时→DATA_FAIL         │ 超时→DATA_FAIL       │  │
   │                      v                       v                     │  │
   │                     RESETTING               RESETTING              │  │
   │                                                                   │  │
   └──────(ACK)──── WAIT_DONE_ACK ←──(DONE已发)──┘                     │
           │  │                                                         │
           │  │ ERROR(x2) / 超时 → RESETTING                            │
           │  v                                                         │
           │  IDLE                                                      │
           │                                                            │
           └── 所有异常路径先进 RESETTING，等 LINK 确认后再到 IDLE ──────┘

RESETTING: 等待 LINK 回复 DATA_RESET_ACK，超时 100ms 重发 DATA_RESET
```

### 1.3 1s 周期调度

Tag 侧 `UwbApp_Task` 主循环：

```
每 1ms 检查:
  1. drain_link_events()      — 处理 LINK 上报的事件，驱动状态机
  2. retry_pending_cmd()      — 如果上周期 app_send_cmd 失败，本周期重试
  3. retry_reset()            — 如果 RESETTING 状态超时未收到 LINK 确认，重发 DATA_RESET
  4. 如果 sess.state == IDLE:
     检查距上次会话结束是否 >= 1s
     → 是: 取 MAC 表中下一个 Anchor，发起会话
  5. 如果 sess.state != IDLE && sess.state != RESETTING:
     检查会话超时 (整体 > 5s → 触发复位)
```

伪代码：

```c
// 文件级变量
static uint32_t g_last_session_end_ms = 0;  // 上次会话结束时间

void UwbApp_Task(void *argument) {
    for (;;) {
        drain_link_events();       // 处理 LINK 事件，推动状态机
        retry_pending_cmd();       // 重试上次失败的 APP→LINK 命令
        retry_reset();             // 重试未确认的 DATA_RESET

        if (sess.state == APP_SESS_IDLE) {
            if (HAL_GetTick() - g_last_session_end_ms >= 1000) {
                uint16_t anchor = mac_table_next();
                if (anchor != 0) {
                    session_start(anchor);
                }
            }
        } else if (sess.state != APP_SESS_RESETTING) {
            if (HAL_GetTick() - sess.session_start_ms > 5000) {
                session_request_reset();
            }
        }
        osDelay(1);
    }
}
```

### 1.4 命令下发与 LINK 消费等待

**核心约束**: APP 下发命令前，确认 LINK 命令队列有空位（即上一条已被消费）。

如果队列满，不丢弃命令，而是暂存到 `pending_cmd`，下个主循环周期重试。

```c
// 发送命令（非阻塞），失败则暂存待重试
static bool app_send_cmd(UwbLinkCmd *cmd) {
    if (!UwbLink_CmdQueueHasSpace()) {
        // 暂存命令，下周期重试（覆盖旧暂存，最新命令优先）
        memcpy(&sess.pending_cmd, cmd, sizeof(UwbLinkCmd));
        sess.has_pending_cmd = true;
        return false;
    }
    UwbLink_SendCmd(cmd);
    sess.last_cmd_ms = HAL_GetTick();
    sess.has_pending_cmd = false;
    return true;
}

// 每周期调用：重试上次失败的暂存命令
static void retry_pending_cmd(void) {
    if (!sess.has_pending_cmd) return;
    if (UwbLink_CmdQueueHasSpace()) {
        UwbLink_SendCmd(&sess.pending_cmd);
        sess.last_cmd_ms = HAL_GetTick();
        sess.has_pending_cmd = false;
    }
}
```

事件处理函数中的用法不变，`app_send_cmd` 返回 false 时不切换状态——下周期重试成功后，由 `retry_pending_cmd` 发出命令，下一个 LINK 事件的到达自然推动状态前进。关键点是：**事件处理函数中如果 app_send_cmd 失败，不切换状态，也不丢弃事件中已提取的数据**（如果是 DATA_FRAG，slot 中的数据已经拷入 rx_buf 才调 app_send_cmd）。

### 1.5 各阶段事件处理

#### drain_link_events() 核心分发

```c
static void drain_link_events(void) {
    UwbLinkAppEvent evt;
    while (xQueueReceive(g_app_evt_queue, &evt, 0)) {

        // RESETTING 状态下只接受 RESET_ACK，丢弃其他事件
        if (sess.state == APP_SESS_RESETTING) {
            if (evt.type == UWB_LINK_APP_EVT_DATA_RESET_ACK) {
                session_reset_confirm();   // LINK 确认复位完成
            }
            // 其他事件丢弃（旧会话的残留事件）
            if (evt.type == UWB_LINK_APP_EVT_DATA_FRAG) {
                UwbSlots_Free(evt.data.data_frag.slot_index); // 释放残留 slot
            }
            continue;
        }

        switch (evt.type) {
        case UWB_LINK_APP_EVT_DATA_ACK:
            sess.error_count = 0;
            handle_ack(&evt);
            break;
        case UWB_LINK_APP_EVT_DATA_FRAG:
            sess.error_count = 0;
            handle_frag(&evt);
            break;
        case UWB_LINK_APP_EVT_DATA_ERROR:
            sess.error_count++;
            if (sess.error_count >= 2) {
                session_request_reset();  // 进入 RESETTING，等 LINK 确认
            }
            break;
        case UWB_LINK_APP_EVT_DATA_COMPLETE:
            session_request_reset();      // 正常结束也走 reset 流程清理 LINK
            break;
        case UWB_LINK_APP_EVT_DATA_FAIL:
            session_request_reset();
            break;
        case UWB_LINK_APP_EVT_TWR_EXCHANGE:
            handle_twr_exchange(&evt.data.twr);
            break;
        }
    }
}
```

#### WAIT_CFG_ACK → 收到 ACK

```c
static void handle_ack_wait_cfg(UwbLinkAppEvent *evt) {
    // CFG_REQ 的 ACK 已收到，Anchor 在线
    // 等待 LINK 空闲，然后发 GET_INFO
    if (app_send_cmd(&(UwbLinkCmd){
        .type = UWB_LINK_CMD_SEND_DATA_CTRL,
        .target_id = sess.anchor_id,
        .ctrl_type = UWB_DATA_CTRL_GET_INFO,
    })) {
        sess.state = APP_SESS_WAIT_INFO;
        sess.retry_count = 0;
    }
}
```

#### WAIT_INFO → 收到 DATA_FRAG(meta)

```c
static void handle_frag_meta(UwbLinkAppEvent *evt) {
    uwb_slot_t *slot = UwbSlots_Get(evt->data.data_frag.slot_index);
    // 解析 meta: data_type(1B) + data_length(2B) + frag_count(1B) + total_crc(2B)
    uint8_t *p = slot->data + UWB_PROTO_MAC_HEADER_LEN + UWB_PROTO_COMMON_HDR_LEN
               + 4 /* ext_header */;
    sess.total_data_len = UwbProtocol_ReadLe16(&p[1]);
    sess.total_frags    = p[3];
    sess.total_crc      = UwbProtocol_ReadLe16(&p[4]);
    sess.next_frag      = 0;
    memset(sess.frag_bitmap, 0, sizeof(sess.frag_bitmap));
    UwbSlots_Free(evt->data.data_frag.slot_index);  // APP 释放 slot

    // 发第一个 PULL
    if (app_send_cmd(&(UwbLinkCmd){
        .type = UWB_LINK_CMD_SEND_DATA_CTRL,
        .target_id = sess.anchor_id,
        .ctrl_type = UWB_DATA_CTRL_PULL,
        .frag_id = 0,
    })) {
        sess.state = APP_SESS_PULLING;
        sess.retry_count = 0;
    }
}
```

#### PULLING → 收到 DATA_FRAG(data)

```c
static void handle_frag_data(UwbLinkAppEvent *evt) {
    uint8_t frag_id = evt->data.data_frag.frag_id;
    uwb_slot_t *slot = UwbSlots_Get(evt->data.data_frag.slot_index);

    // CRC 校验
    uint16_t payload_len = evt->data.data_frag.payload_len;
    uint8_t *payload = slot->data + UWB_PROTO_MAC_HEADER_LEN + UWB_PROTO_COMMON_HDR_LEN
                     + 4 /* ext_header */;
    uint16_t calc_crc = crc16(payload, payload_len - 2);
    uint16_t recv_crc = UwbProtocol_ReadLe16(&payload[payload_len - 2]);

    if (calc_crc != recv_crc) {
        // CRC 错误 → NACK
        app_send_cmd(&(UwbLinkCmd){
            .type = UWB_LINK_CMD_SEND_DATA_CTRL,
            .target_id = sess.anchor_id,
            .ctrl_type = UWB_DATA_CTRL_NACK,
            .frag_id = frag_id,
        });
        UwbSlots_Free(evt->data.data_frag.slot_index);
        return;
    }

    // 拼入 rx_buf（支持乱序）
    uint16_t data_len = payload_len - 2;
    uint16_t offset = frag_id * 62;
    memcpy(&sess.rx_buf[offset], payload, data_len);
    sess.frag_bitmap[frag_id / 8] |= (1 << (frag_id % 8));
    UwbSlots_Free(evt->data.data_frag.slot_index);

    // 检查是否所有分片都已收齐
    if (frag_bitmap_complete()) {
        // 计算实际总长
        uint16_t total_rx = 0;
        for (uint8_t i = 0; i < sess.total_frags; i++) {
            total_rx += (i == sess.total_frags - 1)
                ? (sess.total_data_len - i * 62)   // 最后一片实际长度
                : 62;
        }
        // 校验总 CRC
        uint16_t total_crc = crc16(sess.rx_buf, sess.total_data_len);
        if (total_crc == sess.total_crc) {
            app_send_cmd(&(UwbLinkCmd){
                .type = UWB_LINK_CMD_SEND_DATA_CTRL,
                .target_id = sess.anchor_id,
                .ctrl_type = UWB_DATA_CTRL_DONE,
            });
            sess.state = APP_SESS_WAIT_DONE_ACK;
        } else {
            session_request_reset();  // 总 CRC 不对
        }
    } else {
        // 拉下一片（跳过已收到的）
        sess.next_frag = next_missing_frag();
        app_send_cmd(&(UwbLinkCmd){
            .type = UWB_LINK_CMD_SEND_DATA_CTRL,
            .target_id = sess.anchor_id,
            .ctrl_type = UWB_DATA_CTRL_PULL,
            .frag_id = sess.next_frag,
        });
    }
}

// 检查 frag_bitmap 是否完整
static bool frag_bitmap_complete(void) {
    for (uint8_t i = 0; i < sess.total_frags; i++) {
        if (!(sess.frag_bitmap[i / 8] & (1 << (i % 8)))) return false;
    }
    return true;
}

// 返回第一个未收到的 frag_id
static uint8_t next_missing_frag(void) {
    for (uint8_t i = 0; i < sess.total_frags; i++) {
        if (!(sess.frag_bitmap[i / 8] & (1 << (i % 8)))) return i;
    }
    return sess.total_frags;  // 全部收齐
}
```

#### WAIT_DONE_ACK → 收到 ACK

```c
static void handle_ack_done(UwbLinkAppEvent *evt) {
    // DONE 已确认，发布数据
    publish_data_result(sess.anchor_id, sess.rx_buf, sess.total_data_len);
    session_request_reset();  // 正常结束，走 reset 流程清理 LINK
}
```

### 1.6 超时与重试

每个阶段有独立的重试计数。超时由 LINK 层检测并通过 `DATA_FAIL` 事件上报 APP。

| 阶段 | LINK 超时 | 重试次数 | 超时后动作 |
|------|----------|---------|-----------|
| WAIT_CFG_ACK | 50ms | 3 | DATA_FAIL → session_reset |
| WAIT_INFO | 50ms | 3 | DATA_FAIL → session_reset |
| PULLING | 50ms | 3 每片 | DATA_FAIL → session_reset |
| WAIT_DONE_ACK | 50ms | 3 | DATA_FAIL → session_reset |

### 1.7 会话复位（异步确认）

复位分两步：先发 DATA_RESET 给 LINK，等 LINK 确认后才真正复位自身。

```c
// 发起复位请求（进入 RESETTING 状态）
static void session_request_reset(void) {
    sess.state = APP_SESS_RESETTING;
    sess.reset_retry = 0;

    // 发送 DATA_RESET 命令
    if (UwbLink_CmdQueueHasSpace()) {
        UwbLink_SendCmd(&(UwbLinkCmd){ .type = UWB_LINK_CMD_DATA_RESET });
        sess.last_reset_ms = HAL_GetTick();
    }
}

// 每周期调用：如果 RESETTING 状态超时未确认，重发 DATA_RESET
static void retry_reset(void) {
    if (sess.state != APP_SESS_RESETTING) return;
    if (HAL_GetTick() - sess.last_reset_ms < 100) return;  // 100ms 重试间隔

    sess.reset_retry++;
    if (sess.reset_retry > 10) {
        // 10 次重试仍未确认，强制复位（LINK 可能卡死，slot 由 LINK 超时扫描回收）
        session_reset_confirm();
        return;
    }
    if (UwbLink_CmdQueueHasSpace()) {
        UwbLink_SendCmd(&(UwbLinkCmd){ .type = UWB_LINK_CMD_DATA_RESET });
        sess.last_reset_ms = HAL_GetTick();
    }
}

// LINK 确认复位完成（收到 DATA_RESET_ACK 事件时调用）
static void session_reset_confirm(void) {
    // 记录会话结束时间（用于 1s 间隔计时）
    g_last_session_end_ms = HAL_GetTick();

    // 复位自身状态
    memset(&sess, 0, sizeof(sess));
    sess.state = APP_SESS_IDLE;
}
```

**复位时序**:
```
APP (连续 2 ERROR)                LINK
  │                                │
  │──CMD(DATA_RESET)──────────────→│  link_data_slot_free_all()
  │                                │  data_xfer_state = IDLE
  │←──EVT(DATA_RESET_ACK)────────│  (LINK 确认复位完成)
  │                                │
  │  session_reset_confirm()       │
  │  sess.state = IDLE             │
```

**RESETTING 状态下的事件处理**: APP 只接受 `DATA_RESET_ACK`，丢弃其他事件。对于携带 slot_index 的事件（如 DATA_FRAG），需要先释放 slot 再丢弃，防止 slot 泄漏。

---

## 二、Anchor 侧 APP 设计

### 2.1 会话结构

```c
typedef enum {
    ANCHOR_SESS_IDLE = 0,
    ANCHOR_SESS_ACTIVE,       // 正在传输数据
    ANCHOR_SESS_LAST_SENT,    // 最后一片已发送，等 DONE
} anchor_session_state_t;

typedef struct {
    anchor_session_state_t state;
    uint16_t tag_id;             // 请求方 Tag
    uint32_t session_start_ms;

    /* 预打包数据 */
    uint8_t  tx_buf[512];
    uint16_t tx_len;
    uint8_t  total_frags;
    uint16_t total_crc;

    /* 发送进度 */
    uint8_t  last_sent_frag;     // 上次发送的分片号
    bool     meta_sent;          // meta 分片是否已发

    /* APP 层分配的待发送 slot (需在重启时释放) */
    int8_t   pending_slots[4];   // 最多同时持有 4 个 APP_OWN slot
    uint8_t  pending_slot_count;
} anchor_data_session_t;
```

### 2.2 会话状态机

```
IDLE
  │ LINK 上报 DATA_CFG 事件 → anchor_handle_cfg()
  │ → APP 准备数据 (切片 + CRC)
  │ → state = ACTIVE
  v
ACTIVE
  │ LINK 上报 DATA_CTRL(GET_INFO)
  │   → 检查会话: 如果未结束 → 先下发 meta，再重启（见 2.3）
  │   → 写 meta 到 slot → CMD(SEND_FRAG, frag=meta)
  │ LINK 上报 DATA_CTRL(PULL, frag=N)
  │   → 检查会话: 如果未结束 → 先下发 frag N，再重启
  │   → 写 frag N 到 slot → CMD(SEND_FRAG, frag=N)
  │   → 如果 N == total_frags-1 → state = LAST_SENT
  │ LINK 上报 DATA_CTRL(DONE)
  │   → 写 ACK 到 slot → CMD(SEND_ACK)
  │   → state = IDLE (会话结束)
  │ 超时 1s 无请求 → IDLE (释放旧 session slot)
  v
LAST_SENT
  │ LINK 上报 DATA_CTRL(DONE)
  │   → 写 ACK → CMD(SEND_ACK)
  │   → state = IDLE
  │ LINK 上报 DATA_CTRL(PULL/NACK) — 对方没收到最后一片，重发
  │   → 写 frag(last) 到 slot → CMD(SEND_FRAG)
  │ 超时 1s → IDLE
```

### 2.2.1 Anchor 事件分发

```c
// Anchor 侧 drain_link_events (在 UwbApp_Task 中)
static void anchor_drain_link_events(void) {
    UwbLinkAppEvent evt;
    while (xQueueReceive(g_app_evt_queue, &evt, 0)) {
        switch (evt.type) {
        case UWB_LINK_APP_EVT_DATA_CFG:
            anchor_handle_cfg(&evt.data.data_cfg);
            break;
        case UWB_LINK_APP_EVT_DATA_CTRL:
            anchor_handle_ctrl(&evt.data.data_ctrl);
            break;
        // TWR 事件不变...
        }
    }
}

// 收到 CFG_REQ: 初始化会话
static void anchor_handle_cfg(UwbDataCfgEvent *cfg) {
    // 如果旧会话未结束，先释放旧会话的 pending slot
    if (anch_sess.state != ANCHOR_SESS_IDLE) {
        anchor_free_pending_slots();
    }
    anchor_session_init(cfg->src_id);
}
```

### 2.3 请求到达时的会话检查（核心逻辑）

```
收到请求时:
  1. 检查当前会话是否已结束 (state == IDLE)
     → 已结束: 正常处理，启动新会话
     → 未结束: 先下发当前请求的应答，然后复位重启会话
```

"先下发，再重启"的含义：当旧会话尚未结束时新请求到达，不应简单丢弃旧会话或新请求，而是：

1. **先下发**: 正常处理新请求，准备对应的分片数据，通过 LINK 发送
2. **再重启**: 发送完成后，复位会话状态，以新请求为起点重新开始

这确保了即使状态不同步，当前请求也不会丢失。

```c
static void anchor_handle_ctrl(UwbLinkAppEvent *evt) {
    UwbDataCtrlEvent *ctrl = &evt->data.data_ctrl;
    bool need_restart = false;

    // 检查会话状态
    if (anch_sess.state != ANCHOR_SESS_IDLE) {
        // 旧会话未结束 — 先下发当前请求，然后标记重启
        need_restart = true;
    }

    // 如果当前无会话，初始化
    if (anch_sess.state == ANCHOR_SESS_IDLE) {
        anchor_session_init(ctrl->src_id);
    }

    // 按请求类型准备并下发数据
    switch (ctrl->ctrl_type) {
    case UWB_DATA_CTRL_GET_INFO:
        anchor_prepare_meta_slot();
        UwbLink_SendCmd(&(UwbLinkCmd){
            .type = UWB_LINK_CMD_SEND_FRAG,
            .slot_index = meta_slot,
            .frag_id = 0xFF,  // meta 用特殊 frag_id
        });
        anch_sess.meta_sent = true;
        break;

    case UWB_DATA_CTRL_PULL:
        anchor_prepare_frag_slot(ctrl->frag_id);
        UwbLink_SendCmd(&(UwbLinkCmd){
            .type = UWB_LINK_CMD_SEND_FRAG,
            .slot_index = frag_slot,
            .frag_id = ctrl->frag_id,
        });
        anch_sess.last_sent_frag = ctrl->frag_id;
        if (ctrl->frag_id == anch_sess.total_frags - 1) {
            anch_sess.state = ANCHOR_SESS_LAST_SENT;
        }
        break;

    case UWB_DATA_CTRL_NACK:
        // 重发指定分片
        anchor_prepare_frag_slot(ctrl->frag_id);
        UwbLink_SendCmd(&(UwbLinkCmd){
            .type = UWB_LINK_CMD_SEND_FRAG,
            .slot_index = frag_slot,
            .frag_id = ctrl->frag_id,
        });
        break;

    case UWB_DATA_CTRL_DONE:
        anchor_prepare_ack_slot();
        UwbLink_SendCmd(&(UwbLinkCmd){
            .type = UWB_LINK_CMD_SEND_ACK,
            .slot_index = ack_slot,
        });
        anch_sess.state = ANCHOR_SESS_IDLE;  // 会话结束
        break;
    }

    // 如果需要重启（旧会话未结束），在下发完成后复位
    if (need_restart) {
        anchor_session_restart(ctrl->src_id);
    }
}
```

### 2.4 数据预打包

收到 CFG_REQ 时，Anchor APP 预打包全部数据：

```c
static void anchor_session_init(uint16_t tag_id) {
    memset(&anch_sess, 0, sizeof(anch_sess));
    anch_sess.tag_id = tag_id;
    anch_sess.session_start_ms = HAL_GetTick();
    anch_sess.state = ANCHOR_SESS_ACTIVE;

    // 从传感器缓冲区读取待发送数据
    anch_sess.tx_len = anchor_fetch_data(anch_sess.tx_buf, sizeof(anch_sess.tx_buf));
    anch_sess.total_frags = (anch_sess.tx_len + 61) / 62;  // 每片 62B 净载荷
    anch_sess.total_crc = crc16(anch_sess.tx_buf, anch_sess.tx_len);
}
```

### 2.5 分片打包（写入 slot）

```c
static bool anchor_prepare_frag_slot(uint8_t frag_id) {
    // 边界检查: frag_id 必须在有效范围内
    if (frag_id >= anch_sess.total_frags) return false;

    int8_t idx = UwbSlots_Alloc(UWB_SLOT_APP_OWN);
    if (idx < 0) return false;

    uwb_slot_t *slot = UwbSlots_Get(idx);
    uint16_t offset = frag_id * 62;
    uint16_t frag_len = (offset + 62 <= anch_sess.tx_len) ? 62 : (anch_sess.tx_len - offset);

    // 写 payload: data[N] + crc16(2B)
    memcpy(slot->data, &anch_sess.tx_buf[offset], frag_len);
    uint16_t frag_crc = crc16(&anch_sess.tx_buf[offset], frag_len);
    UwbProtocol_WriteLe16(&slot->data[frag_len], frag_crc);
    slot->data_len = frag_len + 2;

    // 记录到 pending_slots，以便重启时释放
    if (anch_sess.pending_slot_count < 4) {
        anch_sess.pending_slots[anch_sess.pending_slot_count++] = idx;
    }
    return true;
}

// 释放 APP 层持有的所有 slot
static void anchor_free_pending_slots(void) {
    for (uint8_t i = 0; i < anch_sess.pending_slot_count; i++) {
        if (anch_sess.pending_slots[i] >= 0) {
            UwbSlots_Free(anch_sess.pending_slots[i]);
        }
    }
    anch_sess.pending_slot_count = 0;
}
```

### 2.6 会话重启

```c
static void anchor_session_restart(uint16_t new_tag_id) {
    // 1. 释放旧 session 的 APP_OWN slot
    anchor_free_pending_slots();

    // 2. 保留 tx_buf 中已打包的数据（可选：如果 tag_id 变了，重新取数据）
    if (anch_sess.tag_id != new_tag_id) {
        anch_sess.tx_len = anchor_fetch_data(anch_sess.tx_buf, sizeof(anch_sess.tx_buf));
        anch_sess.total_frags = (anch_sess.tx_len + 61) / 62;
        anch_sess.total_crc = crc16(anch_sess.tx_buf, anch_sess.tx_len);
    }
    anch_sess.tag_id = new_tag_id;
    anch_sess.session_start_ms = HAL_GetTick();
    anch_sess.state = ANCHOR_SESS_ACTIVE;
    anch_sess.meta_sent = false;
    anch_sess.last_sent_frag = 0;
}
```

---

## 三、APP ↔ LINK 接口

### 3.1 APP → LINK 命令队列

在 link 层现有 `g_app_evt_queue` 基础上，新增反向通道：

```c
// uwb_link.h 新增

typedef enum {
    UWB_LINK_CMD_NONE = 0,
    UWB_LINK_CMD_SEND_DATA_CFG,     // Tag: 发起 CFG_REQ
    UWB_LINK_CMD_SEND_DATA_CTRL,    // Tag: 发送 CTRL 帧
    UWB_LINK_CMD_SEND_FRAG,         // Anchor: 发送一个分片
    UWB_LINK_CMD_SEND_ACK,          // Anchor: 发送 ACK
    UWB_LINK_CMD_DATA_RESET,        // APP → LINK: 复位数据槽
} UwbLinkCmdType;

typedef struct {
    UwbLinkCmdType type;
    int8_t   slot_index;     // SEND_FRAG/SEND_ACK 时: APP 写好的 slot
    uint16_t target_id;
    uint8_t  ctrl_type;      // SEND_DATA_CTRL 时: GET_INFO/PULL/NACK/DONE
    uint8_t  frag_id;        // CTRL 或 FRAG 的 frag_id
} UwbLinkCmd;

// API
bool UwbLink_SendCmd(const UwbLinkCmd *cmd);
bool UwbLink_CmdQueueHasSpace(void);
```

**队列参数**: depth=4, 元素=UwbLinkCmd

### 3.2 LINK → APP 事件（扩展现有队列）

```c
// uwb_link.h 扩展 UwbLinkAppEventType

typedef enum {
    UWB_LINK_APP_EVT_NONE = 0,
    UWB_LINK_APP_EVT_TWR_EXCHANGE,    // 现有: TWR 测距数据
    UWB_LINK_APP_EVT_NEIGHBOR_SEEN,   // 现有: 邻居发现
    // --- plan-v4 新增 ---
    UWB_LINK_APP_EVT_DATA_CFG,        // Anchor: 收到 CFG_REQ (src_id)
    UWB_LINK_APP_EVT_DATA_CTRL,       // Anchor: 收到 CTRL 帧
    UWB_LINK_APP_EVT_DATA_FRAG,       // Tag: 收到分片 (slot_index, frag_id)
    UWB_LINK_APP_EVT_DATA_ACK,        // Tag: 收到 ACK
    UWB_LINK_APP_EVT_DATA_ERROR,      // 收到 ERROR 帧 (用于连续计数)
    UWB_LINK_APP_EVT_DATA_COMPLETE,   // Tag: 传输成功完成
    UWB_LINK_APP_EVT_DATA_FAIL,       // 传输失败 (超时/重试耗尽)
    UWB_LINK_APP_EVT_DATA_RESET_ACK,  // LINK → APP: DATA_RESET 确认 (slot 已全部回收)
} UwbLinkAppEventType;

// 事件结构扩展
typedef struct {
    UwbLinkAppEventType type;
    union {
        UwbTwrExchange twr;
        struct { uint16_t short_id; uint16_t capability; } neighbor;
        // --- plan-v4 新增 ---
        struct { uint16_t src_id; } data_cfg;
        struct {
            uint16_t src_id;
            uint8_t  ctrl_type;      // GET_INFO/PULL/NACK/DONE
            uint8_t  frag_id;
        } data_ctrl;
        struct {
            uint16_t src_id;
            uint8_t  frag_id;
            uint8_t  total_frags;
            uint8_t  flags;          // bit0: meta
            int8_t   slot_index;     // slot 持有帧数据
            uint16_t payload_len;
        } data_frag;
        struct { uint16_t src_id; } data_ack;
        struct { uint16_t src_id; uint8_t reason; } data_error;
        struct { uint16_t src_id; } data_complete;
        struct { uint16_t src_id; uint8_t reason; } data_fail;
        struct { uint8_t success; } data_reset_ack;  // LINK→APP: 复位确认
    } data;
} UwbLinkAppEvent;
```

### 3.3 交互流程总览

```
Tag APP                Tag LINK              PHY              Anchor LINK           Anchor APP
   │                      │                   │                   │                      │
   │──CMD(CFG)──────────→│                   │                   │                      │
   │                      │──CFG_REQ─────────→│──CFG_REQ─────────→│                      │
   │                      │                   │←──ACK(fast)──────│                      │
   │                      │←──TX_DONE────────│                   │                      │
   │                      │←──RX_SLOT(ACK)───│                   │                      │
   │←──EVT(ACK)─────────│                   │                   │                      │
   │                      │                   │                   │←──EVT(DATA_CFG)────│
   │──CMD(CTRL,GET_INFO)→│                   │                   │                      │
   │                      │──CTRL(GET_INFO)──→│──CTRL(GET_INFO)──→│                      │
   │                      │                   │                   │←──EVT(DATA_CTRL)───│
   │                      │                   │                   │──CMD(SEND_FRAG)──→│(meta)
   │                      │                   │←──FRAG(meta)─────│                      │
   │                      │←──RX_SLOT(meta)──│                   │                      │
   │←──EVT(FRAG,meta)───│                   │                   │                      │
   │                      │                   │                   │                      │
   │──CMD(CTRL,PULL,0)──→│                   │                   │                      │
   │                      │──CTRL(PULL,0)────→│──CTRL(PULL,0)────→│                      │
   │                      │                   │                   │←──EVT(DATA_CTRL)───│
   │                      │                   │                   │──CMD(SEND_FRAG)──→│(frag=0)
   │                      │                   │←──FRAG(0)────────│                      │
   │                      │←──RX_SLOT(frag0)─│                   │                      │
   │←──EVT(FRAG,0)──────│                   │                   │                      │
   │ ... (重复 PULL/FRAG 直到最后一片)        │                   │                      │
   │                      │                   │                   │                      │
   │──CMD(CTRL,DONE)────→│                   │                   │                      │
   │                      │──CTRL(DONE)──────→│──CTRL(DONE)──────→│                      │
   │                      │                   │                   │←──EVT(DATA_CTRL)───│
   │                      │                   │                   │──CMD(SEND_ACK)────→│
   │                      │                   │←──ACK────────────│                      │
   │                      │←──RX_SLOT(ACK)───│                   │                      │
   │←──EVT(ACK)─────────│                   │                   │                      │
   │(会话完成)            │                   │                   │(会话完成)            │
```

---

## 四、LINK 层数据挂起表

LINK 层维护 `g_data_slots[8]` 数据 slot 挂起表，跟踪当前线程所有数据相关 slot（TX_FRAME / RX_FRAME / BACKUP）。APP 下发 `DATA_RESET` 时 LINK 调用 `link_data_slot_free_all()` 批量回收。详细结构与操作见 [link层数据帧方案.md - 数据挂起表](link层数据帧方案.md#数据挂起表)。

复位流程：
```
APP (连续 2 ERROR)
  │
  ├─→ 1. session_request_reset()
  │      UwbLink_SendCmd(DATA_RESET) → 进入 RESETTING
  │
  ├─→ 2. LINK drain_app_cmds():
  │      link_data_slot_free_all()
  │      data_xfer_state = IDLE
  │      → 回复 EVT(DATA_RESET_ACK)
  │
  └─→ 3. APP 收到 DATA_RESET_ACK
         session_reset_confirm()
         sess.state = IDLE
```

---

## 五、MAC 表

Tag 侧维护 MAC 表，记录发现的 Anchor，作为轮询目标。

```c
#define MAC_TABLE_SIZE 8
#define MAC_ENTRY_TIMEOUT_MS  (3 * 60 * 1000)  // 3 分钟无通信剔除

typedef struct {
    bool     valid;
    uint16_t anchor_id;
    uint32_t last_seen_ms;
    uint32_t last_poll_ms;   // 上次轮询时间
} mac_entry_t;

mac_entry_t g_mac_table[MAC_TABLE_SIZE];
```

**轮询策略**: 每 1s 取一个 Anchor 发起数据会话，轮转。

```c
static uint16_t mac_table_next(void) {
    static int round_robin = 0;
    for (int i = 0; i < MAC_TABLE_SIZE; i++) {
        int idx = (round_robin + i) % MAC_TABLE_SIZE;
        if (g_mac_table[idx].valid) {
            round_robin = (idx + 1) % MAC_TABLE_SIZE;
            return g_mac_table[idx].anchor_id;
        }
    }
    return 0;  // 无可用 Anchor
}
```

**MAC 表更新**: 在 Discovery/Ranging 流程中，每收到一个 Anchor 的响应，更新/插入 MAC 表。

---

## 六、关键约束

| 约束 | 值 | 说明 |
|------|----|------|
| Tag 轮询间隔 | 1s | 每 1s 对一个 Anchor 发起会话 |
| 会话总超时 | 5s | 超过则复位 |
| ERROR 容忍 | 连续 2 次 | 达到后进入 RESETTING 状态 |
| DATA_RESET 重试间隔 | 100ms | RESETTING 状态下重发间隔 |
| DATA_RESET 最大重试 | 10 次 | 超过后强制复位 |
| 单操作重试 | 3 次 | LINK 层负责，超时后上报 DATA_FAIL |
| Anchor 超时 | 1s | 无请求则回收会话 |
| MAC 表大小 | 8 条 | 3 分钟无通信剔除 |
| 命令队列深度 | 4 | APP→LINK 命令队列 |

---

## 七、文件修改清单

| 文件 | 改动 |
|------|------|
| `uwb_app.h` | 新增 `app_tag_session_t`、`anchor_data_session_t`、`mac_entry_t`；声明 `UwbApp_SessionReset()` |
| `uwb_app.c` | Tag 会话状态机 (`drain_link_events`、状态处理函数)；Anchor 会话状态机 (`anchor_handle_*`)；MAC 表；1s 轮询调度；数据切片/拼装/CRC；ERROR 计数和会话复位 |
| `uwb_link.h` | 新增 `UwbLinkCmdType`、`UwbLinkCmd`、`UwbLink_SendCmd()`；扩展 `UwbLinkAppEventType` 和事件联合体 |
| `uwb_link.c` | APP→LINK 命令队列；挂起表 (`g_data_slots` + 操作函数)；`drain_app_cmds()`；`DATA_RESET` 处理 |
| `uwb_stack_types.h` | `UwbDataCtrlEvent`、`UwbDataFragEvent`、`mac_entry_t` |
| `uwb_slots.h` | 新增 `UWB_SLOT_APP_OWN`；`UWB_SLOT_NUM` 8 → 16 |

---

## 八、实现顺序

1. **类型定义**: `uwb_stack_types.h` 新增结构体，`uwb_slots.h` 扩容 + 新增 APP_OWN
2. **LINK 命令队列 + 挂起表**: `uwb_link.c/h` APP↔LINK 双向通道
3. **Tag 侧会话状态机**: `uwb_app.c` IDLE→WAIT_CFG_ACK→WAIT_INFO→PULLING→WAIT_DONE_ACK
4. **Anchor 侧会话状态机**: `uwb_app.c` IDLE→ACTIVE→LAST_SENT
5. **MAC 表 + 1s 轮询**: `uwb_app.c` MAC 表管理 + 调度
6. **ERROR 复位链路**: APP ERROR 计数 → session_request_reset → DATA_RESET CMD → LINK free_all + DATA_RESET_ACK → APP session_reset_confirm → IDLE
7. **端到端测试**: 三阶段帧日志 + 异常注入验证
