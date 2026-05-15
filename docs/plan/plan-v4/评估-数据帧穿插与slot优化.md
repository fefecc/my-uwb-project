# 数据帧穿插调度与 Slot 分配 — 评估与优化方案

> 创建时间: 2026-05-14
> 基于: 当前 UWB-V2 分支实际代码 + plan-v4 设计文档对比分析

---

## 一、当前实现状态总结

plan-v4 的三阶段数据帧传输（CFG→META→PULL/FRAG→DONE）已在代码中完整实现，包含：

- PHY 层: DATA_CFG_REQ 快速 ACK 应答、keep_slot 机制
- LINK 层: 2+1 交织调度、数据挂起表 (8条)、超时重发、APP↔LINK 命令队列
- APP 层: Tag 5 状态会话状态机、Anchor 3 状态会话状态机、MAC 表、CRC16、1s 轮询

---

## 二、问题分析

### 问题 1: 测距帧在数据会话期间完全停止

**根因**: [uwb_link.c:982-1022](APP/UWB/uwb_link.c#L982-L1022)

当前 2+1 交织逻辑：

```
if (data_xfer_state != IDLE) {
    // 只做超时检查+重试，不发 Discovery
    link_tag_handle_data_xfer();
} else if (!disc_exchange_active) {
    if (disc_count >= 2 && UwbLink_HasDataCmd()) {
        // 进入数据会话
    } else {
        link_tag_send_disc();  // 只在 IDLE 状态才发 Discovery
    }
}
```

**影响量化**:

一个完整的数据会话（9 片数据，每片 TAG→CTRL→Anchor→FRAG→TAG 约 15-50ms）耗时约 **300-600ms**。在此期间 `data_xfer_state != IDLE`，`link_tag_send_disc()` 永远不会被调用。

| 场景 | 无测距时长 | 丢失测距次数 (200ms周期) |
|------|-----------|------------------------|
| 1 片数据 | ~50ms | 0 |
| 4 片数据 | ~200ms | 1 |
| 9 片数据 | ~450ms | 2-3 |

测距频率在数据会话期间从 5Hz 降到 0Hz，这对实时定位应用不可接受。

**数据会话期间的时序**（当前）:
```
DISC#1  DISC#2  [CFG→ACK→GET_INFO→META→PULL→FRAG#0→PULL→FRAG#1→...→DONE→ACK]
 200ms   200ms   ←———— 约 300-600ms，零测距 ————→
```

### 问题 2: Slot 分配与回收不一致

#### 2.1 SEND_ACK 双重分配

[uwb_link.c:626-658](APP/UWB/uwb_link.c#L626-L658): APP 先分配 `APP_OWN` slot 写 ACK payload，LINK 收到后 `Free` 掉再重新 `Alloc(LINK_OWN)` 构建完整帧。这是两次分配一次释放，非零拷贝，且增加了 slot 池压力。

#### 2.2 SEND_ACK keep_slot=false 与挂起表冲突

[uwb_link.c:645-653](APP/UWB/uwb_link.c#L645-L653): SEND_ACK 先 `link_data_slot_register(idx, TX_FRAME, ...)` 注册到挂起表，然后 `keep_slot = false` 交给 PHY。PHY 在 TX_DONE FINISH 中释放该 slot。但挂起表中的条目未被 `unregister`。后续 `link_data_slot_free_all()` 会 double-free。

#### 2.3 Discovery TX slot 由 PHY 释放，数据帧 TX slot 由 LINK 释放

plan-v4 设计原则是 "PHY 层不再释放任何 slot，统一由 LINK 回收"，但 Discovery 帧仍使用 `keep_slot=false`（默认值，[uwb_link.c:752](APP/UWB/uwb_link.c#L752) 未设置），PHY 在 [uwb_phy.c:641](APP/UWB/uwb_phy.c#L641) 直接 free。数据帧使用 `keep_slot=true`，LINK 管理。两条路径不一致，增加理解和维护成本。

#### 2.4 Anchor 旧会话 pending_slots 泄漏

[uwb_app.c:762-764](APP/UWB/uwb_app.c#L762-L764): `anchor_handle_ctrl()` 只在 `state == IDLE` 时调用 `anchor_session_init()`（内部有 `anchor_free_pending_slots()`）。如果 state 已是 ACTIVE 或 LAST_SENT，收到新请求时直接跳到 switch 处理，旧 `pending_slots[4]` 中的 slot 不会被释放。

#### 2.5 测距帧和数据帧无 slot 隔离

16 个 slot 共享池。数据会话活跃时可能同时持有:
- 1 个 `data_retry_slot`（LINK TX 备份）
- 1 个正在接收的 FRAG slot（PHY→LINK→APP）
- 最多 4 个 pending_slots（Anchor APP）

共最多 6 个 slot 被数据帧占用。加上 Discovery 一次需要 1 TX + 最多 4 RX = 5 个 slot。短期峰值可达 11 个，接近 16 个的上限。虽然通常不会耗尽，但没有保护机制保证测距帧永远能分配到 slot。

### 问题 3: 代码重复 — 数据帧处理逻辑存在两份

`drain_phy_events()` ([uwb_link.c:300-501](APP/UWB/uwb_link.c#L300-L501)) 和 `link_tag_send_disc()` 的阻塞等待循环 ([uwb_link.c:776-910](APP/UWB/uwb_link.c#L776-L910)) 中各有一份几乎完全相同的数据帧处理代码（DATA_CTRL_RESP、DATA_FRAG、DATA_CFG、DATA_CTRL 四个分支）。新增帧类型需要改两处，容易遗漏。

### 问题 4: 数据帧逻辑小问题

#### 4.1 LinkCmd 中 total_frags/flags/payload_len 未在 CTRL 命令中使用

`UwbLinkCmd` 结构体有 `total_frags`、`flags`、`payload_len` 字段。SEND_DATA_CTRL 和 SEND_FRAG 共用同一个结构体。SEND_DATA_CTRL 时这些字段无意义但未清零，可能导致误用。

#### 4.2 TAG 侧 link_tag_send_disc 中收到数据帧时的处理

[uwb_link.c:814-883](APP/UWB/uwb_link.c#L814-L883): `link_tag_send_disc()` 是 DISC_REQ 的发送函数，其阻塞等待循环中处理了数据帧（DATA_ACK、DATA_FRAG、DATA_CFG、DATA_CTRL）。但此时 `data_xfer_state` 可能为 IDLE（因为发送 DISC_REQ 时 `disc_exchange_active=true`，主循环不会进入数据分支），导致收到数据帧时 `link_data_tx_confirmed()` 被调用但没有对应的 data_retry_slot，以及事件发送给 APP 但 APP 可能不在正确的状态。

---

## 三、优化方案

### 方案 A: 细粒度穿插 — 数据帧与测距帧交替

**核心思想**: 不再等整个数据会话完成才恢复测距，而是在每完成一次数据帧交换后穿插一次 Discovery。

**修改点**: LINK 层主循环 [uwb_link.c:975-1039](APP/UWB/uwb_link.c#L975-L1039)

```
新逻辑:
  每个主循环周期:
    1. drain_phy_events()
    2. drain_app_cmds()
    3. 如果 data_xfer_state != IDLE:
       a. 如果有待处理的 RX 应答 → 已在 drain_phy_events 中处理
       b. 如果等待 ACK 且超时 → 重试
       c. 如果刚刚收到应答/分片 (本轮 drain_phy_events 处理了数据帧):
          → 说明一次数据帧交换已完成
          → 如果 disc_exchange_active == false:
              → 发一次 DISC_REQ (穿插测距)
              → 标记 data_round_complete = true
       d. 如果 data_round_complete && !disc_exchange_active:
          → 说明测距穿插完成，可以处理下一个数据命令
          → 但 drain_app_cmds 已经消费了命令（如果有的话）
          → 实际上这个逻辑自然流转: APP 收到 LINK 事件后会发新命令
    4. 否则 (data_xfer_state == IDLE):
       → 按原来的 2+1 逻辑发 DISC_REQ 或启动数据会话
```

**关键**: 不在 `data_xfer_state != IDLE` 时阻止 Discovery，而是：
- 每次 `drain_phy_events()` 处理完一个数据帧交换（例如 TAG 收到 ACK 或 FRAG），允许发一次 DISC_REQ
- DISC_REQ 完成后，drain_app_cmds 自然消费下一个 APP 命令

**实现方式**（简化版）:

```c
// 在 data_xfer_state != IDLE 分支中:
if (g_link.data_xfer_state != UWB_LINK_DATA_IDLE) {
    // 超时检查 + 重试（不变）
    link_data_check_timeout();

    // 穿插测距: 如果当前没有活跃的 DISC 交换，发一次
    if (!g_link.disc_exchange_active) {
        // 检查数据帧最小间隔 (15ms)
        if (HAL_GetTick() - g_link.last_data_tx_ms >= LINK_DATA_FRAME_INTERVAL_MS) {
            link_tag_send_disc();
        }
    }
}
```

**效果**:
```
Before: DISC#1  DISC#2  [CFG→ACK  GET_INFO→META  PULL#0→FRAG#0  ...  DONE→ACK]
                         ←—————— 零测距 300-600ms ——————→

After:  DISC#1  DISC#2  CFG→ACK  DISC  GET_INFO→META  DISC  PULL#0→FRAG#0  DISC  ...
                         ← 测距帧和数据帧交替，测距间隔最大 ~50ms →
```

**时序估计**:
- 一次数据帧交换 (CTRL→FRAG): ~15-30ms
- 一次 Discovery (DISC_REQ→RESP→4槽RX): ~10-15ms
- 测距间隔: 最大约 50ms (一次数据交换 + 一次 Discovery)，约 20Hz

**注意事项**:
1. 穿插的 DISC_REQ 不应影响 `disc_count` 的 2+1 计数逻辑（因为数据会话已由命令驱动启动）
2. 数据帧最小间隔 15ms 依然遵守
3. DISC_REQ 的 `keep_slot` 应统一为 true，由 LINK 管理 slot

### 方案 B: Slot 分区预留

**目标**: 保证测距帧永远有 slot 可用。

**方案 B1 — 软预留**（推荐，改动最小）:

在 `UwbSlots_Alloc()` 中增加优先级参数:

```c
typedef enum {
    UWB_SLOT_PRIO_NORMAL = 0,  // 数据帧
    UWB_SLOT_PRIO_HIGH   = 1,  // 测距帧 (Discovery/TWR)
} uwb_slot_prio_t;

int8_t UwbSlots_Alloc(uwb_slot_owner_t owner, uwb_slot_prio_t prio);
```

- `PRIO_HIGH`: 扫描全部 16 个 slot，包括被数据帧占用的也可以"抢占"（实际是等待）
- `PRIO_NORMAL`: 只能使用 slot 4-15（给测距预留 4 个）

或者更简单：测距帧分配时如果池满，记录一个 pending 事件，下次再试。

**方案 B2 — 硬分区**:

```c
#define UWB_SLOT_RANGING_BASE  0
#define UWB_SLOT_RANGING_NUM   4   // slot 0-3: 仅测距
#define UWB_SLOT_DATA_BASE     4
#define UWB_SLOT_DATA_NUM      12  // slot 4-15: 数据帧
```

但这需要两套 Alloc/Free，增加复杂度。

**推荐**: 方案 B1 的简化版——给测距帧分配时使用完整 16 槽扫描，数据帧分配时如果空闲槽不足 4 个则拒绝（等测距释放）。

### 方案 C: 数据帧逻辑修复

#### C.1 统一 slot 回收策略

所有 TX 帧统一使用 `keep_slot = true`，由 LINK 负责释放：

| 帧类型 | 当前 | 修改后 |
|--------|------|--------|
| DISC_REQ | keep_slot=false, PHY 释放 | keep_slot=true, LINK 在 TX_DONE 后释放 |
| DATA_CFG | keep_slot=true, LINK 释放 | 不变 |
| DATA_CTRL | keep_slot=true, LINK 释放 | 不变 |
| SEND_FRAG | keep_slot=true, LINK 释放 | 不变 |
| SEND_ACK | keep_slot=false, PHY 释放 | keep_slot=true, LINK 在 TX_DONE 后释放 |

改造后 PHY 层完全不再调用 `UwbSlots_Free()`，PHY 看门狗也改为只发 ERROR 事件（携带 slot_index），由 LINK 释放。

#### C.2 SEND_ACK 改为零拷贝

当前:
```
APP Alloc(APP_OWN) → 写 payload → CMD(SEND_ACK, slot=idx)
LINK: Free(idx) → Alloc(LINK_OWN) → 构建完整帧 → PHY_CMD
```

修改为:
```
APP Alloc(APP_OWN) → 写 payload → CMD(SEND_ACK, slot=idx)
LINK: 直接在 slot idx 上 Encode 完整帧 (覆盖 APP payload) → PHY_CMD
```

LINK 不需要重新分配 slot，直接在 APP 传过来的 slot 上 Encode（因为 ACK 帧很小，payload 只有 2 字节，帧头可以覆盖写入）。

#### C.3 Anchor 旧会话处理

在 `anchor_handle_ctrl()` 开头统一处理:

```c
static void anchor_handle_ctrl(UwbLinkAppEvent *evt) {
    // 如果旧会话未结束，先释放旧资源
    if (g_anch_sess.state != ANCHOR_SESS_IDLE) {
        anchor_free_pending_slots();
    }
    // 如果没有活跃会话，初始化
    if (g_anch_sess.state == ANCHOR_SESS_IDLE) {
        anchor_session_init(ctrl->src_id);
    }
    // ... 正常处理请求
}
```

#### C.4 消除 drain_phy_events 和 link_tag_send_disc 中的重复代码

将数据帧的 RX 处理逻辑抽取为共用函数:

```c
// 新增: 处理接收到的数据帧 (共用于 drain_phy_events 和 link_tag_send_disc)
static void link_process_rx_data_frame(uwb_slot_t *s, int8_t slot_index);
```

这个函数包含 DATA_CTRL_RESP、DATA_FRAG、DATA_CFG、DATA_CTRL 四个分支的处理逻辑（目前约 80 行代码出现了两次）。

### 方案 D: 数据会话超时与异常处理增强

#### D.1 分片级别的超时

当前只有单帧 50ms 超时和总会话 5s 超时。建议增加分片级别的追踪:

```c
// 在 link_context_t 中
uint32_t g_data_frag_start_ms;  // 当前分片开始时间
uint8_t  g_data_frag_retry;     // 当前分片重试次数
```

每开始一个新的分片（CTRL/PULL 发送）时记录时间，超时 50ms 未收到 FRAG 则重试，超过 3 次上报 DATA_FAIL。

#### D.2 Anchor 侧 NACK 处理增强

当前 Anchor 收到 NACK 会重新准备分片并发送。但如果是连续多次 NACK 同一个分片，说明链路质量太差，应该上报异常或直接终止会话。

---

## 四、推荐实施优先级

| 优先级 | 改动 | 工作量 | 影响 |
|--------|------|--------|------|
| **P0** | 方案 A: 细粒度穿插 | 中 (修改 LINK 主循环) | 解决测距连续性问题 |
| **P0** | C.1: 统一 slot 回收 + C.2: SEND_ACK 零拷贝 | 小 | 消除 double-free 隐患 |
| **P1** | C.3: Anchor 旧会话处理 | 小 (加 3 行) | 防止 slot 泄漏 |
| **P1** | C.4: 消除重复代码 | 中 (重构) | 维护性 |
| **P2** | 方案 B: Slot 分区预留 | 小 (软预留) | 防御性 |
| **P2** | D.2: NACK 增强 | 小 | 健壮性 |

---

## 五、方案 A 详细设计（重点）

### 5.1 修改点

**文件**: `APP/UWB/uwb_link.c`

**修改 1**: LINK 主循环中 `data_xfer_state != IDLE` 分支

当前代码 (line 982-1011):
```c
if (g_link.cfg.role == APP_ROLE_TAG) {
    if (g_link.data_xfer_state != UWB_LINK_DATA_IDLE) {
        // 超时检查
        ...
    } else if (!g_link.disc_exchange_active) {
        if (g_link.disc_count >= 2 && UwbLink_HasDataCmd()) {
            g_link.disc_count = 0;
        } else {
            link_tag_send_disc();
        }
    }
}
```

修改为:
```c
if (g_link.cfg.role == APP_ROLE_TAG) {
    if (g_link.data_xfer_state != UWB_LINK_DATA_IDLE) {
        // 1. 超时检查 + 重试（不变）
        link_data_check_timeout();

        // 2. ★穿插测距: 数据会话期间只要 DISC 空闲就发
        if (!g_link.disc_exchange_active) {
            uint32_t now = HAL_GetTick();
            if (now - g_link.last_data_tx_ms >= LINK_DATA_FRAME_INTERVAL_MS) {
                link_tag_send_disc();
            }
        }
    } else if (!g_link.disc_exchange_active) {
        // 空闲状态: 原来的 2+1 逻辑
        if (g_link.disc_count >= 2 && UwbLink_HasDataCmd()) {
            g_link.disc_count = 0;
        } else {
            link_tag_send_disc();
        }
    }
}
```

**修改 2**: 增加 `link_data_check_timeout()` 辅助函数

把当前主循环中的超时检查逻辑抽取到独立函数，保持主循环简洁。

**修改 3**: Discovery TX slot 统一使用 keep_slot=true

[uwb_link.c:752](APP/UWB/uwb_link.c#L752) 附近，PHY_CMD_TX_FRAME 增加 `.keep_slot = true`:
```c
phy_cmd_t cmd;
memset(&cmd, 0, sizeof(cmd));
cmd.type           = PHY_CMD_TX_FRAME;
cmd.slot_index     = idx;
// ...
cmd.keep_slot      = true;  // ★ 统一由 LINK 管理 slot
```

同时在 `drain_phy_events()` 的 `PHY_EVT_TX_DONE` 分支中增加 DISC_REQ TX slot 的回收:
```c
case PHY_EVT_TX_DONE:
    if (evt.slot_index >= 0) {
        uwb_slot_t *s = UwbSlots_Get(evt.slot_index);
        if (s != NULL && s->frame_type == UWB_FUNC_DISCOVERY_REQ) {
            UwbSlots_Free(evt.slot_index);  // DISC_REQ TX slot 回收
        }
    }
    // 数据帧 TX 时间戳记录（不变）
    ...
    break;
```

### 5.2 不变量保证

- 数据帧最小间隔 15ms 始终遵守（`last_data_tx_ms` 检查）
- DISC_REQ 间隔由 `disc_exchange_active` 标志保证不会并发
- 数据会话期间 `disc_count` 继续递增但在数据会话中无意义（因为不再由 2+1 驱动）
- `data_xfer_state` 回到 IDLE 后恢复正常的 2+1 逻辑

### 5.3 时序示意

```
时间轴 (每格 ~15ms):

DISC#1  DISC#2  CFG→ACK  DISC#3  GET_INFO→META  DISC#4  PULL#0→FRAG#0  DISC#5  PULL#1→FRAG#1  ...
  |       |        |        |          |           |          |            |          |
  0      200      400      415       430         445       460          475       490

数据帧耗时: 15ms (单次交换)
Discovery 耗时: ~12ms (TX + 4×2ms 槽)
测距间隔: 15ms (数据) + 12ms (测距) + 3ms (处理) ≈ 30ms → 33Hz
```

相比当前的 200ms 测距周期 + 600ms 盲区，穿插后测距间隔稳定在 ~30ms，测距连续性大幅提升。

---

## 六、附: 当前代码中的帧处理流程图

```
TAG LINK 主循环 (每 1ms)
│
├─ drain_phy_events()
│   ├─ PHY_EVT_TX_DONE → 记录 last_data_tx_ms
│   ├─ PHY_EVT_RX_SLOT_DONE
│   │   ├─ DISC_RESP → 解析 TWR 时间戳 → 通知 APP TWR_EXCHANGE
│   │   ├─ DATA_CTRL_RESP → link_data_tx_confirmed() → 通知 APP DATA_ACK
│   │   ├─ DATA_FRAG → link_data_tx_confirmed() → 通知 APP DATA_FRAG (slot 不释放)
│   │   ├─ DATA_CFG → 通知 APP DATA_CFG → data_xfer_state = ANCHOR_WAIT_CTRL
│   │   └─ DATA_CTRL → link_data_tx_confirmed() → 通知 APP DATA_CTRL
│   ├─ PHY_EVT_RX_WINDOW_END → disc_count++
│   ├─ PHY_EVT_RX_FRAME (IDLE 收帧, Anchor 侧)
│   └─ PHY_EVT_RX_TIMEOUT / PHY_EVT_ERROR
│
├─ drain_app_cmds()
│   ├─ CMD_SEND_DATA_CFG → 构建 CFG_REQ, PHY_CMD_TX_FRAME, state=TAG_WAIT_ACK
│   ├─ CMD_SEND_DATA_CTRL → 构建 CTRL, PHY_CMD_TX_FRAME
│   ├─ CMD_SEND_FRAG → link_add_data_frag_header(), PHY_CMD_TX_FRAME
│   ├─ CMD_SEND_ACK → Free APP slot → Alloc LINK slot → 构建 ACK → PHY_CMD
│   └─ CMD_DATA_RESET → link_data_slot_free_all(), state=IDLE, 回复 RESET_ACK
│
├─ [TAG] data_xfer_state != IDLE → 超时/重试检查
├─ [TAG] data_xfer_state == IDLE → 2+1 交织 → link_tag_send_disc()
└─ [Anchor] 数据会话超时检查 (1s)

TAG APP 主循环 (每 1ms)
│
├─ drain_link_events()
│   ├─ TWR_EXCHANGE → handle_twr_exchange() → DS-TWR/SS-TWR 计算 → 发布结果
│   ├─ DATA_ACK → handle_tag_ack()
│   │   ├─ WAIT_CFG_ACK → CMD_SEND_DATA_CTRL(GET_INFO), state=WAIT_INFO
│   │   └─ WAIT_DONE_ACK → session_request_reset()
│   ├─ DATA_FRAG → handle_tag_frag()
│   │   ├─ META → 解析 total_frags/total_data_len/total_crc
│   │   │        → CMD_SEND_DATA_CTRL(PULL, 0), state=PULLING
│   │   └─ DATA → CRC16 校验 → 拼入 rx_buf
│   │            → 收齐 → 总 CRC 校验 → CMD_SEND_DATA_CTRL(DONE)
│   │            → 未收齐 → CMD_SEND_DATA_CTRL(PULL, next_missing)
│   └─ DATA_ERROR → error_count++ → >=2 → session_request_reset()
│
├─ retry_pending_cmd() → 重试暂存的 APP→LINK 命令
├─ retry_reset() → RESETTING 状态 100ms 重发 DATA_RESET
└─ app_tag_poll_schedule() → IDLE + 1s 间隔 → MAC 表取下一个 Anchor → CMD_SEND_DATA_CFG

Anchor APP 主循环 (每 1ms)
│
├─ drain_link_events()
│   ├─ DATA_CFG → anchor_session_init() → 预打包数据
│   └─ DATA_CTRL → anchor_handle_ctrl()
│       ├─ GET_INFO → anchor_prepare_meta_slot() → CMD_SEND_FRAG(meta)
│       ├─ PULL → anchor_prepare_frag_slot(N) → CMD_SEND_FRAG(frag N)
│       ├─ NACK → anchor_prepare_frag_slot(N) → CMD_SEND_FRAG(重发)
│       └─ DONE → anchor_prepare_ack_slot() → CMD_SEND_ACK → state=IDLE
│
└─ 超时检查: 5s 无请求 → 释放 pending slots → state=IDLE
```
