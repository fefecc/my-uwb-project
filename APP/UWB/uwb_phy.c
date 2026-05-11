/**
 * @file uwb_phy.c
 * @brief PHY 层 V3 重构 - 步进式状态机 + 零拷贝 + 快速应答
 *
 * 状态机: IDLE/TX/RX_SLOT × PREPARE/WAIT/FINISH
 * LISTEN 是函数调用，不是状态。
 */

#include "uwb_phy.h"
#include <string.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "uwb_buffers.h"
#include "uwb_slots.h"
#include "uwb_protocol.h"
#include "uwb_timestamp.h"
#include "../service/log_service.h"
#include "../service/time_service.h"
#include "main.h"

/* ---- 状态/中断 清除掩码 ---- */
#define PHY_STATUS_CLEAR_MASK                     \
    (SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | \
     SYS_STATUS_ALL_RX_ERR | SYS_STATUS_RXOVRR |  \
     SYS_STATUS_HPDWARN | SYS_STATUS_TXBERR |     \
     SYS_STATUS_SLP2INIT | SYS_STATUS_RFPLL_LL | SYS_STATUS_CLKPLL_LL)

#define PHY_INT_MASK                              \
    (DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | \
     DWT_INT_RFCE | DWT_INT_RPHE | DWT_INT_RFSL | \
     DWT_INT_RXOVRR | DWT_INT_RXPTO | DWT_INT_SFDT | DWT_INT_ARFE)

/* ---- 快速应答参数 ---- */
#define PHY_DISC_REPLY_RX_AFTER false /* ACK 发完后不等 RX */

/* ---- 通知位 ---- */
#define PHY_NOTIFY_IRQ (1UL << 0)
#define PHY_NOTIFY_CMD (1UL << 1)

/* ---- 上下文 ---- */
typedef struct {
    uint16_t pan_id;
    uint16_t short_addr;
    AppDeviceRole role;

    uwb_phy_state_t state;
    uwb_phy_step_t step;

    /* TX 参数 (由 CMD 或快速应答填充) */
    int8_t tx_slot;
    uint64_t tx_time; /* 0 = 立即 */
    bool pending_rx;
    uint16_t pending_rx_timeout_us;
    uint8_t pending_rx_count;

    /* 快速应答内部 TX 缓冲 (不占 slot) */
    uint8_t tx_buf[UWB_STACK_MAX_FRAME_LEN];
    uint16_t tx_buf_len;
    bool fast_reply_active;

    /* RX slot 状态 */
    int8_t rx_slot;
    uint8_t rx_done_count;
    uint8_t rx_total_count;
    bool rx_got_frame;
    uint8_t rx_frame_count; /* 本窗口收到的有效帧总数 */

    /* 窗口号 */
    uint16_t window_id;

    /* 错误计数 */
    uint32_t hw_error_count;
    uint32_t rx_err_cnt; /* 连续 RX 错误计数, 成功后清零 */

    /* 帧协议配置 (Anchor 快速应答用) */
    UwbStackConfig stack_cfg;
} phy_context_t;

static phy_context_t g_phy;
static TaskHandle_t g_phy_task;

/* ================================================================
 *  DW1000 工具函数
 * ================================================================ */

uint64_t UwbPhy_UsToDwTime(uint32_t us)
{
    /* 499200 kHz × 128 = 63,897,600 kHz; ÷1000 → counts/µs = 63,897.6 */
    return (uint64_t)us * 499200ULL * 128ULL / 1000ULL;
}

uint64_t UwbPhy_ReadRxTimestamp(void)
{
    uwb_timestamp_t ts;
    memset(&ts, 0, sizeof(ts));
    dwt_readrxtimestamp(ts.bytes);
    return uwb_timestamp_to_u64(&ts);
}

uint64_t UwbPhy_ReadTxTimestamp(void)
{
    uwb_timestamp_t ts;
    memset(&ts, 0, sizeof(ts));
    dwt_readtxtimestamp(ts.bytes);
    return uwb_timestamp_to_u64(&ts);
}

/* ================================================================
 *  进入监听 (不是状态机的一部分)
 * ================================================================ */

static void enter_listening(void)
{
    dwt_forcetrxoff();

    /* 清除残留 status, 确保 IRQ 引脚回到 LOW */
    {
        uint32_t residual = dwt_read32bitreg(SYS_STATUS_ID);
        uint32_t clr      = residual & PHY_STATUS_CLEAR_MASK;
        if (clr) dwt_write32bitreg(SYS_STATUS_ID, clr);
    }

    /* 清除 EXTI pending, 防止残留事件干扰 */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);

    dwt_setrxtimeout(0); /* 0 = 无超时, 持续监听 */
    dwt_rxenable(0);
    g_phy.state = UWB_PHY_ST_IDLE;
    g_phy.step  = UWB_PHY_STEP_PREPARE;
}

/**
 * @brief 所有 RX 槽结束, 发 RX_WINDOW_END 并回到监听
 */
static void finish_rx_window(void)
{
    phy_evt_t end = {.type       = PHY_EVT_RX_WINDOW_END,
                     .slot_index = -1,
                     .rx_seq     = g_phy.rx_frame_count};
    UwbBuffers_SendEvt(&end, 0);
    enter_listening();
}

/* ================================================================
 *  IRQ 处理
 * ================================================================ */

/**
 * @brief 构建 DISC_ACK 快速应答帧到内部 tx_buf
 * @return 帧长度, 0=失败
 */
static uint16_t build_fast_reply_ack(uint16_t dst_short, uint8_t seq)
{
    UwbProtocolFrame frame;
    UwbProtocol_InitFrame(&frame,
                          &g_phy.stack_cfg,
                          dst_short,
                          seq,
                          UWB_FUNC_DISCOVERY_RESP);
    frame.common.ext_header_len = 0;
    frame.common.payload_len    = 0;

    size_t tx_len = 0;
    if (!UwbProtocol_Encode(&frame, g_phy.tx_buf,
                            sizeof(g_phy.tx_buf), &tx_len)) {
        return 0;
    }
    return (uint16_t)tx_len;
}

static void irq_rx_ok(uint32_t status)
{
    /* PHY 层零日志原则: 不打逐帧日志, 由 LINK 汇总 */

    /* 成功收帧, 清除连续错误计数 */
    g_phy.rx_err_cnt = 0;

    /* 分配 slot 存收到的帧 */
    int8_t idx = UwbSlots_Alloc(UWB_SLOT_PHY_OWN);
    if (idx < 0) {
        app_log_warn("[PHY] RX_OK no slot");
        /* RX_SLOT 状态下: 跳 FINISH 让状态机正常回收 tx_slot */
        if (g_phy.state == UWB_PHY_ST_RX_SLOT) {
            g_phy.rx_got_frame = false;
            g_phy.step         = UWB_PHY_STEP_FINISH;
        } else {
            enter_listening();
        }
        return;
    }

    uwb_slot_t *s = UwbSlots_Get(idx);

    /* 读帧数据到 slot (唯一必要拷贝: DW1000 SPI → slot) */
    uint32_t frame_info = dwt_read32bitreg(RX_FINFO_ID);
    uint16_t frame_len  = (uint16_t)(frame_info & RX_FINFO_RXFL_MASK_1023);
    if (frame_len > UWB_SLOT_DATA_SIZE) frame_len = UWB_SLOT_DATA_SIZE;

    dwt_readrxdata(s->data, frame_len, 0);
    s->data_len  = frame_len;
    s->rx_ts     = UwbPhy_ReadRxTimestamp();
    s->window_id = g_phy.window_id;

    /* RX 质量 */
    memset(&s->quality, 0, sizeof(s->quality));
    s->quality.rx_pacc = (uint16_t)((frame_info & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT);

    /* 解析帧头用于地址过滤和快速应答 */
    UwbProtocolFrame frame;
    if (!UwbProtocol_Decode(&frame, s->data, s->data_len)) {
        UwbSlots_Free(idx);
        enter_listening();
        return;
    }

    s->frame_type = frame.common.func_code;
    s->src_short  = frame.mac.src16;

    /* 地址过滤 */
    if (frame.mac.pan_id != g_phy.pan_id ||
        (frame.mac.dst16 != g_phy.short_addr &&
         frame.mac.dst16 != UWB_STACK_BROADCAST_SHORT_ID)) {
        UwbSlots_Free(idx);
        enter_listening();
        return;
    }

    /* 记录 rx_slot */
    g_phy.rx_slot      = idx;
    g_phy.rx_got_frame = true;

    /* ---- Anchor 快速应答: 收到 DISC_REQ 时延迟发送 ACK ---- */
    if (g_phy.role == APP_ROLE_ANCHOR &&
        frame.common.func_code == (uint8_t)UWB_FUNC_DISCOVERY_REQ) {

        uint8_t assigned_slot = (g_phy.short_addr - ANCHOR_ADDR_BASE) % DISC_RX_SLOT_COUNT;

        g_phy.tx_buf_len = build_fast_reply_ack(frame.mac.src16, frame.mac.seq);
        if (g_phy.tx_buf_len > 0) {
            dwt_writetxdata(g_phy.tx_buf_len + 2U, g_phy.tx_buf, 0);
            dwt_writetxfctrl(g_phy.tx_buf_len + 2U, 0);

            /* DELAYED TX: 基于 RX 时间戳的精确槽对齐 */
            uint64_t rx_ts    = s->rx_ts;
            uint32_t delay_us = ANCHOR_REPLY_GUARD_US + assigned_slot * DISC_SLOT_WIDTH_US;
            uint64_t tx_time  = rx_ts + UwbPhy_UsToDwTime(delay_us);
            dwt_setdelayedtrxtime((uint32_t)(tx_time >> 8));
            int ret = dwt_starttx(DWT_START_TX_DELAYED);

            if (ret == 0) {
                g_phy.fast_reply_active = true;
                app_log_info("[PHY] TX_STARTED fast_reply slot=%u ",
                             (unsigned)assigned_slot);
                /* 先上报 RX 事件, 然后等 TX_DONE */
                phy_evt_t rx_evt = {.type       = PHY_EVT_RX_FRAME,
                                    .slot_index = g_phy.rx_slot};
                UwbBuffers_SendEvt(&rx_evt, 0);
                g_phy.rx_slot    = -1;
                g_phy.state      = UWB_PHY_ST_TX;
                g_phy.step       = UWB_PHY_STEP_WAIT;
                g_phy.tx_slot    = -1;
                g_phy.pending_rx = false;
                return;
            }
            app_log_warn("[PHY] TX_FAIL fast_reply slot=%u delay=%luus",
                         (unsigned)assigned_slot, (unsigned long)delay_us);
            g_phy.fast_reply_active = false;
        }
    }

    /* ---- IDLE 状态下收帧: 直接上报 + re-listen ---- */
    if (g_phy.state == UWB_PHY_ST_IDLE) {
        phy_evt_t evt = {.type       = PHY_EVT_RX_FRAME,
                         .slot_index = g_phy.rx_slot};
        UwbBuffers_SendEvt(&evt, 0);
        g_phy.rx_slot      = -1;
        g_phy.rx_got_frame = false;
        enter_listening();
        return;
    }

    /* RX_SLOT 状态下收帧: 跳 FINISH 由状态机处理 */
    g_phy.step = UWB_PHY_STEP_FINISH;
}

static void irq_tx_done(void)
{
    g_phy.step = UWB_PHY_STEP_FINISH;
}

static void irq_rx_timeout(void)
{
    g_phy.rx_got_frame = false;
    g_phy.step         = UWB_PHY_STEP_FINISH;
}

static void irq_rx_error(uint32_t status)
{
    g_phy.rx_err_cnt++;
    dwt_forcetrxoff();
    dwt_rxreset();
    g_phy.rx_got_frame = false;

    /* IDLE 状态下: 环境噪声触发的 RX 错误是正常现象,
     * 每 50 次才记录一条日志, 避免刷屏 */
    if (g_phy.state == UWB_PHY_ST_IDLE) {
        if (g_phy.rx_err_cnt <= 1 || (g_phy.rx_err_cnt % 50) == 0) {
            app_log_warn("[PHY] RX_ERR(idle) st=0x%08lX cnt=%lu",
                         (unsigned long)status,
                         (unsigned long)g_phy.rx_err_cnt);
        }
        enter_listening();
        return;
    }

    /* 活跃状态 (TX/RX_SLOT): 始终记录 */
    app_log_warn("[PHY] RX_ERR st=0x%08lX cnt=%lu",
                 (unsigned long)status,
                 (unsigned long)g_phy.rx_err_cnt);

    /* RX_SLOT 状态下: 跳 FINISH 由状态机处理 */
    g_phy.step = UWB_PHY_STEP_FINISH;
}

static void irq_tx_error(uint32_t status)
{
    app_log_warn("[PHY] TX_ERR st=0x%08lX", (unsigned long)status);
    dwt_forcetrxoff();

    /* 上报错误 */
    phy_evt_t evt = {.type = PHY_EVT_ERROR, .slot_index = g_phy.tx_slot};
    UwbBuffers_SendEvt(&evt, 0);

    if (g_phy.tx_slot >= 0) {
        g_phy.tx_slot = -1;
    }
    enter_listening();
}

/* 前向声明: dispatch_status 中 TXFRS+RXFCG 组合处理需要 */
static void run_state_machine(void);

/**
 * @brief 根据已读取的 status 分发处理 (线程中调用)
 */
static void dispatch_status(uint32_t st)
{
    if (st == 0) return;

    /* 当 TXFRS 和 RXFCG 同时置位 (TX 完成后 Anchor 快速回复):
     * 必须先完成 TX FINISH 流程 (上报 TX_DONE + 进入 RX_SLOT),
     * 然后再处理 RXFCG */
    if ((st & SYS_STATUS_TXFRS) && (st & SYS_STATUS_RXFCG)) {
        irq_tx_done();       /* step → FINISH */
        run_state_machine(); /* TX FINISH → RX_SLOT PREPARE */
        run_state_machine(); /* RX_SLOT PREPARE → WAIT */
        irq_rx_ok(st);       /* 处理已收到的帧 */
        return;
    }

    if (st & SYS_STATUS_TXFRS) {
        irq_tx_done();
    } else if (st & SYS_STATUS_RXFCG) {
        irq_rx_ok(st);
    } else if (st & SYS_STATUS_RXRFTO) {
        irq_rx_timeout();
    } else if (st & SYS_STATUS_ALL_RX_ERR) {
        irq_rx_error(st);
    } else if (st & SYS_STATUS_TXBERR) {
        irq_tx_error(st);
    }
}

/**
 * @brief 从 ISR 缓存中取出 status 并分发处理
 *
 * SPI 容错: 如果读取的 status 包含 reserved bit19 (DW1000 永远不会置位),
 * 说明 SPI 读到了垃圾数据。最多重试 3 次, 任何一次读到合法值则正常处理;
 * 3 次全部失败则恢复到监听模式并上报错误。
 */
static void handle_irq(void)
{
    uint32_t st;
    int retry;

    for (retry = 0; retry < 3; retry++) {
        st = dwt_read32bitreg(SYS_STATUS_ID);
        if (!(st & SYS_STATUS_reserved)) {
            break; /* 合法值, 跳出 */
        }
    }

    if (retry > 0) {
        app_log_warn("[PHY] IRQ status retry=%d st=0x%08lX",
                     retry, (unsigned long)st);
    }

    /* 3 次全部读到垃圾: SPI 链路故障, 恢复监听 + 上报错误 */
    if (st & SYS_STATUS_reserved) {
        app_log_warn("[PHY] IRQ SPI_FAIL st=0x%08lX, recover",
                     (unsigned long)st);
        g_phy.hw_error_count++;
        dwt_forcetrxoff();
        if (g_phy.tx_slot >= 0) {
            UwbSlots_Free(g_phy.tx_slot);
            g_phy.tx_slot = -1;
        }
        if (g_phy.rx_slot >= 0) {
            UwbSlots_Free(g_phy.rx_slot);
            g_phy.rx_slot = -1;
        }
        phy_evt_t evt = {.type = PHY_EVT_ERROR, .slot_index = -1};
        UwbBuffers_SendEvt(&evt, 0);
        enter_listening();
        return;
    }

    /* 合法 status: 正常清除并处理 */
    uint32_t cl = st & PHY_STATUS_CLEAR_MASK;
    if (cl) dwt_write32bitreg(SYS_STATUS_ID, cl);

    /* 清除 EXTI pending, 防止 status 清除后残留的上升沿 */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);

    dispatch_status(st);
}

/* ================================================================
 *  CMD 处理
 * ================================================================ */

static void process_cmd(void)
{
    phy_cmd_t cmd;
    while (UwbBuffers_RecvCmd(&cmd, 0)) {
        switch (cmd.type) {
            case PHY_CMD_TX_FRAME:
                app_log_info("[PHY] CMD_TX slot=%d win=%u",
                             (int)cmd.slot_index, g_phy.window_id);
                dwt_forcetrxoff(); /* 停止正在进行的 RX */
                g_phy.tx_slot               = cmd.slot_index;
                g_phy.tx_time               = cmd.tx_time;
                g_phy.pending_rx            = cmd.has_pending_rx;
                g_phy.pending_rx_timeout_us = cmd.rx_timeout_us;
                g_phy.pending_rx_count      = cmd.rx_slot_count;
                g_phy.fast_reply_active     = false;
                g_phy.state                 = UWB_PHY_ST_TX;
                g_phy.step                  = UWB_PHY_STEP_PREPARE;
                break;

            case PHY_CMD_RESET:
                app_log_info("[PHY] CMD_RESET");
                dwt_forcetrxoff();
                enter_listening();
                break;

            case PHY_CMD_ENTER_LISTEN:
                enter_listening();
                break;
        }
    }
}

/* ================================================================
 *  状态机
 * ================================================================ */

static void run_state_machine(void)
{
    switch (g_phy.state) {

        /* ---- TX ---- */
        case UWB_PHY_ST_TX:
            switch (g_phy.step) {
                case UWB_PHY_STEP_PREPARE: {
                    uwb_slot_t *s = UwbSlots_Get(g_phy.tx_slot);
                    if (s == NULL) {
                        app_log_warn("[PHY] TX slot NULL");
                        enter_listening();
                        break;
                    }

                    /* 停止收发 + 清除残留 status, 确保 IRQ 引脚回到 LOW
                     * 防止 TXFRS 因引脚已高而无法产生上升沿 */
                    dwt_forcetrxoff();
                    {
                        uint32_t residual = dwt_read32bitreg(SYS_STATUS_ID);
                        uint32_t clr      = residual & PHY_STATUS_CLEAR_MASK;
                        if (clr) dwt_write32bitreg(SYS_STATUS_ID, clr);
                    }
                    /* 清除 EXTI pending */
                    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);

                    /* slot→SPI 拷贝 */
                    dwt_writetxdata(s->data_len + 2U, s->data, 0);
                    dwt_writetxfctrl(s->data_len + 2U, 0);

                    int ret;
                    if (g_phy.tx_time != 0) {
                        dwt_setdelayedtrxtime((uint32_t)(g_phy.tx_time >> 8));
                        ret = dwt_starttx(DWT_START_TX_DELAYED);
                        if (ret != 0) {
                            app_log_warn("[PHY] TX_DELAY_FAIL");
                            UwbSlots_Free(g_phy.tx_slot);
                            g_phy.tx_slot = -1;
                            phy_evt_t evt = {.type = PHY_EVT_ERROR, .slot_index = -1};
                            UwbBuffers_SendEvt(&evt, 0);
                            enter_listening();
                            break;
                        }
                    } else {
                        ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
                        if (ret != 0) {
                            app_log_warn("[PHY] TX_IMM_FAIL");
                            UwbSlots_Free(g_phy.tx_slot);
                            g_phy.tx_slot = -1;
                            phy_evt_t evt = {.type = PHY_EVT_ERROR, .slot_index = -1};
                            UwbBuffers_SendEvt(&evt, 0);
                            enter_listening();
                            break;
                        }
                    }
                    app_log_info("[PHY] TX_STARTED imm=%u", g_phy.tx_time == 0 ? 1U : 0U);
                    g_phy.step = UWB_PHY_STEP_WAIT;
                    break;
                }

                case UWB_PHY_STEP_WAIT:
                    break; /* 等 irq_tx_done → step=FINISH */

                case UWB_PHY_STEP_FINISH: {
                    /* 快速应答完成: 只需读 TX 时间戳并 re-listen */
                    if (g_phy.fast_reply_active) {
                        uint64_t tx_ts = UwbPhy_ReadTxTimestamp();
                        app_log_info("[LINK] fast_reply=1 tx=0x%02lX%08lX",
                                     (uint32_t)(tx_ts >> 32),
                                     (uint32_t)(tx_ts & 0xFFFFFFFF));
                        g_phy.fast_reply_active = false;
                        g_phy.tx_slot           = -1;
                        enter_listening();
                        break;
                    }

                    /* 填 TX 时间戳到 slot */
                    uwb_slot_t *s = UwbSlots_Get(g_phy.tx_slot);
                    if (s != NULL) {
                        s->tx_ts = UwbPhy_ReadTxTimestamp();
                    }

                    /* PHY 层直接回收 TX slot */
                    if (g_phy.tx_slot >= 0) {
                        UwbSlots_Free(g_phy.tx_slot);
                    }

                    /* 上报 TX_DONE (slot 已回收, index=-1) */
                    phy_evt_t evt = {.type = PHY_EVT_TX_DONE, .slot_index = -1};
                    UwbBuffers_SendEvt(&evt, 0);

                    if (g_phy.pending_rx) {
                        /* 进入 RX_SLOT 等待应答 */
                        g_phy.tx_slot        = -1;
                        g_phy.rx_done_count  = 0;
                        g_phy.rx_total_count = g_phy.pending_rx_count;
                        g_phy.rx_got_frame   = false;
                        g_phy.rx_slot        = -1;
                        g_phy.rx_frame_count = 0; /* ★ 新增 */
                        g_phy.state          = UWB_PHY_ST_RX_SLOT;
                        g_phy.step           = UWB_PHY_STEP_PREPARE;
                    } else {
                        g_phy.tx_slot = -1;
                        enter_listening();
                    }
                    break;
                }
            }
            break;

        /* ---- RX_SLOT ---- */
        case UWB_PHY_ST_RX_SLOT:
            switch (g_phy.step) {
                case UWB_PHY_STEP_PREPARE: {
                    /* 清除残留 status, 确保 IRQ 引脚回到 LOW */
                    uint32_t residual = dwt_read32bitreg(SYS_STATUS_ID);
                    uint32_t clr      = residual & PHY_STATUS_CLEAR_MASK;
                    if (clr) dwt_write32bitreg(SYS_STATUS_ID, clr);
                    /* 清除 EXTI pending */
                    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);

                    /* 第一槽用 3ms (容纳 GUARD 处理开销), 后续槽用标准 2ms */
                    uint16_t slot_timeout = (g_phy.rx_done_count == 0)
                                                ? DISC_FIRST_SLOT_TIMEOUT_US
                                                : g_phy.pending_rx_timeout_us;
                    dwt_setrxtimeout(slot_timeout);
                    if (dwt_rxenable(0) != 0) {
                        /* RX 启用失败: 当前槽标记为空, 上报 */
                        phy_evt_t evt = {.type       = PHY_EVT_RX_SLOT_DONE,
                                         .slot_index = -1,
                                         .rx_seq     = g_phy.rx_done_count};
                        UwbBuffers_SendEvt(&evt, 0);
                        g_phy.rx_done_count++;
                        if (g_phy.rx_done_count < g_phy.rx_total_count) {
                            g_phy.step = UWB_PHY_STEP_PREPARE;
                        } else {
                            finish_rx_window();
                        }
                        break;
                    }
                    g_phy.step = UWB_PHY_STEP_WAIT;
                    break;
                }

                case UWB_PHY_STEP_WAIT:
                    break; /* 等 irq_rx_ok/timeout → step=FINISH */

                case UWB_PHY_STEP_FINISH: {
                    uint8_t seq = g_phy.rx_done_count;
                    bool got    = (g_phy.rx_got_frame && g_phy.rx_slot >= 0);

                    if (got) {
                        /* 有帧: 上报 slot_index + 序号 */
                        phy_evt_t evt = {.type       = PHY_EVT_RX_SLOT_DONE,
                                         .slot_index = g_phy.rx_slot,
                                         .rx_seq     = seq};
                        UwbBuffers_SendEvt(&evt, 0);
                        g_phy.rx_slot = -1;
                        g_phy.rx_frame_count++;
                    } else {
                        /* 空/超时/错误: 上报 -1 + 序号, 不分配物理 slot */
                        phy_evt_t evt = {.type       = PHY_EVT_RX_SLOT_DONE,
                                         .slot_index = -1,
                                         .rx_seq     = seq};
                        UwbBuffers_SendEvt(&evt, 0);
                    }

                    g_phy.rx_done_count++;
                    g_phy.rx_got_frame = false;

                    if (g_phy.rx_done_count < g_phy.rx_total_count) {
                        g_phy.step = UWB_PHY_STEP_PREPARE; /* 下一个 slot */
                    } else {
                        finish_rx_window();
                    }
                    break;
                }
            }
            break;

        case UWB_PHY_ST_IDLE:
            break;
    }
}

/* ================================================================
 *  PHY 线程
 * ================================================================ */

bool UwbPhy_InitWithConfig(uint16_t pan_id, uint16_t short_addr,
                           AppDeviceRole role,
                           const UwbStackConfig *stack_cfg)
{
    memset(&g_phy, 0, sizeof(g_phy));
    g_phy.pan_id     = pan_id;
    g_phy.short_addr = short_addr;
    g_phy.role       = role;
    g_phy.tx_slot    = -1;
    g_phy.rx_slot    = -1;
    g_phy.state      = UWB_PHY_ST_IDLE;
    g_phy.step       = UWB_PHY_STEP_PREPARE;
    if (stack_cfg != NULL) {
        g_phy.stack_cfg = *stack_cfg;
    }
    return true;
}

bool UwbPhy_Init(uint16_t pan_id, uint16_t short_addr)
{
    return UwbPhy_InitWithConfig(pan_id, short_addr, APP_ROLE_TAG, NULL);
}

void UwbPhy_NotifyIrqFromISR(void)
{
    if (g_phy_task == NULL) return;
    BaseType_t higher = pdFALSE;
    xTaskNotifyFromISR(g_phy_task, PHY_NOTIFY_IRQ, eSetBits, &higher);
    portYIELD_FROM_ISR(higher);
}

void UwbPhy_NotifyCmd(void)
{
    if (g_phy_task == NULL) return;
    (void)xTaskNotify(g_phy_task, PHY_NOTIFY_CMD, eSetBits);
}

bool UwbPhy_StartThread(const osThreadAttr_t *attr)
{
    if (g_phy_task != NULL) return true;
    g_phy_task = osThreadNew(UwbPhy_Task, NULL, attr);
    return g_phy_task != NULL;
}

void UwbPhy_Task(void *argument)
{
    (void)argument;

    g_phy_task = xTaskGetCurrentTaskHandle();
    app_log_info("[PHY] START short=0x%04X role=%u",
                 g_phy.short_addr, (unsigned)g_phy.role);

    enter_listening();

    for (;;) {
        uint32_t notify = 0;

        /* 根据状态选择等待时间:
         *   - PREPARE 待执行: 不阻塞, 立即执行
         *   - 活跃状态 (TX/RX_SLOT) WAIT: 5ms 看门狗
         *   - IDLE 监听状态: 1s 保护超时
         */
        TickType_t wait_ticks;
        if (g_phy.state != UWB_PHY_ST_IDLE && g_phy.step == UWB_PHY_STEP_PREPARE) {
            wait_ticks = 0;
        } else if (g_phy.state != UWB_PHY_ST_IDLE) {
            wait_ticks = pdMS_TO_TICKS(UWB_PHY_WATCHDOG_MS);
        } else {
            wait_ticks = pdMS_TO_TICKS(UWB_PHY_IDLE_GUARD_MS);
        }

        BaseType_t got = xTaskNotifyWait(
            0, UINT32_MAX, &notify, wait_ticks);

        /* ---- 软件轮询兜底: 补偿 EXTI 上升沿丢失 ---- */
        if (g_phy.state != UWB_PHY_ST_IDLE) {
            uint32_t poll_st = dwt_read32bitreg(SYS_STATUS_ID);

            /* reserved bit19 检测: DW1000 永远不会置位此位,
             * 如果读到说明 SPI 返回垃圾数据.
             * 注意: 不 continue, 让执行流落到看门狗处理 */
            if (poll_st & SYS_STATUS_reserved) {
                app_log_warn("[PHY] POLL SPI_GARBAGE st=0x%08lX",
                             (unsigned long)poll_st);
                /* 不处理垃圾 status, 也不 continue,
                 * 落到下面看门狗超时后自然恢复 */
            } else if (poll_st & (SYS_STATUS_RXFCG | SYS_STATUS_TXFRS |
                                  SYS_STATUS_RXRFTO | SYS_STATUS_ALL_RX_ERR |
                                  SYS_STATUS_TXBERR)) {
                /* 有可操作的 status 位, 清除并处理 */
                uint32_t cl = poll_st & PHY_STATUS_CLEAR_MASK;
                if (cl) dwt_write32bitreg(SYS_STATUS_ID, cl);
                __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
                if (!(notify & PHY_NOTIFY_IRQ)) {
                    app_log_warn("[PHY] POLL_CATCH st=0x%08lX",
                                 (unsigned long)poll_st);
                }
                dispatch_status(poll_st);
                run_state_machine();
                continue;
            }
        }

        /* ---- 看门狗: 活跃状态等 DW1000 中断超时 ---- */
        if (got != pdPASS && wait_ticks > 0 && g_phy.state != UWB_PHY_ST_IDLE) {
            /* 诊断: 读 DW1000 status + IRQ 引脚电平 */
            uint32_t diag_st  = dwt_read32bitreg(SYS_STATUS_ID);
            uint8_t pin_level = (uint8_t)HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
            app_log_warn("[PHY] WATCHDOG state=%u step=%u st=0x%08lX pin=%u err=%lu",
                         (unsigned)g_phy.state, (unsigned)g_phy.step,
                         (unsigned long)diag_st, (unsigned)pin_level,
                         (unsigned long)g_phy.hw_error_count);
            g_phy.hw_error_count++;
            dwt_forcetrxoff();
            /* 释放占用的 slot */
            if (g_phy.tx_slot >= 0) {
                UwbSlots_Free(g_phy.tx_slot);
                g_phy.tx_slot = -1;
            }
            if (g_phy.rx_slot >= 0) {
                UwbSlots_Free(g_phy.rx_slot);
                g_phy.rx_slot = -1;
            }
            /* 始终上报错误给 LINK 层, 无论是 TX 还是 RX 超时 */
            phy_evt_t evt = {.type = PHY_EVT_ERROR, .slot_index = -1};
            UwbBuffers_SendEvt(&evt, 0);
            enter_listening();
            continue;
        }

        /* ---- IDLE 保护: 1s 超时只做静默重新监听, 不计错误 ---- */
        if (got != pdPASS && g_phy.state == UWB_PHY_ST_IDLE) {
            /* 监听模式下没有活跃操作, 不需要报错
             * 只重新启动监听, 防止 DW1000 接收机卡死 */
            enter_listening();
            continue;
        }

        /* ---- IRQ ---- */
        if (notify & PHY_NOTIFY_IRQ) {
            handle_irq();
        }

        /* ---- CMD ---- */
        if (notify & PHY_NOTIFY_CMD) {
            process_cmd();
        }

        /* ---- 状态机步进 ---- */
        run_state_machine();
    }
}
