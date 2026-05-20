/**
 * @file uwb_link.c
 * @brief LINK 层 (plan-v4) - DISC/TWR 主链路 + APP 驱动 DATA 传输
 */

#include "uwb_link.h"

#include <string.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "uwb_phy.h"
#include "uwb_buffers.h"
#include "uwb_slots.h"
#include "uwb_protocol.h"
#include "uwb_timestamp.h"
#include "uwb_loss_test.h"
#include "../service/log_service.h"

/* LINK/PHY 调试日志会明显影响串口吞吐；默认只输出 START 和 WARN/ERROR。 */
#undef app_log_info
#define app_log_info(...) do { if (0) LogService_Write(APP_LOG_INFO, __VA_ARGS__); } while (0)

/* ---- 时序参数 ---- */
#define DISC_RX_SLOT_COUNT        4U
#define DISC_PIPELINE_MAX         1U
#define LINK_DISC_CMD_TIMEOUT_MS  100U

#define LINK_DATA_TIMEOUT_MS      50U
#define LINK_DATA_MAX_RETRY       3U
#define LINK_DATA_SLOT_MAX        4U

typedef enum {
    LINK_ERR_NONE = 0,
    LINK_ERR_DISC_TIMEOUT,
    LINK_ERR_DISC_PHY_ERROR,
} link_error_t;

typedef enum {
    LINK_DATA_IDLE = 0,
    LINK_DATA_WAIT_TX_DONE,
    LINK_DATA_WAIT_RX,
} link_data_state_t;

typedef struct {
    uint32_t disc_cmd_sent_ms;
} link_timeout_ctx_t;

typedef struct {
    UwbStackConfig cfg;
    uint16_t window_id;
    uint8_t  seq;
    int8_t   rx_slot_results[DISC_RX_SLOT_COUNT];
    int8_t   tx_slot_idx;
    int8_t   misc_tx_slot_idx;
} link_context_t;

typedef struct {
    bool     valid;
    int8_t   slot_index;
    uint16_t target_id;
    uint16_t session_id;
    uint8_t  frag_id;
    uint8_t  flags;
} link_data_slot_entry_t;

typedef struct {
    link_data_state_t state;
    bool pending_cmd_valid;
    UwbLinkCmd pending_cmd;
    UwbLinkCmd active_cmd;
    int8_t tx_slot;
    uint8_t retry_count;
    uint32_t started_ms;
    bool rx_window_active;
    bool response_seen;
} link_data_context_t;

static link_context_t g_link;
static TaskHandle_t   g_link_task;

static uint8_t  g_disc_outstanding;
static uint8_t  g_disc_count;
static link_timeout_ctx_t g_to;

static link_data_context_t g_data;
static link_data_slot_entry_t g_data_slots[LINK_DATA_SLOT_MAX];

/* ---- LINK → APP 事件队列 ---- */
#define LINK_APP_EVT_QUEUE_LEN  8
static StaticQueue_t g_app_evt_queue_ctrl;
static uint8_t g_app_evt_queue_buf[LINK_APP_EVT_QUEUE_LEN * sizeof(UwbLinkAppEvent)];
static QueueHandle_t g_app_evt_queue;

QueueHandle_t UwbLink_AppEventQueue(void)
{
    return g_app_evt_queue;
}

static uint8_t next_seq(void)
{
    return g_link.seq++;
}

static void post_app_event(const UwbLinkAppEvent *evt)
{
    if (evt == NULL || g_app_evt_queue == NULL) return;
    if (xQueueSend(g_app_evt_queue, evt, 0) != pdPASS) {
        app_log_warn("[LINK] APP_EVT_DROP type=%u", (unsigned)evt->type);
    }
}

/* ================================================================
 *  DISC 帧构建
 * ================================================================ */

static uint16_t build_disc_req(uint8_t *buf, size_t buf_size, uint8_t *seq_out)
{
    UwbProtocolFrame frame;
    uint8_t seq = next_seq();
    UwbProtocol_InitFrame(&frame,
                          &g_link.cfg,
                          UWB_STACK_BROADCAST_SHORT_ID,
                          seq,
                          UWB_FUNC_DISCOVERY_REQ);

    frame.common.ext_header_len = 10U;
    UwbProtocol_WriteLe32(&frame.ext_header[0], g_link.window_id);
    UwbProtocol_WriteLe16(&frame.ext_header[4], 1U);
    UwbProtocol_WriteLe16(&frame.ext_header[6], 5000U);
    UwbProtocol_WriteLe16(&frame.ext_header[8], 1U);

    size_t tx_len = 0;
    if (!UwbProtocol_Encode(&frame, buf, buf_size, &tx_len)) {
        return 0;
    }
    if (seq_out != NULL) {
        *seq_out = seq;
    }
    return (uint16_t)tx_len;
}

/* ================================================================
 *  DATA 帧构建
 * ================================================================ */

static uint16_t build_data_cfg_req(uint8_t *buf, size_t buf_size,
                                   uint16_t target_id, uint16_t session_id)
{
    UwbProtocolFrame frame;
    UwbProtocol_InitFrame(&frame, &g_link.cfg, target_id, next_seq(),
                          UWB_FUNC_APP_DATA_CFG);
    frame.common.ext_header_len = 2U;
    UwbProtocol_WriteLe16(&frame.ext_header[0], session_id);

    size_t tx_len = 0;
    if (!UwbProtocol_Encode(&frame, buf, buf_size, &tx_len)) return 0;
    return (uint16_t)tx_len;
}

static uint16_t build_data_ctrl(uint8_t *buf, size_t buf_size,
                                uint16_t target_id, uint16_t session_id,
                                uint8_t ctrl_type, uint8_t frag_id)
{
    UwbProtocolFrame frame;
    UwbProtocol_InitFrame(&frame, &g_link.cfg, target_id, next_seq(),
                          UWB_FUNC_APP_DATA_CTRL);
    frame.common.ext_header_len = 4U;
    UwbProtocol_WriteLe16(&frame.ext_header[0], session_id);
    frame.ext_header[2] = ctrl_type;
    frame.ext_header[3] = frag_id;

    size_t tx_len = 0;
    if (!UwbProtocol_Encode(&frame, buf, buf_size, &tx_len)) return 0;
    return (uint16_t)tx_len;
}

static uint16_t build_data_ctrl_resp(uint8_t *buf, size_t buf_size,
                                     uint16_t target_id, uint8_t resp_type,
                                     uint8_t extra)
{
    UwbProtocolFrame frame;
    UwbProtocol_InitFrame(&frame, &g_link.cfg, target_id, next_seq(),
                          UWB_FUNC_APP_DATA_CTRL_RESP);
    frame.common.ext_header_len = 2U;
    frame.ext_header[0] = resp_type;
    frame.ext_header[1] = extra;

    size_t tx_len = 0;
    if (!UwbProtocol_Encode(&frame, buf, buf_size, &tx_len)) return 0;
    return (uint16_t)tx_len;
}

static bool build_data_frag_in_slot(int8_t slot_index, uint16_t target_id,
                                    uint16_t session_id, uint8_t frag_id)
{
    uwb_slot_t *s = UwbSlots_Get(slot_index);
    if (s == NULL || s->data_len > UWB_PROTO_MAX_PAYLOAD_LEN) return false;

    uint8_t payload[UWB_PROTO_MAX_PAYLOAD_LEN];
    uint16_t payload_len = s->data_len;
    memcpy(payload, s->data, payload_len);

    UwbProtocolFrame frame;
    UwbProtocol_InitFrame(&frame, &g_link.cfg, target_id, next_seq(),
                          UWB_FUNC_APP_DATA_FRAG);
    frame.common.ext_header_len = 5U;
    UwbProtocol_WriteLe16(&frame.ext_header[0], session_id);
    frame.ext_header[2] = frag_id;
    frame.ext_header[3] = s->total_frags;
    frame.ext_header[4] = s->data_flags;
    frame.common.payload_len = payload_len;
    memcpy(frame.payload, payload, payload_len);

    size_t tx_len = 0;
    if (!UwbProtocol_Encode(&frame, s->data, sizeof(s->data), &tx_len)) {
        return false;
    }

    s->data_len    = (uint16_t)tx_len;
    s->frame_type  = (uint8_t)UWB_FUNC_APP_DATA_FRAG;
    s->src_short   = g_link.cfg.short_addr;
    s->session_id  = session_id;
    s->frag_id     = frag_id;
    return true;
}

/* ================================================================
 *  DATA slot 挂起表
 * ================================================================ */

static void link_data_slot_unregister(int8_t slot_index)
{
    for (uint8_t i = 0; i < LINK_DATA_SLOT_MAX; i++) {
        if (g_data_slots[i].valid && g_data_slots[i].slot_index == slot_index) {
            g_data_slots[i].valid = false;
        }
    }
}

static bool link_data_slot_register(int8_t slot_index, uint16_t target_id,
                                    uint16_t session_id, uint8_t frag_id,
                                    uint8_t flags)
{
    link_data_slot_unregister(slot_index);

    for (uint8_t i = 0; i < LINK_DATA_SLOT_MAX; i++) {
        if (g_data_slots[i].valid) {
            uwb_slot_t *s = UwbSlots_Get(g_data_slots[i].slot_index);
            if (s == NULL || s->owner == UWB_SLOT_FREE) {
                g_data_slots[i].valid = false;
            }
        }
    }

    for (uint8_t i = 0; i < LINK_DATA_SLOT_MAX; i++) {
        if (!g_data_slots[i].valid) {
            g_data_slots[i].valid      = true;
            g_data_slots[i].slot_index = slot_index;
            g_data_slots[i].target_id  = target_id;
            g_data_slots[i].session_id = session_id;
            g_data_slots[i].frag_id    = frag_id;
            g_data_slots[i].flags      = flags;
            return true;
        }
    }

    return false;
}

static link_data_slot_entry_t *link_data_slot_find(uint8_t frag_id)
{
    for (uint8_t i = 0; i < LINK_DATA_SLOT_MAX; i++) {
        if (g_data_slots[i].valid && g_data_slots[i].frag_id == frag_id) {
            uwb_slot_t *s = UwbSlots_Get(g_data_slots[i].slot_index);
            if (s != NULL && s->owner != UWB_SLOT_FREE) {
                return &g_data_slots[i];
            }
        }
    }
    return NULL;
}

static void link_data_slot_free_all(void)
{
    for (uint8_t i = 0; i < LINK_DATA_SLOT_MAX; i++) {
        if (g_data_slots[i].valid) {
            UwbSlots_Free(g_data_slots[i].slot_index);
            g_data_slots[i].valid = false;
        }
    }
}

static void phy_clear_pending(void)
{
    phy_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = PHY_CMD_LOAD_PENDING;
    cmd.slot_index = -1;
    if (UwbBuffers_SendCmd(&cmd, 0)) {
        UwbPhy_NotifyCmd();
    }
}

static void data_sm_reset(bool notify_app)
{
    uint16_t fail_src = g_data.active_cmd.target_id;
    uint16_t fail_session = g_data.active_cmd.session_id;

    app_log_warn("[LINK] DATA_RESET notify=%u dst=0x%04X sess=0x%04X tx_slot=%d",
                 notify_app ? 1U : 0U, fail_src, fail_session,
                 (int)g_data.tx_slot);

    link_data_slot_free_all();

    if (g_data.tx_slot >= 0) {
        UwbSlots_Free(g_data.tx_slot);
    }

    memset(&g_data, 0, sizeof(g_data));
    g_data.state = LINK_DATA_IDLE;
    g_data.tx_slot = -1;
    phy_clear_pending();

    if (notify_app) {
        UwbLinkAppEvent evt;
        memset(&evt, 0, sizeof(evt));
        evt.type = UWB_LINK_APP_EVT_DATA_FAIL;
        evt.data.data_ack.src_id = fail_src;
        evt.data.data_ack.session_id = fail_session;
        post_app_event(&evt);
    }
}

/* ================================================================
 *  DATA 交换状态
 * ================================================================ */

static void data_free_tx_slot(void)
{
    if (g_data.tx_slot >= 0) {
        UwbSlots_Free(g_data.tx_slot);
        g_data.tx_slot = -1;
    }
}

static void data_finish_response(void)
{
    app_log_info("[LINK] DATA_DONE type=%u dst=0x%04X sess=0x%04X frag=%u retry=%u",
                 (unsigned)g_data.active_cmd.type,
                 g_data.active_cmd.target_id,
                 g_data.active_cmd.session_id,
                 (unsigned)g_data.active_cmd.frag_id,
                 (unsigned)g_data.retry_count);
    data_free_tx_slot();
    g_data.state = LINK_DATA_IDLE;
    g_data.pending_cmd_valid = false;
    g_data.response_seen = true;
}

static bool data_send_current_slot(void)
{
    if (g_data.tx_slot < 0) {
        app_log_warn("[LINK] DATA_SEND_NO_SLOT");
        return false;
    }

    phy_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type           = PHY_CMD_TX_FRAME;
    cmd.slot_index     = g_data.tx_slot;
    cmd.tx_time        = 0;
    cmd.has_pending_rx = true;
    cmd.rx_timeout_us  = UWB_PHY_RX_SLOT_TIMEOUT_US;
    cmd.rx_slot_count  = 1U;

    if (!UwbBuffers_SendCmd(&cmd, 0)) {
        app_log_warn("[LINK] DATA_PHY_CMD_FULL slot=%d", (int)g_data.tx_slot);
        return false;
    }

    UwbPhy_NotifyCmd();
    app_log_info("[LINK] DATA_PHY_TX slot=%d type=%u dst=0x%04X sess=0x%04X frag=%u retry=%u",
                 (int)g_data.tx_slot,
                 (unsigned)g_data.active_cmd.type,
                 g_data.active_cmd.target_id,
                 g_data.active_cmd.session_id,
                 (unsigned)g_data.active_cmd.frag_id,
                 (unsigned)g_data.retry_count);
    g_data.state = LINK_DATA_WAIT_TX_DONE;
    g_data.started_ms = HAL_GetTick();
    g_data.rx_window_active = false;
    g_data.response_seen = false;
    return true;
}

static void data_retry_or_fail(void)
{
    g_data.rx_window_active = false;

    uwb_slot_t *txs = UwbSlots_Get(g_data.tx_slot);
    if (txs == NULL || txs->owner == UWB_SLOT_FREE) {
        app_log_warn("[LINK] DATA_RETRY_SLOT_GONE slot=%d", (int)g_data.tx_slot);
        g_data.retry_count = LINK_DATA_MAX_RETRY;
    }

    if (g_data.retry_count >= LINK_DATA_MAX_RETRY) {
        app_log_warn("[LINK] DATA_FAIL dst=0x%04X sess=0x%04X frag=%u retry=%u",
                     g_data.active_cmd.target_id,
                     g_data.active_cmd.session_id,
                     (unsigned)g_data.active_cmd.frag_id,
                     (unsigned)g_data.retry_count);
        UwbLinkAppEvent evt;
        memset(&evt, 0, sizeof(evt));
        evt.type = UWB_LINK_APP_EVT_DATA_FAIL;
        evt.data.data_ack.src_id = g_data.active_cmd.target_id;
        evt.data.data_ack.session_id = g_data.active_cmd.session_id;
        post_app_event(&evt);
        data_free_tx_slot();
        g_data.state = LINK_DATA_IDLE;
        g_data.pending_cmd_valid = false;
        return;
    }

    g_data.retry_count++;
    app_log_warn("[LINK] DATA_RETRY dst=0x%04X sess=0x%04X frag=%u retry=%u",
                 g_data.active_cmd.target_id,
                 g_data.active_cmd.session_id,
                 (unsigned)g_data.active_cmd.frag_id,
                 (unsigned)g_data.retry_count);
    if (!data_send_current_slot()) {
        g_data.state = LINK_DATA_WAIT_TX_DONE;
        g_data.started_ms = HAL_GetTick();
    }
}

static bool start_tag_data_cmd(const UwbLinkCmd *cmd)
{
    if (cmd == NULL) return false;
    if (cmd->type != LINK_CMD_SEND_CFG_REQ &&
        cmd->type != LINK_CMD_SEND_CTRL) {
        app_log_warn("[LINK] DATA_TAG_CMD_INVALID type=%u", (unsigned)cmd->type);
        return false;
    }

    int8_t idx = UwbSlots_Alloc(UWB_SLOT_LINK_OWN);
    if (idx < 0) {
        app_log_warn("[LINK] DATA_TX_NO_SLOT type=%u", (unsigned)cmd->type);
        return false;
    }

    uwb_slot_t *s = UwbSlots_Get(idx);
    if (s == NULL) {
        UwbSlots_Free(idx);
        app_log_warn("[LINK] DATA_TX_SLOT_NULL slot=%d", (int)idx);
        return false;
    }

    if (cmd->type == LINK_CMD_SEND_CFG_REQ) {
        s->data_len = build_data_cfg_req(s->data, sizeof(s->data),
                                         cmd->target_id, cmd->session_id);
        s->frame_type = (uint8_t)UWB_FUNC_APP_DATA_CFG;
    } else {
        s->data_len = build_data_ctrl(s->data, sizeof(s->data),
                                      cmd->target_id, cmd->session_id,
                                      cmd->ctrl_type, cmd->frag_id);
        s->frame_type = (uint8_t)UWB_FUNC_APP_DATA_CTRL;
    }

    if (s->data_len == 0) {
        app_log_warn("[LINK] DATA_BUILD_FAIL type=%u dst=0x%04X sess=0x%04X",
                     (unsigned)cmd->type, cmd->target_id, cmd->session_id);
        UwbSlots_Free(idx);
        return false;
    }

    s->session_id = cmd->session_id;
    s->frag_id = cmd->frag_id;

    memset(&g_data.active_cmd, 0, sizeof(g_data.active_cmd));
    g_data.active_cmd = *cmd;
    g_data.tx_slot = idx;
    g_data.retry_count = 0;
    g_disc_count = 0;

    if (!data_send_current_slot()) {
        UwbSlots_Free(idx);
        g_data.tx_slot = -1;
        return false;
    }

    app_log_info("[LINK] DATA_TX type=%u dst=0x%04X sess=0x%04X frag=%u",
                 (unsigned)cmd->type, cmd->target_id,
                 cmd->session_id, (unsigned)cmd->frag_id);
    return true;
}

/* ================================================================
 *  Anchor APP 命令
 * ================================================================ */

static bool load_pending_frag(int8_t slot_index, uint16_t target_id,
                              uint16_t session_id, uint8_t frag_id)
{
    uwb_slot_t *s = UwbSlots_Get(slot_index);
    if (s == NULL) {
        app_log_warn("[LINK] DATA_PENDING_SLOT_NULL slot=%d frag=%u",
                     (int)slot_index, (unsigned)frag_id);
        return false;
    }

    if (!build_data_frag_in_slot(slot_index, target_id, session_id, frag_id)) {
        app_log_warn("[LINK] DATA_FRAG_BUILD_FAIL slot=%d sess=0x%04X frag=%u len=%u",
                     (int)slot_index, session_id, (unsigned)frag_id, s->data_len);
        UwbSlots_Free(slot_index);
        return false;
    }

    if (!link_data_slot_register(slot_index, target_id, session_id,
                                 frag_id, s->data_flags)) {
        app_log_warn("[LINK] DATA_PENDING_TABLE_FULL slot=%d frag=%u",
                     (int)slot_index, (unsigned)frag_id);
        UwbSlots_Free(slot_index);
        return false;
    }

    phy_cmd_t pcmd;
    memset(&pcmd, 0, sizeof(pcmd));
    pcmd.type = PHY_CMD_LOAD_PENDING;
    pcmd.slot_index = slot_index;
    pcmd.session_id = session_id;
    pcmd.frag_id = frag_id;

    if (!UwbBuffers_SendCmd(&pcmd, 0)) {
        app_log_warn("[LINK] DATA_LOAD_PENDING_CMD_FULL slot=%d frag=%u",
                     (int)slot_index, (unsigned)frag_id);
        link_data_slot_unregister(slot_index);
        UwbSlots_Free(slot_index);
        return false;
    }

    UwbPhy_NotifyCmd();
    app_log_info("[LINK] DATA_PENDING dst=0x%04X sess=0x%04X frag=%u slot=%d",
                 target_id, session_id, (unsigned)frag_id, (int)slot_index);
    return true;
}

static bool send_anchor_ack(uint16_t target_id, uint8_t resp_type)
{
    int8_t idx = UwbSlots_Alloc(UWB_SLOT_LINK_OWN);
    if (idx < 0) return false;

    uwb_slot_t *s = UwbSlots_Get(idx);
    if (s == NULL) {
        UwbSlots_Free(idx);
        return false;
    }

    s->data_len = build_data_ctrl_resp(s->data, sizeof(s->data),
                                       target_id, resp_type, 0);
    if (s->data_len == 0) {
        UwbSlots_Free(idx);
        return false;
    }
    s->frame_type = (uint8_t)UWB_FUNC_APP_DATA_CTRL_RESP;

    phy_cmd_t pcmd;
    memset(&pcmd, 0, sizeof(pcmd));
    pcmd.type = PHY_CMD_TX_FRAME;
    pcmd.slot_index = idx;
    pcmd.has_pending_rx = false;

    if (!UwbBuffers_SendCmd(&pcmd, 0)) {
        UwbSlots_Free(idx);
        return false;
    }

    g_link.misc_tx_slot_idx = idx;
    UwbPhy_NotifyCmd();
    return true;
}

static void link_dispatch_app_cmd(const UwbLinkCmd *cmd)
{
    if (cmd == NULL) return;

    app_log_info("[LINK] APP_CMD type=%u dst=0x%04X sess=0x%04X ctrl=%u frag=%u slot=%d",
                 (unsigned)cmd->type, cmd->target_id, cmd->session_id,
                 (unsigned)cmd->ctrl_type, (unsigned)cmd->frag_id,
                 (int)cmd->slot_index);

    if (cmd->type == LINK_CMD_SESSION_RESET) {
        data_sm_reset(false);
        return;
    }

    if (g_link.cfg.role == APP_ROLE_TAG) {
        if (cmd->type == LINK_CMD_SEND_CFG_REQ ||
            cmd->type == LINK_CMD_SEND_CTRL) {
            g_data.pending_cmd = *cmd;
            g_data.pending_cmd_valid = true;
            app_log_info("[LINK] DATA_PENDING_CMD type=%u sess=0x%04X frag=%u",
                         (unsigned)cmd->type, cmd->session_id,
                         (unsigned)cmd->frag_id);
        }
        return;
    }

    if (g_link.cfg.role == APP_ROLE_ANCHOR) {
        switch (cmd->type) {
            case LINK_CMD_SEND_FRAG:
                (void)load_pending_frag(cmd->slot_index, cmd->target_id,
                                         cmd->session_id, cmd->frag_id);
                break;

            case LINK_CMD_SEND_ACK:
                (void)send_anchor_ack(cmd->target_id, cmd->resp_type);
                break;

            default:
                break;
        }
    }
}

static void drain_app_cmds(void)
{
    UwbLinkCmd cmd;

    if (g_link.cfg.role == APP_ROLE_TAG) {
        if (!g_data.pending_cmd_valid && g_data.state == LINK_DATA_IDLE &&
            UwbLinkCmd_Recv(&cmd, 0)) {
            link_dispatch_app_cmd(&cmd);
        }
        return;
    }

    while (UwbLinkCmd_Recv(&cmd, 0)) {
        link_dispatch_app_cmd(&cmd);
    }
}

/* ================================================================
 *  DISC 错误处理
 * ================================================================ */

static void disc_sm_on_error(link_error_t err)
{
    app_log_warn("[LINK] disc error=%u outstanding=%u",
                 (unsigned)err, (unsigned)g_disc_outstanding);

    if (g_link.tx_slot_idx >= 0) {
        UwbSlots_Free(g_link.tx_slot_idx);
        g_link.tx_slot_idx = -1;
    }

    g_disc_outstanding = 0;
    g_to.disc_cmd_sent_ms = 0;

    phy_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = PHY_CMD_RESET;
    if (UwbBuffers_SendCmd(&cmd, 0)) {
        UwbPhy_NotifyCmd();
    }

    phy_evt_t evt;
    while (UwbBuffers_RecvEvt(&evt, 0)) {
        if (evt.slot_index >= 0) {
            UwbSlots_Free(evt.slot_index);
        }
    }
}

/* ================================================================
 *  PHY 事件解析
 * ================================================================ */

static void handle_disc_rx_slot_done(const phy_evt_t *evt)
{
    if (evt->rx_seq < DISC_RX_SLOT_COUNT) {
        g_link.rx_slot_results[evt->rx_seq] = evt->slot_index;
    }
    if (evt->slot_index < 0) return;

    uwb_slot_t *s = UwbSlots_Get(evt->slot_index);
    if (s != NULL && s->frame_type == (uint8_t)UWB_FUNC_DISCOVERY_RESP) {
        UwbProtocolFrame frame;
        if (UwbProtocol_Decode(&frame, s->data, s->data_len) &&
            frame.common.ext_header_len >= 10) {

            uint64_t anchor_rx_ts = UwbProtocol_ReadLe32(&frame.ext_header[0])
                                  | ((uint64_t)frame.ext_header[4] << 32);
            uint64_t anchor_tx_ts = UwbProtocol_ReadLe32(&frame.ext_header[5])
                                  | ((uint64_t)frame.ext_header[9] << 32);

            UwbTwrExchange twr;
            memset(&twr, 0, sizeof(twr));
            twr.anchor_id    = s->src_short;
            twr.exchange_seq = frame.mac.seq;
            twr.response_slot_id = evt->rx_seq;
            {
                uwb_slot_t *tx_slot = UwbSlots_Get(g_link.tx_slot_idx);
                twr.tag_tx_ts = (tx_slot != NULL) ? tx_slot->tx_ts : 0;
                twr.tag_tx_local_tick_20k =
                    (tx_slot != NULL) ? tx_slot->tx_local_tick_20k : 0;
            }
            twr.anchor_rx_ts = anchor_rx_ts;
            twr.anchor_tx_ts = anchor_tx_ts;
            twr.tag_rx_ts    = s->rx_ts;
            twr.tag_rx_local_tick_20k = s->rx_local_tick_20k;
            twr.quality      = s->quality;

            (void)UwbLossTest_PostRangeRx((uint8_t)frame.mac.seq,
                                          twr.anchor_id,
                                          (uint8_t)twr.response_slot_id);

            UwbLinkAppEvent app_evt;
            memset(&app_evt, 0, sizeof(app_evt));
            app_evt.type     = UWB_LINK_APP_EVT_TWR_EXCHANGE;
            app_evt.data.twr = twr;
            post_app_event(&app_evt);

            app_log_info("[LINK] TWR anchor=0x%04X t1=0x%02lX%08lX t2=0x%02lX%08lX t3=0x%02lX%08lX t4=0x%02lX%08lX",
                         twr.anchor_id,
                         (uint32_t)(twr.tag_tx_ts >> 32), (uint32_t)(twr.tag_tx_ts & 0xFFFFFFFF),
                         (uint32_t)(twr.anchor_rx_ts >> 32), (uint32_t)(twr.anchor_rx_ts & 0xFFFFFFFF),
                         (uint32_t)(twr.anchor_tx_ts >> 32), (uint32_t)(twr.anchor_tx_ts & 0xFFFFFFFF),
                         (uint32_t)(twr.tag_rx_ts >> 32), (uint32_t)(twr.tag_rx_ts & 0xFFFFFFFF));
        }
    }

    UwbSlots_Free(evt->slot_index);
}

static void post_data_resp_event(UwbLinkAppEventType type, uint8_t resp_type,
                                 uint16_t src_id)
{
    UwbLinkAppEvent evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = type;
    evt.data.data_ack.src_id = src_id;
    evt.data.data_ack.session_id = g_data.active_cmd.session_id;
    evt.data.data_ack.resp_type = resp_type;
    post_app_event(&evt);
}

static void handle_data_rx_slot_done(const phy_evt_t *evt)
{
    if (evt->slot_index < 0) return;

    uwb_slot_t *s = UwbSlots_Get(evt->slot_index);
    UwbProtocolFrame frame;
    if (s == NULL || !UwbProtocol_Decode(&frame, s->data, s->data_len)) {
        UwbSlots_Free(evt->slot_index);
        return;
    }

    if (frame.common.func_code == (uint8_t)UWB_FUNC_APP_DATA_CTRL_RESP &&
        frame.common.ext_header_len >= 2) {
        uint8_t resp_type = frame.ext_header[0];
        app_log_info("[LINK] DATA_RESP src=0x%04X sess=0x%04X resp=%u",
                     frame.mac.src16, g_data.active_cmd.session_id,
                     (unsigned)resp_type);
        if (resp_type == UWB_DATA_RESP_WAIT) {
            post_data_resp_event(UWB_LINK_APP_EVT_DATA_WAIT, resp_type, frame.mac.src16);
        } else if (resp_type == UWB_DATA_RESP_ERROR) {
            post_data_resp_event(UWB_LINK_APP_EVT_DATA_ERROR, resp_type, frame.mac.src16);
        } else {
            post_data_resp_event(UWB_LINK_APP_EVT_DATA_ACK, resp_type, frame.mac.src16);
        }
        UwbSlots_Free(evt->slot_index);
        data_finish_response();
        return;
    }

    if (frame.common.func_code == (uint8_t)UWB_FUNC_APP_DATA_FRAG &&
        frame.common.ext_header_len >= 5) {
        UwbLinkAppEvent app_evt;
        memset(&app_evt, 0, sizeof(app_evt));
        app_evt.type = UWB_LINK_APP_EVT_DATA_FRAG;
        app_evt.data.data_frag.src_id = frame.mac.src16;
        app_evt.data.data_frag.session_id = UwbProtocol_ReadLe16(&frame.ext_header[0]);
        app_evt.data.data_frag.frag_id = frame.ext_header[2];
        app_evt.data.data_frag.total_frags = frame.ext_header[3];
        app_evt.data.data_frag.flags = frame.ext_header[4];
        app_evt.data.data_frag.slot_index = evt->slot_index;
        app_evt.data.data_frag.payload_len = frame.common.payload_len;
        app_log_info("[LINK] DATA_FRAG_RX src=0x%04X sess=0x%04X frag=%u total=%u flags=0x%02X len=%u slot=%d",
                     app_evt.data.data_frag.src_id,
                     app_evt.data.data_frag.session_id,
                     (unsigned)app_evt.data.data_frag.frag_id,
                     (unsigned)app_evt.data.data_frag.total_frags,
                     (unsigned)app_evt.data.data_frag.flags,
                     app_evt.data.data_frag.payload_len,
                     (int)evt->slot_index);
        s->owner = UWB_SLOT_APP_OWN;
        post_app_event(&app_evt);
        data_finish_response();
        return;
    }

    UwbSlots_Free(evt->slot_index);
}

static void handle_idle_rx_frame(const phy_evt_t *evt)
{
    if (evt->slot_index < 0) return;

    uwb_slot_t *s = UwbSlots_Get(evt->slot_index);
    UwbProtocolFrame frame;
    if (s == NULL || !UwbProtocol_Decode(&frame, s->data, s->data_len)) {
        UwbSlots_Free(evt->slot_index);
        return;
    }

    if (frame.common.func_code == (uint8_t)UWB_FUNC_APP_DATA_CFG &&
        frame.common.ext_header_len >= 2) {
        UwbLinkAppEvent app_evt;
        memset(&app_evt, 0, sizeof(app_evt));
        app_evt.type = UWB_LINK_APP_EVT_DATA_CFG;
        app_evt.data.data_cfg.src_id = frame.mac.src16;
        app_evt.data.data_cfg.session_id = UwbProtocol_ReadLe16(&frame.ext_header[0]);
        post_app_event(&app_evt);
        app_log_info("[LINK] DATA_CFG src=0x%04X sess=0x%04X",
                     app_evt.data.data_cfg.src_id,
                     app_evt.data.data_cfg.session_id);
        UwbSlots_Free(evt->slot_index);
        return;
    }

    if (frame.common.func_code == (uint8_t)UWB_FUNC_APP_DATA_CTRL &&
        frame.common.ext_header_len >= 4) {
        UwbLinkAppEvent app_evt;
        memset(&app_evt, 0, sizeof(app_evt));
        app_evt.type = UWB_LINK_APP_EVT_DATA_CTRL;
        app_evt.data.data_ctrl.src_id = frame.mac.src16;
        app_evt.data.data_ctrl.session_id = UwbProtocol_ReadLe16(&frame.ext_header[0]);
        app_evt.data.data_ctrl.ctrl_type = frame.ext_header[2];
        app_evt.data.data_ctrl.frag_id = frame.ext_header[3];
        post_app_event(&app_evt);
        app_log_info("[LINK] DATA_CTRL src=0x%04X sess=0x%04X ctrl=%u frag=%u",
                     app_evt.data.data_ctrl.src_id,
                     app_evt.data.data_ctrl.session_id,
                     (unsigned)app_evt.data.data_ctrl.ctrl_type,
                     (unsigned)app_evt.data.data_ctrl.frag_id);
        UwbSlots_Free(evt->slot_index);
        return;
    }

    app_log_info("[LINK] RX_FRAME type=%u src=0x%04X (no handler)",
                 s->frame_type, s->src_short);
    UwbSlots_Free(evt->slot_index);
}

static void handle_data_sent_evt(const phy_evt_t *evt)
{
    if (g_link.cfg.role != APP_ROLE_ANCHOR ||
        evt == NULL || evt->slot_index < 0) {
        return;
    }

    uwb_slot_t *s = UwbSlots_Get(evt->slot_index);
    if (s == NULL || s->owner == UWB_SLOT_FREE ||
        s->frame_type != (uint8_t)UWB_FUNC_APP_DATA_FRAG) {
        return;
    }

    UwbLinkAppEvent app_evt;
    memset(&app_evt, 0, sizeof(app_evt));
    app_evt.type = UWB_LINK_APP_EVT_DATA_SENT;
    app_evt.data.data_sent.session_id = s->session_id;
    app_evt.data.data_sent.frag_id = s->frag_id;
    post_app_event(&app_evt);
}

static void handle_data_retry_evt(const phy_evt_t *evt)
{
    link_data_slot_entry_t *entry = link_data_slot_find(evt->frag_id);
    if (entry == NULL) {
        app_log_info("[LINK] DATA_RETRY_NO_BACKUP frag=%u", (unsigned)evt->frag_id);
        return;
    }

    phy_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = PHY_CMD_LOAD_PENDING;
    cmd.slot_index = entry->slot_index;
    cmd.session_id = entry->session_id;
    cmd.frag_id = entry->frag_id;

    if (UwbBuffers_SendCmd(&cmd, 0)) {
        UwbPhy_NotifyCmd();
        app_log_info("[LINK] DATA_RELOAD_PENDING sess=0x%04X frag=%u slot=%d",
                     entry->session_id, (unsigned)entry->frag_id,
                     (int)entry->slot_index);
    } else {
        app_log_warn("[LINK] DATA_RELOAD_CMD_FULL frag=%u", (unsigned)evt->frag_id);
    }
}

static void link_dispatch_phy_evt(const phy_evt_t *evt)
{
    switch (evt->type) {

    case PHY_EVT_TX_DONE:
        if (g_data.state == LINK_DATA_WAIT_TX_DONE &&
            evt->slot_index == g_data.tx_slot) {
            app_log_info("[LINK] DATA_TX_DONE slot=%d sess=0x%04X frag=%u",
                         (int)evt->slot_index,
                         g_data.active_cmd.session_id,
                         (unsigned)g_data.active_cmd.frag_id);
            g_data.state = LINK_DATA_WAIT_RX;
            g_data.rx_window_active = true;
            g_data.started_ms = HAL_GetTick();
        } else if (evt->slot_index >= 0 &&
                   evt->slot_index == g_link.misc_tx_slot_idx) {
            UwbSlots_Free(evt->slot_index);
            g_link.misc_tx_slot_idx = -1;
        } else if (evt->slot_index >= 0) {
            g_link.tx_slot_idx = evt->slot_index;
        }
        break;

    case PHY_EVT_RX_SLOT_DONE:
        if (g_data.rx_window_active) {
            handle_data_rx_slot_done(evt);
        } else {
            handle_disc_rx_slot_done(evt);
        }
        break;

    case PHY_EVT_RX_WINDOW_END:
        if (g_data.rx_window_active) {
            bool got_response = g_data.response_seen;
            g_data.rx_window_active = false;
            g_data.response_seen = false;
            if (!got_response && g_data.state == LINK_DATA_WAIT_RX) {
                app_log_warn("[LINK] DATA_RX_WINDOW_EMPTY sess=0x%04X frag=%u",
                             g_data.active_cmd.session_id,
                             (unsigned)g_data.active_cmd.frag_id);
                data_retry_or_fail();
            }
            break;
        }

        app_log_info("[LINK] RX_WIN ok=%u/%u r=[%d,%d,%d,%d]",
                     (unsigned)evt->rx_seq,
                     (unsigned)DISC_RX_SLOT_COUNT,
                     (int)g_link.rx_slot_results[0],
                     (int)g_link.rx_slot_results[1],
                     (int)g_link.rx_slot_results[2],
                     (int)g_link.rx_slot_results[3]);
        memset(g_link.rx_slot_results, -1, sizeof(g_link.rx_slot_results));
        if (g_link.tx_slot_idx >= 0) {
            UwbSlots_Free(g_link.tx_slot_idx);
            g_link.tx_slot_idx = -1;
        }
        if (g_disc_outstanding > 0) {
            g_disc_outstanding--;
        }
        g_disc_count++;
        break;

    case PHY_EVT_ERROR:
        app_log_warn("[LINK] PHY_ERROR");
        if (g_data.state != LINK_DATA_IDLE) {
            data_retry_or_fail();
        } else if (g_disc_outstanding > 0) {
            disc_sm_on_error(LINK_ERR_DISC_PHY_ERROR);
        }
        break;

    case PHY_EVT_RX_FRAME:
        handle_idle_rx_frame(evt);
        break;

    case PHY_EVT_RX_TIMEOUT:
        app_log_info("[LINK] RX_TIMEOUT");
        break;

    case PHY_EVT_DATA_RETRY:
        handle_data_retry_evt(evt);
        break;

    case PHY_EVT_DATA_SENT:
        handle_data_sent_evt(evt);
        break;
    }
}

/* ================================================================
 *  事件监控与调度
 * ================================================================ */

static void link_check_timeouts(void)
{
    uint32_t now = HAL_GetTick();

    if (g_disc_outstanding > 0 &&
        g_to.disc_cmd_sent_ms > 0 &&
        now - g_to.disc_cmd_sent_ms >= LINK_DISC_CMD_TIMEOUT_MS) {
        disc_sm_on_error(LINK_ERR_DISC_TIMEOUT);
    }

    if (g_data.state != LINK_DATA_IDLE &&
        g_data.started_ms > 0 &&
        now - g_data.started_ms >= LINK_DATA_TIMEOUT_MS) {
        app_log_warn("[LINK] DATA_TIMEOUT state=%u sess=0x%04X frag=%u elapsed=%lu",
                     (unsigned)g_data.state,
                     g_data.active_cmd.session_id,
                     (unsigned)g_data.active_cmd.frag_id,
                     (unsigned long)(now - g_data.started_ms));
        data_retry_or_fail();
    }
}

static void link_event_monitor(void)
{
    phy_evt_t evt;
    while (UwbBuffers_RecvEvt(&evt, 0)) {
        link_dispatch_phy_evt(&evt);
    }

    drain_app_cmds();
    link_check_timeouts();
}

static bool link_tag_send_disc(void)
{
    if (g_disc_outstanding >= DISC_PIPELINE_MAX ||
        g_data.state != LINK_DATA_IDLE ||
        g_data.rx_window_active) {
        return false;
    }

    int8_t idx = UwbSlots_Alloc(UWB_SLOT_LINK_OWN);
    if (idx < 0) return false;

    uwb_slot_t *s = UwbSlots_Get(idx);
    g_link.window_id++;
    s->window_id = g_link.window_id;
    uint8_t tx_seq = 0U;
    s->data_len  = build_disc_req(s->data, sizeof(s->data), &tx_seq);

    if (s->data_len == 0) {
        app_log_warn("[LINK] build DISC_REQ fail");
        UwbSlots_Free(idx);
        return false;
    }
    s->frame_type = (uint8_t)UWB_FUNC_DISCOVERY_REQ;

    phy_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type           = PHY_CMD_TX_FRAME;
    cmd.slot_index     = idx;
    cmd.tx_time        = 0;
    cmd.has_pending_rx = true;
    cmd.rx_timeout_us  = UWB_PHY_RX_SLOT_TIMEOUT_US;
    cmd.rx_slot_count  = DISC_RX_SLOT_COUNT;

    if (!UwbBuffers_SendCmd(&cmd, 0)) {
        app_log_warn("[LINK] cmd queue full");
        UwbSlots_Free(idx);
        return false;
    }

    UwbPhy_NotifyCmd();
    (void)UwbLossTest_PostRangeTx(tx_seq);
    g_disc_outstanding++;
    g_link.tx_slot_idx = -1;
    g_to.disc_cmd_sent_ms = HAL_GetTick();
    return true;
}

static void link_tag_schedule(void)
{
    if (g_data.state != LINK_DATA_IDLE || g_data.rx_window_active) return;

    if (g_data.pending_cmd_valid &&
        g_disc_count >= 2U &&
        g_disc_outstanding == 0U) {
        if (start_tag_data_cmd(&g_data.pending_cmd)) {
            g_data.pending_cmd_valid = false;
            return;
        }
    }

    (void)link_tag_send_disc();
}

/* ================================================================
 *  公开接口
 * ================================================================ */

bool UwbLink_SendCmd(const UwbLinkCmd *cmd)
{
    return UwbLinkCmd_Send(cmd, 0);
}

void UwbLink_DataSlotFreeAll(void)
{
    link_data_slot_free_all();
    phy_clear_pending();
}

/* ================================================================
 *  初始化 & 任务
 * ================================================================ */

bool UwbLink_Init(const UwbStackConfig *cfg)
{
    if (cfg == NULL) return false;

    memset(&g_link, 0, sizeof(g_link));
    g_link.cfg = *cfg;
    g_link.tx_slot_idx = -1;
    g_link.misc_tx_slot_idx = -1;
    memset(g_link.rx_slot_results, -1, sizeof(g_link.rx_slot_results));

    g_disc_outstanding = 0;
    g_disc_count = 0;
    memset(&g_to, 0, sizeof(g_to));
    memset(&g_data, 0, sizeof(g_data));
    g_data.state = LINK_DATA_IDLE;
    g_data.tx_slot = -1;
    memset(g_data_slots, 0, sizeof(g_data_slots));

    if (g_app_evt_queue == NULL) {
        g_app_evt_queue = xQueueCreateStatic(
            LINK_APP_EVT_QUEUE_LEN, sizeof(UwbLinkAppEvent),
            g_app_evt_queue_buf, &g_app_evt_queue_ctrl);
    }

    return true;
}

bool UwbLink_StartThread(const osThreadAttr_t *attr)
{
    if (g_link_task != NULL) return true;
    g_link_task = osThreadNew(UwbLink_Task, NULL, attr);
    return g_link_task != NULL;
}

void UwbLink_Task(void *argument)
{
    (void)argument;

    LogService_Write(APP_LOG_INFO, "[LINK] START short=0x%04X role=%u",
                     g_link.cfg.short_addr, (unsigned)g_link.cfg.role);

    for (;;) {
        link_event_monitor();

        if (g_link.cfg.role == APP_ROLE_TAG) {
            link_tag_schedule();
        }

        osDelay(1);
    }
}
