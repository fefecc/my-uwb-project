#include "uwb_link.h"

#include <string.h>

#include "uwb_phy.h"
#include "uwb_protocol.h"
#include "uwb_timestamp.h"
#include "main.h"
#include "../service/log_service.h"

#define UWB_LINK_APP_EVT_QUEUE_LENGTH (8U)
#define UWB_LINK_POLL_TIMEOUT_MS      (100U)
#define UWB_LINK_DISCOVERY_PERIOD_MS  (1000U)
#define UWB_LINK_RANGE_PERIOD_MS      (250U)
#define UWB_LINK_RX_TIMEOUT_UUS       (5000U)
#define UWB_LINK_ANCHOR_RESP_DELAY_TICKS (4992000ULL)
#define UWB_LINK_RESP_SLOT_WIDTH      (4000U)
#define UWB_LINK_RESP_SLOT_BASE       (1U)
#define UWB_LINK_RESP_SLOT_COUNT      (4U)

typedef enum {
    UWB_LINK_PENDING_NONE = 0,
    UWB_LINK_PENDING_DISCOVERY_REQ,
    UWB_LINK_PENDING_DISCOVERY_RESP,
    UWB_LINK_PENDING_TWR_START,
    UWB_LINK_PENDING_TWR_RESP,
} UwbPendingTx;

typedef struct {
    UwbStackConfig cfg;
    UwbLinkState state;
    uint8_t seq;
    uint16_t exchange_seq;
    uint32_t discovery_epoch;
    uint32_t last_discovery_ms;
    uint32_t last_range_ms;
    UwbPendingTx pending_tx;
    uint16_t pending_anchor_id;
    uint16_t pending_response_slot_id;
    uint64_t pending_tag_tx_ts;
} UwbLinkContext;

static UwbLinkContext g_link;
static StaticQueue_t g_app_evt_queue_ctrl;
static uint8_t g_app_evt_queue_storage[UWB_LINK_APP_EVT_QUEUE_LENGTH * sizeof(UwbLinkAppEvent)];
static QueueHandle_t g_app_evt_queue;
static TaskHandle_t g_link_task;

static uint8_t next_seq(void)
{
    return g_link.seq++;
}

static void post_app_event(const UwbLinkAppEvent *event)
{
    if (g_app_evt_queue == NULL || event == NULL) {
        return;
    }

    if (xQueueSend(g_app_evt_queue, event, 0) != pdPASS) {
        app_log_warn("UWB app event queue full: evt=%u", (unsigned)event->type);
    }
}

static bool post_rx_enable(uint16_t timeout_uus)
{
    UwbPhyCommand cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_type = UWB_PHY_CMD_RX_ENABLE;
    cmd.rx_timeout_uus = timeout_uus;
    return UwbPhy_PostCommand(&cmd, 0);
}

static bool send_frame(const UwbProtocolFrame *frame,
                       UwbPhyCommandType tx_type,
                       uint64_t delayed_time,
                       UwbPendingTx pending)
{
    UwbPhyCommand cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_type = tx_type;
    cmd.tx_delay_time = delayed_time;

    size_t tx_len = 0;
    if (!UwbProtocol_Encode(frame, cmd.tx_buf, sizeof(cmd.tx_buf), &tx_len)) {
        return false;
    }

    cmd.tx_len = (uint16_t)tx_len;
    if (!UwbPhy_PostCommand(&cmd, 0)) {
        return false;
    }

    g_link.pending_tx = pending;
    g_link.state = UWB_LINK_STATE_TX_WAIT_DONE;
    return true;
}

static void send_discovery_req(void)
{
    UwbProtocolFrame frame;
    UwbProtocol_InitFrame(&frame,
                          &g_link.cfg,
                          UWB_STACK_BROADCAST_SHORT_ID,
                          next_seq(),
                          UWB_FUNC_DISCOVERY_REQ);

    frame.common.ext_header_len = 10U;
    UwbProtocol_WriteLe32(&frame.ext_header[0], g_link.discovery_epoch++);
    UwbProtocol_WriteLe16(&frame.ext_header[4], UWB_LINK_RESP_SLOT_BASE);
    UwbProtocol_WriteLe16(&frame.ext_header[6], UWB_LINK_RESP_SLOT_WIDTH);
    UwbProtocol_WriteLe16(&frame.ext_header[8], UWB_LINK_RESP_SLOT_COUNT);

    if (!send_frame(&frame, UWB_PHY_CMD_TX_NOW, 0, UWB_LINK_PENDING_DISCOVERY_REQ)) {
        app_log_warn("UWB discovery request send failed");
    }
}

static void send_discovery_resp(uint16_t dst16, uint32_t epoch)
{
    UwbProtocolFrame frame;
    UwbProtocol_InitFrame(&frame, &g_link.cfg, dst16, next_seq(), UWB_FUNC_DISCOVERY_RESP);

    frame.common.ext_header_len = 8U;
    UwbProtocol_WriteLe32(&frame.ext_header[0], epoch);
    UwbProtocol_WriteLe16(&frame.ext_header[4], g_link.cfg.short_addr);
    UwbProtocol_WriteLe16(&frame.ext_header[6], 0U);

    if (!send_frame(&frame, UWB_PHY_CMD_TX_NOW, 0, UWB_LINK_PENDING_DISCOVERY_RESP)) {
        app_log_warn("UWB discovery response send failed");
    }
}

static void send_twr_start(uint16_t target_short_id)
{
    UwbProtocolFrame frame;
    UwbProtocol_InitFrame(&frame, &g_link.cfg, target_short_id, next_seq(), UWB_FUNC_TWR_TAG_START);

    uint16_t exchange_seq = ++g_link.exchange_seq;
    uint16_t slot_id = (uint16_t)(UWB_LINK_RESP_SLOT_BASE +
                       (g_link.cfg.short_addr % UWB_LINK_RESP_SLOT_COUNT));

    frame.common.ext_header_len = 8U;
    UwbProtocol_WriteLe16(&frame.ext_header[0], exchange_seq);
    UwbProtocol_WriteLe16(&frame.ext_header[2], target_short_id);
    UwbProtocol_WriteLe16(&frame.ext_header[4], slot_id);
    UwbProtocol_WriteLe16(&frame.ext_header[6], UWB_LINK_RESP_SLOT_WIDTH);

    g_link.pending_anchor_id = target_short_id;
    g_link.pending_response_slot_id = slot_id;
    g_link.pending_tag_tx_ts = 0;

    if (!send_frame(&frame, UWB_PHY_CMD_TX_NOW, 0, UWB_LINK_PENDING_TWR_START)) {
        app_log_warn("UWB TWR start send failed");
    }
}

static void send_twr_anchor_resp(uint16_t dst16,
                                 uint16_t exchange_seq,
                                 uint64_t anchor_rx_ts,
                                 uint16_t response_slot_id)
{
    UwbProtocolFrame frame;
    UwbProtocol_InitFrame(&frame, &g_link.cfg, dst16, next_seq(), UWB_FUNC_TWR_ANCHOR_RESP);

    uint64_t anchor_tx_ts = (anchor_rx_ts + UWB_LINK_ANCHOR_RESP_DELAY_TICKS) & ((1ULL << 40U) - 1ULL);
    frame.common.ext_header_len = 18U;
    UwbProtocol_WriteLe16(&frame.ext_header[0], exchange_seq);
    UwbProtocol_WriteLe64(&frame.ext_header[2], anchor_rx_ts);
    UwbProtocol_WriteLe64(&frame.ext_header[10], anchor_tx_ts);

    g_link.pending_response_slot_id = response_slot_id;
    if (!send_frame(&frame, UWB_PHY_CMD_TX_DELAYED, anchor_tx_ts, UWB_LINK_PENDING_TWR_RESP)) {
        app_log_warn("UWB TWR anchor response send failed");
    }
}

static void handle_discovery_req(const UwbProtocolFrame *frame)
{
    if (g_link.cfg.role != APP_ROLE_ANCHOR || frame->common.ext_header_len < 10U) {
        return;
    }

    uint32_t epoch = UwbProtocol_ReadLe32(&frame->ext_header[0]);
    send_discovery_resp(frame->mac.src16, epoch);
}

static void handle_discovery_resp(const UwbProtocolFrame *frame)
{
    if (g_link.cfg.role != APP_ROLE_TAG || frame->common.ext_header_len < 8U) {
        return;
    }

    UwbLinkAppEvent event;
    memset(&event, 0, sizeof(event));
    event.type = UWB_LINK_APP_EVT_NEIGHBOR_SEEN;
    event.data.neighbor.short_id = UwbProtocol_ReadLe16(&frame->ext_header[4]);
    event.data.neighbor.capability = UwbProtocol_ReadLe16(&frame->ext_header[6]);
    post_app_event(&event);
}

static void handle_twr_tag_start(const UwbProtocolFrame *frame, const UwbPhySlot *slot)
{
    if (g_link.cfg.role != APP_ROLE_ANCHOR ||
        frame->common.ext_header_len < 8U ||
        slot == NULL) {
        return;
    }

    uint16_t exchange_seq = UwbProtocol_ReadLe16(&frame->ext_header[0]);
    uint16_t target_short_id = UwbProtocol_ReadLe16(&frame->ext_header[2]);
    uint16_t response_slot_id = UwbProtocol_ReadLe16(&frame->ext_header[4]);

    if (target_short_id != g_link.cfg.short_addr &&
        target_short_id != UWB_STACK_BROADCAST_SHORT_ID) {
        return;
    }

    send_twr_anchor_resp(frame->mac.src16,
                         exchange_seq,
                         slot->rx_ts,
                         response_slot_id);
}

static void handle_twr_anchor_resp(const UwbProtocolFrame *frame, const UwbPhySlot *slot)
{
    if (g_link.cfg.role != APP_ROLE_TAG ||
        frame->common.ext_header_len < 18U ||
        slot == NULL) {
        return;
    }

    UwbLinkAppEvent event;
    memset(&event, 0, sizeof(event));
    event.type = UWB_LINK_APP_EVT_TWR_EXCHANGE;
    event.data.twr.anchor_id = frame->mac.src16;
    event.data.twr.exchange_seq = UwbProtocol_ReadLe16(&frame->ext_header[0]);
    event.data.twr.response_slot_id = g_link.pending_response_slot_id;
    event.data.twr.status_flags = 0;
    event.data.twr.tag_tx_ts = g_link.pending_tag_tx_ts;
    event.data.twr.anchor_rx_ts = UwbProtocol_ReadLe64(&frame->ext_header[2]);
    event.data.twr.anchor_tx_ts = UwbProtocol_ReadLe64(&frame->ext_header[10]);
    event.data.twr.tag_rx_ts = slot->rx_ts;
    event.data.twr.quality = slot->quality;
    event.data.twr.retry_count = 0;
    post_app_event(&event);
}

static void handle_rx_frame(const UwbPhyEvent *event)
{
    UwbPhySlot *slot = UwbPhy_GetSlot(event->slot_id, event->generation);
    if (slot == NULL) {
        return;
    }

    UwbProtocolFrame frame;
    if (!UwbProtocol_Decode(&frame, slot->rx_data, slot->rx_len)) {
        UwbPhy_ReleaseSlot(event->slot_id, event->generation);
        return;
    }

    if (frame.mac.pan_id != g_link.cfg.pan_id ||
        (frame.mac.dst16 != g_link.cfg.short_addr &&
         frame.mac.dst16 != UWB_STACK_BROADCAST_SHORT_ID)) {
        UwbPhy_ReleaseSlot(event->slot_id, event->generation);
        return;
    }

    g_link.state = UWB_LINK_STATE_RX_PROCESS;
    switch ((UwbFuncCode)frame.common.func_code) {
        case UWB_FUNC_DISCOVERY_REQ:
            handle_discovery_req(&frame);
            break;

        case UWB_FUNC_DISCOVERY_RESP:
            handle_discovery_resp(&frame);
            break;

        case UWB_FUNC_TWR_TAG_START:
            handle_twr_tag_start(&frame, slot);
            break;

        case UWB_FUNC_TWR_ANCHOR_RESP:
            handle_twr_anchor_resp(&frame, slot);
            break;

        default:
            break;
    }

    UwbPhy_ReleaseSlot(event->slot_id, event->generation);
    if (g_link.state != UWB_LINK_STATE_TX_WAIT_DONE) {
        g_link.state = UWB_LINK_STATE_IDLE;
    }
}

static void handle_phy_event(const UwbPhyEvent *event)
{
    if (event == NULL) {
        return;
    }

    switch (event->event_type) {
        case UWB_PHY_EVT_RX_OK:
            handle_rx_frame(event);
            break;

        case UWB_PHY_EVT_TX_DONE:
            if (g_link.pending_tx == UWB_LINK_PENDING_TWR_START) {
                g_link.pending_tag_tx_ts = event->dw_ts;
            }
            g_link.pending_tx = UWB_LINK_PENDING_NONE;
            g_link.state = UWB_LINK_STATE_IDLE;
            (void)post_rx_enable(UWB_LINK_RX_TIMEOUT_UUS);
            break;

        case UWB_PHY_EVT_RX_TIMEOUT:
            g_link.state = UWB_LINK_STATE_IDLE;
            break;

        case UWB_PHY_EVT_RX_ERROR:
        case UWB_PHY_EVT_TX_ERROR:
            g_link.pending_tx = UWB_LINK_PENDING_NONE;
            g_link.state = UWB_LINK_STATE_RECOVER;
            (void)post_rx_enable(UWB_LINK_RX_TIMEOUT_UUS);
            g_link.state = UWB_LINK_STATE_IDLE;
            break;

        case UWB_PHY_EVT_NONE:
        default:
            break;
    }
}

static void run_scheduler(void)
{
    if (g_link.state == UWB_LINK_STATE_TX_WAIT_DONE) {
        return;
    }

    uint32_t now = HAL_GetTick();

    if (g_link.cfg.role == APP_ROLE_TAG) {
        if ((now - g_link.last_discovery_ms) >= UWB_LINK_DISCOVERY_PERIOD_MS) {
            g_link.last_discovery_ms = now;
            send_discovery_req();
            return;
        }

        if ((now - g_link.last_range_ms) >= UWB_LINK_RANGE_PERIOD_MS) {
            g_link.last_range_ms = now;
            send_twr_start(UWB_STACK_BROADCAST_SHORT_ID);
            return;
        }
    }

    if (g_link.state == UWB_LINK_STATE_IDLE) {
        if (post_rx_enable(UWB_LINK_RX_TIMEOUT_UUS)) {
            g_link.state = UWB_LINK_STATE_RX_ON;
        }
    }
}

bool UwbLink_Init(const UwbStackConfig *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    memset(&g_link, 0, sizeof(g_link));
    g_link.cfg = *cfg;
    g_link.state = UWB_LINK_STATE_IDLE;

    if (g_app_evt_queue == NULL) {
        g_app_evt_queue = xQueueCreateStatic(UWB_LINK_APP_EVT_QUEUE_LENGTH,
                                             sizeof(UwbLinkAppEvent),
                                             g_app_evt_queue_storage,
                                             &g_app_evt_queue_ctrl);
    }

    return g_app_evt_queue != NULL;
}

bool UwbLink_StartThread(const osThreadAttr_t *attr)
{
    if (g_link_task != NULL) {
        return true;
    }

    g_link_task = osThreadNew(UwbLink_Task, NULL, attr);
    return g_link_task != NULL;
}

QueueHandle_t UwbLink_AppEventQueue(void)
{
    return g_app_evt_queue;
}

void UwbLink_Task(void *argument)
{
    (void)argument;

    QueueHandle_t phy_events = UwbPhy_EventQueue();
    UwbPhyEvent event;

    for (;;) {
        if (phy_events != NULL &&
            xQueueReceive(phy_events, &event, pdMS_TO_TICKS(UWB_LINK_POLL_TIMEOUT_MS)) == pdPASS) {
            handle_phy_event(&event);
        }

        run_scheduler();
    }
}
