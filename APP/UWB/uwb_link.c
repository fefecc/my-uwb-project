/**
 * @file uwb_link.c
 * @brief LINK 层 (V3 极简版) - Tag 100ms 循环发 DISC_REQ + 回收 slot
 *
 * 简化版只做两件事:
 *   1. Tag: 每 100ms 发一次 DISC_REQ，等 PHY 事件，回收 slot
 *   2. Anchor: 只回收 PHY 上报的 slot (快速应答由 PHY 自动完成)
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
#include "../service/log_service.h"

/* ---- 时序参数 ---- */
#define LINK_DISC_PERIOD_MS       200U
#define LINK_POLL_TIMEOUT_MS      50U
#define LINK_PHY_ALIVE_MS         500U

/* ---- 上下文 ---- */
typedef struct {
    UwbStackConfig cfg;
    uint16_t window_id;
    uint8_t  seq;
    uint32_t last_disc_ms;
    uint32_t last_phy_evt_ms;
} link_context_t;

static link_context_t g_link;
static TaskHandle_t g_link_task;

/* ---- LINK → APP 事件队列 ---- */
#define LINK_APP_EVT_QUEUE_LEN  4
static StaticQueue_t g_app_evt_queue_ctrl;
static uint8_t g_app_evt_queue_buf[LINK_APP_EVT_QUEUE_LEN * sizeof(UwbLinkAppEvent)];
static QueueHandle_t g_app_evt_queue;

QueueHandle_t UwbLink_AppEventQueue(void)
{
    return g_app_evt_queue;
}

/* ================================================================
 *  辅助
 * ================================================================ */

static uint8_t next_seq(void)
{
    return g_link.seq++;
}

/* ---- 构建 Discovery REQ 帧 ---- */
static uint16_t build_disc_req(uint8_t *buf, size_t buf_size)
{
    UwbProtocolFrame frame;
    UwbProtocol_InitFrame(&frame,
                          &g_link.cfg,
                          UWB_STACK_BROADCAST_SHORT_ID,
                          next_seq(),
                          UWB_FUNC_DISCOVERY_REQ);

    frame.common.ext_header_len = 10U;
    UwbProtocol_WriteLe32(&frame.ext_header[0], g_link.window_id);
    UwbProtocol_WriteLe16(&frame.ext_header[4], 1U);     /* slot base */
    UwbProtocol_WriteLe16(&frame.ext_header[6], 5000U);  /* slot width us */
    UwbProtocol_WriteLe16(&frame.ext_header[8], 1U);     /* slot count */

    size_t tx_len = 0;
    if (!UwbProtocol_Encode(&frame, buf, buf_size, &tx_len)) {
        return 0;
    }
    return (uint16_t)tx_len;
}

/* ---- 回收 PHY 事件 ---- */
static void drain_phy_events(void)
{
    phy_evt_t evt;
    while (UwbBuffers_RecvEvt(&evt, 0)) {
        g_link.last_phy_evt_ms = HAL_GetTick();

        switch (evt.type) {
            case PHY_EVT_TX_DONE: {
                uwb_slot_t *s = UwbSlots_Get(evt.slot_index);
                if (s != NULL) {
                    app_log_info("[LINK] TX_DONE win=%u tx=0x%02lX%08lX",
                                 s->window_id,
                                 (uint32_t)(s->tx_ts >> 32),
                                 (uint32_t)(s->tx_ts & 0xFFFFFFFF));
                }
                if (evt.slot_index >= 0) UwbSlots_Free(evt.slot_index);
                break;
            }

            case PHY_EVT_RX_FRAME: {
                uwb_slot_t *s = UwbSlots_Get(evt.slot_index);
                if (s != NULL) {
                    app_log_info("[LINK] RX type=%u src=0x%04X rx=0x%02lX%08lX",
                                 s->frame_type, s->src_short,
                                 (uint32_t)(s->rx_ts >> 32),
                                 (uint32_t)(s->rx_ts & 0xFFFFFFFF));
                }
                if (evt.slot_index >= 0) UwbSlots_Free(evt.slot_index);
                break;
            }

            case PHY_EVT_RX_TIMEOUT:
                app_log_info("[LINK] RX_TIMEOUT");
                break;

            case PHY_EVT_ERROR:
                app_log_warn("[LINK] PHY_ERROR slot=%d", (int)evt.slot_index);
                if (evt.slot_index >= 0) UwbSlots_Free(evt.slot_index);
                break;
        }
    }
}

/* ---- Tag: 发送 DISC_REQ ---- */
static void link_tag_send_disc(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - g_link.last_disc_ms) < LINK_DISC_PERIOD_MS) {
        return;
    }
    g_link.last_disc_ms = now;

    app_log_info("[LINK] preparing DISC_REQ tick=%lu", (unsigned long)now);

    int8_t idx = UwbSlots_Alloc(UWB_SLOT_LINK_OWN);
    if (idx < 0) {
        app_log_warn("[LINK] no slot for TX");
        return;
    }

    uwb_slot_t *s = UwbSlots_Get(idx);
    g_link.window_id++;
    s->window_id = g_link.window_id;
    s->data_len  = build_disc_req(s->data, sizeof(s->data));

    if (s->data_len == 0) {
        app_log_warn("[LINK] build DISC_REQ fail");
        UwbSlots_Free(idx);
        return;
    }

    phy_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type           = PHY_CMD_TX_FRAME;
    cmd.slot_index     = idx;
    cmd.tx_time        = 0;   /* 立即发送 */
    cmd.has_pending_rx = true;
    cmd.rx_timeout_us  = UWB_PHY_RX_SLOT_TIMEOUT_US;
    cmd.rx_slot_count  = 1;

    if (!UwbBuffers_SendCmd(&cmd, 0)) {
        app_log_warn("[LINK] cmd queue full");
        UwbSlots_Free(idx);
        return;
    }

    UwbPhy_NotifyCmd();
    app_log_info("[LINK] DISC_REQ win=%u seq=%u",
                 g_link.window_id, (unsigned)(g_link.seq - 1));
}

/* ---- LINK 层看门狗 ---- */
static void link_watchdog(void)
{
    uint32_t now = HAL_GetTick();
    if (g_link.last_phy_evt_ms == 0) {
        g_link.last_phy_evt_ms = now;
        return;
    }

    if ((now - g_link.last_phy_evt_ms) >= LINK_PHY_ALIVE_MS) {
        app_log_warn("[LINK] PHY no response");
        g_link.last_phy_evt_ms = now;

        phy_cmd_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.type       = PHY_CMD_RESET;
        cmd.slot_index = -1;
        UwbBuffers_SendCmd(&cmd, 0);
        UwbPhy_NotifyCmd();
    }
}

/* ================================================================
 *  LINK 接口
 * ================================================================ */

bool UwbLink_Init(const UwbStackConfig *cfg)
{
    if (cfg == NULL) return false;

    memset(&g_link, 0, sizeof(g_link));
    g_link.cfg = *cfg;

    /* 创建 LINK→APP 事件队列 */
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

    app_log_info("[LINK] START short=0x%04X role=%u",
                 g_link.cfg.short_addr, (unsigned)g_link.cfg.role);

    g_link.last_disc_ms    = HAL_GetTick();
    g_link.last_phy_evt_ms = HAL_GetTick();

    for (;;) {
        /* 回收 PHY 事件 */
        drain_phy_events();

        /* Tag 定时发送 */
        if (g_link.cfg.role == APP_ROLE_TAG) {
            link_tag_send_disc();
        }

        /* 看门狗 */
        link_watchdog();

        osDelay(LINK_POLL_TIMEOUT_MS);
    }
}
