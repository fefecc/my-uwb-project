/**
 * @file uwb_link.c
 * @brief LINK 层 (V4 极简版) - Tag 定时发 DISC_REQ, PHY 负责回收 TX slot
 *
 * 流程:
 *   1. LINK 分配 slot, 打包 DISC_REQ 帧数据
 *   2. 通过 cmd_queue 发送 slot_index 给 PHY
 *   3. 阻塞等待 PHY 的 TX_DONE/ERROR 事件 (带超时保护)
 *   4. TX slot 由 PHY 层回收, LINK 只处理事件通知
 *   5. RX slot 仍由 LINK 回收 (需要读取帧数据做业务处理)
 *   6. Anchor: 只回收 PHY 上报的 RX slot (快速应答由 PHY 自动完成)
 *
 * 对 PHY 的 RESET 能力保留但暂不使用。
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
#define LINK_DISC_PERIOD_MS       200U   /* 发送 DISC_REQ 的周期 */
#define LINK_TX_TIMEOUT_MS        50U    /* 等待 PHY 完成 TX 的超时 */
#define LINK_POLL_TIMEOUT_MS      50U    /* 主循环轮询间隔 (Anchor 用) */

/* ---- 上下文 ---- */
typedef struct {
    UwbStackConfig cfg;
    uint16_t window_id;
    uint8_t  seq;
    uint32_t last_disc_ms;
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

/* ---- 回收 PHY 事件 (非阻塞, 处理 RX 帧和超时等) ---- */
static void drain_phy_events(void)
{
    phy_evt_t evt;
    while (UwbBuffers_RecvEvt(&evt, 0)) {

        switch (evt.type) {
            case PHY_EVT_TX_DONE:
                /* TX slot 已由 PHY 回收, LINK 只记录日志 */
                app_log_info("[LINK] TX_DONE (slot recycled by PHY)");
                break;

            case PHY_EVT_RX_FRAME: {
                /* RX slot 由 LINK 回收 (需要读取帧数据) */
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
                /* TX slot 已由 PHY 回收, LINK 只记录日志 */
                app_log_warn("[LINK] PHY_ERROR (slot recycled by PHY)");
                break;
        }
    }
}

/* ---- Tag: 发送 DISC_REQ (内含超时等待) ---- */
static void link_tag_send_disc(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - g_link.last_disc_ms) < LINK_DISC_PERIOD_MS) {
        return;
    }
    g_link.last_disc_ms = now;

    app_log_info("[LINK] preparing DISC_REQ tick=%lu", (unsigned long)now);

    /* 1. 分配 slot, 打包帧数据 */
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

    /* 2. 通过消息队列将 slot 位置发送给 PHY */
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
    app_log_info("[LINK] DISC_REQ win=%u seq=%u slot=%d",
                 g_link.window_id, (unsigned)(g_link.seq - 1), (int)idx);

    /* 3. 超时等待 PHY 完成 TX (slot 已交给 PHY, 由 PHY 回收)
     *    等到 TX_DONE/ERROR 事件, 或超时后放弃等待
     *    超时不需要回收 slot, PHY 层的看门狗会兜底 */
    phy_evt_t evt;
    bool got = UwbBuffers_RecvEvt(&evt, pdMS_TO_TICKS(LINK_TX_TIMEOUT_MS));
    if (got) {
        /* 处理 TX 结果事件 */
        switch (evt.type) {
            case PHY_EVT_TX_DONE:
                app_log_info("[LINK] TX_DONE (slot recycled by PHY)");
                break;
            case PHY_EVT_ERROR:
                app_log_warn("[LINK] TX failed (slot recycled by PHY)");
                break;
            case PHY_EVT_RX_FRAME: {
                /* 可能在等待期间收到 RX 帧 */
                uwb_slot_t *rs = UwbSlots_Get(evt.slot_index);
                if (rs != NULL) {
                    app_log_info("[LINK] RX type=%u src=0x%04X",
                                 rs->frame_type, rs->src_short);
                }
                if (evt.slot_index >= 0) UwbSlots_Free(evt.slot_index);
                break;
            }
            case PHY_EVT_RX_TIMEOUT:
                app_log_info("[LINK] RX_TIMEOUT after TX");
                break;
        }
    } else {
        app_log_warn("[LINK] TX timeout (%ums), slot owned by PHY",
                     (unsigned)LINK_TX_TIMEOUT_MS);
    }
}

/* ---- LINK 层对 PHY 的复位 (保留代码, 暂不使用) ---- */
#if 0
static void link_reset_phy(void)
{
    phy_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type       = PHY_CMD_RESET;
    cmd.slot_index = -1;
    UwbBuffers_SendCmd(&cmd, 0);
    UwbPhy_NotifyCmd();
}
#endif

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

    g_link.last_disc_ms = HAL_GetTick();

    for (;;) {
        /* 回收 PHY 事件 (非阻塞, 处理上一轮残留的 RX 事件等) */
        drain_phy_events();

        /* Tag 定时发送 (内含 TX 超时等待) */
        if (g_link.cfg.role == APP_ROLE_TAG) {
            link_tag_send_disc();
        }

        /* Anchor 只需轮询等待 */
        osDelay(LINK_POLL_TIMEOUT_MS);
    }
}
