#include "uwb_app.h"

#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "uwb_link.h"
#include "uwb_timestamp.h"
#include "../service/data_service.h"
#include "../service/log_service.h"
#include "../service/time_service.h"

#define UWB_APP_DW_TIME_UNIT      (1.0 / (499.2e6 * 128.0))
#define UWB_APP_SPEED_OF_LIGHT    (299702547.0)
#define UWB_APP_ANT_DELAY_COMP_M  (0.0)

#define TWR_MAX_ANCHORS  4

/* ---- per-Anchor 滑窗记录 ---- */
typedef struct {
    bool     valid;
    uint16_t anchor_id;
    uint64_t anchor_tx_ts;     /* t3: 上一次 Anchor 发 RESP 的时间 */
    uint64_t tag_rx_ts;        /* t4: 上一次 Tag 收 RESP 的时间 */
} twr_anchor_record_t;

typedef enum {
    TWR_MODE_DS,    /* DS-TWR: 两次交换消除钟差 */
    TWR_MODE_SS,    /* SS-TWR: 单次交换退化模式 */
} twr_mode_t;

static UwbStackConfig g_app_cfg;
static twr_anchor_record_t g_anchor_records[TWR_MAX_ANCHORS];
static TaskHandle_t g_uwb_app_task;

/* ================================================================
 *  DS-TWR 测距计算 (两次交换, 消除钟差)
 *  prev: 上一次交换的 t3(anchor_tx), t4(tag_rx)
 *  cur:  当前交换的 t1(tag_tx), t2(anchor_rx), t3(anchor_tx), t4(tag_rx)
 * ================================================================ */
static bool compute_range(const UwbTwrExchange *prev,
                          const UwbTwrExchange *cur,
                          UwbRangeResult *out)
{
    if (prev == NULL || cur == NULL || out == NULL ||
        prev->anchor_id != cur->anchor_id) {
        return false;
    }

    double t3 = (double)prev->anchor_tx_ts;
    double t4 = (double)prev->tag_rx_ts;
    double t5 = (double)cur->tag_tx_ts;
    double t6 = (double)cur->anchor_rx_ts;
    double t7 = (double)cur->anchor_tx_ts;
    double t8 = (double)cur->tag_rx_ts;

    double ra = (double)get_timestamp_difference_u64((uint64_t)t6, (uint64_t)t3);
    double da = (double)get_timestamp_difference_u64((uint64_t)t5, (uint64_t)t4);
    double rb = (double)get_timestamp_difference_u64((uint64_t)t8, (uint64_t)t5);
    double db = (double)get_timestamp_difference_u64((uint64_t)t7, (uint64_t)t6);
    double denom = ra + rb + da + db;

    if (denom <= 0.0) {
        return false;
    }

    double tof_ticks = ((ra * rb) - (da * db)) / denom;
    double distance = tof_ticks * UWB_APP_DW_TIME_UNIT * UWB_APP_SPEED_OF_LIGHT -
                      UWB_APP_ANT_DELAY_COMP_M;

    memset(out, 0, sizeof(*out));
    out->anchor_id       = cur->anchor_id;
    out->tag_id          = g_app_cfg.short_addr;
    out->exchange_seq    = cur->exchange_seq;
    out->response_slot_id = cur->response_slot_id;
    out->status_flags    = cur->status_flags;
    out->distance_m      = distance;
    out->range_quality   = (int16_t)cur->quality.rx_pacc;
    out->retry_count     = cur->retry_count;
    out->tag_tx_ts       = cur->tag_tx_ts;
    out->anchor_rx_ts    = cur->anchor_rx_ts;
    out->anchor_tx_ts    = cur->anchor_tx_ts;
    out->tag_rx_ts       = cur->tag_rx_ts;
    return true;
}

/* ================================================================
 *  SS-TWR 单次测距 (退化模式)
 *  ToF = ((t4-t1) - (t3-t2)) / 2
 * ================================================================ */
static bool compute_range_ss(const UwbTwrExchange *cur, UwbRangeResult *out)
{
    if (cur == NULL || out == NULL) return false;

    uint64_t t1 = cur->tag_tx_ts;
    uint64_t t2 = cur->anchor_rx_ts;
    uint64_t t3 = cur->anchor_tx_ts;
    uint64_t t4 = cur->tag_rx_ts;

    uint64_t t_round = get_timestamp_difference_u64(t4, t1);
    uint64_t t_reply = get_timestamp_difference_u64(t3, t2);
    if (t_round <= t_reply) return false;

    int64_t tof_ticks = (int64_t)(t_round - t_reply) / 2;
    if (tof_ticks < 0) return false;

    double distance = (double)tof_ticks * UWB_APP_DW_TIME_UNIT * UWB_APP_SPEED_OF_LIGHT -
                      UWB_APP_ANT_DELAY_COMP_M;

    memset(out, 0, sizeof(*out));
    out->anchor_id       = cur->anchor_id;
    out->tag_id          = g_app_cfg.short_addr;
    out->exchange_seq    = cur->exchange_seq;
    out->response_slot_id = cur->response_slot_id;
    out->status_flags    = cur->status_flags;
    out->distance_m      = distance;
    out->range_quality   = (int16_t)cur->quality.rx_pacc;
    out->retry_count     = cur->retry_count;
    out->tag_tx_ts       = cur->tag_tx_ts;
    out->anchor_rx_ts    = cur->anchor_rx_ts;
    out->anchor_tx_ts    = cur->anchor_tx_ts;
    out->tag_rx_ts       = cur->tag_rx_ts;
    return true;
}

static void publish_range_result(const UwbRangeResult *result)
{
    if (result == NULL) {
        return;
    }

    AppDataNode node;
    memset(&node, 0, sizeof(node));
    node.source = APP_DATA_SRC_UWB;
    (void)TimeService_GetTimestamp(&node.timestamp);
    node.payload.uwb.anchor_id       = result->anchor_id;
    node.payload.uwb.tag_id          = result->tag_id;
    node.payload.uwb.exchange_seq    = result->exchange_seq;
    node.payload.uwb.response_slot_id= result->response_slot_id;
    node.payload.uwb.status_flags    = result->status_flags;
    node.payload.uwb.distance_m      = result->distance_m;
    node.payload.uwb.range_quality   = result->range_quality;
    node.payload.uwb.retry_count     = result->retry_count;
    node.payload.uwb.tag_tx_ts       = result->tag_tx_ts;
    node.payload.uwb.anchor_rx_ts    = result->anchor_rx_ts;
    node.payload.uwb.anchor_tx_ts    = result->anchor_tx_ts;
    node.payload.uwb.tag_rx_ts       = result->tag_rx_ts;

    if (!DataService_Send(&node, 0)) {
        app_log_warn("UWB data queue full");
    }
}

/* ---- 查找或分配 Anchor 记录槽位 ---- */
static twr_anchor_record_t *find_anchor_record(uint16_t anchor_id)
{
    for (int i = 0; i < TWR_MAX_ANCHORS; i++) {
        if (g_anchor_records[i].valid && g_anchor_records[i].anchor_id == anchor_id) {
            return &g_anchor_records[i];
        }
    }
    for (int i = 0; i < TWR_MAX_ANCHORS; i++) {
        if (!g_anchor_records[i].valid) {
            g_anchor_records[i].valid     = true;
            g_anchor_records[i].anchor_id = anchor_id;
            g_anchor_records[i].anchor_tx_ts = 0;
            g_anchor_records[i].tag_rx_ts    = 0;
            return &g_anchor_records[i];
        }
    }
    return NULL;
}

static void handle_twr_exchange(const UwbTwrExchange *exchange)
{
    if (exchange == NULL || exchange->tag_tx_ts == 0U) {
        return;
    }

    twr_anchor_record_t *rec = find_anchor_record(exchange->anchor_id);
    UwbRangeResult result;
    twr_mode_t mode;

    if (rec != NULL && rec->anchor_tx_ts != 0) {
        /*
         * 滑窗 DS-TWR (标准公式消除钟差):
         *   prev: t3=rec->anchor_tx_ts, t4=rec->tag_rx_ts
         *   cur:  t5=exchange->tag_tx_ts, t6=exchange->anchor_rx_ts
         *         t7=exchange->anchor_tx_ts, t8=exchange->tag_rx_ts
         */
        UwbTwrExchange prev;
        memset(&prev, 0, sizeof(prev));
        prev.anchor_id    = rec->anchor_id;
        prev.anchor_tx_ts = rec->anchor_tx_ts;
        prev.tag_rx_ts    = rec->tag_rx_ts;

        if (compute_range(&prev, exchange, &result)) {
            mode = TWR_MODE_DS;
            result.status_flags |= 0x01;  /* bit0 = DS-TWR 标记 */
            publish_range_result(&result);
            app_log_info("[APP] DS-TWR anchor=0x%04X dist=%.2fm",
                         exchange->anchor_id, result.distance_m);
        } else {
            mode = TWR_MODE_SS;
        }
    } else {
        mode = TWR_MODE_SS;
    }

    /* SS-TWR 退化: 首次交换或 DS 计算失败 */
    if (mode == TWR_MODE_SS) {
        if (compute_range_ss(exchange, &result)) {
            publish_range_result(&result);
            app_log_info("[APP] SS-TWR anchor=0x%04X dist=%.2fm",
                         exchange->anchor_id, result.distance_m);
        }
    }

    /* 更新 Anchor 记录: 保存本次交换的 anchor_tx/ts 和 tag_rx/ts */
    if (rec != NULL) {
        rec->anchor_tx_ts = exchange->anchor_tx_ts;
        rec->tag_rx_ts    = exchange->tag_rx_ts;
    }
}

bool UwbApp_Init(const UwbStackConfig *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    g_app_cfg = *cfg;
    memset(g_anchor_records, 0, sizeof(g_anchor_records));
    return true;
}

bool UwbApp_StartThread(const osThreadAttr_t *attr)
{
    if (g_uwb_app_task != NULL) {
        return true;
    }

    g_uwb_app_task = osThreadNew(UwbApp_Task, NULL, attr);
    return g_uwb_app_task != NULL;
}

void UwbApp_Task(void *argument)
{
    (void)argument;

    QueueHandle_t queue = UwbLink_AppEventQueue();
    UwbLinkAppEvent event;

    for (;;) {
        if (queue == NULL ||
            xQueueReceive(queue, &event, portMAX_DELAY) != pdPASS) {
            continue;
        }

        switch (event.type) {
            case UWB_LINK_APP_EVT_TWR_EXCHANGE:
                handle_twr_exchange(&event.data.twr);
                break;

            case UWB_LINK_APP_EVT_NEIGHBOR_SEEN:
                app_log_info("UWB neighbor short=0x%04X cap=0x%04X",
                             event.data.neighbor.short_id,
                             event.data.neighbor.capability);
                break;

            case UWB_LINK_APP_EVT_NONE:
            default:
                break;
        }
    }
}