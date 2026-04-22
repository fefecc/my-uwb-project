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

typedef struct {
    bool valid;
    UwbTwrExchange exchange;
} UwbExchangeCache;

static UwbStackConfig g_app_cfg;
static UwbExchangeCache g_last_exchange;
static TaskHandle_t g_uwb_app_task;

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
    out->anchor_id = cur->anchor_id;
    out->tag_id = g_app_cfg.short_addr;
    out->exchange_seq = cur->exchange_seq;
    out->response_slot_id = cur->response_slot_id;
    out->status_flags = cur->status_flags;
    out->distance_m = distance;
    out->range_quality = (int16_t)cur->quality.rx_pacc;
    out->retry_count = cur->retry_count;
    out->tag_tx_ts = cur->tag_tx_ts;
    out->anchor_rx_ts = cur->anchor_rx_ts;
    out->anchor_tx_ts = cur->anchor_tx_ts;
    out->tag_rx_ts = cur->tag_rx_ts;
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
    node.payload.uwb.anchor_id = result->anchor_id;
    node.payload.uwb.tag_id = result->tag_id;
    node.payload.uwb.exchange_seq = result->exchange_seq;
    node.payload.uwb.response_slot_id = result->response_slot_id;
    node.payload.uwb.status_flags = result->status_flags;
    node.payload.uwb.distance_m = result->distance_m;
    node.payload.uwb.range_quality = result->range_quality;
    node.payload.uwb.retry_count = result->retry_count;
    node.payload.uwb.tag_tx_ts = result->tag_tx_ts;
    node.payload.uwb.anchor_rx_ts = result->anchor_rx_ts;
    node.payload.uwb.anchor_tx_ts = result->anchor_tx_ts;
    node.payload.uwb.tag_rx_ts = result->tag_rx_ts;

    if (!DataService_Send(&node, 0)) {
        app_log_warn("UWB data queue full");
    }
}

static void handle_twr_exchange(const UwbTwrExchange *exchange)
{
    if (exchange == NULL || exchange->tag_tx_ts == 0U) {
        return;
    }

    UwbRangeResult result;
    if (g_last_exchange.valid &&
        compute_range(&g_last_exchange.exchange, exchange, &result)) {
        publish_range_result(&result);
    }

    g_last_exchange.exchange = *exchange;
    g_last_exchange.valid = true;
}

bool UwbApp_Init(const UwbStackConfig *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    g_app_cfg = *cfg;
    memset(&g_last_exchange, 0, sizeof(g_last_exchange));
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
