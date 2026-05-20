#include "uwb_loss_test.h"

#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "main.h"
#include "../service/log_service.h"
#include "../task/app_tasks.h"

#define LOSS_WINDOW_MS        3000U
#define LOSS_BUCKET_MS        500U
#define LOSS_BUCKET_COUNT     (LOSS_WINDOW_MS / LOSS_BUCKET_MS)
#define LOSS_WARMUP_MS        1000U
#define LOSS_PEER_MAX         8U
#define LOSS_AGG_PEER_MAX     (LOSS_PEER_MAX * LOSS_BUCKET_COUNT)
#define LOSS_EVENT_QUEUE_LEN  64U

typedef enum {
    LOSS_TEST_WARMUP = 0,
    LOSS_TEST_COUNTING,
} LossTestState;

typedef enum {
    LOSS_EVT_RANGE_TX = 0,
    LOSS_EVT_RANGE_RX,
} LossEventKind;

typedef struct {
    LossEventKind kind;
    uint8_t seq;
    uint16_t anchor_id;
    uint8_t response_slot_id;
} LossEvent;

typedef struct {
    bool valid;
    uint16_t anchor_id;
    uint8_t response_slot_id;
    uint32_t rx_count;
} LossPeer;

typedef struct {
    uint32_t start_ms;
    uint32_t tx_count;
    LossPeer peers[LOSS_PEER_MAX];
} LossBucket;

static StaticQueue_t g_loss_event_queue_ctrl;
static uint8_t g_loss_event_queue_buf[LOSS_EVENT_QUEUE_LEN * sizeof(LossEvent)];
static QueueHandle_t g_loss_event_queue;

static TaskHandle_t g_loss_task;

static LossTestState g_state = LOSS_TEST_WARMUP;
static uint32_t g_phase_started_ms;
static uint32_t g_counting_started_ms;
static LossBucket g_buckets[LOSS_BUCKET_COUNT];
static uint8_t g_cur_bucket;

static void clear_bucket(LossBucket *bucket, uint32_t start_ms)
{
    if (bucket == NULL) {
        return;
    }

    memset(bucket, 0, sizeof(*bucket));
    bucket->start_ms = start_ms;
}

static void reset_buckets(void)
{
    memset(g_buckets, 0, sizeof(g_buckets));
    g_counting_started_ms = 0U;
    g_cur_bucket = 0U;
}

static LossPeer *find_or_alloc_peer(LossPeer *peers,
                                    uint32_t peer_count,
                                    uint16_t anchor_id,
                                    uint8_t response_slot_id)
{
    if (peers == NULL) {
        return NULL;
    }

    for (uint32_t i = 0; i < peer_count; i++) {
        if (peers[i].valid &&
            peers[i].anchor_id == anchor_id &&
            peers[i].response_slot_id == response_slot_id) {
            return &peers[i];
        }
    }

    for (uint32_t i = 0; i < peer_count; i++) {
        if (!peers[i].valid) {
            memset(&peers[i], 0, sizeof(peers[i]));
            peers[i].valid = true;
            peers[i].anchor_id = anchor_id;
            peers[i].response_slot_id = response_slot_id;
            return &peers[i];
        }
    }

    return NULL;
}

static void calc_led_outputs(LedLossMode mode,
                             bool valid,
                             uint8_t rate,
                             uint8_t *led0,
                             uint8_t *led1,
                             uint8_t *led2)
{
    uint8_t out0 = 0U;
    uint8_t out1 = 0U;
    uint8_t out2 = 0U;

    if (mode == LED_LOSS_MODE_CONFIRM) {
        out0 = 1U;
        out1 = 1U;
        out2 = 1U;
    } else if (mode == LED_LOSS_MODE_RATE && valid) {
        out0 = (rate >= 80U) ? 1U : 0U;
        out1 = (rate >= 90U) ? 1U : 0U;
        out2 = (rate >= 95U) ? 1U : 0U;
    }

    if (led0 != NULL) {
        *led0 = out0;
    }
    if (led1 != NULL) {
        *led1 = out1;
    }
    if (led2 != NULL) {
        *led2 = out2;
    }
}

static bool send_led_cmd(LedLossMode mode,
                         bool valid,
                         uint8_t rate,
                         uint8_t led0,
                         uint8_t led1,
                         uint8_t led2)
{
    LedLossCmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.mode = mode;
    cmd.valid = valid ? 1U : 0U;
    cmd.rate = rate;
    cmd.led0 = led0;
    cmd.led1 = led1;
    cmd.led2 = led2;

    return AppTasks_SendLedLossCmd(&cmd);
}

static void start_loss_test(uint32_t now)
{
    reset_buckets();
    g_state = LOSS_TEST_WARMUP;
    g_phase_started_ms = now;
    (void)send_led_cmd(LED_LOSS_MODE_CONFIRM, false, 0U, 1U, 1U, 1U);
}

static uint8_t calc_display_rate(bool *valid)
{
    LossPeer total_peers[LOSS_AGG_PEER_MAX];
    uint32_t total_tx = 0U;
    uint32_t best_rate = 0U;

    memset(total_peers, 0, sizeof(total_peers));

    if (valid != NULL) {
        *valid = false;
    }

    for (uint32_t bucket_idx = 0; bucket_idx < LOSS_BUCKET_COUNT; bucket_idx++) {
        const LossBucket *bucket = &g_buckets[bucket_idx];
        total_tx += bucket->tx_count;

        for (uint32_t peer_idx = 0; peer_idx < LOSS_PEER_MAX; peer_idx++) {
            const LossPeer *src = &bucket->peers[peer_idx];
            LossPeer *dst;

            if (!src->valid) {
                continue;
            }

            dst = find_or_alloc_peer(total_peers,
                                     LOSS_AGG_PEER_MAX,
                                     src->anchor_id,
                                     src->response_slot_id);
            if (dst == NULL) {
                continue;
            }

            if (UINT32_MAX - dst->rx_count < src->rx_count) {
                dst->rx_count = UINT32_MAX;
            } else {
                dst->rx_count += src->rx_count;
            }
        }
    }

    if (total_tx == 0U) {
        return 0U;
    }

    if (valid != NULL) {
        *valid = true;
    }

    for (uint32_t i = 0; i < LOSS_AGG_PEER_MAX; i++) {
        if (!total_peers[i].valid) {
            continue;
        }

        uint32_t rate = (total_peers[i].rx_count * 100U) / total_tx;
        if (rate > best_rate) {
            best_rate = rate;
        }
    }

    if (best_rate > 100U) {
        best_rate = 100U;
    }

    return (uint8_t)best_rate;
}

static void update_led_from_window(void)
{
    bool valid = false;
    uint8_t rate = calc_display_rate(&valid);
    uint8_t led0 = 0U;
    uint8_t led1 = 0U;
    uint8_t led2 = 0U;

    calc_led_outputs(LED_LOSS_MODE_RATE, valid, rate, &led0, &led1, &led2);

    bool queued = send_led_cmd(LED_LOSS_MODE_RATE,
                               valid,
                               rate,
                               led0,
                               led1,
                               led2);

    LogService_Write(APP_LOG_INFO,
                     "[LOSS] rx_rate=%u%% led_cmd={mode=RATE valid=%u rate=%u%% led0=%u led1=%u led2=%u} queued=%u",
                     (unsigned)rate,
                     valid ? 1U : 0U,
                     (unsigned)rate,
                     (unsigned)led0,
                     (unsigned)led1,
                     (unsigned)led2,
                     queued ? 1U : 0U);
}

static void begin_counting(uint32_t now)
{
    reset_buckets();
    g_state = LOSS_TEST_COUNTING;
    g_counting_started_ms = now;
    clear_bucket(&g_buckets[0], now);
}

static void advance_state(uint32_t now)
{
    if (g_state == LOSS_TEST_WARMUP) {
        if ((uint32_t)(now - g_phase_started_ms) >= LOSS_WARMUP_MS) {
            begin_counting(now);
        }
        return;
    }

    if (g_state != LOSS_TEST_COUNTING) {
        return;
    }

    while ((uint32_t)(now - g_buckets[g_cur_bucket].start_ms) >= LOSS_BUCKET_MS) {
        uint32_t bucket_end_ms = g_buckets[g_cur_bucket].start_ms + LOSS_BUCKET_MS;

        if ((uint32_t)(bucket_end_ms - g_counting_started_ms) >= LOSS_WINDOW_MS) {
            update_led_from_window();
        }

        g_cur_bucket = (uint8_t)((g_cur_bucket + 1U) % LOSS_BUCKET_COUNT);
        clear_bucket(&g_buckets[g_cur_bucket], bucket_end_ms);
    }
}

static void process_event(const LossEvent *evt)
{
    LossBucket *bucket;

    if (evt == NULL) {
        return;
    }

    if (g_state != LOSS_TEST_COUNTING) {
        return;
    }

    bucket = &g_buckets[g_cur_bucket];

    if (evt->kind == LOSS_EVT_RANGE_TX) {
        if (bucket->tx_count < UINT32_MAX) {
            bucket->tx_count++;
        }
        return;
    }

    if (evt->kind == LOSS_EVT_RANGE_RX) {
        LossPeer *peer = find_or_alloc_peer(bucket->peers,
                                            LOSS_PEER_MAX,
                                             evt->anchor_id,
                                             evt->response_slot_id);
        if (peer == NULL) {
            return;
        }

        if (peer->rx_count < UINT32_MAX) {
            peer->rx_count++;
        }
    }
}

bool UwbLossTest_Init(void)
{
    if (g_loss_event_queue == NULL) {
        g_loss_event_queue = xQueueCreateStatic(
            LOSS_EVENT_QUEUE_LEN,
            sizeof(LossEvent),
            g_loss_event_queue_buf,
            &g_loss_event_queue_ctrl);
    }

    reset_buckets();
    g_state = LOSS_TEST_WARMUP;
    g_phase_started_ms = 0U;

    return g_loss_event_queue != NULL;
}

bool UwbLossTest_StartThread(const osThreadAttr_t *attr)
{
    if (g_loss_task != NULL) return true;
    g_loss_task = osThreadNew(UwbLossTest_Task, NULL, attr);
    return g_loss_task != NULL;
}

bool UwbLossTest_PostRangeTx(uint8_t seq)
{
    if (g_loss_event_queue == NULL) return false;

    LossEvent evt;
    memset(&evt, 0, sizeof(evt));
    evt.kind = LOSS_EVT_RANGE_TX;
    evt.seq = seq;
    return xQueueSend(g_loss_event_queue, &evt, 0) == pdPASS;
}

bool UwbLossTest_PostRangeRx(uint8_t seq,
                             uint16_t anchor_id,
                             uint8_t response_slot_id)
{
    if (g_loss_event_queue == NULL) return false;

    LossEvent evt;
    memset(&evt, 0, sizeof(evt));
    evt.kind = LOSS_EVT_RANGE_RX;
    evt.seq = seq;
    evt.anchor_id = anchor_id;
    evt.response_slot_id = response_slot_id;
    return xQueueSend(g_loss_event_queue, &evt, 0) == pdPASS;
}

void UwbLossTest_Task(void *argument)
{
    (void)argument;

    start_loss_test(HAL_GetTick());

    for (;;) {
        uint32_t now = HAL_GetTick();
        LossEvent evt;

        advance_state(now);

        if (g_loss_event_queue != NULL &&
            xQueueReceive(g_loss_event_queue, &evt, pdMS_TO_TICKS(10U)) == pdPASS) {
            now = HAL_GetTick();
            advance_state(now);
            process_event(&evt);

            while (xQueueReceive(g_loss_event_queue, &evt, 0) == pdPASS) {
                now = HAL_GetTick();
                advance_state(now);
                process_event(&evt);
            }
        }
        
        advance_state(HAL_GetTick());
    }
}
