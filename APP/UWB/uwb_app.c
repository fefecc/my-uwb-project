/**
 * @file uwb_app.c
 * @brief APP 层 - TWR 测距计算 + plan-v4 数据会话
 */

#include "uwb_app.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "uwb_link.h"
#include "uwb_protocol.h"
#include "uwb_slots.h"
#include "uwb_buffers.h"
#include "uwb_timestamp.h"
#include "../service/data_service.h"
#include "../service/log_service.h"
#include "../service/time_service.h"
#include "../task/app_tasks.h"
#include "main.h"

#define UWB_APP_DW_TIME_UNIT      (1.0 / (499.2e6 * 128.0))
#define UWB_APP_SPEED_OF_LIGHT    (299702547.0)
#define UWB_APP_ANT_DELAY_COMP_M  (0.0)

#define TWR_MAX_ANCHORS  8
#define TWR_DS_MAX_GAP_MS  12U
#define TWR_DS_MAX_GAP_TICKS \
    ((uint64_t)((TIME_SERVICE_LOCAL_TICKS_PER_SECOND * TWR_DS_MAX_GAP_MS) / 1000U))
#define TWR_DS_DROP_GAP_MS  30U
#define TWR_DS_DROP_GAP_TICKS \
    ((uint64_t)((TIME_SERVICE_LOCAL_TICKS_PER_SECOND * TWR_DS_DROP_GAP_MS) / 1000U))

#define DATA_BUF_SIZE              512U
#define DATA_WAIT_RETRY_MS         10U
#define DATA_SESSION_TIMEOUT_MS    5000U
#define DATA_META_PAYLOAD_LEN      6U
#define DATA_FRAG_CRC_LEN          2U
#define TAG_PULL_MAX_ANCHORS       TWR_MAX_ANCHORS
#define TAG_PULL_TRIGGER_DIST_M    (50.0)

#define PROX_ANCHOR_ID_MIN         (0x0020U)
#define PROX_ANCHOR_ID_MAX         (0x0050U)
#define PROX_PEER_MAX              (8U)
#define PROX_TABLE_MAX_ENTRIES     PROX_PEER_MAX
#define PROX_BUILD_WINDOW_MS       (3000U)
#define PROX_DISCOVERY_INTERVAL_MS (20U)
#define PROX_TABLE_VERSION         (1U)
#define PROX_MIN_VALID_SAMPLES     (8U)
#define PROX_TABLE_HEADER_LEN      (8U)
#define PROX_TABLE_ENTRY_LEN       (35U)
#define PROX_HEX_DUMP_ENABLED      (0U)
#define PROX_RING_START_DELAY_MS   (30U)
#define PROX_RING_NOTIFY_RETRY_MS  (10U)
#define PROX_ENTRY_FLAG_VALID      (1U << 0)
#define PROX_ENTRY_FLAG_LOW_SAMPLE (1U << 1)
#define PROX_ENTRY_FLAG_RX_ERROR   (1U << 2)

#define PROX_SCORE_PACC_REF        (1024U)
#define PROX_SCORE_FP_AMPL_REF     (4096U)
#define PROX_SCORE_STD_NOISE_GOOD  (128U)
#define PROX_SCORE_STD_NOISE_BAD   (1024U)
#define PROX_SCORE_MAX_NOISE_GOOD  (256U)
#define PROX_SCORE_MAX_NOISE_BAD   (2048U)
#define PROX_SCORE_RX_POWER_GOOD   (-5500)
#define PROX_SCORE_RX_POWER_BAD    (-9500)
#define PROX_SCORE_FP_POWER_GOOD   (-6000)
#define PROX_SCORE_FP_POWER_BAD    (-10000)
#define PROX_SCORE_FP_INDEX_STD_GOOD (16U)
#define PROX_SCORE_FP_INDEX_STD_BAD  (128U)

typedef struct {
    bool     valid;
    uint16_t anchor_id;
    bool     have_prev_exchange;
    uint64_t anchor_tx_ts;
    uint64_t tag_rx_ts;
    uint64_t tag_rx_local_tick_20k;
    uint64_t last_published_frame_local_tick_20k;
} twr_anchor_record_t;

typedef enum {
    TAG_DATA_IDLE = 0,
    TAG_DATA_WAIT_CFG_ACK,
    TAG_DATA_WAIT_META,
    TAG_DATA_PULLING,
    TAG_DATA_WAIT_DONE_ACK,
} tag_data_state_t;

typedef struct {
    tag_data_state_t state;
    uint16_t target_id;
    uint16_t session_id;
    uint16_t total_len;
    uint16_t total_crc;
    uint8_t  total_frags;
    uint8_t  next_frag;
    uint32_t session_started_ms;
    uint32_t retry_at_ms;
    uint32_t cmd_count;
    uint32_t ack_count;
    uint32_t wait_count;
    uint32_t frag_count;
    uint32_t drop_count;
    bool     cmd_pending;
    bool     completion_logged;
    UwbLinkCmd current_cmd;
} tag_data_context_t;

typedef struct {
    bool     valid;
    bool     started;
    uint16_t anchor_id;
    double   trigger_distance_m;
    uint32_t trigger_ms;
} tag_pull_record_t;

typedef struct {
    bool     active;
    uint16_t tag_id;
    uint16_t session_id;
    uint8_t  total_frags;
    uint32_t last_ms;
    uint32_t cfg_count;
    uint32_t ctrl_count;
    uint32_t pull_count;
    uint32_t frag_ready_count;
    uint32_t drop_count;
} anchor_data_context_t;

typedef enum {
    PROX_BUILD_IDLE = 0,
    PROX_BUILD_RUNNING,
} prox_build_state_t;

typedef struct {
    bool     valid;
    uint16_t peer_anchor;
    uint16_t sample_count;
    uint16_t valid_count;
    uint32_t pacc_sum;
    uint32_t fp_index_sum;
    uint64_t fp_index_sq_sum;
    uint32_t fp_ampl1_sum;
    uint32_t fp_ampl2_sum;
    uint32_t fp_ampl3_sum;
    uint32_t std_noise_sum;
    uint32_t max_noise_sum;
    int32_t  rx_power_dbm_x100_sum;
    int32_t  fp_power_dbm_x100_sum;
    uint16_t lde_status_or;
    uint32_t rx_error_flags_or;
    double   distance_sum_m;
    double   distance_sq_sum_m;
    uint32_t last_seen_ms;
} prox_peer_accum_t;

static UwbStackConfig g_app_cfg;
static twr_anchor_record_t g_anchor_records[TWR_MAX_ANCHORS];
static TaskHandle_t g_uwb_app_task;

static uint8_t  g_tx_buf[DATA_BUF_SIZE];
static uint16_t g_tx_len;
static uint8_t  g_rx_buf[DATA_BUF_SIZE];
static uint16_t g_rx_len;

static uint16_t g_next_session_id = 1U;
static tag_data_context_t g_tag_data;
static tag_pull_record_t g_tag_pull_records[TAG_PULL_MAX_ANCHORS];
static anchor_data_context_t g_anchor_data;
static volatile prox_build_state_t g_prox_state;
static volatile bool g_prox_request_pending;
static uint16_t g_prox_init_seq;
static uint32_t g_prox_started_ms;
static uint32_t g_prox_next_disc_ms;
static uint32_t g_prox_disc_request_count;
static uint32_t g_prox_disc_queued_count;
static uint16_t g_prox_ring_next_anchor;
static uint16_t g_prox_ring_origin_anchor;
static uint16_t g_prox_ring_token_seq;
static uint16_t g_prox_ring_prev_anchor;
static bool g_prox_ring_start_pending;
static uint32_t g_prox_ring_start_ms;
static bool g_prox_ring_notify_pending;
static uint16_t g_prox_ring_notify_target;
static uint32_t g_prox_ring_notify_next_ms;
static prox_peer_accum_t g_prox_peers[PROX_TABLE_MAX_ENTRIES];

/* ================================================================
 *  工具
 * ================================================================ */

static uint16_t min_u16(uint16_t a, uint16_t b)
{
    return (a < b) ? a : b;
}

static size_t local_bounded_strlen(const char *text, size_t max_len)
{
    size_t len = 0U;

    if (text == NULL) {
        return 0U;
    }

    while (len < max_len && text[len] != '\0') {
        len++;
    }

    return len;
}

static void prox_log_direct(const char *text)
{
    if (text == NULL) {
        return;
    }

    AppTasks_LogWriteText(text, local_bounded_strlen(text, 192U));
}

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8U; bit++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static uint8_t calc_total_frags(uint16_t len)
{
    if (len == 0U) return 0U;
    return (uint8_t)((len + UWB_DATA_FRAG_PAYLOAD_SIZE - 1U) /
                     UWB_DATA_FRAG_PAYLOAD_SIZE);
}

static void prox_prepare_empty_tx_table(void)
{
    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    g_tx_buf[0] = PROX_TABLE_VERSION;
    g_tx_buf[1] = 0U;
    UwbProtocol_WriteLe16(&g_tx_buf[2], g_app_cfg.short_addr);
    UwbProtocol_WriteLe16(&g_tx_buf[4], g_prox_init_seq);
    UwbProtocol_WriteLe16(&g_tx_buf[6], 0U);
    UwbProtocol_WriteLe16(&g_tx_buf[6],
                          crc16_ccitt(g_tx_buf, PROX_TABLE_HEADER_LEN));
    g_tx_len = PROX_TABLE_HEADER_LEN;
    g_anchor_data.total_frags = calc_total_frags(g_tx_len);
}

static void log_twr_frame(const char *mode,
                          const UwbTwrExchange *exchange,
                          const UwbRangeResult *result)
{
    if (mode == NULL || exchange == NULL || result == NULL) return;

    double frame_local_ms =
        ((double)result->frame_local_tick_20k * 1000.0) /
        (double)TIME_SERVICE_LOCAL_TICKS_PER_SECOND;

    app_log_info("[APP] TWR_FRAME mode=%s anchor=0x%04X seq=%u slot=%u dist=%.2fm pacc=%u frame_local_ms=%.3f",
                 mode,
                 exchange->anchor_id,
                 (unsigned)exchange->exchange_seq,
                 (unsigned)exchange->response_slot_id,
                 result->distance_m,
                 (unsigned)result->quality.rx_pacc,
                 frame_local_ms);
}

static double local_tick_to_ms(uint64_t local_tick_20k)
{
    return ((double)local_tick_20k * 1000.0) /
           (double)TIME_SERVICE_LOCAL_TICKS_PER_SECOND;
}

typedef enum {
    TWR_GAP_SHORT = 0,
    TWR_GAP_LONG,
    TWR_GAP_DROP,
} twr_gap_class_t;

static bool twr_get_published_gap_info(uint64_t last_published_frame_local_tick_20k,
                                       uint64_t candidate_frame_local_tick_20k,
                                       uint64_t *gap_ticks,
                                       twr_gap_class_t *gap_class)
{
    if (gap_ticks == NULL || gap_class == NULL ||
        candidate_frame_local_tick_20k == 0U) {
        return false;
    }

    if (last_published_frame_local_tick_20k == 0U) {
        *gap_ticks = 0U;
        *gap_class = TWR_GAP_SHORT;
        return true;
    }

    if (candidate_frame_local_tick_20k < last_published_frame_local_tick_20k) {
        return false;
    }

    *gap_ticks = candidate_frame_local_tick_20k -
                 last_published_frame_local_tick_20k;
    if (*gap_ticks <= TWR_DS_MAX_GAP_TICKS) {
        *gap_class = TWR_GAP_SHORT;
    } else if (*gap_ticks <= TWR_DS_DROP_GAP_TICKS) {
        *gap_class = TWR_GAP_LONG;
    } else {
        *gap_class = TWR_GAP_DROP;
    }
    return true;
}

/* ================================================================
 *  DS/SS-TWR 测距计算
 * ================================================================ */

static bool compute_range(const UwbTwrExchange *prev,
                          const UwbTwrExchange *cur,
                          UwbRangeResult *out)
{
    if (prev == NULL || cur == NULL || out == NULL ||
        prev->anchor_id != cur->anchor_id) {
        app_log_warn("[APP] TWR_DBG_DS_FAIL reason=invalid_input");
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
        app_log_warn("[APP] TWR_DBG_DS_FAIL anchor=0x%04X seq=%u reason=denom ra=%.0f da=%.0f rb=%.0f db=%.0f denom=%.0f",
                     cur->anchor_id,
                     (unsigned)cur->exchange_seq,
                     ra, da, rb, db, denom);
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
    out->quality         = cur->quality;
    out->retry_count     = cur->retry_count;
    out->tag_tx_ts       = cur->tag_tx_ts;
    out->anchor_rx_ts    = cur->anchor_rx_ts;
    out->anchor_tx_ts    = cur->anchor_tx_ts;
    out->tag_rx_ts       = cur->tag_rx_ts;
    out->frame_local_tick_20k =
        (prev->tag_rx_local_tick_20k / 2ULL) + (cur->tag_rx_local_tick_20k / 2ULL) +
        ((prev->tag_rx_local_tick_20k & 1ULL) &&
         (cur->tag_rx_local_tick_20k & 1ULL) ? 1ULL : 0ULL);
    return true;
}

static void publish_range_result(const UwbRangeResult *result)
{
    if (result == NULL) return;

    AppDataNode node;
    memset(&node, 0, sizeof(node));
    node.source = APP_DATA_SRC_UWB;
    (void)TimeService_CaptureNow(&node.time_capture);
    node.time_capture.local_tick_20k = result->frame_local_tick_20k;
    node.payload.uwb.anchor_id       = result->anchor_id;
    node.payload.uwb.tag_id          = result->tag_id;
    node.payload.uwb.exchange_seq    = result->exchange_seq;
    node.payload.uwb.response_slot_id= result->response_slot_id;
    node.payload.uwb.status_flags    = result->status_flags;
    node.payload.uwb.distance_m      = result->distance_m;
    node.payload.uwb.retry_count     = result->retry_count;
    node.payload.uwb.rx_pacc         = result->quality.rx_pacc;
    node.payload.uwb.fp_index        = result->quality.fp_index;
    node.payload.uwb.fp_ampl1        = result->quality.fp_ampl1;
    node.payload.uwb.fp_ampl2        = result->quality.fp_ampl2;
    node.payload.uwb.fp_ampl3        = result->quality.fp_ampl3;
    node.payload.uwb.std_noise       = result->quality.std_noise;
    node.payload.uwb.max_noise       = result->quality.max_noise;
    node.payload.uwb.tag_tx_ts       = result->tag_tx_ts;
    node.payload.uwb.anchor_rx_ts    = result->anchor_rx_ts;
    node.payload.uwb.anchor_tx_ts    = result->anchor_tx_ts;
    node.payload.uwb.tag_rx_ts       = result->tag_rx_ts;

    if (!DataService_Send(&node, 0)) {
        app_log_warn("UWB data queue full");
    }
}

static bool prox_anchor_id_in_range(uint16_t anchor_id)
{
    return anchor_id >= PROX_ANCHOR_ID_MIN &&
           anchor_id <= PROX_ANCHOR_ID_MAX;
}

static bool prox_peer_id_valid(uint16_t peer_anchor)
{
    return prox_anchor_id_in_range(peer_anchor) &&
           peer_anchor != g_app_cfg.short_addr;
}

static void prox_reset_peers(void)
{
    memset(g_prox_peers, 0, sizeof(g_prox_peers));
}

static prox_peer_accum_t *prox_find_or_alloc_peer(uint16_t peer_anchor)
{
    for (uint32_t i = 0; i < PROX_TABLE_MAX_ENTRIES; ++i) {
        if (g_prox_peers[i].valid &&
            g_prox_peers[i].peer_anchor == peer_anchor) {
            return &g_prox_peers[i];
        }
    }

    for (uint32_t i = 0; i < PROX_TABLE_MAX_ENTRIES; ++i) {
        if (!g_prox_peers[i].valid) {
            memset(&g_prox_peers[i], 0, sizeof(g_prox_peers[i]));
            g_prox_peers[i].valid = true;
            g_prox_peers[i].peer_anchor = peer_anchor;
            return &g_prox_peers[i];
        }
    }

    return NULL;
}

static uint8_t prox_compact_and_sort_peers(void)
{
    uint8_t count = 0U;

    for (uint32_t i = 0U; i < PROX_TABLE_MAX_ENTRIES; ++i) {
        if (!g_prox_peers[i].valid) {
            continue;
        }

        if (i != count) {
            g_prox_peers[count] = g_prox_peers[i];
            memset(&g_prox_peers[i], 0, sizeof(g_prox_peers[i]));
        }
        count++;
    }

    for (uint8_t i = 0U; i < count; ++i) {
        for (uint8_t j = (uint8_t)(i + 1U); j < count; ++j) {
            if (g_prox_peers[j].peer_anchor < g_prox_peers[i].peer_anchor) {
                prox_peer_accum_t tmp = g_prox_peers[i];
                g_prox_peers[i] = g_prox_peers[j];
                g_prox_peers[j] = tmp;
            }
        }
    }

    return count;
}

static uint16_t prox_avg_distance_cm(const prox_peer_accum_t *peer)
{
    if (peer == NULL || peer->valid_count == 0U) {
        return 0U;
    }

    double avg_m = peer->distance_sum_m / (double)peer->valid_count;
    if (avg_m <= 0.0) {
        return 0U;
    }

    double cm = avg_m * 100.0 + 0.5;
    if (cm >= 65535.0) {
        return UINT16_MAX;
    }

    return (uint16_t)cm;
}

static uint16_t prox_avg_pacc(const prox_peer_accum_t *peer)
{
    if (peer == NULL || peer->valid_count == 0U) {
        return 0U;
    }

    uint32_t avg = peer->pacc_sum / peer->valid_count;
    return avg > UINT16_MAX ? UINT16_MAX : (uint16_t)avg;
}

static uint16_t prox_avg_u32(uint32_t sum, uint16_t count)
{
    if (count == 0U) {
        return 0U;
    }

    uint32_t avg = sum / count;
    return avg > UINT16_MAX ? UINT16_MAX : (uint16_t)avg;
}

static int16_t prox_avg_i32(int32_t sum, uint16_t count)
{
    if (count == 0U) {
        return 0;
    }

    int32_t avg = sum / (int32_t)count;
    if (avg > INT16_MAX) {
        return INT16_MAX;
    }
    if (avg < INT16_MIN) {
        return INT16_MIN;
    }

    return (int16_t)avg;
}

static uint32_t prox_isqrt_u32(uint32_t value)
{
    uint32_t root = 0U;
    uint32_t bit = 1UL << 30;

    while (bit > value) {
        bit >>= 2;
    }

    while (bit != 0U) {
        if (value >= root + bit) {
            value -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }

    return root;
}

static uint16_t prox_distance_std_cm(const prox_peer_accum_t *peer)
{
    if (peer == NULL || peer->valid_count < 2U) {
        return 0U;
    }

    double count = (double)peer->valid_count;
    double avg_m = peer->distance_sum_m / count;
    double avg_sq_m = peer->distance_sq_sum_m / count;
    double variance_m2 = avg_sq_m - avg_m * avg_m;
    if (variance_m2 <= 0.0) {
        return 0U;
    }

    double variance_cm2 = variance_m2 * 10000.0 + 0.5;
    if (variance_cm2 >= (double)UINT32_MAX) {
        return UINT16_MAX;
    }

    uint32_t std_cm = prox_isqrt_u32((uint32_t)variance_cm2);
    return std_cm > UINT16_MAX ? UINT16_MAX : (uint16_t)std_cm;
}

static uint16_t prox_fp_index_std(const prox_peer_accum_t *peer)
{
    if (peer == NULL || peer->valid_count < 2U) {
        return 0U;
    }

    double count = (double)peer->valid_count;
    double avg = (double)peer->fp_index_sum / count;
    double avg_sq = (double)peer->fp_index_sq_sum / count;
    double variance = avg_sq - avg * avg;
    if (variance <= 0.0) {
        return 0U;
    }

    if (variance >= (double)UINT32_MAX) {
        return UINT16_MAX;
    }

    uint32_t std_value = prox_isqrt_u32((uint32_t)(variance + 0.5));
    return std_value > UINT16_MAX ? UINT16_MAX : (uint16_t)std_value;
}

static uint32_t prox_score_high_u16(uint16_t value, uint16_t ref)
{
    if (ref == 0U) {
        return 0U;
    }

    uint32_t capped = value > ref ? ref : value;
    return capped * 100U / ref;
}

static uint32_t prox_score_low_u16(uint16_t value, uint16_t good, uint16_t bad)
{
    if (value <= good) {
        return 100U;
    }
    if (value >= bad || bad <= good) {
        return 0U;
    }

    return 100U - ((uint32_t)(value - good) * 100U) / (uint32_t)(bad - good);
}

static uint32_t prox_score_power_x100(int16_t value, int32_t good, int32_t bad)
{
    if ((int32_t)value >= good) {
        return 100U;
    }
    if ((int32_t)value <= bad || good <= bad) {
        return 0U;
    }

    return (uint32_t)(((int32_t)value - bad) * 100L / (good - bad));
}

static uint8_t prox_quality(const prox_peer_accum_t *peer)
{
    if (peer == NULL || peer->valid_count == 0U) {
        return 0U;
    }

    uint32_t valid_count = peer->valid_count;
    uint32_t sample_score = (valid_count > 64U ? 64U : valid_count) * 100U / 64U;
    uint32_t dist_std_score = prox_score_low_u16(prox_distance_std_cm(peer),
                                                 20U,
                                                 100U);
    uint32_t avg_pacc = prox_avg_pacc(peer);
    uint32_t pacc_score = prox_score_high_u16((uint16_t)avg_pacc,
                                              PROX_SCORE_PACC_REF);
    uint32_t avg_fp_ampl =
        ((uint32_t)prox_avg_u32(peer->fp_ampl1_sum, peer->valid_count) +
         (uint32_t)prox_avg_u32(peer->fp_ampl2_sum, peer->valid_count) +
         (uint32_t)prox_avg_u32(peer->fp_ampl3_sum, peer->valid_count)) / 3U;
    uint32_t fp_ampl_score = prox_score_high_u16(
        avg_fp_ampl > UINT16_MAX ? UINT16_MAX : (uint16_t)avg_fp_ampl,
        PROX_SCORE_FP_AMPL_REF);
    uint32_t noise_score =
        (prox_score_low_u16(prox_avg_u32(peer->std_noise_sum, peer->valid_count),
                            PROX_SCORE_STD_NOISE_GOOD,
                            PROX_SCORE_STD_NOISE_BAD) +
         prox_score_low_u16(prox_avg_u32(peer->max_noise_sum, peer->valid_count),
                            PROX_SCORE_MAX_NOISE_GOOD,
                            PROX_SCORE_MAX_NOISE_BAD)) / 2U;
    uint32_t power_score =
        (prox_score_power_x100(prox_avg_i32(peer->rx_power_dbm_x100_sum,
                                            peer->valid_count),
                               PROX_SCORE_RX_POWER_GOOD,
                               PROX_SCORE_RX_POWER_BAD) +
         prox_score_power_x100(prox_avg_i32(peer->fp_power_dbm_x100_sum,
                                            peer->valid_count),
                               PROX_SCORE_FP_POWER_GOOD,
                               PROX_SCORE_FP_POWER_BAD)) / 2U;
    uint32_t fp_index_score = prox_score_low_u16(prox_fp_index_std(peer),
                                                 PROX_SCORE_FP_INDEX_STD_GOOD,
                                                 PROX_SCORE_FP_INDEX_STD_BAD);
    uint32_t quality = (sample_score * 10U +
                        dist_std_score * 20U +
                        pacc_score * 15U +
                        fp_ampl_score * 15U +
                        noise_score * 15U +
                        power_score * 15U +
                        fp_index_score * 10U) / 100U;

    if (valid_count < PROX_MIN_VALID_SAMPLES && quality > 60U) {
        quality = 60U;
    }
    if (peer->rx_error_flags_or != 0U && quality > 50U) {
        quality = 50U;
    }
    if (quality > 100U) {
        quality = 100U;
    }

    return (uint8_t)quality;
}

static void prox_log_hex(const uint8_t *buf, uint16_t len)
{
#if PROX_HEX_DUMP_ENABLED
    if (buf == NULL || len == 0U) {
        return;
    }

    for (uint16_t off = 0U; off < len; off = (uint16_t)(off + 16U)) {
        char line[80];
        uint16_t pos = 0U;
        uint16_t chunk = (uint16_t)(len - off);
        if (chunk > 16U) {
            chunk = 16U;
        }

        int wrote = snprintf(line, sizeof(line), "[PROX] hex %03u:", (unsigned)off);
        if (wrote < 0) {
            return;
        }
        pos = (uint16_t)wrote;

        for (uint16_t i = 0U; i < chunk && pos < sizeof(line); ++i) {
            wrote = snprintf(&line[pos], sizeof(line) - pos, " %02X", buf[off + i]);
            if (wrote < 0) {
                return;
            }
            pos = (uint16_t)(pos + (uint16_t)wrote);
        }

        app_log_info("%s", line);
    }
#else
    (void)buf;
    (void)len;
#endif
}

static void prox_log_peer_summary(uint8_t entry_count)
{
    char line[192];
    uint16_t pos = 0U;
    int wrote = snprintf(line, sizeof(line), "[PROX] peers count=%u ids=",
                         (unsigned)entry_count);

    if (wrote < 0) {
        return;
    }
    pos = (uint16_t)wrote;

    if (entry_count == 0U) {
        (void)snprintf(&line[pos], sizeof(line) - pos, "none");
    } else {
        for (uint8_t i = 0U; i < entry_count && pos < sizeof(line); ++i) {
            const prox_peer_accum_t *peer = &g_prox_peers[i];
            uint8_t quality = prox_quality(peer);
            wrote = snprintf(&line[pos],
                             sizeof(line) - pos,
                             "%s0x%04X:s%u/q%u",
                             i == 0U ? "" : ",",
                             peer->peer_anchor,
                             (unsigned)peer->valid_count,
                             (unsigned)quality);
            if (wrote < 0) {
                return;
            }
            pos = (uint16_t)(pos + (uint16_t)wrote);
        }
    }

    app_log_info("%s", line);
}

static uint16_t prox_select_ring_next(uint8_t entry_count)
{
    uint16_t best_after_self = 0U;
    uint16_t best_wrap = 0U;
    uint16_t self = g_app_cfg.short_addr;

    for (uint8_t i = 0U; i < entry_count; ++i) {
        uint16_t peer_anchor = g_prox_peers[i].peer_anchor;

        if (!g_prox_peers[i].valid || !prox_peer_id_valid(peer_anchor)) {
            continue;
        }

        if (peer_anchor > self &&
            (best_after_self == 0U || peer_anchor < best_after_self)) {
            best_after_self = peer_anchor;
        }

        if (best_wrap == 0U || peer_anchor < best_wrap) {
            best_wrap = peer_anchor;
        }
    }

    return best_after_self != 0U ? best_after_self : best_wrap;
}

static void prox_log_ring_next(uint8_t entry_count)
{
    g_prox_ring_next_anchor = prox_select_ring_next(entry_count);

    if (g_prox_ring_next_anchor != 0U) {
        app_log_info("[PROX] ring next self=0x%04X next=0x%04X range=0x%04X-0x%04X peers=%u",
                     g_app_cfg.short_addr,
                     g_prox_ring_next_anchor,
                     (unsigned)PROX_ANCHOR_ID_MIN,
                     (unsigned)PROX_ANCHOR_ID_MAX,
                     (unsigned)entry_count);
    } else {
        app_log_warn("[PROX] ring next none self=0x%04X range=0x%04X-0x%04X peers=%u",
                     g_app_cfg.short_addr,
                     (unsigned)PROX_ANCHOR_ID_MIN,
                     (unsigned)PROX_ANCHOR_ID_MAX,
                     (unsigned)entry_count);
    }
}

static uint16_t prox_next_ring_seq(void)
{
    uint16_t next = (uint16_t)(g_prox_init_seq + 1U);
    return next == 0U ? 1U : next;
}

static void prox_queue_ring_notify(uint16_t target_anchor)
{
    if (!prox_anchor_id_in_range(target_anchor) ||
        target_anchor == g_app_cfg.short_addr) {
        app_log_warn("[PROX] ring notify skip invalid self=0x%04X target=0x%04X",
                     g_app_cfg.short_addr, target_anchor);
        AppTasks_SetAnchorNotifyInitLed(false);
        return;
    }

    if (g_prox_ring_origin_anchor == 0U) {
        g_prox_ring_origin_anchor = g_app_cfg.short_addr;
    }
    if (g_prox_ring_token_seq == 0U) {
        g_prox_ring_token_seq = prox_next_ring_seq();
    }

    g_prox_ring_notify_pending = true;
    g_prox_ring_notify_target = target_anchor;
    g_prox_ring_notify_next_ms = HAL_GetTick();

    AppTasks_SetAnchorNotifyInitLed(true);
    app_log_info("[PROX] ring notify queue self=0x%04X next=0x%04X origin=0x%04X seq=%u",
                 g_app_cfg.short_addr,
                 target_anchor,
                 g_prox_ring_origin_anchor,
                 (unsigned)g_prox_ring_token_seq);
}

static void prox_ring_notify_poll(void)
{
    if (!g_prox_ring_notify_pending ||
        g_app_cfg.role != APP_ROLE_ANCHOR) {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - g_prox_ring_notify_next_ms) < 0) {
        return;
    }

    UwbLinkCmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = LINK_CMD_RING_INIT_NOTIFY;
    cmd.slot_index = -1;
    cmd.target_id = g_prox_ring_notify_target;
    cmd.origin_id = g_prox_ring_origin_anchor;
    cmd.session_id = g_prox_ring_token_seq;

    if (UwbLink_SendCmd(&cmd)) {
        app_log_info("[PROX] ring notify sent self=0x%04X next=0x%04X origin=0x%04X seq=%u",
                     g_app_cfg.short_addr,
                     cmd.target_id,
                     cmd.origin_id,
                     (unsigned)cmd.session_id);
        g_prox_ring_notify_pending = false;
    } else {
        g_prox_ring_notify_next_ms = now + PROX_RING_NOTIFY_RETRY_MS;
        app_log_warn("[PROX] ring notify cmd full self=0x%04X next=0x%04X",
                     g_app_cfg.short_addr,
                     g_prox_ring_notify_target);
    }
}

static void prox_finalize_table(void)
{
    uint8_t entry_count = prox_compact_and_sort_peers();
    uint16_t len = (uint16_t)(PROX_TABLE_HEADER_LEN +
                              entry_count * PROX_TABLE_ENTRY_LEN);
    uint16_t offset = PROX_TABLE_HEADER_LEN;

    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    g_tx_buf[0] = PROX_TABLE_VERSION;
    g_tx_buf[1] = entry_count;
    UwbProtocol_WriteLe16(&g_tx_buf[2], g_app_cfg.short_addr);
    UwbProtocol_WriteLe16(&g_tx_buf[4], g_prox_init_seq);
    UwbProtocol_WriteLe16(&g_tx_buf[6], 0U);

    if (entry_count == 0U) {
        app_log_warn("[PROX] no peer discovered");
    }

    for (uint32_t i = 0U; i < entry_count; ++i) {
        const prox_peer_accum_t *peer = &g_prox_peers[i];
        uint16_t dist_cm = prox_avg_distance_cm(peer);
        uint16_t dist_std_cm = prox_distance_std_cm(peer);
        uint16_t avg_pacc = prox_avg_pacc(peer);
        uint16_t avg_fp_index = prox_avg_u32(peer->fp_index_sum,
                                             peer->valid_count);
        uint16_t avg_fp_ampl1 = prox_avg_u32(peer->fp_ampl1_sum,
                                             peer->valid_count);
        uint16_t avg_fp_ampl2 = prox_avg_u32(peer->fp_ampl2_sum,
                                             peer->valid_count);
        uint16_t avg_fp_ampl3 = prox_avg_u32(peer->fp_ampl3_sum,
                                             peer->valid_count);
        uint16_t avg_std_noise = prox_avg_u32(peer->std_noise_sum,
                                              peer->valid_count);
        uint16_t avg_max_noise = prox_avg_u32(peer->max_noise_sum,
                                              peer->valid_count);
        int16_t avg_rx_power = prox_avg_i32(peer->rx_power_dbm_x100_sum,
                                            peer->valid_count);
        int16_t avg_fp_power = prox_avg_i32(peer->fp_power_dbm_x100_sum,
                                            peer->valid_count);
        uint8_t samples = peer->valid_count > UINT8_MAX ?
                          UINT8_MAX : (uint8_t)peer->valid_count;
        uint8_t quality = prox_quality(peer);
        uint8_t flags = PROX_ENTRY_FLAG_VALID;

        if (peer->valid_count < PROX_MIN_VALID_SAMPLES) {
            flags |= PROX_ENTRY_FLAG_LOW_SAMPLE;
        }
        if (peer->rx_error_flags_or != 0U) {
            flags |= PROX_ENTRY_FLAG_RX_ERROR;
        }

        UwbProtocol_WriteLe16(&g_tx_buf[offset + 0U], g_app_cfg.short_addr);
        UwbProtocol_WriteLe16(&g_tx_buf[offset + 2U], peer->peer_anchor);
        UwbProtocol_WriteLe16(&g_tx_buf[offset + 4U], dist_cm);
        UwbProtocol_WriteLe16(&g_tx_buf[offset + 6U], dist_std_cm);
        UwbProtocol_WriteLe16(&g_tx_buf[offset + 8U], avg_pacc);
        UwbProtocol_WriteLe16(&g_tx_buf[offset + 10U], avg_fp_index);
        UwbProtocol_WriteLe16(&g_tx_buf[offset + 12U], avg_fp_ampl1);
        UwbProtocol_WriteLe16(&g_tx_buf[offset + 14U], avg_fp_ampl2);
        UwbProtocol_WriteLe16(&g_tx_buf[offset + 16U], avg_fp_ampl3);
        UwbProtocol_WriteLe16(&g_tx_buf[offset + 18U], avg_std_noise);
        UwbProtocol_WriteLe16(&g_tx_buf[offset + 20U], avg_max_noise);
        UwbProtocol_WriteLe16(&g_tx_buf[offset + 22U], (uint16_t)avg_rx_power);
        UwbProtocol_WriteLe16(&g_tx_buf[offset + 24U], (uint16_t)avg_fp_power);
        g_tx_buf[offset + 26U] = samples;
        g_tx_buf[offset + 27U] = quality;
        g_tx_buf[offset + 28U] = flags;
        UwbProtocol_WriteLe32(&g_tx_buf[offset + 29U], peer->rx_error_flags_or);
        UwbProtocol_WriteLe16(&g_tx_buf[offset + 33U], peer->lde_status_or);
        offset = (uint16_t)(offset + PROX_TABLE_ENTRY_LEN);

        app_log_info("[PROX] peer=0x%04X dist=%ucm std=%ucm samples=%u quality=%u flags=0x%02X",
                     peer->peer_anchor,
                     (unsigned)dist_cm,
                     (unsigned)dist_std_cm,
                     (unsigned)samples,
                     (unsigned)quality,
                     (unsigned)flags);
        app_log_info("[PROX] q peer=0x%04X pacc=%u fpidx=%u amp=%u/%u/%u noise=%u/%u pwr=%d/%d err=0x%08lX lde=0x%04X",
                     peer->peer_anchor,
                     (unsigned)avg_pacc,
                     (unsigned)avg_fp_index,
                     (unsigned)avg_fp_ampl1,
                     (unsigned)avg_fp_ampl2,
                     (unsigned)avg_fp_ampl3,
                     (unsigned)avg_std_noise,
                     (unsigned)avg_max_noise,
                     (int)avg_rx_power,
                     (int)avg_fp_power,
                     (unsigned long)peer->rx_error_flags_or,
                     (unsigned)peer->lde_status_or);
    }

    uint16_t crc = crc16_ccitt(g_tx_buf, len);
    UwbProtocol_WriteLe16(&g_tx_buf[6], crc);
    g_tx_len = len;
    g_anchor_data.total_frags = calc_total_frags(g_tx_len);

    app_log_info("[PROX] table ready self=0x%04X seq=%u entries=%u max=%u len=%u frags=%u crc=0x%04X disc_req=%lu queued=%lu",
                 g_app_cfg.short_addr,
                 (unsigned)g_prox_init_seq,
                 (unsigned)entry_count,
                 (unsigned)PROX_TABLE_MAX_ENTRIES,
                 (unsigned)g_tx_len,
                 (unsigned)g_anchor_data.total_frags,
                 crc,
                 (unsigned long)g_prox_disc_request_count,
                 (unsigned long)g_prox_disc_queued_count);
    prox_log_peer_summary(entry_count);
    prox_log_ring_next(entry_count);
    if (g_prox_ring_next_anchor != 0U) {
        prox_queue_ring_notify(g_prox_ring_next_anchor);
    } else {
        AppTasks_SetAnchorNotifyInitLed(false);
    }
    prox_log_hex(g_tx_buf, g_tx_len);
}

static void prox_start_build(uint32_t now)
{
    prox_reset_peers();
    g_prox_state = PROX_BUILD_RUNNING;
    g_prox_request_pending = false;
    g_prox_started_ms = now;
    g_prox_next_disc_ms = now;
    g_prox_disc_request_count = 0U;
    g_prox_disc_queued_count = 0U;
    g_prox_ring_next_anchor = 0U;
    g_prox_ring_start_pending = false;
    if (g_prox_ring_origin_anchor == 0U) {
        g_prox_ring_origin_anchor = g_app_cfg.short_addr;
    }
    if (g_prox_ring_token_seq == 0U) {
        g_prox_ring_token_seq = prox_next_ring_seq();
    }
    g_prox_init_seq = g_prox_ring_token_seq;

    AppTasks_SetAnchorLocalInitLed(true);
    app_log_info("[PROX] build start self=0x%04X origin=0x%04X seq=%u window=%ums slots=4",
                 g_app_cfg.short_addr,
                 g_prox_ring_origin_anchor,
                 (unsigned)g_prox_init_seq,
                 (unsigned)PROX_BUILD_WINDOW_MS);
}

static void prox_request_discovery(void)
{
    UwbLinkCmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = LINK_CMD_PROX_DISCOVERY;
    cmd.slot_index = -1;

    g_prox_disc_request_count++;
    if (UwbLink_SendCmd(&cmd)) {
        g_prox_disc_queued_count++;
    } else {
        app_log_warn("[PROX] disc request queue full req=%lu queued=%lu",
                     (unsigned long)g_prox_disc_request_count,
                     (unsigned long)g_prox_disc_queued_count);
    }
}

static void prox_build_poll(void)
{
    if (g_app_cfg.role != APP_ROLE_ANCHOR ||
        !prox_anchor_id_in_range(g_app_cfg.short_addr)) {
        g_prox_request_pending = false;
        return;
    }

    uint32_t now = HAL_GetTick();

    if (g_prox_ring_start_pending &&
        (int32_t)(now - g_prox_ring_start_ms) >= 0 &&
        g_prox_state == PROX_BUILD_IDLE) {
        g_prox_ring_start_pending = false;
        g_prox_request_pending = true;
        app_log_info("[PROX] ring delayed start self=0x%04X prev=0x%04X origin=0x%04X seq=%u",
                     g_app_cfg.short_addr,
                     g_prox_ring_prev_anchor,
                     g_prox_ring_origin_anchor,
                     (unsigned)g_prox_ring_token_seq);
    }

    if (g_prox_request_pending && g_prox_state == PROX_BUILD_IDLE) {
        prox_start_build(now);
    }

    if (g_prox_state != PROX_BUILD_RUNNING) {
        return;
    }

    if ((uint32_t)(now - g_prox_started_ms) >= PROX_BUILD_WINDOW_MS) {
        prox_finalize_table();
        AppTasks_SetAnchorLocalInitLed(false);
        g_prox_state = PROX_BUILD_IDLE;
        return;
    }

    if ((int32_t)(now - g_prox_next_disc_ms) >= 0) {
        prox_request_discovery();
        g_prox_next_disc_ms = now + PROX_DISCOVERY_INTERVAL_MS;
    }
}

static void prox_accumulate_result(const UwbRangeResult *result)
{
    if (result == NULL ||
        g_prox_state != PROX_BUILD_RUNNING ||
        g_app_cfg.role != APP_ROLE_ANCHOR ||
        !prox_peer_id_valid(result->anchor_id) ||
        result->distance_m <= 0.0 ||
        result->distance_m >= 655.35) {
        return;
    }

    prox_peer_accum_t *peer = prox_find_or_alloc_peer(result->anchor_id);
    if (peer == NULL) {
        app_log_warn("[PROX] peer table full drop peer=0x%04X max=%u",
                     result->anchor_id,
                     (unsigned)PROX_TABLE_MAX_ENTRIES);
        return;
    }

    peer->sample_count++;
    peer->valid_count++;
    peer->distance_sum_m += result->distance_m;
    peer->distance_sq_sum_m += result->distance_m * result->distance_m;
    peer->pacc_sum += result->quality.rx_pacc;
    peer->fp_index_sum += result->quality.fp_index;
    peer->fp_index_sq_sum +=
        (uint64_t)result->quality.fp_index * (uint64_t)result->quality.fp_index;
    peer->fp_ampl1_sum += result->quality.fp_ampl1;
    peer->fp_ampl2_sum += result->quality.fp_ampl2;
    peer->fp_ampl3_sum += result->quality.fp_ampl3;
    peer->std_noise_sum += result->quality.std_noise;
    peer->max_noise_sum += result->quality.max_noise;
    peer->rx_power_dbm_x100_sum += result->quality.rx_power_dbm_x100;
    peer->fp_power_dbm_x100_sum += result->quality.fp_power_dbm_x100;
    peer->lde_status_or |= result->quality.lde_status;
    peer->rx_error_flags_or |= result->quality.rx_error_flags;
    peer->last_seen_ms = HAL_GetTick();
}

static void tag_start_session(uint16_t target_id);

static tag_pull_record_t *tag_find_pull_record(uint16_t anchor_id)
{
    for (uint32_t i = 0U; i < TAG_PULL_MAX_ANCHORS; ++i) {
        if (g_tag_pull_records[i].valid &&
            g_tag_pull_records[i].anchor_id == anchor_id) {
            return &g_tag_pull_records[i];
        }
    }
    return NULL;
}

static tag_pull_record_t *tag_alloc_pull_record(uint16_t anchor_id)
{
    for (uint32_t i = 0U; i < TAG_PULL_MAX_ANCHORS; ++i) {
        if (!g_tag_pull_records[i].valid) {
            memset(&g_tag_pull_records[i], 0, sizeof(g_tag_pull_records[i]));
            g_tag_pull_records[i].valid = true;
            g_tag_pull_records[i].anchor_id = anchor_id;
            return &g_tag_pull_records[i];
        }
    }

    app_log_warn("[APP] TAG_PULL_TABLE_FULL drop anchor=0x%04X max=%u",
                 anchor_id,
                 (unsigned)TAG_PULL_MAX_ANCHORS);
    return NULL;
}

static tag_pull_record_t *tag_next_pending_pull_record(void)
{
    tag_pull_record_t *best = NULL;

    for (uint32_t i = 0U; i < TAG_PULL_MAX_ANCHORS; ++i) {
        if (!g_tag_pull_records[i].valid ||
            g_tag_pull_records[i].started) {
            continue;
        }

        if (best == NULL ||
            g_tag_pull_records[i].trigger_ms < best->trigger_ms) {
            best = &g_tag_pull_records[i];
        }
    }
    return best;
}

static void tag_maybe_queue_data_pull(const UwbRangeResult *result)
{
    if (result == NULL ||
        g_app_cfg.role != APP_ROLE_TAG ||
        result->anchor_id == 0U ||
        result->distance_m <= 0.0 ||
        result->distance_m > TAG_PULL_TRIGGER_DIST_M) {
        return;
    }

    if (tag_find_pull_record(result->anchor_id) != NULL) {
        return;
    }

    tag_pull_record_t *rec = tag_alloc_pull_record(result->anchor_id);
    if (rec == NULL) {
        app_log_warn("[APP] TAG_PULL_QUEUE_FAIL anchor=0x%04X dist=%.2fm",
                     result->anchor_id, result->distance_m);
        return;
    }

    rec->trigger_distance_m = result->distance_m;
    rec->trigger_ms = HAL_GetTick();
    rec->started = false;

    app_log_info("[APP] TAG_PULL_TRIGGER anchor=0x%04X dist=%.2fm threshold=%.2fm action=%s",
                 rec->anchor_id,
                 rec->trigger_distance_m,
                 TAG_PULL_TRIGGER_DIST_M,
                 g_tag_data.state == TAG_DATA_IDLE ? "start" : "queue");
}

static twr_anchor_record_t *find_anchor_record(uint16_t anchor_id)
{
    for (int i = 0; i < TWR_MAX_ANCHORS; i++) {
        if (g_anchor_records[i].valid && g_anchor_records[i].anchor_id == anchor_id) {
            return &g_anchor_records[i];
        }
    }
    for (int i = 0; i < TWR_MAX_ANCHORS; i++) {
        if (!g_anchor_records[i].valid) {
            g_anchor_records[i].valid = true;
            g_anchor_records[i].anchor_id = anchor_id;
            g_anchor_records[i].have_prev_exchange = false;
            g_anchor_records[i].anchor_tx_ts = 0;
            g_anchor_records[i].tag_rx_ts = 0;
            g_anchor_records[i].tag_rx_local_tick_20k = 0;
            g_anchor_records[i].last_published_frame_local_tick_20k = 0;
            return &g_anchor_records[i];
        }
    }
    return NULL;
}

static void twr_record_exchange(twr_anchor_record_t *rec,
                                const UwbTwrExchange *exchange)
{
    if (rec == NULL || exchange == NULL) return;

    rec->have_prev_exchange = true;
    rec->anchor_tx_ts = exchange->anchor_tx_ts;
    rec->tag_rx_ts = exchange->tag_rx_ts;
    rec->tag_rx_local_tick_20k = exchange->tag_rx_local_tick_20k;
}

static void handle_twr_exchange(const UwbTwrExchange *exchange)
{
    if (exchange == NULL) return;

    if (exchange->tag_tx_ts == 0U) {
        app_log_warn("[APP] TWR_RX_INVALID anchor=0x%04X seq=%u slot=%u tag_tx=0",
                     exchange->anchor_id,
                     (unsigned)exchange->exchange_seq,
                     (unsigned)exchange->response_slot_id);
        return;
    }

    twr_anchor_record_t *rec = find_anchor_record(exchange->anchor_id);
    if (rec == NULL) {
        app_log_warn("[APP] TWR_REC_FULL anchor=0x%04X", exchange->anchor_id);
        return;
    }

    if (!rec->have_prev_exchange) {
        rec->last_published_frame_local_tick_20k = 0U;
        twr_record_exchange(rec, exchange);
        app_log_info("[APP] TWR_DBG_WAIT_PAIR anchor=0x%04X seq=%u",
                     exchange->anchor_id,
                     (unsigned)exchange->exchange_seq);
        return;
    }

    UwbRangeResult result;
    UwbTwrExchange prev;
    memset(&prev, 0, sizeof(prev));
    prev.anchor_id    = rec->anchor_id;
    prev.anchor_tx_ts = rec->anchor_tx_ts;
    prev.tag_rx_ts    = rec->tag_rx_ts;
    prev.tag_rx_local_tick_20k = rec->tag_rx_local_tick_20k;

    if (!compute_range(&prev, exchange, &result)) {
        app_log_warn("[APP] TWR_DBG_DS_DROP anchor=0x%04X seq=%u action=rebuild_window reason=compute",
                     exchange->anchor_id,
                     (unsigned)exchange->exchange_seq);
        rec->last_published_frame_local_tick_20k = 0U;
        twr_record_exchange(rec, exchange);
        return;
    }

    {
        uint64_t gap_ticks = 0U;
        twr_gap_class_t gap_class = TWR_GAP_SHORT;
        if (!twr_get_published_gap_info(rec->last_published_frame_local_tick_20k,
                                        result.frame_local_tick_20k,
                                        &gap_ticks,
                                        &gap_class)) {
            app_log_warn("[APP] TWR_DBG_DS_DROP anchor=0x%04X seq=%u action=rebuild_window reason=frame_time",
                         exchange->anchor_id,
                         (unsigned)exchange->exchange_seq);
            rec->last_published_frame_local_tick_20k = 0U;
            twr_record_exchange(rec, exchange);
            return;
        }

        app_log_info("[APP] TWR_DBG_GAP anchor=0x%04X seq=%u last_ms=%.3f cand_ms=%.3f gap_ms=%.3f gap_class=%s",
                     exchange->anchor_id,
                     (unsigned)exchange->exchange_seq,
                     local_tick_to_ms(rec->last_published_frame_local_tick_20k),
                     local_tick_to_ms(result.frame_local_tick_20k),
                     local_tick_to_ms(gap_ticks),
                     gap_class == TWR_GAP_SHORT ? "SHORT" :
                     (gap_class == TWR_GAP_LONG ? "LONG" : "DROP"));

        if (gap_class == TWR_GAP_DROP) {
            app_log_warn("[APP] TWR_DBG_DS_DROP anchor=0x%04X seq=%u action=rebuild_window reason=gap_timeout gap_ms=%.3f",
                         exchange->anchor_id,
                         (unsigned)exchange->exchange_seq,
                         local_tick_to_ms(gap_ticks));
            rec->last_published_frame_local_tick_20k = 0U;
            twr_record_exchange(rec, exchange);
            return;
        }

        result.status_flags &= (uint16_t)~(APP_UWB_STATUS_FLAG_TWR_DS |
                                           APP_UWB_STATUS_FLAG_TWR_DS_SHORT |
                                           APP_UWB_STATUS_FLAG_TWR_DS_LONG);
        result.status_flags |= APP_UWB_STATUS_FLAG_TWR_DS;
        result.status_flags |= gap_class == TWR_GAP_SHORT ?
            APP_UWB_STATUS_FLAG_TWR_DS_SHORT :
            APP_UWB_STATUS_FLAG_TWR_DS_LONG;

        prox_accumulate_result(&result);
        tag_maybe_queue_data_pull(&result);
        publish_range_result(&result);
        log_twr_frame(gap_class == TWR_GAP_SHORT ? "DS_SHORT" : "DS_LONG",
                      exchange, &result);
    }
    twr_record_exchange(rec, exchange);
    rec->last_published_frame_local_tick_20k = result.frame_local_tick_20k;
}

/* ================================================================
 *  Tag DATA 会话
 * ================================================================ */

static void tag_reset_session(void)
{
    memset(&g_tag_data.current_cmd, 0, sizeof(g_tag_data.current_cmd));
    g_tag_data.state = TAG_DATA_IDLE;
    g_tag_data.target_id = 0;
    g_tag_data.session_id = 0;
    g_tag_data.total_len = 0;
    g_tag_data.total_crc = 0;
    g_tag_data.total_frags = 0;
    g_tag_data.next_frag = 0;
    g_tag_data.session_started_ms = 0;
    g_tag_data.retry_at_ms = 0;
    g_tag_data.cmd_count = 0;
    g_tag_data.ack_count = 0;
    g_tag_data.wait_count = 0;
    g_tag_data.frag_count = 0;
    g_tag_data.drop_count = 0;
    g_tag_data.cmd_pending = false;
    g_tag_data.completion_logged = false;
    g_rx_len = 0;
}

static void tag_submit_cmd(const UwbLinkCmd *cmd)
{
    if (cmd == NULL) return;
    g_tag_data.current_cmd = *cmd;
    g_tag_data.cmd_pending = true;
    g_tag_data.retry_at_ms = HAL_GetTick();
}

static void tag_send_reset_cmd(uint16_t session_id)
{
    UwbLinkCmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = LINK_CMD_SESSION_RESET;
    cmd.session_id = session_id;
    (void)UwbLink_SendCmd(&cmd);
}

static void tag_log_data_stats(const char *label)
{
    if (label == NULL) return;

    app_log_info("[APP] %s anchor=0x%04X sess=0x%04X cmd=%lu ack=%lu wait=%lu frag=%lu drop=%lu len=%u/%u",
                 label,
                 g_tag_data.target_id,
                 g_tag_data.session_id,
                 (unsigned long)g_tag_data.cmd_count,
                 (unsigned long)g_tag_data.ack_count,
                 (unsigned long)g_tag_data.wait_count,
                 (unsigned long)g_tag_data.frag_count,
                 (unsigned long)g_tag_data.drop_count,
                 g_rx_len,
                 g_tag_data.total_len);
}

static void tag_request_cfg(uint16_t target_id, uint16_t session_id)
{
    UwbLinkCmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = LINK_CMD_SEND_CFG_REQ;
    cmd.slot_index = -1;
    cmd.target_id = target_id;
    cmd.session_id = session_id;
    tag_submit_cmd(&cmd);
}

static void tag_request_ctrl(uint8_t ctrl_type, uint8_t frag_id)
{
    UwbLinkCmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = LINK_CMD_SEND_CTRL;
    cmd.slot_index = -1;
    cmd.target_id = g_tag_data.target_id;
    cmd.session_id = g_tag_data.session_id;
    cmd.ctrl_type = ctrl_type;
    cmd.frag_id = frag_id;
    tag_submit_cmd(&cmd);
}

static void tag_try_send_pending_cmd(void)
{
    if (!g_tag_data.cmd_pending) return;

    uint32_t now = HAL_GetTick();
    if (now < g_tag_data.retry_at_ms) return;

    if (UwbLink_SendCmd(&g_tag_data.current_cmd)) {
        g_tag_data.cmd_count++;
        g_tag_data.cmd_pending = false;
    } else {
        g_tag_data.retry_at_ms = now + 1U;
    }
}

static void tag_start_session(uint16_t target_id)
{
    tag_reset_session();

    uint16_t session_id = g_next_session_id++;
    if (g_next_session_id == 0U) g_next_session_id = 1U;
    if (session_id == 0U) session_id = g_next_session_id++;

    g_tag_data.state = TAG_DATA_WAIT_CFG_ACK;
    g_tag_data.target_id = target_id;
    g_tag_data.session_id = session_id;
    g_tag_data.session_started_ms = HAL_GetTick();
    memset(g_rx_buf, 0, sizeof(g_rx_buf));
    g_rx_len = 0;

    app_log_info("[APP] DATA_SESSION_START anchor=0x%04X sess=0x%04X",
                 target_id, session_id);
    tag_request_cfg(target_id, session_id);
}

static void tag_schedule_wait_retry(void)
{
    g_tag_data.wait_count++;
    g_tag_data.cmd_pending = true;
    g_tag_data.retry_at_ms = HAL_GetTick() + DATA_WAIT_RETRY_MS;
    app_log_info("[APP] DATA_WAIT retry ctrl=%u frag=%u after=%ums",
                 (unsigned)g_tag_data.current_cmd.ctrl_type,
                 (unsigned)g_tag_data.current_cmd.frag_id,
                 (unsigned)DATA_WAIT_RETRY_MS);
}

static void tag_log_session_complete(void)
{
    if (g_tag_data.completion_logged) return;

    uint32_t now = HAL_GetTick();
    uint32_t duration_ms = 0;
    if (g_tag_data.session_started_ms > 0U) {
        duration_ms = now - g_tag_data.session_started_ms;
    }

    app_log_info("[APP] SESSION_COMPLETE anchor=0x%04X sess=0x%04X dur=%lums frags=%lu/%u len=%u/%u",
                 g_tag_data.target_id,
                 g_tag_data.session_id,
                 (unsigned long)duration_ms,
                 (unsigned long)g_tag_data.frag_count,
                 (unsigned)g_tag_data.total_frags,
                 g_rx_len,
                 g_tag_data.total_len);
    g_tag_data.completion_logged = true;
}

static bool tag_verify_prox_table(void)
{
    uint16_t got_crc = crc16_ccitt(g_rx_buf, g_rx_len);
    uint16_t table_crc;
    uint8_t entry_count;
    uint16_t table_self;
    uint16_t table_seq;
    uint16_t table_len;
    uint8_t crc_lo;
    uint8_t crc_hi;
    uint8_t log_count;

    if (g_rx_len < PROX_TABLE_HEADER_LEN ||
        g_tag_data.total_len != g_rx_len ||
        g_tag_data.total_crc != got_crc) {
        app_log_warn("[APP] DATA_VERIFY_FAIL anchor=0x%04X len=%u/%u rx_crc=0x%04X meta_crc=0x%04X reason=meta",
                     g_tag_data.target_id,
                     g_rx_len,
                     g_tag_data.total_len,
                     got_crc,
                     g_tag_data.total_crc);
        return false;
    }

    if (g_rx_buf[0] != PROX_TABLE_VERSION) {
        app_log_warn("[APP] PROX_TABLE_INVALID anchor=0x%04X version=%u expect=%u",
                     g_tag_data.target_id,
                     (unsigned)g_rx_buf[0],
                     (unsigned)PROX_TABLE_VERSION);
        return false;
    }

    entry_count = g_rx_buf[1];
    table_len = (uint16_t)(PROX_TABLE_HEADER_LEN +
                           entry_count * PROX_TABLE_ENTRY_LEN);
    if (table_len != g_rx_len) {
        app_log_warn("[APP] PROX_TABLE_INVALID anchor=0x%04X entries=%u len=%u expect=%u",
                     g_tag_data.target_id,
                     (unsigned)entry_count,
                     g_rx_len,
                     table_len);
        return false;
    }

    table_crc = UwbProtocol_ReadLe16(&g_rx_buf[6]);
    crc_lo = g_rx_buf[6];
    crc_hi = g_rx_buf[7];
    g_rx_buf[6] = 0U;
    g_rx_buf[7] = 0U;
    got_crc = crc16_ccitt(g_rx_buf, g_rx_len);
    g_rx_buf[6] = crc_lo;
    g_rx_buf[7] = crc_hi;

    if (got_crc != table_crc) {
        app_log_warn("[APP] PROX_TABLE_CRC_FAIL anchor=0x%04X got=0x%04X expect=0x%04X",
                     g_tag_data.target_id,
                     got_crc,
                     table_crc);
        return false;
    }

    table_self = UwbProtocol_ReadLe16(&g_rx_buf[2]);
    table_seq = UwbProtocol_ReadLe16(&g_rx_buf[4]);
    app_log_info("[APP] DATA_VERIFY_OK anchor=0x%04X len=%u frags=%u crc=0x%04X",
                 g_tag_data.target_id,
                 g_rx_len,
                 (unsigned)g_tag_data.total_frags,
                 table_crc);
    app_log_info("[APP] PROX_TABLE_RX anchor=0x%04X table_self=0x%04X seq=%u entries=%u len=%u",
                 g_tag_data.target_id,
                 table_self,
                 (unsigned)table_seq,
                 (unsigned)entry_count,
                 g_rx_len);

    log_count = entry_count > 4U ? 4U : entry_count;
    for (uint8_t i = 0U; i < log_count; ++i) {
        uint16_t off = (uint16_t)(PROX_TABLE_HEADER_LEN +
                                  i * PROX_TABLE_ENTRY_LEN);
        uint16_t self = UwbProtocol_ReadLe16(&g_rx_buf[off + 0U]);
        uint16_t peer = UwbProtocol_ReadLe16(&g_rx_buf[off + 2U]);
        uint16_t dist_cm = UwbProtocol_ReadLe16(&g_rx_buf[off + 4U]);
        uint16_t std_cm = UwbProtocol_ReadLe16(&g_rx_buf[off + 6U]);
        uint8_t samples = g_rx_buf[off + 26U];
        uint8_t quality = g_rx_buf[off + 27U];
        uint8_t flags = g_rx_buf[off + 28U];

        app_log_info("[APP] PROX_ENTRY_RX anchor=0x%04X self=0x%04X peer=0x%04X dist=%ucm std=%ucm samples=%u quality=%u flags=0x%02X",
                     g_tag_data.target_id,
                     self,
                     peer,
                     (unsigned)dist_cm,
                     (unsigned)std_cm,
                     (unsigned)samples,
                     (unsigned)quality,
                     (unsigned)flags);
    }

    if (entry_count > log_count) {
        app_log_info("[APP] PROX_ENTRY_RX anchor=0x%04X omitted=%u",
                     g_tag_data.target_id,
                     (unsigned)(entry_count - log_count));
    }

    return true;
}

static void handle_tag_ack(const UwbLinkAppEvent *evt)
{
    (void)evt;
    g_tag_data.ack_count++;

    if (g_tag_data.state == TAG_DATA_WAIT_CFG_ACK) {
        app_log_info("[APP] DATA_ACK CFG sess=0x%04X", g_tag_data.session_id);
        g_tag_data.state = TAG_DATA_WAIT_META;
        tag_request_ctrl(UWB_DATA_CTRL_GET_INFO, 0);
        return;
    }

    if (g_tag_data.state == TAG_DATA_WAIT_DONE_ACK) {
        app_log_info("[APP] DATA_ACK DONE sess=0x%04X", g_tag_data.session_id);
        tag_log_data_stats("DATA_STATS");
        tag_log_session_complete();
        tag_reset_session();
    }
}

static void handle_tag_data_fail(void)
{
    if (g_tag_data.state == TAG_DATA_WAIT_DONE_ACK &&
        g_tag_data.completion_logged) {
        app_log_warn("[APP] DATA_DONE_ACK_FAIL anchor=0x%04X sess=0x%04X",
                     g_tag_data.target_id, g_tag_data.session_id);
        tag_log_data_stats("DATA_STATS");
        tag_reset_session();
        return;
    }

    app_log_warn("[APP] DATA_SESSION_FAIL anchor=0x%04X sess=0x%04X",
                 g_tag_data.target_id, g_tag_data.session_id);
    tag_log_data_stats("DATA_STATS_FAIL");
    tag_send_reset_cmd(g_tag_data.session_id);
    tag_reset_session();
}

static void handle_tag_frag(const UwbDataFragEvent *evt)
{
    if (evt == NULL || evt->slot_index < 0) return;

    UwbProtocolFrame frame;
    uwb_slot_t *slot = UwbSlots_Get(evt->slot_index);
    if (slot == NULL ||
        !UwbProtocol_Decode(&frame, slot->data, slot->data_len)) {
        app_log_warn("[APP] DATA_FRAG_DECODE_FAIL slot=%d", (int)evt->slot_index);
        g_tag_data.drop_count++;
        UwbSlots_Free(evt->slot_index);
        return;
    }

    if (evt->src_id != g_tag_data.target_id ||
        evt->session_id != g_tag_data.session_id) {
        app_log_warn("[APP] DATA_FRAG_DROP src=0x%04X sess=0x%04X expect=0x%04X/0x%04X",
                     evt->src_id, evt->session_id,
                     g_tag_data.target_id, g_tag_data.session_id);
        g_tag_data.drop_count++;
        UwbSlots_Free(evt->slot_index);
        return;
    }

    if ((evt->flags & UWB_DATA_FRAG_FLAG_META) != 0U) {
        if (g_tag_data.state == TAG_DATA_WAIT_META &&
            frame.common.payload_len >= DATA_META_PAYLOAD_LEN) {
            g_tag_data.total_len = UwbProtocol_ReadLe16(&frame.payload[0]);
            g_tag_data.total_frags = frame.payload[2];
            g_tag_data.total_crc = UwbProtocol_ReadLe16(&frame.payload[4]);
            g_tag_data.next_frag = 1U;
            g_rx_len = 0;

            app_log_info("[APP] DATA_META len=%u total_frags=%u crc=0x%04X",
                         g_tag_data.total_len,
                         (unsigned)g_tag_data.total_frags,
                         g_tag_data.total_crc);

            if (g_tag_data.total_len > DATA_BUF_SIZE ||
                g_tag_data.total_frags == 0U ||
                g_tag_data.total_frags > UWB_DATA_MAX_FRAGS) {
                app_log_warn("[APP] DATA_META_INVALID len=%u total_frags=%u",
                             g_tag_data.total_len,
                             (unsigned)g_tag_data.total_frags);
                UwbSlots_Free(evt->slot_index);
                handle_tag_data_fail();
                return;
            }

            g_tag_data.state = TAG_DATA_PULLING;
            tag_request_ctrl(UWB_DATA_CTRL_PULL, g_tag_data.next_frag);
        }

        UwbSlots_Free(evt->slot_index);
        return;
    }

    if (g_tag_data.state != TAG_DATA_PULLING ||
        evt->frag_id != g_tag_data.next_frag ||
        frame.common.payload_len < DATA_FRAG_CRC_LEN) {
        app_log_warn("[APP] DATA_FRAG_UNEXPECTED state=%u frag=%u expect=%u len=%u",
                     (unsigned)g_tag_data.state,
                     (unsigned)evt->frag_id,
                     (unsigned)g_tag_data.next_frag,
                     frame.common.payload_len);
        g_tag_data.drop_count++;
        UwbSlots_Free(evt->slot_index);
        return;
    }

    uint16_t frag_crc = UwbProtocol_ReadLe16(&frame.payload[0]);
    const uint8_t *frag_data = &frame.payload[DATA_FRAG_CRC_LEN];
    uint16_t frag_len = (uint16_t)(frame.common.payload_len - DATA_FRAG_CRC_LEN);
    uint16_t calc_crc = crc16_ccitt(frag_data, frag_len);

    if (calc_crc != frag_crc) {
        app_log_warn("[APP] DATA_FRAG_CRC_FAIL frag=%u got=0x%04X calc=0x%04X",
                     (unsigned)evt->frag_id, frag_crc, calc_crc);
        g_tag_data.drop_count++;
        tag_request_ctrl(UWB_DATA_CTRL_PULL, g_tag_data.next_frag);
        UwbSlots_Free(evt->slot_index);
        return;
    }

    uint16_t offset = (uint16_t)((evt->frag_id - 1U) * UWB_DATA_FRAG_PAYLOAD_SIZE);
    if (offset + frag_len > DATA_BUF_SIZE ||
        offset + frag_len > g_tag_data.total_len) {
        app_log_warn("[APP] DATA_FRAG_RANGE_FAIL frag=%u off=%u len=%u total=%u",
                     (unsigned)evt->frag_id, offset, frag_len,
                     g_tag_data.total_len);
        g_tag_data.drop_count++;
        UwbSlots_Free(evt->slot_index);
        handle_tag_data_fail();
        return;
    }

    memcpy(&g_rx_buf[offset], frag_data, frag_len);
    g_rx_len = offset + frag_len;
    g_tag_data.frag_count++;
    app_log_info("[APP] DATA_FRAG frag=%u len=%u",
                 (unsigned)evt->frag_id, frag_len);

    UwbSlots_Free(evt->slot_index);

    if (g_tag_data.next_frag < g_tag_data.total_frags) {
        g_tag_data.next_frag++;
        tag_request_ctrl(UWB_DATA_CTRL_PULL, g_tag_data.next_frag);
        return;
    }

    if (g_rx_len != g_tag_data.total_len ||
        crc16_ccitt(g_rx_buf, g_rx_len) != g_tag_data.total_crc) {
        app_log_warn("[APP] DATA_TOTAL_CRC_FAIL len=%u expect_len=%u",
                     g_rx_len, g_tag_data.total_len);
        handle_tag_data_fail();
        return;
    }

    if (!tag_verify_prox_table()) {
        handle_tag_data_fail();
        return;
    }

    tag_log_session_complete();
    g_tag_data.state = TAG_DATA_WAIT_DONE_ACK;
    tag_request_ctrl(UWB_DATA_CTRL_DONE, 0);
}

static void tag_data_poll(void)
{
    if (g_app_cfg.role != APP_ROLE_TAG) return;

    uint32_t now = HAL_GetTick();

    if (g_tag_data.state != TAG_DATA_IDLE &&
        g_tag_data.session_started_ms > 0 &&
        now - g_tag_data.session_started_ms >= DATA_SESSION_TIMEOUT_MS) {
        handle_tag_data_fail();
        return;
    }

    if (g_tag_data.state == TAG_DATA_IDLE) {
        tag_pull_record_t *rec = tag_next_pending_pull_record();
        if (rec != NULL) {
            rec->started = true;
            app_log_info("[APP] TAG_PULL_START anchor=0x%04X dist=%.2fm age=%lums",
                         rec->anchor_id,
                         rec->trigger_distance_m,
                         (unsigned long)(now - rec->trigger_ms));
            tag_start_session(rec->anchor_id);
        }
    }

    tag_try_send_pending_cmd();
}

/* ================================================================
 *  Anchor DATA 会话
 * ================================================================ */

static bool anchor_send_cmd(UwbLinkCmd *cmd)
{
    if (cmd == NULL) return false;
    if (UwbLink_SendCmd(cmd)) return true;

    if (cmd->slot_index >= 0) {
        UwbSlots_Free(cmd->slot_index);
    }
    app_log_warn("[APP] DATA_CMD_SEND_FAIL type=%u dst=0x%04X sess=0x%04X frag=%u",
                 (unsigned)cmd->type, cmd->target_id,
                 cmd->session_id, (unsigned)cmd->frag_id);
    return false;
}

static void anchor_log_data_stats(const char *label)
{
    if (label == NULL) return;

    app_log_info("[APP] %s tag=0x%04X sess=0x%04X cfg=%lu ctrl=%lu pull=%lu frag_ready=%lu drop=%lu",
                 label,
                 g_anchor_data.tag_id,
                 g_anchor_data.session_id,
                 (unsigned long)g_anchor_data.cfg_count,
                 (unsigned long)g_anchor_data.ctrl_count,
                 (unsigned long)g_anchor_data.pull_count,
                 (unsigned long)g_anchor_data.frag_ready_count,
                 (unsigned long)g_anchor_data.drop_count);
}

static bool anchor_send_payload_slot(uint16_t target_id, uint16_t session_id,
                                     uint8_t frag_id, uint8_t flags,
                                     const uint8_t *payload, uint16_t payload_len)
{
    if (payload == NULL || payload_len > UWB_PROTO_MAX_PAYLOAD_LEN) return false;

    int8_t idx = UwbSlots_Alloc(UWB_SLOT_APP_OWN);
    if (idx < 0) return false;

    uwb_slot_t *s = UwbSlots_Get(idx);
    if (s == NULL) {
        UwbSlots_Free(idx);
        return false;
    }

    memcpy(s->data, payload, payload_len);
    s->data_len = payload_len;
    s->session_id = session_id;
    s->frag_id = frag_id;
    s->total_frags = g_anchor_data.total_frags;
    s->data_flags = flags;

    UwbLinkCmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = LINK_CMD_SEND_FRAG;
    cmd.slot_index = idx;
    cmd.target_id = target_id;
    cmd.session_id = session_id;
    cmd.frag_id = frag_id;

    if (!anchor_send_cmd(&cmd)) {
        app_log_warn("[APP] DATA_SEND_FRAG queue full frag=%u", (unsigned)frag_id);
        return false;
    }
    g_anchor_data.frag_ready_count++;
    return true;
}

static void anchor_prepare_meta(uint16_t target_id, uint16_t session_id)
{
    uint8_t payload[DATA_META_PAYLOAD_LEN];
    memset(payload, 0, sizeof(payload));
    UwbProtocol_WriteLe16(&payload[0], g_tx_len);
    payload[2] = g_anchor_data.total_frags;
    payload[3] = UWB_DATA_FRAG_PAYLOAD_SIZE;
    UwbProtocol_WriteLe16(&payload[4], crc16_ccitt(g_tx_buf, g_tx_len));

    (void)anchor_send_payload_slot(target_id, session_id, 0,
                                   UWB_DATA_FRAG_FLAG_META,
                                   payload, sizeof(payload));
}

static void anchor_prepare_data_frag(uint16_t target_id, uint16_t session_id,
                                     uint8_t frag_id)
{
    if (frag_id == 0U || frag_id > g_anchor_data.total_frags) {
        app_log_warn("[APP] DATA_PREP_FRAG_INVALID frag=%u total=%u",
                     (unsigned)frag_id, (unsigned)g_anchor_data.total_frags);
        return;
    }

    uint16_t offset = (uint16_t)((frag_id - 1U) * UWB_DATA_FRAG_PAYLOAD_SIZE);
    uint16_t remain = (offset < g_tx_len) ? (uint16_t)(g_tx_len - offset) : 0U;
    uint16_t frag_len = min_u16(remain, UWB_DATA_FRAG_PAYLOAD_SIZE);

    uint8_t payload[UWB_PROTO_MAX_PAYLOAD_LEN];
    UwbProtocol_WriteLe16(&payload[0], crc16_ccitt(&g_tx_buf[offset], frag_len));
    memcpy(&payload[DATA_FRAG_CRC_LEN], &g_tx_buf[offset], frag_len);

    (void)anchor_send_payload_slot(target_id, session_id, frag_id, 0,
                                   payload, (uint16_t)(frag_len + DATA_FRAG_CRC_LEN));
}

static void handle_anchor_data_cfg(const UwbLinkAppEvent *evt)
{
    if (evt == NULL || g_app_cfg.role != APP_ROLE_ANCHOR) return;

    g_anchor_data.active = true;
    g_anchor_data.tag_id = evt->data.data_cfg.src_id;
    g_anchor_data.session_id = evt->data.data_cfg.session_id;
    g_anchor_data.total_frags = calc_total_frags(g_tx_len);
    g_anchor_data.last_ms = HAL_GetTick();
    g_anchor_data.cfg_count = 1U;
    g_anchor_data.ctrl_count = 0;
    g_anchor_data.pull_count = 0;
    g_anchor_data.frag_ready_count = 0;
    g_anchor_data.drop_count = 0;

    app_log_info("[APP] DATA_CFG tag=0x%04X sess=0x%04X total_frags=%u",
                 g_anchor_data.tag_id, g_anchor_data.session_id,
                 (unsigned)g_anchor_data.total_frags);

    anchor_prepare_meta(g_anchor_data.tag_id, g_anchor_data.session_id);
}

static void handle_anchor_data_ctrl(const UwbDataCtrlEvent *ctrl)
{
    if (ctrl == NULL || g_app_cfg.role != APP_ROLE_ANCHOR) return;

    if (!g_anchor_data.active ||
        ctrl->src_id != g_anchor_data.tag_id ||
        ctrl->session_id != g_anchor_data.session_id) {
        app_log_warn("[APP] DATA_CTRL_DROP src=0x%04X sess=0x%04X ctrl=%u frag=%u active=%u",
                     ctrl->src_id, ctrl->session_id,
                     (unsigned)ctrl->ctrl_type, (unsigned)ctrl->frag_id,
                     g_anchor_data.active ? 1U : 0U);
        g_anchor_data.drop_count++;
        return;
    }

    g_anchor_data.last_ms = HAL_GetTick();
    g_anchor_data.ctrl_count++;

    if (ctrl->ctrl_type == UWB_DATA_CTRL_GET_INFO) {
        anchor_prepare_meta(ctrl->src_id, ctrl->session_id);
        return;
    }

    if (ctrl->ctrl_type == UWB_DATA_CTRL_PULL) {
        g_anchor_data.pull_count++;
        anchor_prepare_data_frag(ctrl->src_id, ctrl->session_id, ctrl->frag_id);
        return;
    }

    if (ctrl->ctrl_type == UWB_DATA_CTRL_DONE ||
        ctrl->ctrl_type == UWB_DATA_CTRL_STOP) {
        app_log_info("[APP] DATA_SESSION_DONE tag=0x%04X sess=0x%04X",
                     ctrl->src_id, ctrl->session_id);
        anchor_log_data_stats("DATA_ANCHOR_STATS");
        g_anchor_data.active = false;
        UwbLink_DataSlotFreeAll();
    }
}

static void handle_anchor_data_sent(const UwbLinkAppEvent *evt)
{
    if (evt == NULL || g_app_cfg.role != APP_ROLE_ANCHOR) return;

    uint16_t session_id = evt->data.data_sent.session_id;
    uint8_t frag_id = evt->data.data_sent.frag_id;

    if (!g_anchor_data.active ||
        session_id != g_anchor_data.session_id) {
        return;
    }

    g_anchor_data.last_ms = HAL_GetTick();
    g_anchor_data.ctrl_count++;

    if (frag_id == 0U) {
        anchor_prepare_data_frag(g_anchor_data.tag_id, session_id, 1U);
        return;
    }

    g_anchor_data.pull_count++;
    if (frag_id < g_anchor_data.total_frags) {
        anchor_prepare_data_frag(g_anchor_data.tag_id, session_id,
                                 (uint8_t)(frag_id + 1U));
    }
}

static void anchor_data_poll(void)
{
    if (g_app_cfg.role != APP_ROLE_ANCHOR || !g_anchor_data.active) return;

    uint32_t now = HAL_GetTick();
    if (now - g_anchor_data.last_ms >= DATA_SESSION_TIMEOUT_MS) {
        app_log_warn("[APP] DATA_SESSION_TIMEOUT tag=0x%04X sess=0x%04X",
                     g_anchor_data.tag_id, g_anchor_data.session_id);
        anchor_log_data_stats("DATA_ANCHOR_TIMEOUT_STATS");
        g_anchor_data.active = false;
        UwbLink_DataSlotFreeAll();
    }
}

static void handle_ring_init_notify(const UwbLinkAppEvent *evt)
{
    if (evt == NULL || g_app_cfg.role != APP_ROLE_ANCHOR) {
        return;
    }

    uint16_t src_id = evt->data.ring_init.src_id;
    uint16_t origin_id = evt->data.ring_init.origin_id;
    uint16_t ring_seq = evt->data.ring_init.seq;

    if (!prox_anchor_id_in_range(g_app_cfg.short_addr) ||
        !prox_anchor_id_in_range(origin_id)) {
        app_log_warn("[PROX] ring notify ignored self=0x%04X src=0x%04X origin=0x%04X seq=%u",
                     g_app_cfg.short_addr,
                     src_id,
                     origin_id,
                     (unsigned)ring_seq);
        return;
    }

    g_prox_ring_prev_anchor = src_id;
    g_prox_ring_origin_anchor = origin_id;
    g_prox_ring_token_seq = ring_seq == 0U ? 1U : ring_seq;

    if (origin_id == g_app_cfg.short_addr) {
        g_prox_ring_start_pending = false;
        g_prox_request_pending = false;
        AppTasks_SetAnchorGlobalInitLed(false);
        AppTasks_SetAnchorLocalInitLed(false);
        AppTasks_SetAnchorNotifyInitLed(false);
        app_log_info("[PROX] ring complete self=0x%04X from=0x%04X origin=0x%04X seq=%u",
                     g_app_cfg.short_addr,
                     src_id,
                     origin_id,
                     (unsigned)g_prox_ring_token_seq);
        return;
    }

    g_prox_ring_start_pending = true;
    g_prox_ring_start_ms = HAL_GetTick() + PROX_RING_START_DELAY_MS;
    AppTasks_SetAnchorLocalInitLed(true);
    app_log_info("[PROX] ring notify app self=0x%04X from=0x%04X origin=0x%04X seq=%u delay=%ums",
                 g_app_cfg.short_addr,
                 src_id,
                 origin_id,
                 (unsigned)g_prox_ring_token_seq,
                 (unsigned)PROX_RING_START_DELAY_MS);
}

static void handle_ring_init_acked(const UwbLinkAppEvent *evt)
{
    if (evt == NULL || g_app_cfg.role != APP_ROLE_ANCHOR) {
        return;
    }

    app_log_info("[PROX] ring notify acked self=0x%04X next=0x%04X origin=0x%04X seq=%u",
                 g_app_cfg.short_addr,
                 evt->data.ring_init.src_id,
                 evt->data.ring_init.origin_id,
                 (unsigned)evt->data.ring_init.seq);
    AppTasks_SetAnchorNotifyInitLed(false);
}

/* ================================================================
 *  事件处理
 * ================================================================ */

static void drain_link_events(void)
{
    QueueHandle_t queue = UwbLink_AppEventQueue();
    if (queue == NULL) return;

    UwbLinkAppEvent evt;
    while (xQueueReceive(queue, &evt, 0) == pdPASS) {
        switch (evt.type) {
            case UWB_LINK_APP_EVT_TWR_EXCHANGE:
                handle_twr_exchange(&evt.data.twr);
                break;

            case UWB_LINK_APP_EVT_NEIGHBOR_SEEN:
                break;

            case UWB_LINK_APP_EVT_DATA_CFG:
                handle_anchor_data_cfg(&evt);
                break;

            case UWB_LINK_APP_EVT_DATA_CTRL:
                handle_anchor_data_ctrl(&evt.data.data_ctrl);
                break;

            case UWB_LINK_APP_EVT_DATA_ACK:
                if (g_app_cfg.role == APP_ROLE_TAG) handle_tag_ack(&evt);
                break;

            case UWB_LINK_APP_EVT_DATA_WAIT:
                if (g_app_cfg.role == APP_ROLE_TAG) tag_schedule_wait_retry();
                break;

            case UWB_LINK_APP_EVT_DATA_ERROR:
            case UWB_LINK_APP_EVT_DATA_FAIL:
                if (g_app_cfg.role == APP_ROLE_TAG) handle_tag_data_fail();
                break;

            case UWB_LINK_APP_EVT_DATA_FRAG:
                if (g_app_cfg.role == APP_ROLE_TAG) {
                    handle_tag_frag(&evt.data.data_frag);
                } else if (evt.data.data_frag.slot_index >= 0) {
                    UwbSlots_Free(evt.data.data_frag.slot_index);
                }
                break;

            case UWB_LINK_APP_EVT_DATA_SENT:
                handle_anchor_data_sent(&evt);
                break;

            case UWB_LINK_APP_EVT_RING_INIT_NOTIFY:
                handle_ring_init_notify(&evt);
                break;

            case UWB_LINK_APP_EVT_RING_INIT_ACKED:
                handle_ring_init_acked(&evt);
                break;

            default:
                break;
        }
    }
}

/* ================================================================
 *  初始化 & 任务
 * ================================================================ */

bool UwbApp_Init(const UwbStackConfig *cfg)
{
    if (cfg == NULL) return false;

    g_app_cfg = *cfg;
    memset(g_anchor_records, 0, sizeof(g_anchor_records));
    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    memset(g_rx_buf, 0, sizeof(g_rx_buf));
    memset(&g_tag_data, 0, sizeof(g_tag_data));
    memset(g_tag_pull_records, 0, sizeof(g_tag_pull_records));
    memset(&g_anchor_data, 0, sizeof(g_anchor_data));
    memset(g_prox_peers, 0, sizeof(g_prox_peers));
    g_tx_len = 0;
    g_rx_len = 0;
    g_prox_state = PROX_BUILD_IDLE;
    g_prox_request_pending = false;
    g_prox_init_seq = 0U;
    g_prox_started_ms = 0U;
    g_prox_next_disc_ms = 0U;
    g_prox_disc_request_count = 0U;
    g_prox_disc_queued_count = 0U;
    g_prox_ring_next_anchor = 0U;
    g_prox_ring_origin_anchor = 0U;
    g_prox_ring_token_seq = 0U;
    g_prox_ring_prev_anchor = 0U;
    g_prox_ring_start_pending = false;
    g_prox_ring_start_ms = 0U;
    g_prox_ring_notify_pending = false;
    g_prox_ring_notify_target = 0U;
    g_prox_ring_notify_next_ms = 0U;

    if (cfg->role == APP_ROLE_ANCHOR) {
        prox_prepare_empty_tx_table();
    }

    return true;
}

bool UwbApp_StartThread(const osThreadAttr_t *attr)
{
    if (g_uwb_app_task != NULL) return true;
    g_uwb_app_task = osThreadNew(UwbApp_Task, NULL, attr);
    return g_uwb_app_task != NULL;
}

bool UwbApp_RequestProxBuild(void)
{
    char line[160];

    if (g_app_cfg.role != APP_ROLE_ANCHOR ||
        !prox_anchor_id_in_range(g_app_cfg.short_addr)) {
        int n = snprintf(line, sizeof(line),
                         "[PROX] request rejected role=%u self=0x%04X range=0x%04X-0x%04X\r\n",
                         (unsigned)g_app_cfg.role,
                         g_app_cfg.short_addr,
                         (unsigned)PROX_ANCHOR_ID_MIN,
                         (unsigned)PROX_ANCHOR_ID_MAX);
        if (n > 0) {
            prox_log_direct(line);
        }
        return false;
    }

    if (g_prox_state != PROX_BUILD_IDLE ||
        g_prox_ring_start_pending ||
        g_prox_ring_notify_pending) {
        int n = snprintf(line, sizeof(line),
                         "[PROX] request rejected busy state=%u self=0x%04X ring_start=%u ring_notify=%u\r\n",
                         (unsigned)g_prox_state,
                         g_app_cfg.short_addr,
                         g_prox_ring_start_pending ? 1U : 0U,
                         g_prox_ring_notify_pending ? 1U : 0U);
        if (n > 0) {
            prox_log_direct(line);
        }
        return false;
    }

    g_prox_ring_origin_anchor = g_app_cfg.short_addr;
    g_prox_ring_prev_anchor = 0U;
    g_prox_ring_token_seq = prox_next_ring_seq();
    g_prox_request_pending = true;
    AppTasks_SetAnchorGlobalInitLed(true);
    AppTasks_SetAnchorLocalInitLed(true);
    AppTasks_SetAnchorNotifyInitLed(false);
    {
        int n = snprintf(line, sizeof(line),
                         "[PROX] request accepted self=0x%04X origin=0x%04X seq=%u window=%ums interval=%ums\r\n",
                         g_app_cfg.short_addr,
                         g_prox_ring_origin_anchor,
                         (unsigned)g_prox_ring_token_seq,
                         (unsigned)PROX_BUILD_WINDOW_MS,
                         (unsigned)PROX_DISCOVERY_INTERVAL_MS);
        if (n > 0) {
            prox_log_direct(line);
        }
    }
    return true;
}

void UwbApp_Task(void *argument)
{
    (void)argument;

    app_log_info("[APP] START short=0x%04X role=%u",
                 g_app_cfg.short_addr, (unsigned)g_app_cfg.role);

    if (g_app_cfg.role == APP_ROLE_ANCHOR) {
        app_log_info("[APP] PROX_TABLE_BUFFER len=%u frags=%u ready=%u",
                     (unsigned)g_tx_len,
                     (unsigned)g_anchor_data.total_frags,
                     g_tx_len > 0U ? 1U : 0U);
    }

    for (;;) {
        drain_link_events();
        prox_ring_notify_poll();
        prox_build_poll();
        tag_data_poll();
        anchor_data_poll();
        osDelay(1);
    }
}
