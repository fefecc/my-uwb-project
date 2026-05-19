/**
 * @file uwb_app.c
 * @brief APP 层 - TWR 测距计算 + plan-v4 数据会话
 */

#include "uwb_app.h"

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
#include "main.h"

#define UWB_APP_DW_TIME_UNIT      (1.0 / (499.2e6 * 128.0))
#define UWB_APP_SPEED_OF_LIGHT    (299702547.0)
#define UWB_APP_ANT_DELAY_COMP_M  (0.0)

#define TWR_MAX_ANCHORS  4
#define TWR_DS_MAX_GAP_MS  12U
#define TWR_DS_MAX_GAP_TICKS \
    ((uint64_t)((TIME_SERVICE_LOCAL_TICKS_PER_SECOND * TWR_DS_MAX_GAP_MS) / 1000U))

#define DATA_BUF_SIZE              512U
#define DATA_WAIT_RETRY_MS         10U
#define DATA_SESSION_TIMEOUT_MS    5000U
#define DATA_SESSION_INTERVAL_MS   1000U
#define DATA_META_PAYLOAD_LEN      6U
#define DATA_FRAG_CRC_LEN          2U

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
    uint32_t next_session_ms;
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

static UwbStackConfig g_app_cfg;
static twr_anchor_record_t g_anchor_records[TWR_MAX_ANCHORS];
static TaskHandle_t g_uwb_app_task;

static uint8_t  g_tx_buf[DATA_BUF_SIZE];
static uint16_t g_tx_len;
static uint8_t  g_rx_buf[DATA_BUF_SIZE];
static uint16_t g_rx_len;
static uint8_t  g_expected_buf[DATA_BUF_SIZE];
static uint16_t g_expected_len;

static uint16_t g_last_anchor_id;
static uint32_t g_last_anchor_seen_ms;
static uint16_t g_next_session_id = 1U;
static tag_data_context_t g_tag_data;
static anchor_data_context_t g_anchor_data;

static const char *g_test_sentences[] = {
    "Hello from Anchor! This is sentence one for testing.",
    "UWB data transfer works. This is sentence two.",
    "STM32H7 + DW1000. This is sentence three.",
    "FreeRTOS RTOS running. This is sentence four.",
    "Data link complete. This is sentence five.",
};
#define TEST_SENTENCE_COUNT  5

/* ================================================================
 *  工具
 * ================================================================ */

static uint16_t min_u16(uint16_t a, uint16_t b)
{
    return (a < b) ? a : b;
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

static bool append_payload_bytes(uint8_t *dst, uint16_t dst_size,
                                 uint16_t *len,
                                 const uint8_t *src, uint16_t src_len)
{
    if (dst == NULL || len == NULL || src == NULL) return false;
    if ((uint32_t)(*len) + src_len > dst_size) return false;

    memcpy(&dst[*len], src, src_len);
    *len = (uint16_t)(*len + src_len);
    return true;
}

static uint16_t build_fake_payload(uint8_t *dst, uint16_t dst_size)
{
    uint16_t len = 0;
    if (dst == NULL || dst_size == 0U) return 0;

    for (int i = 0; i < TEST_SENTENCE_COUNT; i++) {
        uint16_t slen = (uint16_t)strlen(g_test_sentences[i]);
        if (!append_payload_bytes(dst, dst_size, &len,
                                  (const uint8_t *)g_test_sentences[i],
                                  slen)) {
            break;
        }

        if (i < TEST_SENTENCE_COUNT - 1) {
            uint8_t nl = (uint8_t)'\n';
            if (!append_payload_bytes(dst, dst_size, &len, &nl, 1U)) {
                break;
            }
        }
    }

    return len;
}

static bool find_first_diff(const uint8_t *a, const uint8_t *b,
                            uint16_t len, uint16_t *index)
{
    if (a == NULL || b == NULL || index == NULL) return false;

    for (uint16_t i = 0; i < len; i++) {
        if (a[i] != b[i]) {
            *index = i;
            return true;
        }
    }
    return false;
}

static void log_payload_lines(const char *prefix, const uint8_t *buf, uint16_t len)
{
    if (prefix == NULL || buf == NULL) return;

    uint16_t off = 0;
    while (off < len) {
        uint16_t end = off;
        while (end < len && buf[end] != '\n') end++;
        uint16_t line_len = end - off;
        if (line_len > 0U) {
            char line[128];
            uint16_t copy_len = min_u16(line_len, (uint16_t)(sizeof(line) - 1U));
            memcpy(line, &buf[off], copy_len);
            line[copy_len] = '\0';
            app_log_info("[APP] %s %s", prefix, line);
        }
        off = (end < len) ? (uint16_t)(end + 1U) : end;
    }
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

static bool twr_published_gap_ok(uint64_t last_published_frame_local_tick_20k,
                                 uint64_t candidate_frame_local_tick_20k)
{
    if (candidate_frame_local_tick_20k == 0U) {
        return false;
    }
    if (last_published_frame_local_tick_20k == 0U) {
        return true;
    }
    if (candidate_frame_local_tick_20k < last_published_frame_local_tick_20k) {
        return false;
    }
    return (candidate_frame_local_tick_20k - last_published_frame_local_tick_20k) <=
           TWR_DS_MAX_GAP_TICKS;
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

    if (g_app_cfg.role == APP_ROLE_TAG) {
        g_last_anchor_id = exchange->anchor_id;
        g_last_anchor_seen_ms = HAL_GetTick();
    }

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
        bool gap_ok = twr_published_gap_ok(rec->last_published_frame_local_tick_20k,
                                           result.frame_local_tick_20k);
        app_log_info("[APP] TWR_DBG_GAP anchor=0x%04X seq=%u last_ms=%.3f cand_ms=%.3f gap_ok=%u",
                     exchange->anchor_id,
                     (unsigned)exchange->exchange_seq,
                     local_tick_to_ms(rec->last_published_frame_local_tick_20k),
                     local_tick_to_ms(result.frame_local_tick_20k),
                     (unsigned)gap_ok);

        if (!gap_ok) {
            app_log_warn("[APP] TWR_DBG_DS_DROP anchor=0x%04X seq=%u action=rebuild_window reason=gap",
                         exchange->anchor_id,
                         (unsigned)exchange->exchange_seq);
            rec->last_published_frame_local_tick_20k = 0U;
            twr_record_exchange(rec, exchange);
            return;
        }
    }

    result.status_flags |= 0x01;
    publish_range_result(&result);
    log_twr_frame("DS", exchange, &result);
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

static bool tag_verify_fake_payload(void)
{
    uint16_t got_crc = crc16_ccitt(g_rx_buf, g_rx_len);
    uint16_t exp_crc = crc16_ccitt(g_expected_buf, g_expected_len);

    if (g_rx_len != g_expected_len ||
        g_tag_data.total_len != g_expected_len ||
        g_tag_data.total_crc != exp_crc ||
        got_crc != exp_crc ||
        memcmp(g_rx_buf, g_expected_buf, g_expected_len) != 0) {
        uint16_t cmp_len = min_u16(g_rx_len, g_expected_len);
        uint16_t diff = 0;
        bool has_diff = find_first_diff(g_rx_buf, g_expected_buf,
                                        cmp_len, &diff);

        app_log_warn("[APP] DATA_VERIFY_FAIL rx_len=%u exp_len=%u rx_crc=0x%04X exp_crc=0x%04X meta_crc=0x%04X",
                     g_rx_len, g_expected_len, got_crc, exp_crc,
                     g_tag_data.total_crc);
        if (has_diff) {
            app_log_warn("[APP] DATA_VERIFY_DIFF off=%u got=0x%02X exp=0x%02X",
                         diff, g_rx_buf[diff], g_expected_buf[diff]);
        }
        return false;
    }

    app_log_info("[APP] DATA_VERIFY_OK len=%u frags=%u crc=0x%04X",
                 g_rx_len, (unsigned)g_tag_data.total_frags, got_crc);
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
        g_tag_data.next_session_ms = HAL_GetTick() + DATA_SESSION_INTERVAL_MS;
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
        g_tag_data.next_session_ms = HAL_GetTick() + DATA_SESSION_INTERVAL_MS;
        tag_reset_session();
        return;
    }

    app_log_warn("[APP] DATA_SESSION_FAIL anchor=0x%04X sess=0x%04X",
                 g_tag_data.target_id, g_tag_data.session_id);
    tag_log_data_stats("DATA_STATS_FAIL");
    tag_send_reset_cmd(g_tag_data.session_id);
    g_tag_data.next_session_ms = HAL_GetTick() + DATA_SESSION_INTERVAL_MS;
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

    if (!tag_verify_fake_payload()) {
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

    if (g_tag_data.state == TAG_DATA_IDLE &&
        g_last_anchor_id != 0U &&
        now >= g_tag_data.next_session_ms &&
        now - g_last_anchor_seen_ms < DATA_SESSION_TIMEOUT_MS) {
        tag_start_session(g_last_anchor_id);
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
    memset(g_expected_buf, 0, sizeof(g_expected_buf));
    memset(&g_tag_data, 0, sizeof(g_tag_data));
    memset(&g_anchor_data, 0, sizeof(g_anchor_data));
    g_tx_len = 0;
    g_rx_len = 0;
    g_expected_len = build_fake_payload(g_expected_buf, sizeof(g_expected_buf));
    g_last_anchor_id = 0;
    g_last_anchor_seen_ms = 0;
    g_tag_data.next_session_ms = DATA_SESSION_INTERVAL_MS;

    if (cfg->role == APP_ROLE_ANCHOR) {
        memcpy(g_tx_buf, g_expected_buf, g_expected_len);
        g_tx_len = g_expected_len;
        g_anchor_data.total_frags = calc_total_frags(g_tx_len);
    }

    return true;
}

bool UwbApp_StartThread(const osThreadAttr_t *attr)
{
    if (g_uwb_app_task != NULL) return true;
    g_uwb_app_task = osThreadNew(UwbApp_Task, NULL, attr);
    return g_uwb_app_task != NULL;
}

void UwbApp_Task(void *argument)
{
    (void)argument;

    app_log_info("[APP] START short=0x%04X role=%u",
                 g_app_cfg.short_addr, (unsigned)g_app_cfg.role);

    if (g_app_cfg.role == APP_ROLE_ANCHOR && g_tx_len > 0) {
        app_log_info("[APP] DATA_FAKE_READY len=%u frags=%u crc=0x%04X",
                     (unsigned)g_tx_len,
                     (unsigned)g_anchor_data.total_frags,
                     crc16_ccitt(g_tx_buf, g_tx_len));
        log_payload_lines(">", g_tx_buf, g_tx_len);
    } else if (g_app_cfg.role == APP_ROLE_TAG && g_expected_len > 0) {
        app_log_info("[APP] DATA_FAKE_EXPECT len=%u frags=%u crc=0x%04X",
                     (unsigned)g_expected_len,
                     (unsigned)calc_total_frags(g_expected_len),
                     crc16_ccitt(g_expected_buf, g_expected_len));
    }

    for (;;) {
        drain_link_events();
        tag_data_poll();
        anchor_data_poll();
        osDelay(1);
    }
}
