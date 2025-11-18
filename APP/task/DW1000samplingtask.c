#include "DW1000samplingtask.h"
#include "stdio.h"
#include "bphero_uwb.h"
#include "uwb_device.h"
#include "deca_device_api.h"
#include "trilateration.h"
#include "deca_regs.h"
#include "mydw1000timestamp.h"
#include "uwb_timestamp.h"
#include "EXITcallback.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdint.h"
#include "inttypes.h"
#include "doubleTOchar.h"
#include "app_config.h"
#include "app_log.h"
#include "app_time.h"
#include <stdbool.h>
#include "uwb_timestamp.h"
#include "uwb_frame.h"

TaskHandle_t dw1000samplingTaskNotifyHandle                 = NULL;
static volatile isr_timestamp_packet_t isr_timestamp_packet = {0};

typedef enum {
    TAG_STATE_IDLE,
    TAG_STATE_AWAIT_POLL_TX_CONFIRM,
    TAG_STATE_AWAIT_RESPONSE_RX,
    TAG_STATE_AWAIT_FINAL_TX_CONFIRM,
    TAG_STATE_AWAIT_DISDATA_RX
} Tag_State_t;

typedef enum {
    ANCHOR_STATE_AWAIT_POLL_RX,
    ANCHOR_STATE_AWAIT_RESPONSE_TX_CONFIRM,
    ANCHOR_STATE_AWAIT_FINAL_RX,
    ANCHOR_STATE_AWAIT_DISDATA_TX_CONFIRM
} Anchor_State_t;

void reset_anchor_state_machine(Anchor_State_t *current_anchor_state);
// Force the tag state machine back to IDLE and clear pending notifications
void reset_tag_state_machine(Tag_State_t *current_tag_state);

#define MaxAnchorNum          3

#define UWB_ANCHOR_TABLE_SIZE 3

typedef struct {
    uint16_t short_addr;
    float pos_x;
    float pos_y;
    uwb_timestamp_t poll_tx;
    uwb_timestamp_t poll_rx;
    uwb_timestamp_t resp_tx;
    uwb_timestamp_t resp_rx;
    uwb_timestamp_t final_tx;
    uwb_timestamp_t final_rx;
    double distance_m;
    uint64_t utc_timestamp_ms;
    uint32_t system_tick_ms;
    bool valid;
} uwb_anchor_record_t;

static uwb_anchor_record_t g_anchor_table[UWB_ANCHOR_TABLE_SIZE] = {
    {.short_addr = 0x0032},
    {.short_addr = 0x0033},
    {.short_addr = 0x0034},
};

static uwb_anchor_record_t g_temp_anchor_record = {0};

#define UWB_DELAY_MS 10U

// Tracks the state of the current tag->anchor ranging exchange on the anchor
typedef struct {
    bool active;
    uint16_t tag_addr;
    uint8_t sequence_num;
    uwb_timestamp_t poll_rx_ts;
    uwb_timestamp_t resp_tx_ts;
    uwb_timestamp_t final_rx_ts;
} anchor_session_t;

static anchor_session_t g_anchor_session = {0};

static uwb_anchor_record_t *UWB_FindAnchorRecord(uint16_t short_addr)
{
    for (size_t i = 0; i < UWB_ANCHOR_TABLE_SIZE; ++i) {
        if (g_anchor_table[i].short_addr == short_addr) {
            return &g_anchor_table[i];
        }
    }
    return NULL;
}

static void UWB_ResetAnchorTimestamps(uwb_anchor_record_t *record)
{

    if (record == NULL) {
        return;
    }
    uwb_timestamp_from_u64(0, &record->poll_tx);
    uwb_timestamp_from_u64(0, &record->poll_rx);
    uwb_timestamp_from_u64(0, &record->resp_tx);
    uwb_timestamp_from_u64(0, &record->resp_rx);
    uwb_timestamp_from_u64(0, &record->final_tx);
    uwb_timestamp_from_u64(0, &record->final_rx);
    record->distance_m       = 0.0;
    record->utc_timestamp_ms = 0;
    record->system_tick_ms   = 0;
    record->valid            = false;
}

static void UWB_ClearTempAnchorRecord(void)
{
    uint16_t addr = g_temp_anchor_record.short_addr;
    memset(&g_temp_anchor_record, 0, sizeof(g_temp_anchor_record));
    g_temp_anchor_record.short_addr = addr;
}

static void UWB_CommitTempAnchorRecord(void)
{
    // Persist the most recent ranging exchange into the anchor table with timestamps
    uwb_anchor_record_t *dst = UWB_FindAnchorRecord(g_temp_anchor_record.short_addr);
    if (dst == NULL) {
        return;
    }
    *dst                  = g_temp_anchor_record;
    app_timepoint_t now   = AppTime_Now();
    dst->system_tick_ms   = now.tick_ms;
    dst->utc_timestamp_ms = now.utc_ms;
    dst->valid            = true;
}

static void Anchor_ResetSession(void)
{
    memset(&g_anchor_session, 0, sizeof(g_anchor_session));
}

uint16_t UWB_GetTagFinalDelayMs(void)
{
    return UWB_DELAY_MS;
}

double twr_distance;

dw1000_local_device_t local_device = {0};

static uint8_t dw1000tx_buffer[FRAME_LEN_MAX];
static uint8_t dw1000rx_buffer[FRAME_LEN_MAX];

extern srd_msg_dsss msg_f_send;

static Tag_State_t g_current_tag_state       = TAG_STATE_IDLE;
static Anchor_State_t g_current_anchor_state = ANCHOR_STATE_AWAIT_POLL_RX;

static uint32_t notified_value = 0;

static uint16_t NodeIndex = 0;

// Measurements are pushed to this queue for other tasks to consume
QueueHandle_t dw1000data_queue = NULL;

static float g_anchor_pos_x = 0.0f;
static float g_anchor_pos_y = 0.0f;

// Tag state machine drives POLL/RESP/FINAL exchanges with anchors
void dw1000TagMain(void)
{
    while (1) {
        switch (g_current_tag_state) {
            case TAG_STATE_IDLE: {
                // Trigger next ranging cycle
                osDelay(100);
                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

                uwb_anchor_record_t *active_record = &g_anchor_table[NodeIndex];
                g_temp_anchor_record               = *active_record;
                UWB_ResetAnchorTimestamps(&g_temp_anchor_record);

                local_device.seqNum++;

                uwb_address_t self_addr = {.pan_id = local_device.pan_id, .short_addr = local_device.short_addr};
                uwb_address_t dest_addr = {.pan_id = local_device.pan_id, .short_addr = g_temp_anchor_record.short_addr};
                uwb_frame_t poll_frame;
                uwb_frame_init(&poll_frame, UWB_FRAME_TYPE_POLL, &self_addr, &dest_addr, local_device.seqNum);

                int frame_size = uwb_frame_encode(&poll_frame, dw1000tx_buffer, sizeof(dw1000tx_buffer));
                if (frame_size < 0) {
                    log_error("Failed to encode POLL frame");
                    reset_tag_state_machine(&g_current_tag_state);
                    break;
                }

                dwt_writetxdata(frame_size, dw1000tx_buffer, 0);
                dwt_writetxfctrl(frame_size, 0);
                dwt_starttx(DWT_START_TX_IMMEDIATE);

                g_current_tag_state = TAG_STATE_AWAIT_POLL_TX_CONFIRM;
                break;
            }

            case TAG_STATE_AWAIT_POLL_TX_CONFIRM: {
                // Wait for POLL TX confirmation from DW1000
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_TX_DONE) {
                        g_temp_anchor_record.poll_tx = isr_timestamp_packet.tx;
                        dwt_setrxtimeout(65535);
                        dwt_rxenable(0);
                        g_current_tag_state = TAG_STATE_AWAIT_RESPONSE_RX;
                    } else {
                        log_error("POLL frame interrupt error");
                        UWB_ClearTempAnchorRecord();
                        reset_tag_state_machine(&g_current_tag_state);
                    }
                } else {
                    log_error("AWAIT POLL_TX timeout");
                    UWB_ClearTempAnchorRecord();
                    reset_tag_state_machine(&g_current_tag_state);
                }
                break;
            }

            case TAG_STATE_AWAIT_RESPONSE_RX: {
                // Expecting anchor RESP frame
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_RX_DONE) {
                        uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                        dwt_readrxdata(dw1000rx_buffer, frame_len, 0);

                        uwb_frame_t resp_frame;
                        if (uwb_frame_decode(&resp_frame, dw1000rx_buffer, frame_len) == 0 &&
                            resp_frame.type == UWB_FRAME_TYPE_RESP) {

                            g_temp_anchor_record.resp_rx = isr_timestamp_packet.rx;
                            g_temp_anchor_record.poll_rx = resp_frame.payload.resp.poll_rx;
                            g_temp_anchor_record.resp_tx = resp_frame.payload.resp.resp_tx;

                            uwb_timestamp_t final_tx_ts =
                                uwb_timestamp_add_delay_ms(&g_temp_anchor_record.resp_rx, UWB_DELAY_MS);
                            g_temp_anchor_record.final_tx = final_tx_ts;
                            uint64_t final_ticks          = uwb_timestamp_to_u64(&final_tx_ts);
                            dwt_setdelayedtrxtime((uint32_t)(final_ticks >> 8));

                            uwb_address_t self_addr = {.pan_id = local_device.pan_id, .short_addr = local_device.short_addr};
                            uwb_address_t dest_addr = {.pan_id = local_device.pan_id, .short_addr = g_temp_anchor_record.short_addr};
                            uwb_frame_t final_frame;
                            uwb_frame_init(&final_frame, UWB_FRAME_TYPE_FINAL, &self_addr, &dest_addr, local_device.seqNum);
                            final_frame.payload.final.poll_tx  = g_temp_anchor_record.poll_tx;
                            final_frame.payload.final.poll_rx  = g_temp_anchor_record.poll_rx;
                            final_frame.payload.final.resp_tx  = g_temp_anchor_record.resp_tx;
                            final_frame.payload.final.resp_rx  = g_temp_anchor_record.resp_rx;
                            final_frame.payload.final.final_tx = g_temp_anchor_record.final_tx;

                            int frame_size = uwb_frame_encode(&final_frame, dw1000tx_buffer, sizeof(dw1000tx_buffer));
                            if (frame_size < 0) {
                                log_error("Failed to encode FINAL frame");
                                UWB_ClearTempAnchorRecord();
                                reset_tag_state_machine(&g_current_tag_state);
                                break;
                            }

                            dwt_writetxdata(frame_size, dw1000tx_buffer, 0);
                            dwt_writetxfctrl(frame_size, 0);
                            dwt_starttx(DWT_START_TX_DELAYED);

                            g_current_tag_state = TAG_STATE_AWAIT_FINAL_TX_CONFIRM;
                        } else {
                            log_error("RESPONSE frame decode error");
                            UWB_ClearTempAnchorRecord();
                            reset_tag_state_machine(&g_current_tag_state);
                        }
                    } else {
                        log_error("AWAIT RESPONSE_RX timeout");
                        UWB_ClearTempAnchorRecord();
                        reset_tag_state_machine(&g_current_tag_state);
                    }
                } else {
                    log_error("AWAIT RESPONSE_RX timeout");
                    UWB_ClearTempAnchorRecord();
                    reset_tag_state_machine(&g_current_tag_state);
                }
                break;
            }

            case TAG_STATE_AWAIT_FINAL_TX_CONFIRM: {
                // Confirm FINAL TX before listening for result
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_TX_DONE) {
                        dwt_setrxtimeout(65535);
                        dwt_rxenable(0);
                        g_current_tag_state = TAG_STATE_AWAIT_DISDATA_RX;
                    } else {
                        log_error("AWAIT FINAL_TX timeout");
                        UWB_ClearTempAnchorRecord();
                        reset_tag_state_machine(&g_current_tag_state);
                    }
                } else {
                    log_error("AWAIT FINAL_TX timeout");
                    UWB_ClearTempAnchorRecord();
                    reset_tag_state_machine(&g_current_tag_state);
                }
                break;
            }

            case TAG_STATE_AWAIT_DISDATA_RX: {
                // Await anchor result frame containing timestamps/position
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_RX_DONE) {
                        uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                        dwt_readrxdata(dw1000rx_buffer, frame_len, 0);

                        uwb_frame_t result_frame;
                        if (uwb_frame_decode(&result_frame, dw1000rx_buffer, frame_len) == 0 &&
                            result_frame.type == UWB_FRAME_TYPE_RESULT) {

                            g_temp_anchor_record.poll_tx  = result_frame.payload.result.poll_tx;
                            g_temp_anchor_record.poll_rx  = result_frame.payload.result.poll_rx;
                            g_temp_anchor_record.resp_tx  = result_frame.payload.result.resp_tx;
                            g_temp_anchor_record.resp_rx  = result_frame.payload.result.resp_rx;
                            g_temp_anchor_record.final_tx = result_frame.payload.result.final_tx;
                            g_temp_anchor_record.final_rx = result_frame.payload.result.final_rx;
                            g_temp_anchor_record.pos_x    = result_frame.payload.result.anchor_pos_x;
                            g_temp_anchor_record.pos_y    = result_frame.payload.result.anchor_pos_y;

                            uint64_t poll_tx  = uwb_timestamp_to_u64(&g_temp_anchor_record.poll_tx);
                            uint64_t resp_rx  = uwb_timestamp_to_u64(&g_temp_anchor_record.resp_rx);
                            uint64_t final_tx = uwb_timestamp_to_u64(&g_temp_anchor_record.final_tx);
                            uint64_t poll_rx  = uwb_timestamp_to_u64(&g_temp_anchor_record.poll_rx);
                            uint64_t resp_tx  = uwb_timestamp_to_u64(&g_temp_anchor_record.resp_tx);
                            uint64_t final_rx = uwb_timestamp_to_u64(&g_temp_anchor_record.final_rx);

                            double distance                 = calculate_distance_from_timestamps(poll_tx, resp_rx, final_tx,
                                                                                                 poll_rx, resp_tx, final_rx);
                            g_temp_anchor_record.distance_m = distance;
                            twr_distance                    = distance;

                            UWB_CommitTempAnchorRecord();

                            // Distance queue is disabled for now
                            (void)dw1000data_queue;

                            reset_tag_state_machine(&g_current_tag_state);
                        } else {
                            log_error("RESULT frame decode error");
                            UWB_ClearTempAnchorRecord();
                            reset_tag_state_machine(&g_current_tag_state);
                        }
                    } else {
                        log_error("AWAIT RESULT_RX timeout");
                        UWB_ClearTempAnchorRecord();
                        reset_tag_state_machine(&g_current_tag_state);
                    }
                } else {
                    log_error("AWAIT RESULT_RX timeout");
                    UWB_ClearTempAnchorRecord();
                    reset_tag_state_machine(&g_current_tag_state);
                }
                break;
            }

            default:
                UWB_ClearTempAnchorRecord();
                reset_tag_state_machine(&g_current_tag_state);
                break;
        }
    }
}
void dw1000AnchorMain(void)
{
    // Anchor state machine listens for tag POLL and responds with RESP/RESULT
    reset_anchor_state_machine(&g_current_anchor_state);
    Anchor_ResetSession();

    while (1) {
        switch (g_current_anchor_state) {
            case ANCHOR_STATE_AWAIT_POLL_RX: {
                // Continuously wait for an incoming tag POLL frame
                dwt_setrxtimeout(0);
                dwt_rxenable(0);

                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(3000)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_RX_DONE) {
                        HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                        uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                        dwt_readrxdata(dw1000rx_buffer, frame_len, 0);

                        uwb_frame_t poll_frame;
                        if (uwb_frame_decode(&poll_frame, dw1000rx_buffer, frame_len) == 0) {
                            if (poll_frame.type != UWB_FRAME_TYPE_POLL) {
                                log_warn("Unexpected frame type 0x%02X while waiting for POLL", poll_frame.type);
                                reset_anchor_state_machine(&g_current_anchor_state);
                                break;
                            }

                            Anchor_ResetSession();
                            g_anchor_session.active     = true;
                            g_anchor_session.tag_addr   = poll_frame.header.source_addr;
                            g_anchor_session.poll_rx_ts = isr_timestamp_packet.rx;

                            uwb_timestamp_t resp_tx_ts =
                                uwb_timestamp_add_delay_ms(&g_anchor_session.poll_rx_ts, UWB_DELAY_MS);
                            g_anchor_session.resp_tx_ts = resp_tx_ts;
                            uint64_t resp_ticks         = uwb_timestamp_to_u64(&resp_tx_ts);
                            dwt_setdelayedtrxtime((uint32_t)(resp_ticks >> 8));

                            uwb_address_t self_addr = {.pan_id = local_device.pan_id, .short_addr = local_device.short_addr};
                            uwb_address_t dest_addr = {.pan_id = poll_frame.header.pan_id, .short_addr = poll_frame.header.source_addr};
                            uwb_frame_t resp_frame;
                            uwb_frame_init(&resp_frame, UWB_FRAME_TYPE_RESP, &self_addr, &dest_addr, poll_frame.header.sequence_num);
                            resp_frame.payload.resp.poll_rx = g_anchor_session.poll_rx_ts;
                            resp_frame.payload.resp.resp_tx = resp_tx_ts;

                            int frame_size = uwb_frame_encode(&resp_frame, dw1000tx_buffer, sizeof(dw1000tx_buffer));
                            if (frame_size < 0) {
                                log_error("Failed to encode RESP frame");
                                reset_anchor_state_machine(&g_current_anchor_state);
                                break;
                            }

                            dwt_writetxdata(frame_size, dw1000tx_buffer, 0);
                            dwt_writetxfctrl(frame_size, 0);
                            dwt_starttx(DWT_START_TX_DELAYED);

                            g_current_anchor_state = ANCHOR_STATE_AWAIT_RESPONSE_TX_CONFIRM;
                            break;
                        } else {
                            log_error("Failed to decode POLL frame");
                            reset_anchor_state_machine(&g_current_anchor_state);
                            break;
                        }
                    } else {
                        log_warn("ANCHOR_STATE_AWAIT_POLL_RX unexpected event 0x%08lX", notified_value);
                        reset_anchor_state_machine(&g_current_anchor_state);
                        break;
                    }
                } else {
                    log_warn("ANCHOR_STATE_AWAIT_POLL_RX timeout");
                    reset_anchor_state_machine(&g_current_anchor_state);
                    break;
                }
                break;
            }

            case ANCHOR_STATE_AWAIT_RESPONSE_TX_CONFIRM: {
                // Ensure RESP transmission completed before switching to RX
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_TX_DONE) {
                        dwt_setrxtimeout(65535);
                        dwt_rxenable(0);
                        g_current_anchor_state = ANCHOR_STATE_AWAIT_FINAL_RX;
                        break;
                    } else {
                        log_error("Response TX confirm unexpected event 0x%08lX", notified_value);
                        reset_anchor_state_machine(&g_current_anchor_state);
                        break;
                    }
                } else {
                    log_warn("ANCHOR_STATE_AWAIT_RESPONSE_TX_CONFIRM timeout");
                    reset_anchor_state_machine(&g_current_anchor_state);
                    break;
                }
                break;
            }

            case ANCHOR_STATE_AWAIT_FINAL_RX: {
                // Expect the tag FINAL frame and compute range once received
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(200)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_RX_DONE) {
                        uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                        dwt_readrxdata(dw1000rx_buffer, frame_len, 0);

                        uwb_frame_t final_frame;
                        if (uwb_frame_decode(&final_frame, dw1000rx_buffer, frame_len) == 0 &&
                            final_frame.type == UWB_FRAME_TYPE_FINAL &&
                            g_anchor_session.active &&
                            final_frame.header.source_addr == g_anchor_session.tag_addr) {

                            g_anchor_session.final_rx_ts = isr_timestamp_packet.rx;

                            double distance = calculate_distance_from_timestamps(
                                uwb_timestamp_to_u64(&final_frame.payload.final.poll_tx),
                                uwb_timestamp_to_u64(&final_frame.payload.final.resp_rx),
                                uwb_timestamp_to_u64(&final_frame.payload.final.final_tx),
                                uwb_timestamp_to_u64(&g_anchor_session.poll_rx_ts),
                                uwb_timestamp_to_u64(&g_anchor_session.resp_tx_ts),
                                uwb_timestamp_to_u64(&g_anchor_session.final_rx_ts));
                            twr_distance = distance;
                            log_info("Anchor distance %.2f m -> tag 0x%04X", distance, g_anchor_session.tag_addr);

                            uwb_address_t self_addr = {.pan_id = local_device.pan_id, .short_addr = local_device.short_addr};
                            uwb_address_t dest_addr = {.pan_id = final_frame.header.pan_id, .short_addr = final_frame.header.source_addr};
                            uwb_frame_t result_frame;
                            uwb_frame_init(&result_frame, UWB_FRAME_TYPE_RESULT, &self_addr, &dest_addr, final_frame.header.sequence_num);
                            result_frame.payload.result.poll_tx      = final_frame.payload.final.poll_tx;
                            result_frame.payload.result.poll_rx      = g_anchor_session.poll_rx_ts;
                            result_frame.payload.result.resp_tx      = g_anchor_session.resp_tx_ts;
                            result_frame.payload.result.resp_rx      = final_frame.payload.final.resp_rx;
                            result_frame.payload.result.final_tx     = final_frame.payload.final.final_tx;
                            result_frame.payload.result.final_rx     = g_anchor_session.final_rx_ts;
                            result_frame.payload.result.anchor_pos_x = g_anchor_pos_x;
                            result_frame.payload.result.anchor_pos_y = g_anchor_pos_y;

                            int frame_size = uwb_frame_encode(&result_frame, dw1000tx_buffer, sizeof(dw1000tx_buffer));
                            if (frame_size < 0) {
                                log_error("Failed to encode RESULT frame");
                                reset_anchor_state_machine(&g_current_anchor_state);
                                break;
                            }

                            dwt_writetxdata(frame_size, dw1000tx_buffer, 0);
                            dwt_writetxfctrl(frame_size, 0);
                            dwt_starttx(DWT_START_TX_IMMEDIATE);

                            g_current_anchor_state = ANCHOR_STATE_AWAIT_DISDATA_TX_CONFIRM;
                            break;
                        } else {
                            log_error("Invalid FINAL frame");
                            reset_anchor_state_machine(&g_current_anchor_state);
                            break;
                        }
                    } else if (notified_value & UWB_EVENT_RX_TIMEOUT) {
                        log_warn("ANCHOR_STATE_AWAIT_FINAL_RX timeout waiting for FINAL");
                        reset_anchor_state_machine(&g_current_anchor_state);
                        break;
                    } else {
                        log_error("ANCHOR_STATE_AWAIT_FINAL_RX unexpected event 0x%08lX", notified_value);
                        reset_anchor_state_machine(&g_current_anchor_state);
                        break;
                    }
                } else {
                    log_warn("ANCHOR_STATE_AWAIT_FINAL_RX timeout (no event)");
                    reset_anchor_state_machine(&g_current_anchor_state);
                    break;
                }
                break;
            }

            case ANCHOR_STATE_AWAIT_DISDATA_TX_CONFIRM: {
                // Wait for RESULT frame completion before resetting session
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_TX_DONE) {
                        log_info("RESULT frame sent to tag 0x%04X", g_anchor_session.tag_addr);
                        Anchor_ResetSession();
                        reset_anchor_state_machine(&g_current_anchor_state);
                        break;
                    } else {
                        log_error("RESULT TX confirm unexpected event 0x%08lX", notified_value);
                        reset_anchor_state_machine(&g_current_anchor_state);
                        break;
                    }
                } else {
                    log_warn("ANCHOR_STATE_AWAIT_RESULT_TX timeout");
                    reset_anchor_state_machine(&g_current_anchor_state);
                    break;
                }
                break;
            }

            default:
                reset_anchor_state_machine(&g_current_anchor_state);
                break;
        }
    }
}
typedef struct {
    uint8_t frameCtrl[2];
    uint16_t pan_id;
    uint16_t short_addr;
} dw1000_local_flashData_t;

void DW1000samplingtask(void *argument)
{

    dw1000samplingTaskNotifyHandle =
        xTaskGetCurrentTaskHandle();
    UWB_DeviceInitFromConfig();

    const app_config_t *cfg = AppConfig_Get();
    app_device_role_t role  = APP_DEVICE_ROLE_ANCHOR;
    if (cfg != NULL) {
        if (cfg->device_role == APP_DEVICE_ROLE_TAG || cfg->device_role == APP_DEVICE_ROLE_ANCHOR) {
            role = (app_device_role_t)cfg->device_role;
        } else {
            log_warn("Invalid role %u in config, defaulting to anchor", cfg->device_role);
        }
    } else {
        log_warn("AppConfig_Get returned NULL, defaulting to anchor role");
    }

    if (role == APP_DEVICE_ROLE_TAG) {
        log_info("DW1000 task running in TAG mode");
        dw1000TagMain();
    } else {
        log_info("DW1000 task running in ANCHOR mode");
        dw1000AnchorMain();
    }
}

void uwb_isr_handler(void)
{
    // DW1000 interrupt handler: capture timestamps and map status bits to task events
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    UWB_Event_t event_to_notify = UWB_EVENT_NONE;

    static uint32_t status_reg;

    status_reg = dwt_read32bitoffsetregFromISR(SYS_STATUS_ID, 0);

    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
    uwb_timestamp_read_rx_isr(&isr_timestamp_packet.rx);
    uwb_timestamp_read_tx_isr(&isr_timestamp_packet.tx);
    taskEXIT_CRITICAL_FROM_ISR(saved);

    if (status_reg & SYS_STATUS_TXFRS) {

        event_to_notify |= UWB_EVENT_TX_DONE;
    }

    if (status_reg & SYS_STATUS_RXFCG) {

        event_to_notify |= UWB_EVENT_RX_DONE;
    }

    if (status_reg & SYS_STATUS_RXRFTO) {

        event_to_notify |= UWB_EVENT_RX_TIMEOUT;
    }
    if (status_reg & SYS_STATUS_RXSFDTO) {

        event_to_notify |= UWB_EVENT_SFD_DONE;
    }
    if (status_reg & SYS_STATUS_RXPTO) {

        event_to_notify |= UWB_EVENT_PRE_DONE;
    }

    if (status_reg & SYS_STATUS_AFFREJ) {

        event_to_notify |= UWB_EVENT_FRAME_REJECTED;
    }

    if (status_reg & (SYS_STATUS_RXFCE | SYS_STATUS_RXPHE | SYS_STATUS_RXRFSL)) {

        event_to_notify |= UWB_EVENT_RX_ERROR;
    }

    dwt_write32bitoffsetregFromISR(SYS_STATUS_ID, 0, status_reg);

    if (event_to_notify != UWB_EVENT_NONE &&
        dw1000samplingTaskNotifyHandle != NULL) {
        xTaskNotifyFromISR(dw1000samplingTaskNotifyHandle, event_to_notify,
                           eSetBits, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int16_t clear_node_profile(uwb_node_profile_t *node_profile)
{

    if (node_profile == NULL) {
        return -1;
    }
    uint16_t short_addrtemp = node_profile->short_addr;

    memset(node_profile, 0, sizeof(uwb_node_profile_t));

    node_profile->short_addr = short_addrtemp;

    return 0;
}

void apply_dw1000_optimizations(const dwt_config_t *config)
{

    dwt_writetodevice(AGC_CTRL_ID, AGC_TUNE1_OFFSET, 2, (uint8_t[]){0x9B, 0x88});

    dwt_write32bitoffsetreg(AGC_CTRL_ID, AGC_TUNE2_OFFSET, 0x2502A907);

    dwt_writetodevice(DRX_CONF_ID, DRX_TUNE2_OFFSET, 4, (uint8_t[]){0x5E, 0x01, 0x3B, 0x35});

    dwt_writetodevice(LDE_IF_ID, LDE_CFG2_OFFSET, 2, (uint8_t[]){0x07, 0x06});

    if (config->chan == 5) {

        dwt_writetodevice(RF_CONF_ID, RF_TXCTRL_OFFSET, 3, (uint8_t[]){0xE0, 0x3F, 0x1E});
        dwt_writetodevice(TX_CAL_ID, TC_PGDELAY_OFFSET, 1, (uint8_t[]){0xC0});
        dwt_writetodevice(FS_CTRL_ID, FS_PLLTUNE_OFFSET, 1, (uint8_t[]){0xBE});
    }

    if (config->chan == 2) {

        dwt_writetodevice(RF_CONF_ID, RF_TXCTRL_OFFSET, 3, (uint8_t[]){0xA0, 0x5C, 0x04});
        dwt_writetodevice(TX_CAL_ID, TC_PGDELAY_OFFSET, 1, (uint8_t[]){0xC2});
        dwt_writetodevice(FS_CTRL_ID, FS_PLLTUNE_OFFSET, 1, (uint8_t[]){0x26});
    }
}

double calculate_distance_from_timestamps(uint64_t tag_poll_tx_ts, uint64_t tag_resp_rx_ts, uint64_t tag_final_tx_ts,
                                          uint64_t anchor_poll_rx_ts, uint64_t anchor_resp_tx_ts, uint64_t anchor_final_rx_ts)
{

    double T_round_tag    = (double)get_timestamp_difference_u64(tag_resp_rx_ts, tag_poll_tx_ts);
    double T_reply_anchor = (double)get_timestamp_difference_u64(anchor_resp_tx_ts, anchor_poll_rx_ts);
    double T_round_anchor = (double)get_timestamp_difference_u64(anchor_final_rx_ts, anchor_resp_tx_ts);
    double T_reply_tag    = (double)get_timestamp_difference_u64(tag_final_tx_ts, tag_resp_rx_ts);

    double Tprop_ticks;
    double numerator   = (T_round_tag * T_round_anchor) - (T_reply_tag * T_reply_anchor);
    double denominator = T_round_tag + T_round_anchor + T_reply_tag + T_reply_anchor;

    if (denominator == 0) {
        return -1.0;
    }

    Tprop_ticks = numerator / denominator;

    double Tprop_seconds = Tprop_ticks * DWT_TIME_UNITS;

    double distance_meters = Tprop_seconds * SPEED_OF_LIGHT;

    return distance_meters;
}

#define RX_SOFTRESET_BIT_MASK   (1UL << 28)
#define ALL_SOFTRESET_BITS_MASK (0xFUL << 28)

void reset_tag_state_machine(Tag_State_t *current_tag_state)
{
    // Disable DW1000 interrupts while tearing down the state

    uint32_t interrupt_mask = DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RFCE

                              | DWT_INT_RXPTO | DWT_INT_SFDT

        ;

    dwt_setinterrupt(interrupt_mask, 0);

    dwt_forcetrxoff();

    uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    dwt_write32bitreg(SYS_STATUS_ID, status_reg);

    uint32_t pmsc_ctrl0_value;

    pmsc_ctrl0_value = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);

    dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl0_value & ~RX_SOFTRESET_BIT_MASK);
    osDelay(2);
    dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, (pmsc_ctrl0_value & ~ALL_SOFTRESET_BITS_MASK) | ALL_SOFTRESET_BITS_MASK);

    if (dw1000samplingTaskNotifyHandle != NULL) {
        xTaskNotifyStateClear(dw1000samplingTaskNotifyHandle);
    } else {
        xTaskNotifyStateClear(NULL);
    }

    g_current_tag_state = TAG_STATE_IDLE;
    if (current_tag_state != &g_current_tag_state) {
        *current_tag_state = TAG_STATE_IDLE;
    }

    dwt_setinterrupt(interrupt_mask, 1);
}

// Reset anchor side DW1000 into RX mode for the next ranging exchange
void reset_anchor_state_machine(Anchor_State_t *current_anchor_state)
{

    uint32_t interrupt_mask = DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RFCE

                              | DWT_INT_RXPTO | DWT_INT_SFDT

        ;

    dwt_setinterrupt(interrupt_mask, 0);

    dwt_forcetrxoff();

    uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    dwt_write32bitreg(SYS_STATUS_ID, status_reg);

    uint32_t pmsc_ctrl0_value;

    pmsc_ctrl0_value = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);

    dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl0_value & ~RX_SOFTRESET_BIT_MASK);
    osDelay(2);
    dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, (pmsc_ctrl0_value & ~ALL_SOFTRESET_BITS_MASK) | ALL_SOFTRESET_BITS_MASK);

    if (dw1000samplingTaskNotifyHandle != NULL) {
        xTaskNotifyStateClear(dw1000samplingTaskNotifyHandle);
    } else {
        xTaskNotifyStateClear(NULL);
    }

    *current_anchor_state = ANCHOR_STATE_AWAIT_POLL_RX;
    Anchor_ResetSession();

    dwt_setinterrupt(interrupt_mask, 1);
    dwt_setrxtimeout(0);
    dwt_rxenable(0);
}

void UWBMssageInit(void)
{
    const app_config_t *cfg = AppConfig_Get();
    if (cfg == NULL) {
        cfg = AppConfig_GetDefaults();
        log_warn("AppConfig unavailable, using defaults for anchor metadata");
    }

    if (cfg == NULL) {
        log_error("AppConfig defaults unavailable");
        return;
    }

    g_anchor_pos_x = cfg->anchor_pos_x;
    g_anchor_pos_y = cfg->anchor_pos_y;
    const char *role_str;
    if (cfg->device_role == APP_DEVICE_ROLE_ANCHOR) {
        role_str = "Anchor";
    } else if (cfg->device_role == APP_DEVICE_ROLE_TAG) {
        role_str = "Tag";
    } else {
        role_str = "Unknown";
    }
    log_info("DW1000 identity updated role=%s PAN=0x%04X short=0x%04X", role_str, cfg->pan_id, cfg->short_addr);
}
