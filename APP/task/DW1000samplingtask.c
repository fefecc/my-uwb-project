#include "DW1000samplingtask.h"
#include "stdio.h"
#include "bphero_uwb.h"
#include "uwb_device.h"
#include "deca_device_api.h"
#include "trilateration.h"
#include "deca_regs.h"
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
#include <math.h> // fabs

#define TAG_LOG_VERBOSE 0

#if TAG_LOG_VERBOSE
#define TAG_LOG_ERROR(...) log_error(__VA_ARGS__)
#define TAG_LOG_WARN(...)  log_warn(__VA_ARGS__)
#define TAG_LOG_INFO(...)  log_info(__VA_ARGS__)
#else
#define TAG_LOG_ERROR(...) ((void)0)
#define TAG_LOG_WARN(...)  ((void)0)
#define TAG_LOG_INFO(...)  log_info(__VA_ARGS__)
#endif

TaskHandle_t dw1000samplingTaskNotifyHandle                 = NULL;
static volatile isr_timestamp_packet_t isr_timestamp_packet = {0};

typedef enum {
    TAG_STATE_IDLE,
    TAG_STATE_AWAIT_POLL_TX_CONFIRM,
    TAG_STATE_AWAIT_RESPONSE_RX,
    TAG_STATE_AWAIT_FINAL_TX_CONFIRM,
    TAG_STATE_AWAIT_RESULT_RX
} Tag_State_t;

typedef enum {
    ANCHOR_STATE_AWAIT_POLL_RX,
    ANCHOR_STATE_AWAIT_RESPONSE_TX_CONFIRM,
    ANCHOR_STATE_AWAIT_FINAL_RX,
    ANCHOR_STATE_AWAIT_RESULT_TX_CONFIRM
} Anchor_State_t;

void reset_anchor_state_machine(Anchor_State_t *current_anchor_state);
void reset_tag_state_machine(Tag_State_t *current_tag_state);

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

typedef struct {
    bool active;
    uint16_t tag_addr;
    uint16_t tag_pan_id;
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

static void UWB_CommitTempAnchorRecord(void)
{
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

QueueHandle_t dw1000data_queue = NULL;

static float g_anchor_pos_x = 0.0f;
static float g_anchor_pos_y = 0.0f;

void dw1000TagMain(void)
{
    while (1) {
        switch (g_current_tag_state) {
            case TAG_STATE_IDLE: {
                //  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                osDelay(30);

                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

                uwb_anchor_record_t *active_record = &g_anchor_table[NodeIndex];
                g_temp_anchor_record               = *active_record;
                UWB_ResetAnchorTimestamps(&g_temp_anchor_record);

                local_device.seqNum++; // seq ++

                uwb_address_t self_addr = {.pan_id = local_device.pan_id, .short_addr = local_device.short_addr};
                uwb_address_t dest_addr = {.pan_id = local_device.pan_id, .short_addr = g_temp_anchor_record.short_addr};
                uwb_frame_t poll_frame;
                uwb_frame_init(&poll_frame, UWB_FRAME_TYPE_POLL, &self_addr, &dest_addr, local_device.seqNum);

                int frame_size = uwb_frame_encode(&poll_frame, dw1000tx_buffer, sizeof(dw1000tx_buffer));

                if (frame_size < 0) {
                    log_error("poll frame pack failed, seq = %u", local_device.seqNum);
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
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(100)) == pdTRUE &&
                    (notified_value & UWB_EVENT_TX_DONE)) {
                    g_temp_anchor_record.poll_tx = isr_timestamp_packet.tx; // 发送成功
                    dwt_setrxtimeout(65535);
                    dwt_rxenable(0);
                    g_current_tag_state = TAG_STATE_AWAIT_RESPONSE_RX;
                } else {
                    log_error("poll frame TX failed, seq=%u ", local_device.seqNum);
                    reset_tag_state_machine(&g_current_tag_state);
                }
                break;
            }

            case TAG_STATE_AWAIT_RESPONSE_RX: {
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_RX_DONE) {
                        uint16_t rx_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                        dwt_readrxdata(dw1000rx_buffer, rx_len, 0);
                        uwb_frame_t rx_frame;
                        if (uwb_frame_decode(&rx_frame, dw1000rx_buffer, rx_len) == 0 &&
                            rx_frame.type == UWB_FRAME_TYPE_RESP) {
                            g_temp_anchor_record.resp_rx = isr_timestamp_packet.rx;
                            g_temp_anchor_record.poll_rx = rx_frame.payload.resp.poll_rx;
                            g_temp_anchor_record.resp_tx = rx_frame.payload.resp.resp_tx;

                            uwb_address_t self_addr = {.pan_id = local_device.pan_id, .short_addr = local_device.short_addr};
                            uwb_address_t dest_addr = {.pan_id = rx_frame.header.pan_id, .short_addr = rx_frame.header.source_addr};
                            uwb_frame_t final_frame;
                            uwb_frame_init(&final_frame, UWB_FRAME_TYPE_FINAL, &self_addr, &dest_addr, local_device.seqNum);

                            uwb_timestamp_t final_tx_planned = uwb_timestamp_add_delay_ms(&g_temp_anchor_record.resp_rx, UWB_DELAY_MS);
                            g_temp_anchor_record.final_tx    = final_tx_planned;

                            final_frame.payload.final.poll_tx  = g_temp_anchor_record.poll_tx;
                            final_frame.payload.final.poll_rx  = g_temp_anchor_record.poll_rx;
                            final_frame.payload.final.resp_tx  = g_temp_anchor_record.resp_tx;
                            final_frame.payload.final.resp_rx  = g_temp_anchor_record.resp_rx;
                            final_frame.payload.final.final_tx = final_tx_planned;

                            int final_len = uwb_frame_encode(&final_frame, dw1000tx_buffer, sizeof(dw1000tx_buffer));
                            if (final_len > 0) {
                                uint64_t final_ticks = uwb_timestamp_to_u64(&final_tx_planned);
                                dwt_setdelayedtrxtime((uint32_t)(final_ticks >> 8));
                                dwt_writetxdata(final_len, dw1000tx_buffer, 0);
                                dwt_writetxfctrl(final_len, 0);
                                dwt_starttx(DWT_START_TX_DELAYED);
                                g_current_tag_state = TAG_STATE_AWAIT_FINAL_TX_CONFIRM;
                            } else {
                                log_error("final frame encode failed seq=%u", local_device.seqNum);
                                reset_tag_state_machine(&g_current_tag_state);
                            }
                        } else {
                            log_error("RESP recceived failed seq=%u", rx_frame.header.sequence_num);
                            reset_tag_state_machine(&g_current_tag_state);
                        }
                    } else {
                        log_error("resp recceived failed notified_value=%u", notified_value);
                        reset_tag_state_machine(&g_current_tag_state);
                    }
                } else {
                    log_error("resp recceived failed");
                    reset_tag_state_machine(&g_current_tag_state);
                }
                break;
            }

            case TAG_STATE_AWAIT_FINAL_TX_CONFIRM: {
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(100)) == pdTRUE &&
                    (notified_value & UWB_EVENT_TX_DONE)) {
                    g_temp_anchor_record.final_tx = isr_timestamp_packet.tx;
                    dwt_setrxtimeout(65535);
                    dwt_rxenable(0);
                    g_current_tag_state = TAG_STATE_AWAIT_RESULT_RX;
                } else {
                    log_error("FINAL tx failed seq=%u", local_device.seqNum);
                    reset_tag_state_machine(&g_current_tag_state);
                }
                break;
            }

            case TAG_STATE_AWAIT_RESULT_RX: {
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(200)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_RX_DONE) {
                        uint16_t rx_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                        dwt_readrxdata(dw1000rx_buffer, rx_len, 0);
                        uwb_frame_t rx_frame;
                        if (uwb_frame_decode(&rx_frame, dw1000rx_buffer, rx_len) == 0 &&
                            rx_frame.type == UWB_FRAME_TYPE_RESULT) {
                            g_temp_anchor_record.final_rx = rx_frame.payload.result.final_rx;
                            g_temp_anchor_record.pos_x    = rx_frame.payload.result.anchor_pos_x;
                            g_temp_anchor_record.pos_y    = rx_frame.payload.result.anchor_pos_y;

                            uint64_t tag_poll_tx    = uwb_timestamp_to_u64(&g_temp_anchor_record.poll_tx);
                            uint64_t tag_resp_rx    = uwb_timestamp_to_u64(&g_temp_anchor_record.resp_rx);
                            uint64_t tag_final_tx   = uwb_timestamp_to_u64(&g_temp_anchor_record.final_tx);
                            uint64_t anchor_poll_rx = uwb_timestamp_to_u64(&rx_frame.payload.result.poll_rx);
                            uint64_t anchor_resp_tx = uwb_timestamp_to_u64(&rx_frame.payload.result.resp_tx);
                            uint64_t anchor_final_rx =
                                uwb_timestamp_to_u64(&rx_frame.payload.result.final_rx);

                            g_temp_anchor_record.poll_rx  = rx_frame.payload.result.poll_rx;
                            g_temp_anchor_record.resp_tx  = rx_frame.payload.result.resp_tx;
                            g_temp_anchor_record.final_rx = rx_frame.payload.result.final_rx;

                            double dist = calculate_distance_from_timestamps_v2(tag_poll_tx,
                                                                                anchor_poll_rx,
                                                                                anchor_resp_tx,
                                                                                tag_resp_rx,
                                                                                tag_final_tx,
                                                                                anchor_final_rx);

                            g_temp_anchor_record.distance_m = dist;
                            UWB_CommitTempAnchorRecord();

                            log_info("RESULT seq=%d dist=%.3f m ", rx_frame.header.sequence_num, dist);

                            g_current_tag_state = TAG_STATE_IDLE;
                        } else {
                            log_error("RESULT decode/type failed seq=%u", rx_frame.header.sequence_num);
                            reset_tag_state_machine(&g_current_tag_state);
                        }
                    } else {
                        log_error("result wait failed event=0x%08lX", notified_value);
                        reset_tag_state_machine(&g_current_tag_state);
                    }
                } else {
                    log_error("result wait timeout");
                    reset_tag_state_machine(&g_current_tag_state);
                }
                break;
            }

            default:
                g_current_tag_state = TAG_STATE_IDLE;
                break;
        }
    }
}

void dw1000AnchorMain(void)
{
    Anchor_ResetSession(); // clear anchor session
    reset_anchor_state_machine(&g_current_anchor_state);

    while (1) {

        switch (g_current_anchor_state) {
            case ANCHOR_STATE_AWAIT_POLL_RX: {

                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value,
                                    pdMS_TO_TICKS(5000)) ==
                    pdTRUE) { // pdMS_TO_TICKS(3000) portMAX_DELAY
                    if (notified_value & UWB_EVENT_RX_DONE) {
                        uint16_t frame_len =
                            dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                        dwt_readrxdata(dw1000rx_buffer, frame_len, 0);

                        uwb_frame_t poll_frame;
                        if (uwb_frame_decode(&poll_frame, dw1000rx_buffer,
                                             frame_len) == 0 &&
                            poll_frame.type == UWB_FRAME_TYPE_POLL) {
                            g_anchor_session.sequence_num =
                                poll_frame.header.sequence_num;
                            g_anchor_session.tag_addr   = poll_frame.header.source_addr;
                            g_anchor_session.tag_pan_id = poll_frame.header.pan_id;
                            g_anchor_session.poll_rx_ts = isr_timestamp_packet.rx;
                            local_device.seqNum         = g_anchor_session.sequence_num;

                            uwb_timestamp_t resp_tx = uwb_timestamp_add_delay_ms(
                                &g_anchor_session.poll_rx_ts, UWB_DELAY_MS);
                            g_anchor_session.resp_tx_ts = resp_tx;
                            uint64_t resp_ticks         = uwb_timestamp_to_u64(&resp_tx);
                            dwt_setdelayedtrxtime((uint32_t)(resp_ticks >> 8));

                            uwb_address_t self_addr = {
                                .pan_id     = local_device.pan_id,
                                .short_addr = local_device.short_addr};
                            uwb_address_t dest_addr = {
                                .pan_id     = poll_frame.header.pan_id,
                                .short_addr = poll_frame.header.source_addr};
                            uwb_frame_t resp_frame;
                            uwb_frame_init(&resp_frame, UWB_FRAME_TYPE_RESP, &self_addr,
                                           &dest_addr, g_anchor_session.sequence_num);
                            resp_frame.payload.resp.poll_rx =
                                g_anchor_session.poll_rx_ts;
                            resp_frame.payload.resp.resp_tx =
                                g_anchor_session.resp_tx_ts;

                            int tx_len = uwb_frame_encode(&resp_frame, dw1000tx_buffer,
                                                          sizeof(dw1000tx_buffer));
                            if (tx_len > 0) {
                                dwt_writetxdata(tx_len, dw1000tx_buffer, 0);
                                dwt_writetxfctrl(tx_len, 0);
                                dwt_starttx(DWT_START_TX_DELAYED);
                                g_current_anchor_state =
                                    ANCHOR_STATE_AWAIT_RESPONSE_TX_CONFIRM;
                                break;
                            } else {
                                log_error("anchor resp encode failed seq=%u",
                                          resp_frame.header.sequence_num);
                            }
                        } else {
                            log_error(
                                "anchor poll decode failed or frame type mismatch "
                                "(len=%u)",
                                frame_len);
                        }
                    } else {
                        log_error("anchor rx error waiting POLL (event=0x%08lX)",
                                  notified_value);
                    }
                } else {
                    log_warn("anchor wait POLL notify timeout");
                }
                reset_anchor_state_machine(&g_current_anchor_state);
                break;
            }

            case ANCHOR_STATE_AWAIT_RESPONSE_TX_CONFIRM: {
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(200)) == pdTRUE &&
                    (notified_value & UWB_EVENT_TX_DONE)) {
                    dwt_setrxtimeout(65535);
                    dwt_rxenable(0);
                    g_current_anchor_state = ANCHOR_STATE_AWAIT_FINAL_RX;
                    break;
                } else {
                    log_error("RESP sent failed seq=%u", g_anchor_session.sequence_num);
                }
                reset_anchor_state_machine(&g_current_anchor_state);
                break;
            }

            case ANCHOR_STATE_AWAIT_FINAL_RX: {
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(300)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_RX_DONE) {
                        uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                        dwt_readrxdata(dw1000rx_buffer, frame_len, 0);

                        uwb_frame_t final_frame;
                        if (uwb_frame_decode(&final_frame, dw1000rx_buffer, frame_len) == 0 &&
                            final_frame.type == UWB_FRAME_TYPE_FINAL &&
                            final_frame.header.sequence_num == g_anchor_session.sequence_num) {
                            g_anchor_session.final_rx_ts = isr_timestamp_packet.rx;

                            uwb_frame_t result_frame;
                            uwb_address_t self_addr = {.pan_id = local_device.pan_id, .short_addr = local_device.short_addr};
                            uwb_address_t dest_addr = {.pan_id = g_anchor_session.tag_pan_id, .short_addr = g_anchor_session.tag_addr};
                            uwb_frame_init(&result_frame, UWB_FRAME_TYPE_RESULT, &self_addr, &dest_addr, g_anchor_session.sequence_num);

                            result_frame.payload.result.poll_tx      = final_frame.payload.final.poll_tx;
                            result_frame.payload.result.poll_rx      = g_anchor_session.poll_rx_ts;
                            result_frame.payload.result.resp_tx      = g_anchor_session.resp_tx_ts;
                            result_frame.payload.result.resp_rx      = final_frame.payload.final.resp_rx;
                            result_frame.payload.result.final_tx     = final_frame.payload.final.final_tx;
                            result_frame.payload.result.final_rx     = g_anchor_session.final_rx_ts;
                            result_frame.payload.result.anchor_pos_x = g_anchor_pos_x;
                            result_frame.payload.result.anchor_pos_y = g_anchor_pos_y;

                            int tx_len = uwb_frame_encode(&result_frame, dw1000tx_buffer, sizeof(dw1000tx_buffer));
                            if (tx_len > 0) {
                                dwt_writetxdata(tx_len, dw1000tx_buffer, 0);
                                dwt_writetxfctrl(tx_len, 0);
                                dwt_starttx(DWT_START_TX_IMMEDIATE);
                                g_current_anchor_state = ANCHOR_STATE_AWAIT_RESULT_TX_CONFIRM;
                                break;
                            } else {
                                log_error("anchor result encode failed seq=%u", final_frame.header.sequence_num);
                            }
                        } else {
                            log_error("anchor final decode/type/seq failed (len=%u)", frame_len);
                        }
                    } else {
                        log_error("anchor wait FINAL error event=0x%08lX", notified_value);
                    }
                } else {
                    log_error("anchor wait FINAL timeout");
                }
                reset_anchor_state_machine(&g_current_anchor_state);
                break;
            }

            case ANCHOR_STATE_AWAIT_RESULT_TX_CONFIRM: {
                if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, pdMS_TO_TICKS(200)) == pdTRUE &&
                    (notified_value & UWB_EVENT_TX_DONE)) {
                    log_info("result sent succeed\r\n");
                    reset_anchor_state_machine(&g_current_anchor_state);
                } else {
                    log_error("RESULT sent failed seq=%u", g_anchor_session.sequence_num);
                    reset_anchor_state_machine(&g_current_anchor_state);
                }
                break;
            }

            default: {

                reset_anchor_state_machine(&g_current_anchor_state);
                break;
            }
        }
        osDelay(1);
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
    UWB_DeviceInitFromConfig(); // 初始化本机的配置数据

    const app_config_t *cfg = AppConfig_Get(); // 配置控制功能
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
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    UWB_Event_t event_to_notify = UWB_EVENT_NONE;

    static uint32_t status_reg;

    status_reg = dwt_read32bitoffsetregFromISR(SYS_STATUS_ID, 0);

    bool tx_done  = (status_reg & SYS_STATUS_TXFRS) != 0;
    bool rx_ok    = (status_reg & SYS_STATUS_RXFCG) != 0;
    bool err_bit  = (status_reg & (SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO | SYS_STATUS_RXSFDTO |
                                  SYS_STATUS_AFFREJ | SYS_STATUS_RXFCE | SYS_STATUS_RXPHE |
                                  SYS_STATUS_RXRFSL)) != 0;
    bool multiple = tx_done && rx_ok;

    if (!err_bit && !multiple) {
        if (rx_ok) {
            UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
            uwb_timestamp_read_rx_isr(&isr_timestamp_packet.rx);
            taskEXIT_CRITICAL_FROM_ISR(saved);
            event_to_notify = UWB_EVENT_RX_DONE;
        } else if (tx_done) {
            UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
            uwb_timestamp_read_tx_isr(&isr_timestamp_packet.tx);
            taskEXIT_CRITICAL_FROM_ISR(saved);
            event_to_notify = UWB_EVENT_TX_DONE;
        }
    } else {
        event_to_notify = UWB_EVENT_RX_ERROR;
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

// poll_tx, poll_rx, resp_tx, resp_rx, final_tx, final_rx
double calculate_distance_from_timestamps_v2(uint64_t poll_tx_ts,
                                             uint64_t poll_rx_ts,
                                             uint64_t resp_tx_ts,
                                             uint64_t resp_rx_ts,
                                             uint64_t final_tx_ts,
                                             uint64_t final_rx_ts)
{
    // 假设：get_timestamp_difference_u64(end, start)
    // 返回的是 (end - start) 在 40bit 下做回绕处理后的无符号差值
    double T_round_tag    = (double)get_timestamp_difference_u64(resp_rx_ts, poll_tx_ts);  // t4 - t1
    double T_reply_anchor = (double)get_timestamp_difference_u64(resp_tx_ts, poll_rx_ts);  // t3 - t2
    double T_round_anchor = (double)get_timestamp_difference_u64(final_rx_ts, resp_tx_ts); // t6 - t3
    double T_reply_tag    = (double)get_timestamp_difference_u64(final_tx_ts, resp_rx_ts); // t5 - t4

    double numerator   = (T_round_tag * T_round_anchor) - (T_reply_tag * T_reply_anchor);
    double denominator = T_round_tag + T_round_anchor + T_reply_tag + T_reply_anchor;

    if (denominator == 0.0) {
        return -1.0; // 错误情况，你也可以改成 0 或其它标记值
    }

    double Tprop_ticks = numerator / denominator; // ToF，单位：DW1000 tick
    Tprop_ticks        = fabs(Tprop_ticks);       // 取绝对值，防止出现负的 ToF

    double Tprop_seconds = Tprop_ticks * DWT_TIME_UNITS;   // tick -> 秒
    double distance_m    = Tprop_seconds * SPEED_OF_LIGHT; // 秒 * m/s -> 米

    return distance_m;
}

#define RX_SOFTRESET_BIT_MASK   (1UL << 28)
#define ALL_SOFTRESET_BITS_MASK (0xFUL << 28)

void reset_tag_state_machine(Tag_State_t *current_tag_state)
{
    uint32_t interrupt_mask = DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RFCE |
                              DWT_INT_RXPTO | DWT_INT_SFDT;

    dwt_setinterrupt(interrupt_mask, 0);

    uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    dwt_write32bitreg(SYS_STATUS_ID, status_reg);

    dwt_rxreset();

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

void reset_anchor_state_machine(Anchor_State_t *current_anchor_state)
{

    uint32_t interrupt_mask = DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO |
                              DWT_INT_RFCE | DWT_INT_RXPTO | DWT_INT_SFDT;

    // 1. 关掉相关中断，防止过程中被打断
    dwt_setinterrupt(interrupt_mask, 0);

    // 2. 确保收发器不在忙
    dwt_forcetrxoff();

    // 4. 复位 RX 数字部分
    dwt_rxreset();

    // 5. 清任务通知状态
    // if (dw1000samplingTaskNotifyHandle != NULL) {
    //     xTaskNotifyStateClear(dw1000samplingTaskNotifyHandle);
    // }

    // 6. 重置本地状态机 & 会话
    *current_anchor_state = ANCHOR_STATE_AWAIT_POLL_RX;
    Anchor_ResetSession();

    // 7. 重新打开中断、设置超时并进入接收
    dwt_setinterrupt(interrupt_mask, 1);
    dwt_setrxtimeout(0); // 永久 RX，按需可改
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
