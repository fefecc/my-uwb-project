#include "app_tasks.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "dma.h"
#include "usart.h"

#include "../bsp/bsp_key.h"
#include "../bsp/bsp_led.h"
#include "../device/gnss_parser.h"
#include "../device/imu_device.h"
#include "../service/config_service.h"
#include "../service/data_service.h"
#include "../service/log_service.h"
#include "../service/storage_service.h"
#include "../service/time_service.h"
#include "../UWB/uwb_stack.h"

#define APP_GNSS_RX_BUFFER_SIZE      (1024U)
#define APP_USART_CMD_RX_BUFFER_SIZE (128U)
#define APP_USART_CMD_NOTIFY_RX_IDLE (1UL << 31)
#define APP_USART_CMD_NOTIFY_RX_FULL (1UL << 30)
#define APP_USART_CMD_NOTIFY_TX_DONE (1UL << 29)
#define APP_USART_CMD_NOTIFY_LOG     (1UL << 28)
#define APP_USART_CMD_RX_LEN_MASK    (0x0000FFFFUL)
#define APP_USART_CMD_TX_USE_DMA     (0U)
#define APP_USART_CMD_TX_DMA_SIZE    (256U)
#define APP_USART_CMD_TX_TIMEOUT_MS  (100U)
#define APP_USART_LOG_SLOT_SIZE      (512U)
#define APP_DATA_SORT_WINDOW         (10U)
#define APP_SD_BLOCK_SIZE            (16U * 1024U)
#define APP_ASCII_LINE_SIZE          (384U)
#define APP_KEY_LONG_PRESS_MS        (200U)
#define APP_SUPPRESS_SD_WRITE_ERROR_LOGS (1U)

#if APP_SUPPRESS_SD_WRITE_ERROR_LOGS
static void app_log_sd_error(const char *fmt, ...)
{
    (void)fmt;
}

static void app_log_sd_warn(const char *fmt, ...)
{
    (void)fmt;
}
#else
#define app_log_sd_error(...) app_log_error(__VA_ARGS__)
#define app_log_sd_warn(...)  app_log_warn(__VA_ARGS__)
#endif

#define GNSS_MSG_ID_BESTNAV          (0x0846U)

typedef enum {
    APP_SD_BLOCK_MAIN   = 0,
    APP_SD_BLOCK_BACKUP = 1,
} AppSdBlockId;

typedef enum {
    APP_USART_LOG_SLOT_CMD = 0,
    APP_USART_LOG_SLOT_SYSTEM,
    APP_USART_LOG_SLOT_GNSS,
    APP_USART_LOG_SLOT_IMU,
    APP_USART_LOG_SLOT_UWB,
    APP_USART_LOG_SLOT_DATA,
    APP_USART_LOG_SLOT_SD,
    APP_USART_LOG_SLOT_KEY,
    APP_USART_LOG_SLOT_COUNT,
} AppUsartLogSlotId;

typedef struct {
    uint8_t buffer[APP_USART_LOG_SLOT_SIZE];
    size_t head;
    size_t tail;
    size_t used;
    uint32_t dropped;
    StaticSemaphore_t mutex_ctrl;
    SemaphoreHandle_t mutex;
} AppUsartLogSlot;

typedef struct {
    uint8_t *block[2];
    size_t len[2];
    bool ready[2];
    bool locked[2];
    uint8_t active;
} AppSdFifo;

typedef struct __attribute__((packed)) {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t sync3;
    uint8_t cpu_idle;
    uint16_t message_id;
    uint16_t message_length;
    uint8_t time_ref;
    uint8_t time_status;
    uint16_t week;
    uint32_t week_ms;
    uint32_t version;
    uint8_t reserved;
    uint8_t leap_sec;
    uint16_t delay_ms;
    uint32_t p_sol_status;
    uint32_t pos_type;
    double lat;
    double lon;
    double hgt;
    float undulation;
    uint32_t datum_id;
    float lat_std;
    float lon_std;
    float hgt_std;
    char stn_id[4];
    float diff_age;
    float sol_age;
    uint8_t svs_tracked;
    uint8_t svs_in_sol;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t ext_sol_stat;
    uint8_t gal_bds3_mask;
    uint8_t gps_glonass_bds2_mask;
    uint32_t v_sol_status;
    uint32_t vel_type;
    float latency;
    float age;
    double hor_spd;
    double trk_gnd;
    double vert_spd;
    float ver_spd_std;
    float hor_spd_std;
} GnssBestNavFrame;

static AppMode g_app_mode = APP_MODE_RUN;

static TaskHandle_t g_gnss_task;
static TaskHandle_t g_imu_task;
static TaskHandle_t g_key_task;
static TaskHandle_t g_data_sort_task;
static TaskHandle_t g_sd_writer_task;
static TaskHandle_t g_led_task;
static TaskHandle_t g_usart_cmd_task;

static uint8_t g_gnss_rx_buffer[APP_GNSS_RX_BUFFER_SIZE];
static GnssParser g_gnss_parser;
static uint8_t g_usart_cmd_rx_buffer[APP_USART_CMD_RX_BUFFER_SIZE];
static char g_usart_cmd_line[APP_USART_CMD_RX_BUFFER_SIZE + 1U];
static uint32_t g_usart_cmd_line_len;
static AppConfig g_usart_cmd_config;
static bool g_usart_cmd_config_valid;
static bool g_usart_cmd_config_dirty;
static volatile HAL_StatusTypeDef g_usart_cmd_tx_status = HAL_OK;
static volatile uint32_t g_usart_cmd_tx_error;
static volatile HAL_UART_StateTypeDef g_usart_cmd_tx_state;
static AppUsartLogSlot g_usart_log_slots[APP_USART_LOG_SLOT_COUNT];
static bool g_usart_log_ready;
static uint32_t g_usart_log_next_slot;
static uint8_t g_usart_cmd_tx_dma_buffer[APP_USART_CMD_TX_DMA_SIZE];
static uint16_t g_usart_cmd_tx_dma_len;
static bool g_usart_cmd_tx_dma_pending;
static volatile bool g_usart_cmd_tx_dma_busy;

static uint8_t g_sd_main_block[APP_SD_BLOCK_SIZE];
static uint8_t g_sd_backup_block[APP_SD_BLOCK_SIZE];
static AppSdFifo g_sd_fifo;
static StaticSemaphore_t g_sd_fifo_mutex_ctrl;
static SemaphoreHandle_t g_sd_fifo_mutex;

static uint32_t sd_ready_bit(AppSdBlockId id)
{
    return (uint32_t)(1UL << (uint32_t)id);
}

static const char *sd_block_name(AppSdBlockId id)
{
    return id == APP_SD_BLOCK_MAIN ? "main" : "backup";
}

static const char *data_source_name(AppDataSource source)
{
    switch (source) {
        case APP_DATA_SRC_GNSS:
            return "GNSS";
        case APP_DATA_SRC_IMU:
            return "IMU";
        case APP_DATA_SRC_UWB:
            return "UWB";
        default:
            return "UNKNOWN";
    }
}

static bool scheduler_running(void)
{
    return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
}

static void notify_usart_cmd(uint32_t bits)
{
    if (g_usart_cmd_task == NULL || !scheduler_running()) {
        return;
    }

    (void)xTaskNotify(g_usart_cmd_task, bits, eSetBits);
}

void AppTasks_LogInit(void)
{
    if (g_usart_log_ready) {
        return;
    }

    for (uint32_t i = 0; i < APP_USART_LOG_SLOT_COUNT; ++i) {
        AppUsartLogSlot *slot = &g_usart_log_slots[i];
        memset(slot, 0, sizeof(*slot));
        slot->mutex = xSemaphoreCreateMutexStatic(&slot->mutex_ctrl);
        if (slot->mutex == NULL) {
            return;
        }
    }

    g_usart_log_next_slot      = APP_USART_LOG_SLOT_CMD;
    g_usart_cmd_tx_dma_len     = 0U;
    g_usart_cmd_tx_dma_pending = false;
    g_usart_cmd_tx_dma_busy    = false;
    g_usart_cmd_tx_status      = HAL_OK;
    g_usart_cmd_tx_error       = 0U;
    g_usart_cmd_tx_state       = HAL_UART_STATE_READY;
    g_usart_log_ready          = true;
}

static bool usart_log_slot_take(AppUsartLogSlot *slot)
{
    if (slot == NULL || slot->mutex == NULL) {
        return false;
    }

    if (!scheduler_running()) {
        return true;
    }

    return xSemaphoreTake(slot->mutex, portMAX_DELAY) == pdTRUE;
}

static void usart_log_slot_give(AppUsartLogSlot *slot)
{
    if (slot == NULL || slot->mutex == NULL || !scheduler_running()) {
        return;
    }

    (void)xSemaphoreGive(slot->mutex);
}

static void usart_log_slot_drop_oldest(AppUsartLogSlot *slot, size_t len)
{
    if (slot == NULL || len == 0U) {
        return;
    }

    if (len > slot->used) {
        len = slot->used;
    }

    slot->tail = (slot->tail + len) % APP_USART_LOG_SLOT_SIZE;
    slot->used -= len;
    slot->dropped += (uint32_t)len;
}

static bool usart_log_slot_write(AppUsartLogSlotId id,
                                 const uint8_t *data,
                                 size_t len)
{
    if (!g_usart_log_ready || data == NULL || len == 0U ||
        id >= APP_USART_LOG_SLOT_COUNT) {
        return false;
    }

    AppUsartLogSlot *slot = &g_usart_log_slots[id];
    if (!usart_log_slot_take(slot)) {
        return false;
    }

    if (len > APP_USART_LOG_SLOT_SIZE) {
        data += len - APP_USART_LOG_SLOT_SIZE;
        len = APP_USART_LOG_SLOT_SIZE;
    }

    size_t free_len = APP_USART_LOG_SLOT_SIZE - slot->used;
    if (len > free_len) {
        usart_log_slot_drop_oldest(slot, len - free_len);
    }

    size_t first = APP_USART_LOG_SLOT_SIZE - slot->head;
    if (first > len) {
        first = len;
    }
    memcpy(&slot->buffer[slot->head], data, first);

    size_t second = len - first;
    if (second > 0U) {
        memcpy(slot->buffer, data + first, second);
    }

    slot->head = (slot->head + len) % APP_USART_LOG_SLOT_SIZE;
    slot->used += len;

    usart_log_slot_give(slot);
    notify_usart_cmd(APP_USART_CMD_NOTIFY_LOG);
    return true;
}

static size_t usart_log_slot_read(AppUsartLogSlotId id,
                                  uint8_t *data,
                                  size_t max_len)
{
    if (!g_usart_log_ready || data == NULL || max_len == 0U ||
        id >= APP_USART_LOG_SLOT_COUNT) {
        return 0U;
    }

    AppUsartLogSlot *slot = &g_usart_log_slots[id];
    if (!usart_log_slot_take(slot)) {
        return 0U;
    }

    size_t len = slot->used < max_len ? slot->used : max_len;
    if (len > 0U) {
        size_t first = APP_USART_LOG_SLOT_SIZE - slot->tail;
        if (first > len) {
            first = len;
        }
        memcpy(data, &slot->buffer[slot->tail], first);

        size_t second = len - first;
        if (second > 0U) {
            memcpy(data + first, slot->buffer, second);
        }

        slot->tail = (slot->tail + len) % APP_USART_LOG_SLOT_SIZE;
        slot->used -= len;
    }

    usart_log_slot_give(slot);
    return len;
}

static AppUsartLogSlotId usart_log_slot_for_current_task(void)
{
    if (!scheduler_running()) {
        return APP_USART_LOG_SLOT_SYSTEM;
    }

    TaskHandle_t current = xTaskGetCurrentTaskHandle();
    if (current == g_usart_cmd_task) {
        return APP_USART_LOG_SLOT_CMD;
    }
    if (current == g_gnss_task) {
        return APP_USART_LOG_SLOT_GNSS;
    }
    if (current == g_imu_task) {
        return APP_USART_LOG_SLOT_IMU;
    }
    if (current == g_data_sort_task) {
        return APP_USART_LOG_SLOT_DATA;
    }
    if (current == g_sd_writer_task) {
        return APP_USART_LOG_SLOT_SD;
    }
    if (current == g_key_task || current == g_led_task) {
        return APP_USART_LOG_SLOT_KEY;
    }

    const char *name = pcTaskGetName(NULL);
    if (name != NULL && strncmp(name, "uwb", 3U) == 0) {
        return APP_USART_LOG_SLOT_UWB;
    }

    return APP_USART_LOG_SLOT_SYSTEM;
}

bool AppTasks_LogWriteText(const char *text, size_t len)
{
    if (!g_usart_log_ready) {
        AppTasks_LogInit();
    }

    return usart_log_slot_write(usart_log_slot_for_current_task(),
                                (const uint8_t *)text,
                                len);
}

static bool init_sd_fifo(void)
{
    if (g_sd_fifo_mutex == NULL) {
        g_sd_fifo_mutex = xSemaphoreCreateMutexStatic(&g_sd_fifo_mutex_ctrl);
    }

    if (g_sd_fifo_mutex == NULL) {
        return false;
    }

    memset(&g_sd_fifo, 0, sizeof(g_sd_fifo));
    g_sd_fifo.block[APP_SD_BLOCK_MAIN]   = g_sd_main_block;
    g_sd_fifo.block[APP_SD_BLOCK_BACKUP] = g_sd_backup_block;
    g_sd_fifo.active                     = APP_SD_BLOCK_MAIN;

    return true;
}

static bool sd_fifo_take(void)
{
    return g_sd_fifo_mutex != NULL &&
           xSemaphoreTake(g_sd_fifo_mutex, portMAX_DELAY) == pdTRUE;
}

static void sd_fifo_give(void)
{
    (void)xSemaphoreGive(g_sd_fifo_mutex);
}

static bool sd_fifo_active_writable(void)
{
    uint8_t active = g_sd_fifo.active;
    return active < 2U &&
           !g_sd_fifo.ready[active] &&
           !g_sd_fifo.locked[active];
}

static bool sd_fifo_switch_to_free(void)
{
    uint8_t next = g_sd_fifo.active ^ 1U;

    if (g_sd_fifo.ready[next] || g_sd_fifo.locked[next]) {
        return false;
    }

    g_sd_fifo.active    = next;
    g_sd_fifo.len[next] = 0;
    return true;
}

static void notify_sd_block_ready(AppSdBlockId id)
{
    if (g_sd_writer_task == NULL) {
        app_log_sd_error("SD writer not ready for %s block", sd_block_name(id));
        return;
    }

    if (xTaskNotify(g_sd_writer_task, sd_ready_bit(id), eSetBits) != pdPASS) {
        app_log_sd_error("notify SD writer failed for %s block", sd_block_name(id));
    }
}

static bool sd_fifo_write_ascii(const uint8_t *data, size_t len, AppDataSource source)
{
    const uint8_t *src = data;
    size_t remaining   = len;

    while (remaining > 0U) {
        AppSdBlockId ready_id = APP_SD_BLOCK_MAIN;
        bool has_ready        = false;
        bool no_free          = false;
        size_t copied         = 0;

        if (!sd_fifo_take()) {
            app_log_sd_error("SD FIFO mutex unavailable");
            return false;
        }

        if (!sd_fifo_active_writable() && !sd_fifo_switch_to_free()) {
            no_free = true;
        }

        if (!no_free) {
            uint8_t active = g_sd_fifo.active;
            size_t space   = APP_SD_BLOCK_SIZE - g_sd_fifo.len[active];
            copied         = remaining < space ? remaining : space;

            memcpy(&g_sd_fifo.block[active][g_sd_fifo.len[active]], src, copied);
            g_sd_fifo.len[active] += copied;

            if (g_sd_fifo.len[active] == APP_SD_BLOCK_SIZE) {
                ready_id                = (AppSdBlockId)active;
                g_sd_fifo.ready[active] = true;
                has_ready               = true;

                if (!sd_fifo_switch_to_free()) {
                    no_free = true;
                }
            }
        }

        sd_fifo_give();

        if (has_ready) {
            notify_sd_block_ready(ready_id);
        }

        if (no_free) {
            app_log_sd_error("SD FIFO full: source=%s block=%s",
                             data_source_name(source),
                             has_ready ? sd_block_name(ready_id) : "none");
            return false;
        }

        src += copied;
        remaining -= copied;
    }

    return true;
}

static bool sd_fifo_lock_block(AppSdBlockId id, const uint8_t **data, size_t *len)
{
    uint8_t index = (uint8_t)id;
    bool locked   = false;

    if (data == NULL || len == NULL || index >= 2U || !sd_fifo_take()) {
        return false;
    }

    if (g_sd_fifo.ready[index] && !g_sd_fifo.locked[index]) {
        g_sd_fifo.locked[index] = true;
        *data                   = g_sd_fifo.block[index];
        *len                    = g_sd_fifo.len[index];
        locked                  = true;
    }

    sd_fifo_give();
    return locked;
}

static void sd_fifo_release_block(AppSdBlockId id)
{
    uint8_t index = (uint8_t)id;

    if (index >= 2U || !sd_fifo_take()) {
        return;
    }

    g_sd_fifo.ready[index]  = false;
    g_sd_fifo.locked[index] = false;
    g_sd_fifo.len[index]    = 0;

    sd_fifo_give();
}

static void sd_fifo_unlock_block(AppSdBlockId id)
{
    uint8_t index = (uint8_t)id;

    if (index >= 2U || !sd_fifo_take()) {
        return;
    }

    g_sd_fifo.locked[index] = false;

    sd_fifo_give();
}

static uint32_t sd_fifo_ready_bits(void)
{
    uint32_t bits = 0;

    if (!sd_fifo_take()) {
        return 0U;
    }

    for (uint32_t id = APP_SD_BLOCK_MAIN; id <= APP_SD_BLOCK_BACKUP; ++id) {
        if (g_sd_fifo.ready[id] && !g_sd_fifo.locked[id]) {
            bits |= sd_ready_bit((AppSdBlockId)id);
        }
    }

    sd_fifo_give();
    return bits;
}

static size_t bounded_strlen(const char *text, size_t max_len)
{
    size_t len = 0;

    if (text == NULL) {
        return 0;
    }

    while (len < max_len && text[len] != '\0') {
        len++;
    }

    return len;
}

static size_t format_node_ascii(const AppDataNode *node, char *line, size_t line_size)
{
    int n = 0;

    if (node == NULL || line == NULL || line_size == 0U) {
        return 0;
    }

    const TimeTimestamp *ts = &node->timestamp;
    uint32_t week           = ts->utc_valid ? ts->local_utc.week : 0U;
    uint32_t week_ms        = ts->utc_valid ? ts->local_utc.week_ms : 0U;
    uint32_t week_sec       = week_ms / 1000U;
    uint32_t week_ms_rem    = week_ms % 1000U;

    switch (node->source) {
        case APP_DATA_SRC_GNSS:
            n = snprintf(line, line_size,
                         "GNSS,%lu,%lu.%03lu,%lu,%.3f,%u,%.17f,%.17f,%.17f,%lu,%.9f,%.9f,%.9f,%lu,%lu,%.9f,%.9f,%u,%u\r\n",
                         (unsigned long)week,
                         (unsigned long)week_sec,
                         (unsigned long)week_ms_rem,
                         (unsigned long)ts->local_clock.sec,
                         (double)ts->local_clock.ms,
                         ts->utc_valid ? 1U : 0U,
                         node->payload.gnss.lat,
                         node->payload.gnss.lon,
                         node->payload.gnss.hgt,
                         (unsigned long)node->payload.gnss.datum_id,
                         node->payload.gnss.lat_std,
                         node->payload.gnss.lon_std,
                         node->payload.gnss.hgt_std,
                         (unsigned long)node->payload.gnss.pos_status,
                         (unsigned long)node->payload.gnss.pos_type,
                         node->payload.gnss.diff_age,
                         node->payload.gnss.sol_age,
                         node->payload.gnss.svs_tracked,
                         node->payload.gnss.svs_in_sol);
            break;

        case APP_DATA_SRC_IMU:
            n = snprintf(line, line_size,
                         "IMU,%lu,%lu.%03lu,%lu,%.3f,%u,%d,%d,%d,%d,%d,%d\r\n",
                         (unsigned long)week,
                         (unsigned long)week_sec,
                         (unsigned long)week_ms_rem,
                         (unsigned long)ts->local_clock.sec,
                         (double)ts->local_clock.ms,
                         ts->utc_valid ? 1U : 0U,
                         node->payload.imu.accel[0],
                         node->payload.imu.accel[1],
                         node->payload.imu.accel[2],
                         node->payload.imu.gyro[0],
                         node->payload.imu.gyro[1],
                         node->payload.imu.gyro[2]);
            break;

        case APP_DATA_SRC_UWB:
            n = snprintf(line, line_size,
                         "UWB,%lu,%lu.%03lu,0x%02lX%08lX,%.3f,%u,%u,%u,%u,%u,%u,%.17f,%d,%u,"
                         "0x%02lX%08lX,0x%02lX%08lX,0x%02lX%08lX,0x%02lX%08lX\r\n",
                         (unsigned long)week,
                         (unsigned long)week_sec,
                         (unsigned long)week_ms_rem,
                         (uint32_t)(ts->local_clock.sec >> 32),
                         (uint32_t)(ts->local_clock.sec & 0xFFFFFFFF),
                         (double)ts->local_clock.ms,
                         ts->utc_valid ? 1U : 0U,
                         node->payload.uwb.anchor_id,
                         node->payload.uwb.tag_id,
                         node->payload.uwb.exchange_seq,
                         node->payload.uwb.response_slot_id,
                         node->payload.uwb.status_flags,
                         node->payload.uwb.distance_m,
                         node->payload.uwb.range_quality,
                         node->payload.uwb.retry_count,
                         (uint32_t)(node->payload.uwb.tag_tx_ts >> 32),
                         (uint32_t)(node->payload.uwb.tag_tx_ts & 0xFFFFFFFF),
                         (uint32_t)(node->payload.uwb.anchor_rx_ts >> 32),
                         (uint32_t)(node->payload.uwb.anchor_rx_ts & 0xFFFFFFFF),
                         (uint32_t)(node->payload.uwb.anchor_tx_ts >> 32),
                         (uint32_t)(node->payload.uwb.anchor_tx_ts & 0xFFFFFFFF),
                         (uint32_t)(node->payload.uwb.tag_rx_ts >> 32),
                         (uint32_t)(node->payload.uwb.tag_rx_ts & 0xFFFFFFFF));
            break;

        default:
            return 0;
    }

    if (n <= 0 || (size_t)n >= line_size) {
        return 0;
    }

    return bounded_strlen(line, line_size);
}

static void start_gnss_dma_idle(void)
{
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    (void)HAL_UART_Receive_DMA(&huart3, g_gnss_rx_buffer, APP_GNSS_RX_BUFFER_SIZE);
}

static void gnss_frame_handler(uint16_t msg_id,
                               const uint8_t *frame,
                               uint16_t frame_len,
                               void *user)
{
    (void)user;

    TimeUtcClock utc;
    if (GnssParser_ExtractUtc(frame, frame_len, &utc)) {
        TimeService_WriteUtcCache(&utc);
    }

    if (msg_id != GNSS_MSG_ID_BESTNAV || frame_len < sizeof(GnssBestNavFrame)) {
        return;
    }

    GnssBestNavFrame nav;
    memcpy(&nav, frame, sizeof(nav));

    AppDataNode node = {0};
    node.source      = APP_DATA_SRC_GNSS;
    (void)TimeService_GetTimestamp(&node.timestamp);
    node.payload.gnss.lat         = nav.lat;
    node.payload.gnss.lon         = nav.lon;
    node.payload.gnss.hgt         = nav.hgt;
    node.payload.gnss.datum_id    = nav.datum_id;
    node.payload.gnss.lat_std     = nav.lat_std;
    node.payload.gnss.lon_std     = nav.lon_std;
    node.payload.gnss.hgt_std     = nav.hgt_std;
    node.payload.gnss.pos_status  = nav.p_sol_status;
    node.payload.gnss.pos_type    = nav.pos_type;
    node.payload.gnss.diff_age    = nav.diff_age;
    node.payload.gnss.sol_age     = nav.sol_age;
    node.payload.gnss.svs_tracked = nav.svs_tracked;
    node.payload.gnss.svs_in_sol  = nav.svs_in_sol;

    if (!DataService_Send(&node, 0)) {
        app_log_warn("GNSS data queue full");
    }
}

static int compare_local_clock(const TimeLocalClock *a, const TimeLocalClock *b)
{
    if (a->sec < b->sec) {
        return -1;
    }
    if (a->sec > b->sec) {
        return 1;
    }
    if (a->ms < b->ms) {
        return -1;
    }
    if (a->ms > b->ms) {
        return 1;
    }
    return 0;
}

static int compare_node_time(const AppDataNode *a, const AppDataNode *b)
{
    return compare_local_clock(&a->timestamp.local_clock,
                               &b->timestamp.local_clock);
}

static void sorted_window_insert(AppDataNode *nodes,
                                 uint32_t *count,
                                 uint32_t capacity,
                                 const AppDataNode *node)
{
    if (nodes == NULL || count == NULL || node == NULL || *count >= capacity) {
        return;
    }

    uint32_t pos = *count;
    while (pos > 0U && compare_node_time(&nodes[pos - 1U], node) > 0) {
        nodes[pos] = nodes[pos - 1U];
        pos--;
    }

    nodes[pos] = *node;
    (*count)++;
}

static void write_sorted_node_to_sd(const AppDataNode *node)
{
    if (node == NULL) {
        return;
    }

    char line[APP_ASCII_LINE_SIZE];
    size_t len = format_node_ascii(node, line, sizeof(line));

    if (len == 0U) {
        app_log_warn("format data node failed: source=%s",
                     data_source_name(node->source));
        return;
    }

    if (!sd_fifo_write_ascii((const uint8_t *)line, len, node->source)) {
        app_log_sd_error("write ASCII to SD FIFO failed: source=%s",
                         data_source_name(node->source));
    }
}

bool AppTasks_CreateAll(AppMode mode)
{
    g_app_mode = mode;

    AppTasks_LogInit();
    LogService_Init();
    TimeService_Init();
    BspLed_Init();
    (void)ConfigService_Load();

    if (!DataService_Init() || !init_sd_fifo()) {
        return false;
    }

    const osThreadAttr_t default_attr = {
        .name       = "appDefault",
        .stack_size = 512U * 4U,
        .priority   = osPriorityLow,
    };
    const osThreadAttr_t gnss_attr = {
        .name       = "gnssTask",
        .stack_size = 1024U * 4U,
        .priority   = osPriorityAboveNormal,
    };
    const osThreadAttr_t imu_attr = {
        .name       = "imuTask",
        .stack_size = 768U * 4U,
        .priority   = osPriorityAboveNormal,
    };
    const osThreadAttr_t sort_attr = {
        .name       = "dataSort",
        .stack_size = 1024U * 4U,
        .priority   = osPriorityNormal,
    };
    const osThreadAttr_t sd_attr = {
        .name       = "sdWriter",
        .stack_size = 1024U * 4U,
        .priority   = osPriorityBelowNormal,
    };
    const osThreadAttr_t key_attr = {
        .name       = "keyTask",
        .stack_size = 512U * 4U,
        .priority   = osPriorityLow,
    };
    const osThreadAttr_t led_attr = {
        .name       = "ledTask",
        .stack_size = 512U * 4U,
        .priority   = osPriorityLow,
    };
    const osThreadAttr_t cmd_attr = {
        .name       = "usartCMD",
        .stack_size = 768U * 4U,
        .priority   = osPriorityLow,
    };

    (void)osThreadNew(AppDefaultTask, NULL, &default_attr);
    (void)osThreadNew(AppSdWriterTask, NULL, &sd_attr);
    (void)osThreadNew(AppDataSortTask, NULL, &sort_attr);
    (void)osThreadNew(AppLedTask, NULL, &led_attr);
    (void)osThreadNew(AppKeyTask, NULL, &key_attr);
    (void)osThreadNew(AppUsartCmdTask, NULL, &cmd_attr);

    if (mode != APP_MODE_CONFIG) {
        if (!UwbStack_StartFromConfig()) {
            app_log_error("UWB stack start failed");
        }
        (void)osThreadNew(AppGnssTask, NULL, &gnss_attr);
        (void)osThreadNew(AppImuTask, NULL, &imu_attr);
    }

    return true;
}

void AppDefaultTask(void *argument)
{
    (void)argument;

    for (;;) {
        osDelay(1000U);
    }
}

void AppGnssTask(void *argument)
{
    (void)argument;

    g_gnss_task = xTaskGetCurrentTaskHandle();
    GnssParser_Init(&g_gnss_parser, gnss_frame_handler, NULL);
    start_gnss_dma_idle();

    for (;;) {
        uint32_t dma_len = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &dma_len, portMAX_DELAY) == pdTRUE) {
            if (dma_len > 0U && dma_len <= APP_GNSS_RX_BUFFER_SIZE) {
                GnssParser_ProcessBlock(&g_gnss_parser, g_gnss_rx_buffer, dma_len);
            }
            memset(g_gnss_rx_buffer, 0, sizeof(g_gnss_rx_buffer));
            start_gnss_dma_idle();
        }
    }
}

void AppImuTask(void *argument)
{
    (void)argument;

    g_imu_task = xTaskGetCurrentTaskHandle();

    if (ImuDevice_Init(ImuDevice_DefaultConfig()) != 0) {
        app_log_error("IMU init failed");
        for (;;) {
            osDelay(1000U);
        }
    }

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        AppDataNode node = {0};
        node.source      = APP_DATA_SRC_IMU;
        if (ImuDevice_ReadRaw(&node.payload.imu) == 0) {
            (void)TimeService_GetTimestamp(&node.timestamp);
            if (!DataService_Send(&node, 0)) {
                app_log_warn("IMU data queue full");
            }
        }
    }
}

void AppDataSortTask(void *argument)
{
    (void)argument;

    g_data_sort_task = xTaskGetCurrentTaskHandle();

    AppDataNode window[APP_DATA_SORT_WINDOW + 1U];
    uint32_t count = 0;

    for (;;) {
        AppDataNode node;
        if (!DataService_Receive(&node, portMAX_DELAY)) {
            continue;
        }

        sorted_window_insert(window,
                             &count,
                             APP_DATA_SORT_WINDOW + 1U,
                             &node);

        if (count <= APP_DATA_SORT_WINDOW) {
            continue;
        }

        write_sorted_node_to_sd(&window[0]);
        if (count > 1U) {
            memmove(&window[0], &window[1], (count - 1U) * sizeof(window[0]));
        }
        count--;
    }
}

void AppSdWriterTask(void *argument)
{
    (void)argument;

    FIL file;
    bool mounted = false;
    bool opened  = false;

    g_sd_writer_task = xTaskGetCurrentTaskHandle();

    for (;;) {
        if (!mounted) {
            mounted = StorageService_Mount();
            if (!mounted) {
                osDelay(1000U);
                continue;
            }
        }

        if (!opened) {
            opened = StorageService_OpenNextLog(&file);
            if (!opened) {
                osDelay(1000U);
                continue;
            }
        }

        uint32_t notify_bits = sd_fifo_ready_bits();
        if (notify_bits == 0U) {
            if (xTaskNotifyWait(0U,
                                sd_ready_bit(APP_SD_BLOCK_MAIN) |
                                    sd_ready_bit(APP_SD_BLOCK_BACKUP),
                                &notify_bits,
                                portMAX_DELAY) != pdTRUE) {
                continue;
            }
        }

        for (uint32_t id = APP_SD_BLOCK_MAIN; id <= APP_SD_BLOCK_BACKUP; ++id) {
            bool write_failed = false;

            if ((notify_bits & sd_ready_bit((AppSdBlockId)id)) == 0U) {
                continue;
            }

            const uint8_t *data = NULL;
            size_t len          = 0;
            if (!sd_fifo_lock_block((AppSdBlockId)id, &data, &len)) {
                app_log_sd_warn("SD %s block ready notify without data",
                                sd_block_name((AppSdBlockId)id));
                continue;
            }

            if (len != APP_SD_BLOCK_SIZE ||
                !StorageService_WriteBlock(&file, data, len)) {
                app_log_sd_error("SD write %s block failed",
                                 sd_block_name((AppSdBlockId)id));
                (void)f_close(&file);
                mounted      = false;
                opened       = false;
                write_failed = true;
                sd_fifo_unlock_block((AppSdBlockId)id);
            } else {
                (void)f_sync(&file);
                BspLed_Toggle(BSP_LED_0);
                sd_fifo_release_block((AppSdBlockId)id);
            }

            if (write_failed) {
                break;
            }
        }
    }
}

void AppKeyTask(void *argument)
{
    (void)argument;

    g_key_task = xTaskGetCurrentTaskHandle();
    BspKeyState key;
    BspKey_Init(&key);

    for (;;) {
        BspKeyEvent evt = BspKey_Poll(&key, APP_KEY_LONG_PRESS_MS);
        if (evt == BSP_KEY_EVENT_SHORT_PRESS && g_app_mode == APP_MODE_CONFIG) {
            const AppConfig *cfg = ConfigService_Get();
            if (cfg != NULL) {
                app_log_info("CFG pan=0x%04X short=0x%04X role=%u",
                             cfg->pan_id, cfg->short_addr, cfg->role);
            }
        } else if (evt == BSP_KEY_EVENT_LONG_PRESS) {
            app_log_warn("key long press reset");
            osDelay(20U);
            NVIC_SystemReset();
        }

        osDelay(10U);
    }
}

void AppLedTask(void *argument)
{
    (void)argument;

    g_led_task = xTaskGetCurrentTaskHandle();

    uint32_t index = 0;
    for (;;) {
        if (g_app_mode == APP_MODE_CONFIG) {
            BspLed_AllOff();
            BspLed_Set((BspLedId)(index % BSP_LED_COUNT), true);
            index++;
            osDelay(200U);
        } else {
            BspLed_Toggle(BSP_LED_3);
            osDelay(500U); // 表示正常工作
        }
    }
}

static size_t usart_cmd_fill_tx_dma_buffer(void)
{
    for (uint32_t checked = 0; checked < APP_USART_LOG_SLOT_COUNT; ++checked) {
        AppUsartLogSlotId id =
            (AppUsartLogSlotId)((g_usart_log_next_slot + checked) %
                                APP_USART_LOG_SLOT_COUNT);

        size_t len = usart_log_slot_read(id,
                                         g_usart_cmd_tx_dma_buffer,
                                         sizeof(g_usart_cmd_tx_dma_buffer));
        if (len > 0U) {
            g_usart_log_next_slot = ((uint32_t)id + 1U) %
                                    APP_USART_LOG_SLOT_COUNT;
            return len;
        }
    }

    return 0U;
}

static void usart_cmd_try_start_tx(void)
{
#if APP_USART_CMD_TX_USE_DMA
    if (g_usart_cmd_tx_dma_busy) {
        return;
    }
#endif

    if (!g_usart_cmd_tx_dma_pending) {
        size_t len = usart_cmd_fill_tx_dma_buffer();
        if (len == 0U) {
            return;
        }

        g_usart_cmd_tx_dma_len     = (uint16_t)len;
        g_usart_cmd_tx_dma_pending = true;
    }

#if APP_USART_CMD_TX_USE_DMA
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1,
                                                     g_usart_cmd_tx_dma_buffer,
                                                     g_usart_cmd_tx_dma_len);
    g_usart_cmd_tx_status    = status;
    g_usart_cmd_tx_error     = huart1.ErrorCode;
    g_usart_cmd_tx_state     = huart1.gState;

    if (status == HAL_OK) {
        g_usart_cmd_tx_dma_busy    = true;
        g_usart_cmd_tx_dma_pending = false;
    } else if (status != HAL_BUSY) {
        (void)HAL_UART_AbortTransmit(&huart1);
    }
#else
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1,
                                                 g_usart_cmd_tx_dma_buffer,
                                                 g_usart_cmd_tx_dma_len,
                                                 APP_USART_CMD_TX_TIMEOUT_MS);
    if (status == HAL_BUSY) {
        (void)HAL_UART_AbortTransmit(&huart1);
        osDelay(1U);
        status = HAL_UART_Transmit(&huart1,
                                   g_usart_cmd_tx_dma_buffer,
                                   g_usart_cmd_tx_dma_len,
                                   APP_USART_CMD_TX_TIMEOUT_MS);
    }

    g_usart_cmd_tx_status      = status;
    g_usart_cmd_tx_error       = huart1.ErrorCode;
    g_usart_cmd_tx_state       = huart1.gState;
    g_usart_cmd_tx_dma_pending = false;
#endif
}

static HAL_StatusTypeDef send_text(const char *text)
{
    if (text == NULL) {
        return HAL_ERROR;
    }

    size_t len = bounded_strlen(text, UINT16_MAX);
    if (len == 0U) {
        return HAL_OK;
    }

    if (!usart_log_slot_write(APP_USART_LOG_SLOT_CMD,
                              (const uint8_t *)text,
                              len)) {
        return HAL_ERROR;
    }

    g_usart_log_next_slot = APP_USART_LOG_SLOT_CMD;
    return HAL_OK;
}

static void send_cmd_help(void)
{
    send_text("cmd: read | set <pan_hex> <short_hex> <role> | set pan|short|role|log <value> | save | help\r\n");
}

static void usart_cmd_load_config(void)
{
    const AppConfig *cfg = ConfigService_Get();
    if (cfg != NULL && ConfigService_IsValid(cfg)) {
        g_usart_cmd_config = *cfg;
    } else {
        ConfigService_InitDefaults(&g_usart_cmd_config);
    }

    g_usart_cmd_config_valid = ConfigService_IsValid(&g_usart_cmd_config);
    g_usart_cmd_config_dirty = false;
}

static bool usart_cmd_require_config(void)
{
    if (!g_usart_cmd_config_valid) {
        usart_cmd_load_config();
    }

    if (!g_usart_cmd_config_valid) {
        send_text("ERR config unavailable\r\n");
        return false;
    }

    return true;
}

static void print_config(void)
{
    char line[128];

    if (!usart_cmd_require_config()) {
        return;
    }

    (void)snprintf(line, sizeof(line),
                   "pan=0x%04X short=0x%04X role=%u log=%lu dirty=%u\r\n",
                   g_usart_cmd_config.pan_id,
                   g_usart_cmd_config.short_addr,
                   g_usart_cmd_config.role,
                   (unsigned long)g_usart_cmd_config.log_level,
                   g_usart_cmd_config_dirty ? 1U : 0U);
    send_text(line);
}

static char *trim_ascii_space(char *line)
{
    while (*line == ' ' || *line == '\t') {
        ++line;
    }

    size_t len = strlen(line);
    while (len > 0U && (line[len - 1U] == ' ' || line[len - 1U] == '\t')) {
        line[--len] = '\0';
    }

    return line;
}

static bool command_equals(const char *line, const char *cmd)
{
    return strcmp(line, cmd) == 0;
}

static bool command_starts_with(const char *line, const char *cmd)
{
    size_t cmd_len = strlen(cmd);
    return strncmp(line, cmd, cmd_len) == 0 &&
           (line[cmd_len] == '\0' || line[cmd_len] == ' ' || line[cmd_len] == '\t');
}

static bool parse_unsigned_tail(const char *line, const char *fmt, unsigned int *value)
{
    int consumed = 0;
    return sscanf(line, fmt, value, &consumed) == 1 && line[consumed] == '\0';
}

static bool apply_config_set_line(const char *line)
{
    if (!usart_cmd_require_config()) {
        return true;
    }

    unsigned int pan  = 0;
    unsigned int addr = 0;
    unsigned int role = 0;
    unsigned int log  = 0;
    int consumed      = 0;

    if (sscanf(line, "set %x %x %u %n", &pan, &addr, &role, &consumed) == 3 &&
        line[consumed] == '\0') {
        if (pan > UINT16_MAX || addr > UINT16_MAX || role > APP_ROLE_ANCHOR) {
            send_text("ERR set range\r\n");
            return true;
        }

        g_usart_cmd_config.pan_id     = (uint16_t)pan;
        g_usart_cmd_config.short_addr = (uint16_t)addr;
        g_usart_cmd_config.role       = (uint8_t)role;
        g_usart_cmd_config_dirty      = true;
        send_text("OK set\r\n");
        return true;
    }

    if (parse_unsigned_tail(line, "set pan %x %n", &pan)) {
        if (pan > UINT16_MAX) {
            send_text("ERR set range\r\n");
            return true;
        }
        g_usart_cmd_config.pan_id = (uint16_t)pan;
        g_usart_cmd_config_dirty  = true;
        send_text("OK set pan\r\n");
        return true;
    }

    if (parse_unsigned_tail(line, "set short %x %n", &addr)) {
        if (addr > UINT16_MAX) {
            send_text("ERR set range\r\n");
            return true;
        }
        g_usart_cmd_config.short_addr = (uint16_t)addr;
        g_usart_cmd_config_dirty      = true;
        send_text("OK set short\r\n");
        return true;
    }

    if (parse_unsigned_tail(line, "set role %u %n", &role)) {
        if (role > APP_ROLE_ANCHOR) {
            send_text("ERR set range\r\n");
            return true;
        }
        g_usart_cmd_config.role  = (uint8_t)role;
        g_usart_cmd_config_dirty = true;
        send_text("OK set role\r\n");
        return true;
    }

    if (parse_unsigned_tail(line, "set log %u %n", &log)) {
        g_usart_cmd_config.log_level = (uint32_t)log;
        g_usart_cmd_config_dirty     = true;
        send_text("OK set log\r\n");
        return true;
    }

    send_text("ERR set args\r\n");
    send_cmd_help();
    return true;
}

static bool handle_config_line(char *line)
{
    line = trim_ascii_space(line);
    if (*line == '\0') {
        return true;
    }

    if (command_equals(line, "read")) {
        print_config();
        send_text("OK read\r\n");
        return true;
    }

    if (command_equals(line, "help")) {
        send_cmd_help();
        send_text("OK help\r\n");
        return true;
    }

    if (command_equals(line, "save")) {
        if (!usart_cmd_require_config()) {
            return true;
        }

        if (ConfigService_Save(&g_usart_cmd_config) == HAL_OK) {
            g_usart_cmd_config_dirty = false;
            send_text("OK save\r\n");
        } else {
            send_text("ERR save\r\n");
        }
        return true;
    }

    if (command_starts_with(line, "set")) {
        return apply_config_set_line(line);
    }

    send_text("ERR unknown cmd\r\n");
    send_cmd_help();
    return true;
}

static bool start_usart_cmd_dma_idle(void)
{
    memset(g_usart_cmd_rx_buffer, 0, sizeof(g_usart_cmd_rx_buffer));
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(&huart1,
                                                    g_usart_cmd_rx_buffer,
                                                    sizeof(g_usart_cmd_rx_buffer));
    if (status != HAL_OK) {
        (void)HAL_UART_AbortReceive(&huart1);
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        status = HAL_UART_Receive_DMA(&huart1,
                                      g_usart_cmd_rx_buffer,
                                      sizeof(g_usart_cmd_rx_buffer));
    }

    if (status == HAL_OK) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
        return true;
    }

    return false;
}

static bool process_usart_cmd_frame(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0U || len > APP_USART_CMD_RX_BUFFER_SIZE) {
        return false;
    }

    for (uint32_t i = 0; i < len; ++i) {
        char ch = (char)data[i];

        if (ch == '\r' || ch == '\n') {
            if (g_usart_cmd_line_len > 0U) {
                g_usart_cmd_line[g_usart_cmd_line_len] = '\0';
                if (!handle_config_line(g_usart_cmd_line)) {
                    g_usart_cmd_line_len = 0U;
                    memset(g_usart_cmd_line, 0, sizeof(g_usart_cmd_line));
                    return false;
                }
                g_usart_cmd_line_len = 0U;
                memset(g_usart_cmd_line, 0, sizeof(g_usart_cmd_line));
            }
            continue;
        }

        if (g_usart_cmd_line_len >= APP_USART_CMD_RX_BUFFER_SIZE) {
            g_usart_cmd_line_len = 0U;
            memset(g_usart_cmd_line, 0, sizeof(g_usart_cmd_line));
            send_text("ERR cmd too long\r\n");
            return false;
        }

        g_usart_cmd_line[g_usart_cmd_line_len++] = ch;
    }

    return true;
}

void AppUsartCmdTask(void *argument)
{
    (void)argument;

    g_usart_cmd_task = xTaskGetCurrentTaskHandle();
    usart_cmd_load_config();

    bool rx_enabled = g_app_mode == APP_MODE_CONFIG;
    if (rx_enabled) {
        send_text("config mode\r\n");
        send_cmd_help();
        while (!start_usart_cmd_dma_idle()) {
            osDelay(10U);
        }
    } else {
        send_text("run mode\r\n");
    }

    for (;;) {
        usart_cmd_try_start_tx();

        uint32_t notify = 0;
        if (xTaskNotifyWait(0U,
                            UINT32_MAX,
                            &notify,
                            pdMS_TO_TICKS(20U)) != pdTRUE) {
            continue;
        }

        if ((notify & APP_USART_CMD_NOTIFY_TX_DONE) != 0U) {
            g_usart_cmd_tx_dma_busy = false;
        }

        if (!rx_enabled) {
            continue;
        }

        bool restart_rx = false;
        if ((notify & APP_USART_CMD_NOTIFY_RX_FULL) != 0U) {
            memset(g_usart_cmd_rx_buffer, 0, sizeof(g_usart_cmd_rx_buffer));
            g_usart_cmd_line_len = 0U;
            memset(g_usart_cmd_line, 0, sizeof(g_usart_cmd_line));
            send_text("ERR cmd too long\r\n");
            restart_rx = true;
        }

        if ((notify & APP_USART_CMD_NOTIFY_RX_IDLE) != 0U) {
            uint32_t dma_len = notify & APP_USART_CMD_RX_LEN_MASK;
            if (!process_usart_cmd_frame(g_usart_cmd_rx_buffer, dma_len)) {
                memset(g_usart_cmd_rx_buffer, 0, sizeof(g_usart_cmd_rx_buffer));
            }
            restart_rx = true;
        }

        if (restart_rx) {
            while (!start_usart_cmd_dma_idle()) {
                osDelay(10U);
            }
        }
    }
}

void AppTasks_NotifyImuIrqFromISR(void)
{
    if (g_imu_task == NULL) {
        return;
    }

    BaseType_t higher = pdFALSE;
    vTaskNotifyGiveFromISR(g_imu_task, &higher);
    portYIELD_FROM_ISR(higher);
}

void AppTasks_NotifyKeyIrqFromISR(void)
{
    if (g_key_task == NULL) {
        return;
    }

    BaseType_t higher = pdFALSE;
    vTaskNotifyGiveFromISR(g_key_task, &higher);
    portYIELD_FROM_ISR(higher);
}

void AppTasks_NotifyGnssDmaBlockFromISR(uint32_t len)
{
    if (g_gnss_task == NULL) {
        return;
    }

    BaseType_t higher = pdFALSE;
    xTaskNotifyFromISR(g_gnss_task, len, eSetValueWithOverwrite, &higher);
    portYIELD_FROM_ISR(higher);
}

void AppTasks_NotifyUsartCmdDmaBlockFromISR(uint32_t len)
{
    if (g_usart_cmd_task == NULL) {
        return;
    }

    BaseType_t higher = pdFALSE;
    uint32_t notify   = APP_USART_CMD_NOTIFY_RX_IDLE |
                      (len & APP_USART_CMD_RX_LEN_MASK);
    xTaskNotifyFromISR(g_usart_cmd_task, notify, eSetBits, &higher);
    portYIELD_FROM_ISR(higher);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != USART1) {
        return;
    }

    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    if (g_usart_cmd_task != NULL) {
        BaseType_t higher = pdFALSE;
        xTaskNotifyFromISR(g_usart_cmd_task,
                           APP_USART_CMD_NOTIFY_RX_FULL,
                           eSetBits,
                           &higher);
        portYIELD_FROM_ISR(higher);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != USART1) {
        return;
    }

    g_usart_cmd_tx_dma_busy = false;
    g_usart_cmd_tx_status   = HAL_OK;
    g_usart_cmd_tx_error    = huart1.ErrorCode;
    g_usart_cmd_tx_state    = huart1.gState;

    if (g_usart_cmd_task == NULL) {
        return;
    }

    BaseType_t higher = pdFALSE;
    xTaskNotifyFromISR(g_usart_cmd_task,
                       APP_USART_CMD_NOTIFY_TX_DONE,
                       eSetBits,
                       &higher);
    portYIELD_FROM_ISR(higher);
}

void USART1IdleHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == RESET) {
        return;
    }

    uint32_t remaining = APP_USART_CMD_RX_BUFFER_SIZE;
    if (huart1.hdmarx != NULL) {
        remaining = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    }

    uint32_t dma_len = APP_USART_CMD_RX_BUFFER_SIZE - remaining;
    if (dma_len == 0U || dma_len > APP_USART_CMD_RX_BUFFER_SIZE) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        return;
    }

    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    (void)HAL_UART_AbortReceive(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    AppTasks_NotifyUsartCmdDmaBlockFromISR(dma_len);
}

void GNSSIdleHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) == RESET) {
        return;
    }

    __HAL_UART_CLEAR_IDLEFLAG(&huart3);

    uint32_t remaining = 0;
    if (huart3.hdmarx != NULL) {
        remaining = __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    }

    uint32_t dma_len = APP_GNSS_RX_BUFFER_SIZE - remaining;
    (void)HAL_UART_DMAStop(&huart3);

    if (dma_len > 0U && dma_len <= APP_GNSS_RX_BUFFER_SIZE) {
        AppTasks_NotifyGnssDmaBlockFromISR(dma_len);
    }
}
