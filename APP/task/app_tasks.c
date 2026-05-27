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
#include "../UWB/uwb_loss_test.h"
#include "../UWB/uwb_app.h"
#include "../UWB/uwb_stack.h"

#define APP_GNSS_RX_BUFFER_SIZE          (1024U)
#define APP_USART_CMD_RX_BUFFER_SIZE     (128U)
#define APP_USART_CMD_NOTIFY_RX_IDLE     (1UL << 31)
#define APP_USART_CMD_NOTIFY_RX_FULL     (1UL << 30)
#define APP_USART_CMD_NOTIFY_TX_DONE     (1UL << 29)
#define APP_USART_CMD_NOTIFY_LOG         (1UL << 28)
#define APP_USART_CMD_RX_LEN_MASK        (0x0000FFFFUL)
#define APP_USART_CMD_TX_USE_DMA         (1U)
#define APP_USART_CMD_TX_DMA_SIZE        (1024U)
#define APP_USART_CMD_TX_DMA_BUFFER_COUNT (2U)
#define APP_USART_CMD_TX_TIMEOUT_MS      (100U)
#define APP_USART_LOG_SLOT_SIZE          (512U)
#define APP_USART_DATA_LOG_SLOT_SIZE     (4096U)
#define APP_USART_LOSS_LOG_SLOT_SIZE     (1024U)
#define APP_DATA_SORT_WINDOW             (10U)
#define APP_SD_BLOCK_SIZE                (16U * 1024U)
#define APP_ASCII_LINE_SIZE              (384U)
#define APP_KEY_SHORT_PRESS_MS           (1000U)
#define APP_KEY_LONG_PRESS_MS            (2000U)
#define APP_SUPPRESS_SD_WRITE_ERROR_LOGS (1U)
#define APP_UWB_DEBUG_SD_LOG_ENABLED     (0U)
#define APP_LED_LOSS_CMD_QUEUE_LEN       (4U)
#define APP_KEY_EVENT_QUEUE_LEN          (4U)
#define APP_LED_SD_WRITE_PULSE_MS        (80U)
#define APP_USART_RUN_DATA_ONLY          (0U)
#define APP_SUPPRESS_DATA_ASCII_OUTPUT   (1U)

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

#define GNSS_MSG_ID_BESTNAV (0x0846U)

typedef enum {
    APP_SD_BLOCK_MAIN   = 0,
    APP_SD_BLOCK_BACKUP = 1,
} AppSdBlockId;

/* UWB 日志 SD 用独立的通知位 (bit2, bit3), 与数据 FIFO 的 bit0/bit1 互不干扰 */
#define APP_UWB_SD_NOTIFY_MAIN   (1UL << 2)
#define APP_UWB_SD_NOTIFY_BACKUP (1UL << 3)
#define APP_UWB_SD_NOTIFY_MASK   (APP_UWB_SD_NOTIFY_MAIN | APP_UWB_SD_NOTIFY_BACKUP)

typedef enum {
    APP_USART_LOG_SLOT_CMD = 0,
    APP_USART_LOG_SLOT_SYSTEM,
    APP_USART_LOG_SLOT_GNSS,
    APP_USART_LOG_SLOT_IMU,
    APP_USART_LOG_SLOT_UWB,
    APP_USART_LOG_SLOT_DATA,
    APP_USART_LOG_SLOT_SD,
    APP_USART_LOG_SLOT_KEY,
    APP_USART_LOG_SLOT_LOSS,
    APP_USART_LOG_SLOT_COUNT,
} AppUsartLogSlotId;

typedef enum {
    APP_LED_STATE_CONFIG = 0,
    APP_LED_STATE_NORMAL,
    APP_LED_STATE_LOSS_DISPLAY,
} AppLedState;

typedef enum {
    APP_KEY_EVENT_SHORT_PRESS = 0,
    APP_KEY_EVENT_LONG_PRESS,
} AppKeyEventType;

typedef struct {
    AppKeyEventType type;
} AppKeyEventMsg;

typedef struct {
    uint32_t range_event_seen;
    uint32_t sd_write_event_seen;
    uint32_t sd_pulse_started_ms;
    bool range_led_on;
    bool sd_pulse_active;
} AppTagNormalLedCtx;

typedef struct {
    TimeCapture time_capture;
    bool time_valid;
} AppGnssFrameContext;

typedef struct {
    uint8_t *buffer;
    size_t capacity;
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

/* 前向声明: UWB SD FIFO 写入 (定义在后面) */
static bool uwb_sd_fifo_write(const uint8_t *data, size_t len);

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
static AppGnssFrameContext g_gnss_frame_ctx;
static TimeCapture g_gnss_idle_time_capture;
static volatile bool g_gnss_idle_time_valid;
static TimeCapture g_imu_irq_time_capture;
static volatile bool g_imu_irq_time_valid;
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
static uint8_t g_usart_cmd_tx_dma_buffer[APP_USART_CMD_TX_DMA_BUFFER_COUNT][APP_USART_CMD_TX_DMA_SIZE];
static uint16_t g_usart_cmd_tx_dma_len;
static uint8_t g_usart_cmd_tx_dma_active;
static uint8_t g_usart_cmd_tx_dma_pending_index;
static bool g_usart_cmd_tx_dma_pending;
static volatile bool g_usart_cmd_tx_dma_busy;
static uint8_t g_usart_log_slot_storage[APP_USART_LOG_SLOT_COUNT][APP_USART_LOG_SLOT_SIZE];
static uint8_t g_usart_data_log_slot_storage[APP_USART_DATA_LOG_SLOT_SIZE];
static uint8_t g_usart_loss_log_slot_storage[APP_USART_LOSS_LOG_SLOT_SIZE];

static uint8_t g_sd_main_block[APP_SD_BLOCK_SIZE];
static uint8_t g_sd_backup_block[APP_SD_BLOCK_SIZE];
static AppSdFifo g_sd_fifo;
static StaticSemaphore_t g_sd_fifo_mutex_ctrl;
static SemaphoreHandle_t g_sd_fifo_mutex;

/* UWB 日志 SD FIFO (独立 16KB×2 双缓冲) */
static uint8_t g_uwb_sd_main_block[APP_SD_BLOCK_SIZE];
static uint8_t g_uwb_sd_backup_block[APP_SD_BLOCK_SIZE];
static AppSdFifo g_uwb_sd_fifo;
static StaticSemaphore_t g_uwb_sd_fifo_mutex_ctrl;
static SemaphoreHandle_t g_uwb_sd_fifo_mutex;
static bool g_uwb_sd_ready;

static StaticQueue_t g_led_loss_cmd_queue_ctrl;
static uint8_t g_led_loss_cmd_queue_buf[APP_LED_LOSS_CMD_QUEUE_LEN * sizeof(LedLossCmd)];
static QueueHandle_t g_led_loss_cmd_queue;
static StaticQueue_t g_key_event_queue_ctrl;
static uint8_t g_key_event_queue_buf[APP_KEY_EVENT_QUEUE_LEN * sizeof(AppKeyEventMsg)];
static QueueHandle_t g_key_event_queue;
static volatile bool g_anchor_led_local_init;
static volatile bool g_anchor_led_global_init;
static volatile bool g_anchor_led_notify_init;
static volatile uint32_t g_tag_led_range_event_count;
static volatile uint32_t g_tag_led_sd_write_event_count;

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
        case APP_DATA_SRC_UWB_TWR:
            return "UWB_TWR";
        case APP_DATA_SRC_UWB_ANCHOR_DATA:
            return "UWB_ANCHOR_DATA";
        default:
            return "UNKNOWN";
    }
}

static AppDeviceRole app_current_role(void)
{
    const AppConfig *cfg = ConfigService_Get();

    if (cfg == NULL || !ConfigService_IsValid(cfg)) {
        cfg = ConfigService_GetDefaults();
    }

    if (cfg == NULL || !ConfigService_IsValid(cfg)) {
        return APP_ROLE_ANCHOR;
    }

    return (AppDeviceRole)cfg->role;
}

static uint16_t app_current_short_addr(void)
{
    const AppConfig *cfg = ConfigService_Get();

    if (cfg == NULL || !ConfigService_IsValid(cfg)) {
        cfg = ConfigService_GetDefaults();
    }

    if (cfg == NULL || !ConfigService_IsValid(cfg)) {
        return 0U;
    }

    return cfg->short_addr;
}

static bool app_loss_test_enabled(void)
{
    return app_current_role() == APP_ROLE_TAG;
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
        slot->buffer   = g_usart_log_slot_storage[i];
        slot->capacity = APP_USART_LOG_SLOT_SIZE;
        if (i == APP_USART_LOG_SLOT_DATA) {
            slot->buffer   = g_usart_data_log_slot_storage;
            slot->capacity = APP_USART_DATA_LOG_SLOT_SIZE;
        }
        if (i == APP_USART_LOG_SLOT_LOSS) {
            slot->buffer   = g_usart_loss_log_slot_storage;
            slot->capacity = APP_USART_LOSS_LOG_SLOT_SIZE;
        }
        slot->mutex = xSemaphoreCreateMutexStatic(&slot->mutex_ctrl);
        if (slot->mutex == NULL) {
            return;
        }
    }

    g_usart_log_next_slot      = APP_USART_LOG_SLOT_CMD;
    g_usart_cmd_tx_dma_len     = 0U;
    g_usart_cmd_tx_dma_active  = 0U;
    g_usart_cmd_tx_dma_pending_index = 0U;
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
    if (slot == NULL || slot->buffer == NULL || slot->capacity == 0U ||
        len == 0U) {
        return;
    }

    if (len > slot->used) {
        len = slot->used;
    }

    slot->tail = (slot->tail + len) % slot->capacity;
    slot->used -= len;
    slot->dropped += (uint32_t)len;
}

static bool usart_log_slot_wait_for_space(AppUsartLogSlot *slot,
                                          size_t len)
{
    if (slot == NULL || slot->buffer == NULL || slot->capacity == 0U ||
        len == 0U || len > slot->capacity) {
        return false;
    }

    for (;;) {
        if (!usart_log_slot_take(slot)) {
            return false;
        }

        if (len <= (slot->capacity - slot->used)) {
            return true;
        }

        usart_log_slot_give(slot);

        if (!scheduler_running()) {
            return false;
        }

        notify_usart_cmd(APP_USART_CMD_NOTIFY_LOG);
        osDelay(1U);
    }
}

static bool usart_log_slot_output_enabled(AppUsartLogSlotId id)
{
#if APP_USART_RUN_DATA_ONLY
    if (g_app_mode != APP_MODE_CONFIG && id != APP_USART_LOG_SLOT_DATA) {
        return false;
    }
#else
    (void)id;
#endif

    return true;
}

static bool usart_log_slot_write(AppUsartLogSlotId id,
                                 const uint8_t *data,
                                 size_t len)
{
    if (!g_usart_log_ready || data == NULL || len == 0U ||
        id >= APP_USART_LOG_SLOT_COUNT) {
        return false;
    }

    if (!usart_log_slot_output_enabled(id)) {
        return true;
    }

    AppUsartLogSlot *slot = &g_usart_log_slots[id];
    if (slot->buffer == NULL || slot->capacity == 0U ||
        !usart_log_slot_take(slot)) {
        return false;
    }

    if (len > slot->capacity) {
        if (id == APP_USART_LOG_SLOT_DATA) {
            usart_log_slot_give(slot);
            return false;
        }
        data += len - slot->capacity;
        len = slot->capacity;
    }

    size_t free_len = slot->capacity - slot->used;
    if (len > free_len) {
        if (id == APP_USART_LOG_SLOT_DATA) {
            usart_log_slot_give(slot);
            if (!usart_log_slot_wait_for_space(slot, len)) {
                return false;
            }
        } else {
            usart_log_slot_drop_oldest(slot, len - free_len);
        }
    }

    size_t first = slot->capacity - slot->head;
    if (first > len) {
        first = len;
    }
    memcpy(&slot->buffer[slot->head], data, first);

    size_t second = len - first;
    if (second > 0U) {
        memcpy(slot->buffer, data + first, second);
    }

    slot->head = (slot->head + len) % slot->capacity;
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
    if (slot->buffer == NULL || slot->capacity == 0U ||
        !usart_log_slot_take(slot)) {
        return 0U;
    }

    size_t len = slot->used < max_len ? slot->used : max_len;
    if (len > 0U) {
        size_t first = slot->capacity - slot->tail;
        if (first > len) {
            first = len;
        }
        memcpy(data, &slot->buffer[slot->tail], first);

        size_t second = len - first;
        if (second > 0U) {
            memcpy(data + first, slot->buffer, second);
        }

        slot->tail = (slot->tail + len) % slot->capacity;
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
    if (name != NULL && strcmp(name, "uwbLoss") == 0) {
        return APP_USART_LOG_SLOT_LOSS;
    }

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

bool AppTasks_LogWriteSd(const char *text, size_t len)
{
    return uwb_sd_fifo_write((const uint8_t *)text, len);
}

static bool init_led_loss_queue(void)
{
    if (g_led_loss_cmd_queue == NULL) {
        g_led_loss_cmd_queue = xQueueCreateStatic(
            APP_LED_LOSS_CMD_QUEUE_LEN,
            sizeof(LedLossCmd),
            g_led_loss_cmd_queue_buf,
            &g_led_loss_cmd_queue_ctrl);
    }

    return g_led_loss_cmd_queue != NULL;
}

static bool init_key_event_queue(void)
{
    if (g_key_event_queue == NULL) {
        g_key_event_queue = xQueueCreateStatic(
            APP_KEY_EVENT_QUEUE_LEN,
            sizeof(AppKeyEventMsg),
            g_key_event_queue_buf,
            &g_key_event_queue_ctrl);
    }

    return g_key_event_queue != NULL;
}

bool AppTasks_SendLedLossCmd(const LedLossCmd *cmd)
{
    if (g_led_loss_cmd_queue == NULL || cmd == NULL) {
        return false;
    }

    return xQueueSend(g_led_loss_cmd_queue, cmd, 0) == pdPASS;
}

static void app_led_count_event(volatile uint32_t *counter)
{
    if (counter == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    (*counter)++;
    taskEXIT_CRITICAL();
}

static uint32_t app_led_read_event_count(volatile uint32_t *counter)
{
    uint32_t value = 0U;

    if (counter == NULL) {
        return 0U;
    }

    taskENTER_CRITICAL();
    value = *counter;
    taskEXIT_CRITICAL();
    return value;
}

void AppTasks_SetAnchorLocalInitLed(bool on)
{
    g_anchor_led_local_init = on;
}

void AppTasks_SetAnchorGlobalInitLed(bool on)
{
    g_anchor_led_global_init = on;
}

void AppTasks_SetAnchorNotifyInitLed(bool on)
{
    g_anchor_led_notify_init = on;
}

void AppTasks_NotifyUwbRangeSolved(void)
{
    app_led_count_event(&g_tag_led_range_event_count);
}

void AppTasks_NotifySdWriteDone(void)
{
    app_led_count_event(&g_tag_led_sd_write_event_count);
}

static bool app_key_publish_event(AppKeyEventType type)
{
    AppKeyEventMsg event = {
        .type = type,
    };

    if (g_key_event_queue == NULL) {
        return false;
    }

    return xQueueSend(g_key_event_queue, &event, 0) == pdPASS;
}

static void app_led_loss_queue_reset(void)
{
    if (g_led_loss_cmd_queue != NULL) {
        (void)xQueueReset(g_led_loss_cmd_queue);
    }
}

static const char *app_led_state_name(AppLedState state)
{
    switch (state) {
        case APP_LED_STATE_CONFIG:
            return "CONFIG";

        case APP_LED_STATE_NORMAL:
            return "NORMAL";

        case APP_LED_STATE_LOSS_DISPLAY:
            return "LOSS_DISPLAY";

        default:
            return "UNKNOWN";
    }
}

static const char *app_key_event_type_name(AppKeyEventType type)
{
    switch (type) {
        case APP_KEY_EVENT_SHORT_PRESS:
            return "SHORT_PRESS";

        case APP_KEY_EVENT_LONG_PRESS:
            return "LONG_PRESS";

        default:
            return "UNKNOWN";
    }
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

    /* UWB 日志 FIFO */
    if (g_uwb_sd_fifo_mutex == NULL) {
        g_uwb_sd_fifo_mutex = xSemaphoreCreateMutexStatic(&g_uwb_sd_fifo_mutex_ctrl);
    }
    if (g_uwb_sd_fifo_mutex == NULL) {
        return false;
    }
    memset(&g_uwb_sd_fifo, 0, sizeof(g_uwb_sd_fifo));
    g_uwb_sd_fifo.block[APP_SD_BLOCK_MAIN]   = g_uwb_sd_main_block;
    g_uwb_sd_fifo.block[APP_SD_BLOCK_BACKUP] = g_uwb_sd_backup_block;
    g_uwb_sd_fifo.active                     = APP_SD_BLOCK_MAIN;
    g_uwb_sd_ready                           = false;

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

/* ---- UWB 日志 SD FIFO 操作 (与数据 FIFO 结构相同, 使用独立的缓冲和互斥) ---- */

static bool uwb_sd_fifo_take(void)
{
    return g_uwb_sd_fifo_mutex != NULL &&
           xSemaphoreTake(g_uwb_sd_fifo_mutex, portMAX_DELAY) == pdTRUE;
}

static void uwb_sd_fifo_give(void)
{
    (void)xSemaphoreGive(g_uwb_sd_fifo_mutex);
}

static bool uwb_sd_fifo_active_writable(void)
{
    uint8_t active = g_uwb_sd_fifo.active;
    return active < 2U &&
           !g_uwb_sd_fifo.ready[active] &&
           !g_uwb_sd_fifo.locked[active];
}

static bool uwb_sd_fifo_switch_to_free(void)
{
    uint8_t next = g_uwb_sd_fifo.active ^ 1U;
    if (g_uwb_sd_fifo.ready[next] || g_uwb_sd_fifo.locked[next]) {
        return false;
    }
    g_uwb_sd_fifo.active    = next;
    g_uwb_sd_fifo.len[next] = 0;
    return true;
}

static void notify_uwb_sd_block_ready(AppSdBlockId id)
{
    if (g_sd_writer_task == NULL) {
        return;
    }
    uint32_t bit = (id == APP_SD_BLOCK_MAIN) ? APP_UWB_SD_NOTIFY_MAIN : APP_UWB_SD_NOTIFY_BACKUP;
    (void)xTaskNotify(g_sd_writer_task, bit, eSetBits);
}

static bool uwb_sd_fifo_write(const uint8_t *data, size_t len)
{
    if (!g_uwb_sd_ready) {
        return false;
    }

    const uint8_t *src = data;
    size_t remaining   = len;

    while (remaining > 0U) {
        AppSdBlockId ready_id = APP_SD_BLOCK_MAIN;
        bool has_ready        = false;
        bool no_free          = false;
        size_t copied         = 0;

        if (!uwb_sd_fifo_take()) {
            return false;
        }

        if (!uwb_sd_fifo_active_writable() && !uwb_sd_fifo_switch_to_free()) {
            no_free = true;
        }

        if (!no_free) {
            uint8_t active = g_uwb_sd_fifo.active;
            size_t space   = APP_SD_BLOCK_SIZE - g_uwb_sd_fifo.len[active];
            copied         = remaining < space ? remaining : space;

            memcpy(&g_uwb_sd_fifo.block[active][g_uwb_sd_fifo.len[active]],
                   src, copied);
            g_uwb_sd_fifo.len[active] += copied;

            if (g_uwb_sd_fifo.len[active] == APP_SD_BLOCK_SIZE) {
                ready_id                    = (AppSdBlockId)active;
                g_uwb_sd_fifo.ready[active] = true;
                has_ready                   = true;

                if (!uwb_sd_fifo_switch_to_free()) {
                    no_free = true;
                }
            }
        }

        uwb_sd_fifo_give();

        if (has_ready) {
            notify_uwb_sd_block_ready(ready_id);
        }

        if (no_free) {
            return false; /* 静默丢弃, 避免递归日志 */
        }

        src += copied;
        remaining -= copied;
    }

    return true;
}

#if APP_UWB_DEBUG_SD_LOG_ENABLED
static bool uwb_sd_fifo_lock_block(AppSdBlockId id,
                                   const uint8_t **data, size_t *len)
{
    uint8_t index = (uint8_t)id;
    bool locked   = false;

    if (data == NULL || len == NULL || index >= 2U || !uwb_sd_fifo_take()) {
        return false;
    }

    if (g_uwb_sd_fifo.ready[index] && !g_uwb_sd_fifo.locked[index]) {
        g_uwb_sd_fifo.locked[index] = true;
        *data                       = g_uwb_sd_fifo.block[index];
        *len                        = g_uwb_sd_fifo.len[index];
        locked                      = true;
    }

    uwb_sd_fifo_give();
    return locked;
}

static void uwb_sd_fifo_release_block(AppSdBlockId id)
{
    uint8_t index = (uint8_t)id;
    if (index >= 2U || !uwb_sd_fifo_take()) {
        return;
    }
    g_uwb_sd_fifo.ready[index]  = false;
    g_uwb_sd_fifo.locked[index] = false;
    g_uwb_sd_fifo.len[index]    = 0;
    uwb_sd_fifo_give();
}

static void uwb_sd_fifo_unlock_block(AppSdBlockId id)
{
    uint8_t index = (uint8_t)id;
    if (index >= 2U || !uwb_sd_fifo_take()) {
        return;
    }
    g_uwb_sd_fifo.locked[index] = false;
    uwb_sd_fifo_give();
}

static uint32_t uwb_sd_fifo_ready_bits(void)
{
    uint32_t bits = 0;
    if (!uwb_sd_fifo_take()) {
        return 0U;
    }
    for (uint32_t id = APP_SD_BLOCK_MAIN; id <= APP_SD_BLOCK_BACKUP; ++id) {
        if (g_uwb_sd_fifo.ready[id] && !g_uwb_sd_fifo.locked[id]) {
            bits |= (id == APP_SD_BLOCK_MAIN) ? APP_UWB_SD_NOTIFY_MAIN : APP_UWB_SD_NOTIFY_BACKUP;
        }
    }
    uwb_sd_fifo_give();
    return bits;
}
#endif

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

static void format_u64_dec(uint64_t value, char *out, size_t out_size)
{
    char tmp[21];
    size_t pos = 0U;

    if (out == NULL || out_size == 0U) {
        return;
    }
    out[0] = '\0';

    do {
        tmp[pos++] = (char)('0' + (value % 10ULL));
        value /= 10ULL;
    } while (value != 0ULL && pos < sizeof(tmp));

    if (pos >= out_size) {
        pos = out_size - 1U;
    }

    for (size_t i = 0U; i < pos; ++i) {
        out[i] = tmp[pos - 1U - i];
    }
    out[pos] = '\0';
}

static void format_tick20k_ms(uint64_t tick, char *out, size_t out_size)
{
    char ms_int[21];
    uint64_t whole_ms;
    uint32_t frac_ms_x1000;
    int n;

    if (out == NULL || out_size == 0U) {
        return;
    }
    out[0] = '\0';

    whole_ms      = tick / 20ULL;
    frac_ms_x1000 = (uint32_t)((tick % 20ULL) * 50ULL);
    format_u64_dec(whole_ms, ms_int, sizeof(ms_int));

    n = snprintf(out, out_size, "%s.%03lu",
                 ms_int,
                 (unsigned long)frac_ms_x1000);
    if (n <= 0 || (size_t)n >= out_size) {
        out[0] = '\0';
    }
}

static bool take_cached_time_capture(TimeCapture *out,
                                     TimeCapture *cache,
                                     volatile bool *valid)
{
    bool ok = false;

    if (out == NULL || cache == NULL || valid == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    if (*valid) {
        *out  = *cache;
        *valid = false;
        ok    = true;
    }
    taskEXIT_CRITICAL();

    return ok;
}

static size_t format_node_ascii(const AppDataNode *node,
                                const TimeTimestamp *ts,
                                char *line,
                                size_t line_size)
{
    int n = 0;
    char mono_ms[25];

    if (node == NULL || ts == NULL || line == NULL || line_size == 0U) {
        return 0;
    }

    format_tick20k_ms(node->time_capture.local_tick_20k,
                      mono_ms,
                      sizeof(mono_ms));

    uint32_t week    = ts->utc_valid ? ts->local_utc.week : 0U;
    uint32_t week_ms = ts->utc_valid ? ts->local_utc.week_ms : 0U;

    switch (node->source) {
        case APP_DATA_SRC_GNSS:
            n = snprintf(line, line_size,
                         "%lu,%lu,%u,%s,GNSS,%.9f,%.9f,%.4f,%lu,%.4f,%.4f,%.4f,%lu,%lu,%.3f,%.3f,%u,%u\r\n",
                         (unsigned long)week,
                         (unsigned long)week_ms,
                         ts->utc_valid ? 1U : 0U,
                         mono_ms,
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
                         "%lu,%lu,%u,%s,IMU,%d,%d,%d,%d,%d,%d\r\n",
                         (unsigned long)week,
                         (unsigned long)week_ms,
                         ts->utc_valid ? 1U : 0U,
                         mono_ms,
                         node->payload.imu.accel[0],
                         node->payload.imu.accel[1],
                         node->payload.imu.accel[2],
                         node->payload.imu.gyro[0],
                         node->payload.imu.gyro[1],
                         node->payload.imu.gyro[2]);
            break;

        case APP_DATA_SRC_UWB_TWR:
            n = snprintf(line, line_size,
                         "%lu,%lu,%u,%s,UWB_TWR,0x%04X,0x%04X,%u,0x%04X,%.3f,%u,%u,%u,%u,%u,%u,%u\r\n",
                         (unsigned long)week,
                         (unsigned long)week_ms,
                         ts->utc_valid ? 1U : 0U,
                         mono_ms,
                         (unsigned)node->payload.uwb_twr.anchor_id,
                         (unsigned)node->payload.uwb_twr.tag_id,
                         (unsigned)node->payload.uwb_twr.exchange_seq,
                         (unsigned)node->payload.uwb_twr.status_flags,
                         node->payload.uwb_twr.distance_m,
                         (unsigned)node->payload.uwb_twr.rx_pacc,
                         (unsigned)node->payload.uwb_twr.fp_index,
                         (unsigned)node->payload.uwb_twr.fp_ampl1,
                         (unsigned)node->payload.uwb_twr.fp_ampl2,
                         (unsigned)node->payload.uwb_twr.fp_ampl3,
                         (unsigned)node->payload.uwb_twr.std_noise,
                         (unsigned)node->payload.uwb_twr.max_noise);
            break;

        default:
            return 0;
    }

    if (n <= 0 || (size_t)n >= line_size) {
        return 0;
    }

    return bounded_strlen(line, line_size);
}

static size_t format_anchor_data_ascii(const AppDataNode *node,
                                       const TimeTimestamp *ts,
                                       uint8_t entry_index,
                                       char *line,
                                       size_t line_size)
{
    int n = 0;
    char mono_ms[25];
    const AppUwbAnchorDataSample *sample;
    AppUwbAnchorEntry empty_entry  = {0};
    const AppUwbAnchorEntry *entry = &empty_entry;

    if (node == NULL || ts == NULL || line == NULL || line_size == 0U ||
        node->source != APP_DATA_SRC_UWB_ANCHOR_DATA) {
        return 0;
    }

    sample = &node->payload.uwb_anchor_data;
    if (sample->entry_count > 0U) {
        if (entry_index >= sample->entry_count ||
            entry_index >= APP_UWB_ANCHOR_DATA_MAX_ENTRIES) {
            return 0;
        }
        entry = &sample->entries[entry_index];
    } else if (entry_index != 0U) {
        return 0;
    }

    format_tick20k_ms(node->time_capture.local_tick_20k,
                      mono_ms,
                      sizeof(mono_ms));

    uint32_t week    = ts->utc_valid ? ts->local_utc.week : 0U;
    uint32_t week_ms = ts->utc_valid ? ts->local_utc.week_ms : 0U;

    n = snprintf(line, line_size,
                 "%lu,%lu,%u,%s,UWB_ANCHOR_DATA,0x%04X,%u,%u,%u,0x%04X,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%u,%u,0x%02X,0x%08lX,0x%04X,%u,%u,0x%04X\r\n",
                 (unsigned long)week,
                 (unsigned long)week_ms,
                 ts->utc_valid ? 1U : 0U,
                 mono_ms,
                 (unsigned)sample->source_anchor_id,
                 (unsigned)sample->table_seq,
                 (unsigned)entry_index,
                 (unsigned)sample->entry_count,
                 (unsigned)entry->peer_anchor,
                 (unsigned)entry->dist_cm,
                 (unsigned)entry->dist_std_cm,
                 (unsigned)entry->avg_pacc,
                 (unsigned)entry->avg_fp_index,
                 (unsigned)entry->avg_fp_ampl1,
                 (unsigned)entry->avg_fp_ampl2,
                 (unsigned)entry->avg_fp_ampl3,
                 (unsigned)entry->avg_std_noise,
                 (unsigned)entry->avg_max_noise,
                 (int)entry->avg_rx_power_dbm_x100,
                 (int)entry->avg_fp_power_dbm_x100,
                 (unsigned)entry->samples,
                 (unsigned)entry->quality,
                 (unsigned)entry->flags,
                 (unsigned long)entry->rx_error_flags,
                 (unsigned)entry->lde_status,
                 (unsigned)sample->total_len,
                 (unsigned)sample->total_frags,
                 (unsigned)sample->table_crc);

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
    AppGnssFrameContext *ctx = (AppGnssFrameContext *)user;

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
    if (ctx != NULL && ctx->time_valid) {
        node.time_capture = ctx->time_capture;
    } else {
        (void)TimeService_CaptureNow(&node.time_capture);
    }
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

static int compare_local_tick(uint64_t a, uint64_t b)
{
    if (a < b) {
        return -1;
    }
    if (a > b) {
        return 1;
    }
    return 0;
}

static int compare_node_time(const AppDataNode *a, const AppDataNode *b)
{
    int cmp = compare_local_tick(a->time_capture.local_tick_20k,
                                 b->time_capture.local_tick_20k);
    if (cmp != 0) {
        return cmp;
    }

    if (a->enqueue_seq < b->enqueue_seq) {
        return -1;
    }
    if (a->enqueue_seq > b->enqueue_seq) {
        return 1;
    }
    return 0;
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

static void write_data_ascii_outputs(const char *line,
                                     size_t len,
                                     AppDataSource source)
{
    if (line == NULL || len == 0U) {
        return;
    }

    (void)usart_log_slot_write(APP_USART_LOG_SLOT_DATA,
                               (const uint8_t *)line,
                               len);

    if (!sd_fifo_write_ascii((const uint8_t *)line, len, source)) {
        app_log_sd_error("write ASCII to SD FIFO failed: source=%s",
                         data_source_name(source));
    }
}

static void write_sorted_node_to_outputs(const AppDataNode *node)
{
    if (node == NULL) {
        return;
    }

#if APP_SUPPRESS_DATA_ASCII_OUTPUT
    return;
#endif

    TimeTimestamp ts;
    if (!TimeService_ResolveCapture(&node->time_capture, &ts)) {
        app_log_warn("resolve data node time failed: source=%s",
                     data_source_name(node->source));
        return;
    }

    char line[APP_ASCII_LINE_SIZE];
    if (node->source == APP_DATA_SRC_UWB_ANCHOR_DATA) {
        uint8_t entry_count = node->payload.uwb_anchor_data.entry_count;
        uint8_t line_count  = entry_count == 0U ? 1U : entry_count;
        if (line_count > APP_UWB_ANCHOR_DATA_MAX_ENTRIES) {
            line_count = APP_UWB_ANCHOR_DATA_MAX_ENTRIES;
        }

        for (uint8_t i = 0U; i < line_count; ++i) {
            size_t len = format_anchor_data_ascii(node, &ts, i, line, sizeof(line));
            if (len == 0U) {
                app_log_warn("format data node failed: source=%s",
                             data_source_name(node->source));
                return;
            }
            write_data_ascii_outputs(line, len, node->source);
        }
    } else {
        size_t len = format_node_ascii(node, &ts, line, sizeof(line));
        if (len == 0U) {
            app_log_warn("format data node failed: source=%s",
                         data_source_name(node->source));
            return;
        }
        write_data_ascii_outputs(line, len, node->source);
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

    bool loss_test_enabled = app_loss_test_enabled();

    if (!DataService_Init() ||
        !init_sd_fifo() ||
        !init_key_event_queue() ||
        (loss_test_enabled &&
         (!init_led_loss_queue() || !UwbLossTest_Init()))) {
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
        .stack_size = 2048U * 4U,
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
    const osThreadAttr_t loss_attr = {
        .name       = "uwbLoss",
        .stack_size = 1024U * 4U,
        .priority   = osPriorityNormal,
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

    if (loss_test_enabled && !UwbLossTest_StartThread(&loss_attr)) {
        app_log_error("UWB loss test start failed");
        return false;
    }

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
    GnssParser_Init(&g_gnss_parser, gnss_frame_handler, &g_gnss_frame_ctx);
    start_gnss_dma_idle();

    for (;;) {
        uint32_t dma_len = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &dma_len, portMAX_DELAY) == pdTRUE) {
            if (dma_len > 0U && dma_len <= APP_GNSS_RX_BUFFER_SIZE) {
                g_gnss_frame_ctx.time_valid =
                    take_cached_time_capture(&g_gnss_frame_ctx.time_capture,
                                             &g_gnss_idle_time_capture,
                                             &g_gnss_idle_time_valid);
                GnssParser_ProcessBlock(&g_gnss_parser, g_gnss_rx_buffer, dma_len);
                g_gnss_frame_ctx.time_valid = false;
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

        TimeCapture irq_time_capture;
        bool irq_time_valid =
            take_cached_time_capture(&irq_time_capture,
                                     &g_imu_irq_time_capture,
                                     &g_imu_irq_time_valid);

        AppDataNode node = {0};
        node.source      = APP_DATA_SRC_IMU;
        if (ImuDevice_ReadRaw(&node.payload.imu) == 0) {
            if (irq_time_valid) {
                node.time_capture = irq_time_capture;
            } else {
                (void)TimeService_CaptureNow(&node.time_capture);
            }
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

        write_sorted_node_to_outputs(&window[0]);
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
#if APP_UWB_DEBUG_SD_LOG_ENABLED
    FIL uwb_log_file;
#endif
    bool mounted = false;
    bool opened  = false;
#if APP_UWB_DEBUG_SD_LOG_ENABLED
    bool uwb_log_opened = false;
#endif

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

#if APP_UWB_DEBUG_SD_LOG_ENABLED
        /* UWB debug 日志文件: 独立打开 */
        if (!uwb_log_opened) {
            uwb_log_opened = StorageService_OpenNextUwbLog(&uwb_log_file);
            if (uwb_log_opened) {
                g_uwb_sd_ready = true;
            }
        }
#endif

        uint32_t notify_bits = sd_fifo_ready_bits();
#if APP_UWB_DEBUG_SD_LOG_ENABLED
        notify_bits |= uwb_sd_fifo_ready_bits();
#endif
        if (notify_bits == 0U) {
            uint32_t wait_mask = sd_ready_bit(APP_SD_BLOCK_MAIN) |
                                 sd_ready_bit(APP_SD_BLOCK_BACKUP);
#if APP_UWB_DEBUG_SD_LOG_ENABLED
            wait_mask |= APP_UWB_SD_NOTIFY_MASK;
#endif
            if (xTaskNotifyWait(0U,
                                wait_mask,
                                &notify_bits,
                                portMAX_DELAY) != pdTRUE) {
                continue;
            }
        }

        /* ---- 处理数据 FIFO (GNSS/IMU) ---- */
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
                AppTasks_NotifySdWriteDone();
                sd_fifo_release_block((AppSdBlockId)id);
            }

            if (write_failed) {
                break;
            }
        }

#if APP_UWB_DEBUG_SD_LOG_ENABLED
        /* ---- 处理 UWB debug 日志 FIFO ---- */
        if (uwb_log_opened) {
            for (uint32_t id = APP_SD_BLOCK_MAIN; id <= APP_SD_BLOCK_BACKUP; ++id) {
                uint32_t uwb_bit = (id == APP_SD_BLOCK_MAIN) ? APP_UWB_SD_NOTIFY_MAIN : APP_UWB_SD_NOTIFY_BACKUP;

                if ((notify_bits & uwb_bit) == 0U) {
                    continue;
                }

                const uint8_t *data = NULL;
                size_t len          = 0;
                if (!uwb_sd_fifo_lock_block((AppSdBlockId)id, &data, &len)) {
                    continue;
                }

                if (len != APP_SD_BLOCK_SIZE ||
                    !StorageService_WriteBlock(&uwb_log_file, data, len)) {
                    (void)f_close(&uwb_log_file);
                    uwb_log_opened = false;
                    g_uwb_sd_ready = false;
                    uwb_sd_fifo_unlock_block((AppSdBlockId)id);
                    break;
                } else {
                    (void)f_sync(&uwb_log_file);
                    uwb_sd_fifo_release_block((AppSdBlockId)id);
                }
            }
        }
#endif
    }
}

void AppKeyTask(void *argument)
{
    (void)argument;

    g_key_task = xTaskGetCurrentTaskHandle();
    BspKeyState key;
    BspKey_Init(&key);

    for (;;) {
        AppKeyEventType type;
        bool publish = false;

        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U));

        BspKeyEvent evt = BspKey_Poll(&key,
                                      APP_KEY_SHORT_PRESS_MS,
                                      APP_KEY_LONG_PRESS_MS);
        switch (evt) {
            case BSP_KEY_EVENT_SHORT_PRESS:
                type    = APP_KEY_EVENT_SHORT_PRESS;
                publish = true;
                break;

            case BSP_KEY_EVENT_LONG_PRESS:
                type    = APP_KEY_EVENT_LONG_PRESS;
                publish = true;
                break;

            default:
                break;
        }

        if (publish) {
            if (type == APP_KEY_EVENT_SHORT_PRESS &&
                app_current_role() == APP_ROLE_ANCHOR) {
                uint16_t self_addr = app_current_short_addr();
                bool requested     = UwbApp_RequestProxBuild();

                char line[128];
                int n = snprintf(line, sizeof(line),
                                 "[PROX] key short self=0x%04X request=%u\r\n",
                                 self_addr,
                                 requested ? 1U : 0U);
                if (n > 0) {
                    AppTasks_LogWriteText(line, bounded_strlen(line, sizeof(line)));
                }
                app_log_info("KEY anchor prox_build request=%u self=0x%04X",
                             requested ? 1U : 0U,
                             self_addr);
            }

            bool queued = app_key_publish_event(type);
            app_log_info("KEY event=%s queued=%u",
                         app_key_event_type_name(type),
                         queued ? 1U : 0U);
        }
    }
}

static void app_led_set_loss_bar(uint8_t rate)
{
    BspLed_Set(BSP_LED_0, rate >= 80U);
    BspLed_Set(BSP_LED_1, rate >= 90U);
    BspLed_Set(BSP_LED_2, rate >= 95U);
}

static void app_led_apply_loss_cmd(const LedLossCmd *cmd)
{
    if (cmd == NULL) {
        return;
    }

    switch (cmd->mode) {
        case LED_LOSS_MODE_CONFIRM:
            BspLed_Set(BSP_LED_0, cmd->led0 != 0U);
            BspLed_Set(BSP_LED_1, cmd->led1 != 0U);
            BspLed_Set(BSP_LED_2, cmd->led2 != 0U);
            break;

        case LED_LOSS_MODE_RATE:
            BspLed_Set(BSP_LED_0, cmd->led0 != 0U);
            BspLed_Set(BSP_LED_1, cmd->led1 != 0U);
            BspLed_Set(BSP_LED_2, cmd->led2 != 0U);
            break;

        case LED_LOSS_MODE_OFF:
        default:
            app_led_set_loss_bar(0U);
            break;
    }
}

static void app_led_apply_anchor_state(void)
{
    BspLed_Set(BSP_LED_0, g_anchor_led_global_init);
    BspLed_Set(BSP_LED_1, g_anchor_led_local_init);
    BspLed_Set(BSP_LED_2, g_anchor_led_notify_init);
}

static void app_led_sync_tag_normal_events(AppTagNormalLedCtx *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ctx->range_event_seen =
        app_led_read_event_count(&g_tag_led_range_event_count);
    ctx->sd_write_event_seen =
        app_led_read_event_count(&g_tag_led_sd_write_event_count);
}

static void app_led_apply_tag_normal_state(AppTagNormalLedCtx *ctx,
                                           uint32_t now)
{
    if (ctx == NULL) {
        return;
    }

    uint32_t range_events =
        app_led_read_event_count(&g_tag_led_range_event_count);
    if (range_events != ctx->range_event_seen) {
        ctx->range_event_seen = range_events;
        ctx->range_led_on     = !ctx->range_led_on;
    }

    uint32_t sd_write_events =
        app_led_read_event_count(&g_tag_led_sd_write_event_count);
    if (sd_write_events != ctx->sd_write_event_seen) {
        ctx->sd_write_event_seen  = sd_write_events;
        ctx->sd_pulse_started_ms  = now;
        ctx->sd_pulse_active      = true;
    }

    if (ctx->sd_pulse_active &&
        (uint32_t)(now - ctx->sd_pulse_started_ms) >=
            APP_LED_SD_WRITE_PULSE_MS) {
        ctx->sd_pulse_active = false;
    }

    BspLed_Set(BSP_LED_0, ctx->sd_pulse_active);
    BspLed_Set(BSP_LED_1, ctx->range_led_on);
    BspLed_Set(BSP_LED_2, false);
}

static void app_led_enter_normal_state(AppLedState *state)
{
    if (state != NULL) {
        *state = APP_LED_STATE_NORMAL;
    }

    app_led_loss_queue_reset();
    app_led_set_loss_bar(0U);
}

static void app_led_enter_loss_state(AppLedState *state)
{
    if (state != NULL) {
        *state = APP_LED_STATE_LOSS_DISPLAY;
    }

    app_led_loss_queue_reset();
    app_led_set_loss_bar(0U);
}

static void app_led_handle_key_event(AppLedState *state,
                                     const AppKeyEventMsg *event)
{
    if (state == NULL || event == NULL) {
        return;
    }

    if (*state == APP_LED_STATE_CONFIG) {
        app_log_info("LED key=%s ignored state=%s",
                     app_key_event_type_name(event->type),
                     app_led_state_name(*state));
        return;
    }

    switch (event->type) {
        case APP_KEY_EVENT_SHORT_PRESS:
            if (!app_loss_test_enabled()) {
                app_log_info("LED key=%s ignored role=%u state=%s",
                             app_key_event_type_name(event->type),
                             (unsigned)app_current_role(),
                             app_led_state_name(*state));
                break;
            }

            if (*state == APP_LED_STATE_LOSS_DISPLAY) {
                app_led_enter_normal_state(state);
                app_log_info("LED key=%s state=%s",
                             app_key_event_type_name(event->type),
                             app_led_state_name(*state));
            } else {
                app_led_enter_loss_state(state);
                app_log_info("LED key=%s state=%s",
                             app_key_event_type_name(event->type),
                             app_led_state_name(*state));
            }
            break;

        case APP_KEY_EVENT_LONG_PRESS:
            app_log_info("LED key=%s ignored state=%s",
                         app_key_event_type_name(event->type),
                         app_led_state_name(*state));
            break;

        default:
            break;
    }
}

void AppLedTask(void *argument)
{
    (void)argument;

    g_led_task = xTaskGetCurrentTaskHandle();

    uint32_t index        = 0;
    uint32_t heartbeat_ms = HAL_GetTick();
    AppLedState state     = (g_app_mode == APP_MODE_CONFIG) ? APP_LED_STATE_CONFIG : APP_LED_STATE_NORMAL;
    AppTagNormalLedCtx tag_led_ctx;
    memset(&tag_led_ctx, 0, sizeof(tag_led_ctx));
    app_led_sync_tag_normal_events(&tag_led_ctx);

    for (;;) {
        AppKeyEventMsg event;
        while (g_key_event_queue != NULL &&
               xQueueReceive(g_key_event_queue, &event, 0) == pdPASS) {
            app_led_handle_key_event(&state, &event);
        }

        uint32_t now = HAL_GetTick();

        if (state == APP_LED_STATE_CONFIG) {
            app_led_sync_tag_normal_events(&tag_led_ctx);
            BspLed_AllOff();
            BspLed_Set((BspLedId)(index % BSP_LED_COUNT), true);
            index++;
            osDelay(200U);
            continue;
        }

        if (state == APP_LED_STATE_LOSS_DISPLAY) {
            LedLossCmd cmd;
            while (g_led_loss_cmd_queue != NULL &&
                   xQueueReceive(g_led_loss_cmd_queue, &cmd, 0) == pdPASS) {
                app_led_apply_loss_cmd(&cmd);
            }
            app_led_sync_tag_normal_events(&tag_led_ctx);
        } else if (app_current_role() == APP_ROLE_ANCHOR) {
            app_led_apply_anchor_state();
            app_led_sync_tag_normal_events(&tag_led_ctx);
        } else {
            app_led_apply_tag_normal_state(&tag_led_ctx, now);
        }

        if ((uint32_t)(now - heartbeat_ms) >= 500U) {
            BspLed_Toggle(BSP_LED_3);
            heartbeat_ms = now;
        }

        osDelay(20U);
    }
}

static size_t usart_cmd_fill_tx_dma_buffer(uint8_t *buffer,
                                           size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0U) {
        return 0U;
    }

    for (uint32_t checked = 0; checked < APP_USART_LOG_SLOT_COUNT; ++checked) {
        AppUsartLogSlotId id =
            (AppUsartLogSlotId)((g_usart_log_next_slot + checked) %
                                APP_USART_LOG_SLOT_COUNT);

        size_t len = usart_log_slot_read(id,
                                         buffer,
                                         buffer_size);
        if (len > 0U) {
            g_usart_log_next_slot = ((uint32_t)id + 1U) %
                                    APP_USART_LOG_SLOT_COUNT;
            return len;
        }
    }

    return 0U;
}

static bool usart_cmd_prepare_tx_dma_buffer(void)
{
    if (g_usart_cmd_tx_dma_pending) {
        return true;
    }

    uint8_t fill_index =
        (uint8_t)((g_usart_cmd_tx_dma_active + 1U) %
                  APP_USART_CMD_TX_DMA_BUFFER_COUNT);

    size_t len = usart_cmd_fill_tx_dma_buffer(
        g_usart_cmd_tx_dma_buffer[fill_index],
        sizeof(g_usart_cmd_tx_dma_buffer[fill_index]));
    if (len == 0U) {
        return false;
    }

    g_usart_cmd_tx_dma_pending_index = fill_index;
    g_usart_cmd_tx_dma_len           = (uint16_t)len;
    g_usart_cmd_tx_dma_pending       = true;
    return true;
}

static void usart_cmd_try_start_tx(void)
{
#if APP_USART_CMD_TX_USE_DMA
    if (!g_usart_cmd_tx_dma_pending) {
        (void)usart_cmd_prepare_tx_dma_buffer();
    }

    if (g_usart_cmd_tx_dma_busy || !g_usart_cmd_tx_dma_pending) {
        return;
    }

    uint8_t tx_index = g_usart_cmd_tx_dma_pending_index;

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1,
                                                     g_usart_cmd_tx_dma_buffer[tx_index],
                                                     g_usart_cmd_tx_dma_len);
    g_usart_cmd_tx_status    = status;
    g_usart_cmd_tx_error     = huart1.ErrorCode;
    g_usart_cmd_tx_state     = huart1.gState;

    if (status == HAL_OK) {
        g_usart_cmd_tx_dma_active  = tx_index;
        g_usart_cmd_tx_dma_busy    = true;
        g_usart_cmd_tx_dma_pending = false;
    } else if (status != HAL_BUSY) {
        (void)HAL_UART_AbortTransmit(&huart1);
    }
#else
    if (!g_usart_cmd_tx_dma_pending &&
        !usart_cmd_prepare_tx_dma_buffer()) {
        return;
    }

    uint8_t tx_index = g_usart_cmd_tx_dma_pending_index;
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1,
                                                 g_usart_cmd_tx_dma_buffer[tx_index],
                                                 g_usart_cmd_tx_dma_len,
                                                 APP_USART_CMD_TX_TIMEOUT_MS);
    if (status == HAL_BUSY) {
        (void)HAL_UART_AbortTransmit(&huart1);
        osDelay(1U);
        status = HAL_UART_Transmit(&huart1,
                                   g_usart_cmd_tx_dma_buffer[tx_index],
                                   g_usart_cmd_tx_dma_len,
                                   APP_USART_CMD_TX_TIMEOUT_MS);
    }

    g_usart_cmd_tx_status      = status;
    g_usart_cmd_tx_error       = huart1.ErrorCode;
    g_usart_cmd_tx_state       = huart1.gState;
    g_usart_cmd_tx_dma_active  = tx_index;
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

    TimeCapture capture;
    if (TimeService_CaptureNow(&capture)) {
        g_imu_irq_time_capture = capture;
        g_imu_irq_time_valid   = true;
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

    TimeCapture capture;
    if (TimeService_CaptureNow(&capture)) {
        g_gnss_idle_time_capture = capture;
        g_gnss_idle_time_valid   = true;
    }

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
