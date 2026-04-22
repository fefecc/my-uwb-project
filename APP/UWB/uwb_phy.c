#include "uwb_phy.h"

#include <string.h>

#include "cmsis_os2.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "../service/log_service.h"
#include "../service/time_service.h"

#define UWB_PHY_CMD_QUEUE_LENGTH  (4U)
#define UWB_PHY_EVT_QUEUE_LENGTH  (8U)
#define UWB_PHY_NOTIFY_IRQ        (1UL << 0)
#define UWB_PHY_NOTIFY_CMD        (1UL << 1)
#define UWB_PHY_STATUS_CLEAR_MASK (SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_RXRFTO)

static StaticQueue_t g_phy_cmd_queue_ctrl;
static uint8_t g_phy_cmd_queue_storage[UWB_PHY_CMD_QUEUE_LENGTH * sizeof(UwbPhyCommand)];
static QueueHandle_t g_phy_cmd_queue;

static StaticQueue_t g_phy_evt_queue_ctrl;
static uint8_t g_phy_evt_queue_storage[UWB_PHY_EVT_QUEUE_LENGTH * sizeof(UwbPhyEvent)];
static QueueHandle_t g_phy_evt_queue;

static UwbPhySlot g_phy_slots[UWB_STACK_PHY_SLOT_COUNT];
static TaskHandle_t g_phy_task;

static bool read_local_clock(TimeLocalClock *out)
{
    if (!TimeService_GetLocalClock(out)) {
        memset(out, 0, sizeof(*out));
        return false;
    }
    return true;
}

static void post_event(const UwbPhyEvent *event)
{
    if (g_phy_evt_queue == NULL || event == NULL) {
        return;
    }

    if (xQueueSend(g_phy_evt_queue, event, 0) != pdPASS) {
        app_log_warn("UWB phy event queue full: evt=%u", (unsigned)event->event_type);
    }
}

static UwbPhySlot *alloc_phy_slot(void)
{
    for (uint32_t i = 0; i < UWB_STACK_PHY_SLOT_COUNT; ++i) {
        if (g_phy_slots[i].owner == UWB_SLOT_OWNER_FREE &&
            g_phy_slots[i].state == UWB_SLOT_EMPTY) {
            g_phy_slots[i].owner   = UWB_SLOT_OWNER_PHY;
            g_phy_slots[i].state   = UWB_SLOT_WRITING;
            g_phy_slots[i].slot_id = (uint8_t)i;
            return &g_phy_slots[i];
        }
    }

    return NULL;
}

static uint64_t read_rx_timestamp(void)
{
    uwb_timestamp_t ts;
    memset(&ts, 0, sizeof(ts));
    dwt_readrxtimestamp(ts.bytes);
    return uwb_timestamp_to_u64(&ts);
}

static uint64_t read_tx_timestamp(void)
{
    uwb_timestamp_t ts;
    memset(&ts, 0, sizeof(ts));
    dwt_readtxtimestamp(ts.bytes);
    return uwb_timestamp_to_u64(&ts);
}

static void handle_rx_ok(uint32_t status, const TimeLocalClock *local_now)
{
    UwbPhySlot *slot = alloc_phy_slot();
    if (slot == NULL) {
        app_log_error("UWB phy slot overflow");
        return;
    }

    uint32_t frame_info = dwt_read32bitreg(RX_FINFO_ID);
    uint16_t frame_len  = (uint16_t)(frame_info & RX_FINFO_RXFL_MASK_1023);
    if (frame_len > UWB_STACK_MAX_FRAME_LEN) {
        frame_len = UWB_STACK_MAX_FRAME_LEN;
    }

    memset(slot->rx_data, 0, sizeof(slot->rx_data));
    dwt_readrxdata(slot->rx_data, frame_len, 0);

    slot->rx_len         = frame_len;
    slot->rx_ts          = read_rx_timestamp();
    slot->tx_ts          = 0;
    slot->irq_local_time = *local_now;
    memset(&slot->quality, 0, sizeof(slot->quality));
    slot->quality.rx_pacc        = (uint16_t)((frame_info & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT);
    slot->quality.rx_error_flags = 0;
    slot->error_flags            = 0;
    slot->valid                  = true;
    slot->generation++;
    slot->owner = UWB_SLOT_OWNER_LINK;
    slot->state = UWB_SLOT_READY;

    UwbPhyEvent event = {
        .event_type = UWB_PHY_EVT_RX_OK,
        .slot_id    = slot->slot_id,
        .generation = slot->generation,
        .status     = status,
        .dw_ts      = slot->rx_ts,
        .timestamp  = *local_now,
    };
    post_event(&event);
}

static void handle_tx_done(uint32_t status, const TimeLocalClock *local_now)
{
    UwbPhyEvent event = {
        .event_type = UWB_PHY_EVT_TX_DONE,
        .slot_id    = 0xFFU,
        .generation = 0,
        .status     = status,
        .dw_ts      = read_tx_timestamp(),
        .timestamp  = *local_now,
    };
    post_event(&event);
}

static void handle_rx_error(UwbPhyEventType type, uint32_t status, const TimeLocalClock *local_now)
{
    UwbPhyEvent event = {
        .event_type = type,
        .slot_id    = 0xFFU,
        .generation = 0,
        .status     = status,
        .dw_ts      = 0,
        .timestamp  = *local_now,
    };
    post_event(&event);
}

static void handle_irq(void)
{
    uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
    TimeLocalClock local_now;
    (void)read_local_clock(&local_now);

    if ((status & SYS_STATUS_RXFCG) != 0U) {
        handle_rx_ok(status, &local_now);
    }

    if ((status & SYS_STATUS_TXFRS) != 0U) {
        handle_tx_done(status, &local_now);
    }

    if ((status & SYS_STATUS_RXRFTO) != 0U) {
        handle_rx_error(UWB_PHY_EVT_RX_TIMEOUT, status, &local_now);
    } else if ((status & SYS_STATUS_ALL_RX_ERR) != 0U) {
        handle_rx_error(UWB_PHY_EVT_RX_ERROR, status, &local_now);
        dwt_rxreset();
    }

    if ((status & SYS_STATUS_TXERR) != 0U) {
        handle_rx_error(UWB_PHY_EVT_TX_ERROR, status, &local_now);
    }

    dwt_write32bitreg(SYS_STATUS_ID, status & UWB_PHY_STATUS_CLEAR_MASK);
}

static void execute_command(const UwbPhyCommand *cmd)
{
    if (cmd == NULL) {
        return;
    }

    switch (cmd->cmd_type) {
        case UWB_PHY_CMD_RX_ENABLE:
            dwt_setrxtimeout(cmd->rx_timeout_uus);
            if (dwt_rxenable(0) != 0) {
                app_log_warn("UWB RX enable failed");
            }
            break;

        case UWB_PHY_CMD_TX_NOW:
            if (cmd->tx_len == 0U || cmd->tx_len > UWB_STACK_MAX_FRAME_LEN) {
                app_log_warn("UWB TX_NOW invalid len=%u", (unsigned)cmd->tx_len);
                break;
            }
            (void)dwt_writetxdata(cmd->tx_len, (uint8_t *)cmd->tx_buf, 0);
            (void)dwt_writetxfctrl(cmd->tx_len, 0);
            if (dwt_starttx(DWT_START_TX_IMMEDIATE) != 0) {
                app_log_warn("UWB TX_NOW start failed");
            }
            break;

        case UWB_PHY_CMD_TX_DELAYED:
            if (cmd->tx_len == 0U || cmd->tx_len > UWB_STACK_MAX_FRAME_LEN) {
                app_log_warn("UWB TX_DELAYED invalid len=%u", (unsigned)cmd->tx_len);
                break;
            }
            dwt_setdelayedtrxtime((uint32_t)(cmd->tx_delay_time >> 8));
            (void)dwt_writetxdata(cmd->tx_len, (uint8_t *)cmd->tx_buf, 0);
            (void)dwt_writetxfctrl(cmd->tx_len, 0);
            if (dwt_starttx(DWT_START_TX_DELAYED) != 0) {
                app_log_warn("UWB TX_DELAYED start failed");
            }
            break;

        case UWB_PHY_CMD_NONE:
        default:
            break;
    }
}

bool UwbPhy_Init(void)
{
    if (g_phy_cmd_queue == NULL) {
        g_phy_cmd_queue = xQueueCreateStatic(UWB_PHY_CMD_QUEUE_LENGTH,
                                             sizeof(UwbPhyCommand),
                                             g_phy_cmd_queue_storage,
                                             &g_phy_cmd_queue_ctrl);
    }

    if (g_phy_evt_queue == NULL) {
        g_phy_evt_queue = xQueueCreateStatic(UWB_PHY_EVT_QUEUE_LENGTH,
                                             sizeof(UwbPhyEvent),
                                             g_phy_evt_queue_storage,
                                             &g_phy_evt_queue_ctrl);
    }

    memset(g_phy_slots, 0, sizeof(g_phy_slots));

    for (uint32_t i = 0; i < UWB_STACK_PHY_SLOT_COUNT; ++i) {
        g_phy_slots[i].slot_id = (uint8_t)i;
        g_phy_slots[i].owner   = UWB_SLOT_OWNER_FREE;
        g_phy_slots[i].state   = UWB_SLOT_EMPTY;
    }

    return g_phy_cmd_queue != NULL && g_phy_evt_queue != NULL;
}

bool UwbPhy_StartThread(const osThreadAttr_t *attr)
{
    if (g_phy_task != NULL) {
        return true;
    }

    g_phy_task = osThreadNew(UwbPhy_Task, NULL, attr);
    return g_phy_task != NULL;
}

bool UwbPhy_PostCommand(const UwbPhyCommand *cmd, TickType_t timeout)
{
    if (g_phy_cmd_queue == NULL || cmd == NULL) {
        return false;
    }

    if (xQueueSend(g_phy_cmd_queue, cmd, timeout) != pdPASS) {
        return false;
    }

    if (g_phy_task != NULL) {
        (void)xTaskNotify(g_phy_task, UWB_PHY_NOTIFY_CMD, eSetBits);
    }
    return true;
}

QueueHandle_t UwbPhy_EventQueue(void)
{
    return g_phy_evt_queue;
}

UwbPhySlot *UwbPhy_GetSlot(uint8_t slot_id, uint32_t generation)
{
    if (slot_id >= UWB_STACK_PHY_SLOT_COUNT) {
        return NULL;
    }

    UwbPhySlot *slot = &g_phy_slots[slot_id];
    if (slot->owner != UWB_SLOT_OWNER_LINK ||
        slot->state != UWB_SLOT_READY ||
        slot->generation != generation ||
        !slot->valid) {
        return NULL;
    }

    return slot;
}

void UwbPhy_ReleaseSlot(uint8_t slot_id, uint32_t generation)
{
    if (slot_id >= UWB_STACK_PHY_SLOT_COUNT) {
        return;
    }

    UwbPhySlot *slot = &g_phy_slots[slot_id];
    if (slot->generation != generation) {
        return;
    }

    slot->owner  = UWB_SLOT_OWNER_FREE;
    slot->state  = UWB_SLOT_EMPTY;
    slot->valid  = false;
    slot->rx_len = 0;
}

void UwbPhy_NotifyIrqFromISR(void)
{
    if (g_phy_task == NULL) {
        return;
    }

    BaseType_t higher = pdFALSE;
    xTaskNotifyFromISR(g_phy_task, UWB_PHY_NOTIFY_IRQ, eSetBits, &higher);
    portYIELD_FROM_ISR(higher);
}

void UwbPhy_Task(void *argument)
{
    (void)argument;

    for (;;) {
        uint32_t notify = 0;
        (void)xTaskNotifyWait(0U, UINT32_MAX, &notify, portMAX_DELAY);

        if ((notify & UWB_PHY_NOTIFY_IRQ) != 0U) {
            handle_irq();
        }

        UwbPhyCommand cmd;
        while (xQueueReceive(g_phy_cmd_queue, &cmd, 0) == pdPASS) {
            execute_command(&cmd);
        }
    }
}
