#ifndef APP_UWB_UWB_PHY_H_
#define APP_UWB_UWB_PHY_H_

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "uwb_stack_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UWB_PHY_EVT_NONE = 0,
    UWB_PHY_EVT_RX_OK,
    UWB_PHY_EVT_TX_DONE,
    UWB_PHY_EVT_RX_TIMEOUT,
    UWB_PHY_EVT_RX_ERROR,
    UWB_PHY_EVT_TX_ERROR,
} UwbPhyEventType;

typedef enum {
    UWB_PHY_CMD_NONE = 0,
    UWB_PHY_CMD_RX_ENABLE,
    UWB_PHY_CMD_TX_NOW,
    UWB_PHY_CMD_TX_DELAYED,
} UwbPhyCommandType;

typedef enum {
    UWB_SLOT_OWNER_FREE = 0,
    UWB_SLOT_OWNER_PHY,
    UWB_SLOT_OWNER_LINK,
} UwbSlotOwner;

typedef enum {
    UWB_SLOT_EMPTY = 0,
    UWB_SLOT_WRITING,
    UWB_SLOT_READY,
} UwbSlotState;

typedef struct {
    UwbPhyCommandType cmd_type;
    uint64_t tx_delay_time;
    uint16_t rx_timeout_uus;
    uint16_t rx_slot_interval;
    uint16_t rx_slot_width;
    uint16_t rx_slot_count;
    uint16_t rx_slot_base_id;
    uint8_t tx_buf[UWB_STACK_MAX_FRAME_LEN];
    uint16_t tx_len;
} UwbPhyCommand;

typedef struct {
    uint8_t slot_id;
    uint32_t generation;
    UwbSlotOwner owner;
    UwbSlotState state;
    bool valid;
    uint16_t rx_len;
    uint8_t rx_data[UWB_STACK_MAX_FRAME_LEN];
    uint64_t rx_ts;
    uint64_t tx_ts;
    TimeLocalClock irq_local_time;
    UwbRxQuality quality;
    uint32_t error_flags;
} UwbPhySlot;

typedef struct {
    UwbPhyEventType event_type;
    uint8_t slot_id;
    uint32_t generation;
    uint32_t status;
    uint64_t dw_ts;
    TimeLocalClock timestamp;
} UwbPhyEvent;

bool UwbPhy_Init(void);
bool UwbPhy_StartThread(const osThreadAttr_t *attr);
bool UwbPhy_PostCommand(const UwbPhyCommand *cmd, TickType_t timeout);
QueueHandle_t UwbPhy_EventQueue(void);
UwbPhySlot *UwbPhy_GetSlot(uint8_t slot_id, uint32_t generation);
void UwbPhy_ReleaseSlot(uint8_t slot_id, uint32_t generation);
void UwbPhy_NotifyIrqFromISR(void);
void UwbPhy_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* APP_UWB_UWB_PHY_H_ */
