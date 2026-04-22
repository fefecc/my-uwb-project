#ifndef APP_UWB_UWB_LINK_H_
#define APP_UWB_UWB_LINK_H_

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "uwb_stack_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UWB_LINK_STATE_IDLE = 0,
    UWB_LINK_STATE_RX_ON,
    UWB_LINK_STATE_RX_PROCESS,
    UWB_LINK_STATE_TX_PREPARE,
    UWB_LINK_STATE_TX_WAIT_DONE,
    UWB_LINK_STATE_RECOVER,
} UwbLinkState;

typedef enum {
    UWB_LINK_APP_EVT_NONE = 0,
    UWB_LINK_APP_EVT_TWR_EXCHANGE,
    UWB_LINK_APP_EVT_NEIGHBOR_SEEN,
} UwbLinkAppEventType;

typedef struct {
    UwbLinkAppEventType type;
    union {
        UwbTwrExchange twr;
        struct {
            uint16_t short_id;
            uint16_t capability;
        } neighbor;
    } data;
} UwbLinkAppEvent;

bool UwbLink_Init(const UwbStackConfig *cfg);
bool UwbLink_StartThread(const osThreadAttr_t *attr);
QueueHandle_t UwbLink_AppEventQueue(void);
void UwbLink_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* APP_UWB_UWB_LINK_H_ */
