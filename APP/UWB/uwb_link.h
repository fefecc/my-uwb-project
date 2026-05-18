#ifndef APP_UWB_UWB_LINK_H_
#define APP_UWB_UWB_LINK_H_

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "uwb_stack_types.h"
#include "uwb_buffers.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UWB_LINK_APP_EVT_NONE = 0,
    UWB_LINK_APP_EVT_TWR_EXCHANGE,
    UWB_LINK_APP_EVT_NEIGHBOR_SEEN,

    /* plan-v4 数据帧事件 (暂未启用, 预留) */
    UWB_LINK_APP_EVT_DATA_CFG,
    UWB_LINK_APP_EVT_DATA_CTRL,
    UWB_LINK_APP_EVT_DATA_FRAG,
    UWB_LINK_APP_EVT_DATA_ACK,
    UWB_LINK_APP_EVT_DATA_WAIT,
    UWB_LINK_APP_EVT_DATA_ERROR,
    UWB_LINK_APP_EVT_DATA_COMPLETE,
    UWB_LINK_APP_EVT_DATA_FAIL,
    UWB_LINK_APP_EVT_DATA_SENT,
} UwbLinkAppEventType;

typedef struct {
    UwbLinkAppEventType type;
    union {
        UwbTwrExchange twr;
        struct {
            uint16_t short_id;
            uint16_t capability;
        } neighbor;
        /* plan-v4 数据帧事件 (暂未启用, 预留) */
        UwbDataCtrlEvent  data_ctrl;
        UwbDataFragEvent  data_frag;
        struct {
            uint16_t src_id;
            uint16_t session_id;
            uint8_t  resp_type;
        } data_ack;
        struct {
            uint16_t src_id;
            uint16_t session_id;
        } data_cfg;
        struct {
            uint16_t session_id;
            uint8_t  frag_id;
        } data_sent;
    } data;
} UwbLinkAppEvent;

bool UwbLink_Init(const UwbStackConfig *cfg);
bool UwbLink_StartThread(const osThreadAttr_t *attr);
QueueHandle_t UwbLink_AppEventQueue(void);
void UwbLink_Task(void *argument);

/* plan-v4 APP→LINK 接口 (暂未启用, 预留) */
bool UwbLink_SendCmd(const UwbLinkCmd *cmd);
void UwbLink_DataSlotFreeAll(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_UWB_UWB_LINK_H_ */
