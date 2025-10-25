#ifndef _DW1000SAMPLING_H_
#define _DW1000SAMPLING_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "bphero_uwb.h"

// 定义邮箱结构体
// 这个结构用来记录时间戳
typedef struct {
    uint8_t rxtimestamp[5];
    uint8_t txtimestamp[5];

} isr_timestamp_packet_t;

// 来自中断的事件通知 (使用位掩码)
typedef enum {
    UWB_EVENT_NONE           = 0,
    UWB_EVENT_TX_DONE        = (1 << 0), // 发送完成 0x01
    UWB_EVENT_RX_DONE        = (1 << 1), // 接收成功 0x02
    UWB_EVENT_RX_TIMEOUT     = (1 << 2), // 接收超时 0x04
    UWB_EVENT_RX_ERROR       = (1 << 3), // 接收出错 0x08
    UWB_EVENT_PRE_DONE       = (1 << 4), // 前导码超时 0x10
    UWB_EVENT_SFD_DONE       = (1 << 5), // SFD超时  0x20
    UWB_EVENT_FRAME_REJECTED = (1 << 6)  // 自动帧过滤拒绝事件 ,一个硬件错规则 0x40
} UWB_Event_t;

// 这些数据是时间戳的40位数据
typedef struct {

    uint16_t short_addr;

    /** @brief 本设备发送 Poll 帧的 40 位发送时间戳。 (由 Tag 记录) */
    uint8_t poll_tx_ts[5];

    /** @brief 本设备接收 Poll 帧的 40 位接收时间戳。 (由 Anchor 记录) */
    uint8_t poll_rx_ts[5];

    /** @brief 本设备发送 Response 帧的 40 位发送时间戳。(由 Anchor 记录/计算) */
    uint8_t resp_tx_ts[5];

    /** @brief 本设备接收 Response 帧的 40 位接收时间戳。(由 Tag 记录) */
    uint8_t resp_rx_ts[5];

    /** @brief 本设备发送 Final 帧的 40 位发送时间戳。 (由 Tag 记录) */
    uint8_t final_tx_ts[5];

    /** @brief 本设备接收 Final 帧的 40 位接收时间戳。 (由 Anchor 记录) */
    uint8_t final_rx_ts[5];

} uwb_node_profile_t;

typedef struct {

    /**
     * @brief 节点的 16 位短地址。
     */
    uint16_t short_addr;

    /** @brief 本设备发送 Poll 帧的发送时间戳。 (由 Tag 记录) */
    uint64_t poll_tx_ts;

    /** @brief 本设备接收 Poll 帧的接收时间戳。 (由 Anchor 记录) */
    uint64_t poll_rx_ts;

    /** @brief 本设备发送 Response 帧的发送时间戳。(由 Anchor 记录/计算) */
    uint64_t resp_tx_ts;

    /** @brief 本设备接收 Response 帧的接收时间戳。(由 Tag 记录) */
    uint64_t resp_rx_ts;

    /** @brief 本设备发送 Final 帧的发送时间戳。 (由 Tag 记录) */
    uint64_t final_tx_ts;

    /** @brief 本设备接收 Final 帧的接收时间戳。 (由 Anchor 记录) */
    uint64_t final_rx_ts;

} uwb_node_profile_u64_t;

// 本机的数据
typedef struct {
    uint8 frameCtrl[2]; //  frame control bytes 00-01
    uint8 seqNum;       //  sequence_number 02
    uint16_t pan_id;
    uint16_t short_addr;
} dw1000_local_device_t;

extern TaskHandle_t dw1000samplingTaskNotifyHandle;

extern QueueHandle_t dw1000data_queue;

void DW1000samplingtask(void *argument);
int16_t clear_node_profile(uwb_node_profile_t *node_profile);
double calculate_distance_from_timestamps(uint64_t tag_poll_tx_ts,
                                          uint64_t tag_resp_rx_ts,
                                          uint64_t tag_final_tx_ts,
                                          uint64_t anchor_poll_rx_ts,
                                          uint64_t anchor_resp_tx_ts,
                                          uint64_t anchor_final_rx_ts);
void UWBMssageInit(void);

#endif