#ifndef _DW1000SAMPLING_H_
#define _DW1000SAMPLING_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "bphero_uwb.h"
#include "uwb_timestamp.h"
#include "timestamp.h"

// 定义邮箱结构体
// 这个结构用来记录时间戳
typedef struct {
    uwb_timestamp_t rx;
    uwb_timestamp_t tx;
} isr_timestamp_packet_t;

// 来自中断的事件通知 (使用位掩码)
typedef enum {
    UWB_EVENT_NONE     = 0,
    UWB_EVENT_TX_DONE  = (1 << 0), // 发送完成 0x01
    UWB_EVENT_RX_DONE  = (1 << 1), // 接收成功 0x02
    UWB_EVENT_RX_ERROR = (1 << 2), // 接收出错 0x08

} UWB_Event_t;

// 这些数据是时间戳的40位数据
typedef struct {

    uint16_t short_addr;

    /** @brief 本设备发送 Poll 帧的 40 位发送时间戳。 (由 Tag 记录) */
    uwb_timestamp_t poll_tx_ts;

    /** @brief 本设备接收 Poll 帧的 40 位接收时间戳。 (由 Anchor 记录) */
    uwb_timestamp_t poll_rx_ts;

    /** @brief 本设备发送 Response 帧的 40 位发送时间戳。(由 Anchor 记录/计算) */
    uwb_timestamp_t resp_tx_ts;

    /** @brief 本设备接收 Response 帧的 40 位接收时间戳。(由 Tag 记录) */
    uwb_timestamp_t resp_rx_ts;

    /** @brief 本设备发送 Final 帧的 40 位发送时间戳。 (由 Tag 记录) */
    uwb_timestamp_t final_tx_ts;

    /** @brief 本设备接收 Final 帧的 40 位接收时间戳。 (由 Anchor 记录) */
    uwb_timestamp_t final_rx_ts;

    float anchor_pos_x;
    float anchor_pos_y;

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

typedef struct {

    utc_global_timestamp_t utc_timestamp; // RESULT 中的 UTC 时间戳

    uint16_t anchorId; // 基站 ID (2字节)
    uint16_t tagId;    // 标签 ID (2字节)

    double distance_m; // RESULT 中计算得到的距离，单位：米

    double fpp_dbm; // 首径功率 (8字节)
    double rxp_dbm; // 接收总功率 (8字节)
    double diff_db; // 功率差 (用于判断 NLOS) (8字节)

    uint8_t accum_data[25]; // 累加器数据片段

} uwb_result_queue_item_t;

typedef struct {
    double fpp_dbm; // 首径功率 (First Path Power)
    double rxp_dbm; // 接收总功率 (RX Power)
    double diff_db; // 功率差 (用于判断 NLOS)
} SignalStats_t;

extern TaskHandle_t dw1000samplingTaskNotifyHandle;

extern QueueHandle_t xDW1000DataQueue;

void DW1000samplingtask(void *argument);
int16_t clear_node_profile(uwb_node_profile_t *node_profile);
double calculate_distance_from_timestamps_v2(
    uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t resp_tx_ts,
    uint64_t resp_rx_ts, uint64_t final_tx_ts, uint64_t final_rx_ts);
void UWBMssageInit(void);

#endif
