/**
 * @file uwb_buffers.h
 * @brief UWB 层间队列 (精简版)
 *
 * 队列只传极小的结构体（slot 索引 + 类型），帧数据在共享 slot 池中。
 *
 * 通道:
 *   LINK → PHY : cmd_queue (phy_cmd_t, depth=4)
 *   PHY → LINK : evt_queue (phy_evt_t, depth=8)
 */

#ifndef APP_UWB_UWB_BUFFERS_H_
#define APP_UWB_UWB_BUFFERS_H_

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- LINK → PHY 命令 ---- */

typedef enum {
    PHY_CMD_TX_FRAME = 0,
    PHY_CMD_RESET,
    PHY_CMD_ENTER_LISTEN,
} phy_cmd_type_t;

typedef struct {
    phy_cmd_type_t type;
    int8_t   slot_index;
    uint64_t tx_time;          /* 0 = 立即, 非0 = 延时 */
    bool     has_pending_rx;
    uint16_t rx_timeout_us;
    uint8_t  rx_slot_count;
} phy_cmd_t;

/* ---- PHY → LINK 事件 ---- */

typedef enum {
    PHY_EVT_TX_DONE = 0,
    PHY_EVT_RX_FRAME,
    PHY_EVT_RX_TIMEOUT,
    PHY_EVT_RX_SLOT_DONE,     /* 单个 RX 时间槽完成 (帧 or 空) */
    PHY_EVT_RX_WINDOW_END,    /* 所有 RX 时间槽结束 */
    PHY_EVT_ERROR,
} phy_evt_type_t;

typedef struct {
    phy_evt_type_t type;
    int8_t slot_index;         /* 物理 slot index, -1 = 无帧 */
    uint8_t rx_seq;            /* RX 时间槽序号 (0-3), 仅 RX_SLOT_DONE 有效 */
    uint64_t tx_ts;            /* TX_DONE 时携带 TX 时间戳, 0=无效 */
} phy_evt_t;

/* ---- API ---- */

bool UwbBuffers_Init(void);

bool UwbBuffers_SendCmd(const phy_cmd_t *cmd, TickType_t timeout);
bool UwbBuffers_RecvCmd(phy_cmd_t *cmd, TickType_t timeout);

bool UwbBuffers_SendEvt(const phy_evt_t *evt, TickType_t timeout);
bool UwbBuffers_RecvEvt(phy_evt_t *evt, TickType_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* APP_UWB_UWB_BUFFERS_H_ */
