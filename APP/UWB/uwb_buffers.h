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
    PHY_CMD_SET_ERROR_FLAG,   /* plan-v4: 设置数据帧错误标志 */
    PHY_CMD_LOAD_PENDING,     /* plan-v4: 预载数据分片帧 (slot 转为 PHY_OWN) */
} phy_cmd_type_t;

typedef struct {
    phy_cmd_type_t type;
    int8_t   slot_index;
    uint64_t tx_time;          /* 0 = 立即, 非0 = 延时 */
    bool     has_pending_rx;
    uint16_t rx_timeout_us;
    uint8_t  rx_slot_count;
    uint16_t session_id;       /* plan-v4: LOAD_PENDING 用 */
    uint8_t  frag_id;          /* plan-v4: LOAD_PENDING 用 */
} phy_cmd_t;

/* ---- PHY → LINK 事件 ---- */

typedef enum {
    PHY_EVT_TX_DONE = 0,
    PHY_EVT_RX_FRAME,
    PHY_EVT_RX_TIMEOUT,
    PHY_EVT_RX_SLOT_DONE,     /* 单个 RX 时间槽完成 (帧 or 空) */
    PHY_EVT_RX_WINDOW_END,    /* 所有 RX 时间槽结束 */
    PHY_EVT_ERROR,
    PHY_EVT_DATA_SENT,
    PHY_EVT_DATA_RETRY,       /* plan-v4: 数据帧重试通知 */
} phy_evt_type_t;

typedef struct {
    phy_evt_type_t type;
    int8_t slot_index;         /* 物理 slot index, -1 = 无帧 */
    uint8_t rx_seq;            /* RX 时间槽序号 (0-3), 仅 RX_SLOT_DONE 有效 */
    uint8_t frag_id;           /* plan-v4: DATA_RETRY 时有效 */
    uint64_t tx_ts;            /* TX_DONE 时携带 TX 时间戳, 0=无效 */
} phy_evt_t;

/* ---- APP → LINK 命令 (plan-v4) ---- */

typedef enum {
    LINK_CMD_SEND_CFG_REQ = 0,   /* Tag: 请求发起数据会话 */
    LINK_CMD_SEND_CTRL,          /* Tag: 发送 CTRL 帧 */
    LINK_CMD_SEND_FRAG,          /* Anchor: 发送分片帧 (预载到 pending) */
    LINK_CMD_SEND_ACK,           /* Anchor: 发送快速应答确认 */
    LINK_CMD_SESSION_RESET,      /* Anchor: 重置数据会话 */
    LINK_CMD_PROX_DISCOVERY,     /* Anchor: active prox discovery window */
    LINK_CMD_RING_INIT_NOTIFY,   /* Anchor: unicast ring init token */
} UwbLinkCmdType;

typedef struct {
    UwbLinkCmdType type;
    int8_t   slot_index;       /* -1 = 无 slot */
    uint16_t target_id;        /* APP 选择的对端短地址 */
    uint16_t origin_id;
    uint16_t session_id;
    uint8_t  ctrl_type;        /* LINK_CMD_SEND_CTRL 用 */
    uint8_t  frag_id;          /* LINK_CMD_SEND_FRAG 用 */
    uint8_t  resp_type;        /* LINK_CMD_SEND_ACK 用 */
} UwbLinkCmd;

/* ---- API ---- */

bool UwbBuffers_Init(void);
bool UwbLinkCmd_Init(void);

bool UwbBuffers_SendCmd(const phy_cmd_t *cmd, TickType_t timeout);
bool UwbBuffers_RecvCmd(phy_cmd_t *cmd, TickType_t timeout);

bool UwbBuffers_SendEvt(const phy_evt_t *evt, TickType_t timeout);
bool UwbBuffers_RecvEvt(phy_evt_t *evt, TickType_t timeout);

bool UwbLinkCmd_Send(const UwbLinkCmd *cmd, TickType_t timeout);
bool UwbLinkCmd_Recv(UwbLinkCmd *cmd, TickType_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* APP_UWB_UWB_BUFFERS_H_ */
