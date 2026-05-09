/**
 * @file uwb_buffers.c
 * @brief UWB 层间队列实现 (精简版)
 */

#include "uwb_buffers.h"
#include <string.h>

/* ================================================================
 *  静态队列存储
 * ================================================================ */

#define CMD_QUEUE_LEN  4
#define EVT_QUEUE_LEN  8

static StaticQueue_t g_cmd_queue_ctrl;
static uint8_t g_cmd_queue_buf[CMD_QUEUE_LEN * sizeof(phy_cmd_t)];
static QueueHandle_t g_cmd_queue;

static StaticQueue_t g_evt_queue_ctrl;
static uint8_t g_evt_queue_buf[EVT_QUEUE_LEN * sizeof(phy_evt_t)];
static QueueHandle_t g_evt_queue;

/* ================================================================
 *  初始化
 * ================================================================ */

bool UwbBuffers_Init(void)
{
    if (g_cmd_queue == NULL) {
        g_cmd_queue = xQueueCreateStatic(
            CMD_QUEUE_LEN, sizeof(phy_cmd_t),
            g_cmd_queue_buf, &g_cmd_queue_ctrl);
    }

    if (g_evt_queue == NULL) {
        g_evt_queue = xQueueCreateStatic(
            EVT_QUEUE_LEN, sizeof(phy_evt_t),
            g_evt_queue_buf, &g_evt_queue_ctrl);
    }

    return g_cmd_queue != NULL && g_evt_queue != NULL;
}

/* ================================================================
 *  LINK → PHY 命令
 * ================================================================ */

bool UwbBuffers_SendCmd(const phy_cmd_t *cmd, TickType_t timeout)
{
    if (g_cmd_queue == NULL || cmd == NULL) return false;
    return xQueueSend(g_cmd_queue, cmd, timeout) == pdPASS;
}

bool UwbBuffers_RecvCmd(phy_cmd_t *cmd, TickType_t timeout)
{
    if (g_cmd_queue == NULL || cmd == NULL) return false;
    return xQueueReceive(g_cmd_queue, cmd, timeout) == pdPASS;
}

/* ================================================================
 *  PHY → LINK 事件
 * ================================================================ */

bool UwbBuffers_SendEvt(const phy_evt_t *evt, TickType_t timeout)
{
    if (g_evt_queue == NULL || evt == NULL) return false;
    return xQueueSend(g_evt_queue, evt, timeout) == pdPASS;
}

bool UwbBuffers_RecvEvt(phy_evt_t *evt, TickType_t timeout)
{
    if (g_evt_queue == NULL || evt == NULL) return false;
    return xQueueReceive(g_evt_queue, evt, timeout) == pdPASS;
}
