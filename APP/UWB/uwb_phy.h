/**
 * @file uwb_phy.h
 * @brief PHY 层 - DW1000 硬件直接管理 (V3 重构)
 *
 * 状态机: IDLE / TX / RX_SLOT × PREPARE / WAIT / FINISH
 * LISTEN 不是状态机的一部分，是 enter_listening() 函数调用。
 *
 * 快速应答: IRQ 中判帧类型 → 填 tx_buf/tx_time → 跳 TX/PREPARE
 */

#ifndef APP_UWB_UWB_PHY_H_
#define APP_UWB_UWB_PHY_H_

#include <stdbool.h>
#include <stdint.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uwb_stack_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  超时参数
 * ================================================================ */

#define UWB_PHY_WATCHDOG_MS        50U   /* 活跃状态最后防护, 必须 > RX_SLOT_TIMEOUT (10ms) */
#define UWB_PHY_IDLE_GUARD_MS      5000U /* IDLE 监听态保护超时 (仅防止硬件彻底卡死) */
#define UWB_PHY_RX_SLOT_TIMEOUT_US 2000U /* 2ms, P99(1ms)+4σ 余量 (实测 max=1ms) */
#define UWB_PHY_TX_DELAYED_MIN_US  500U  /* 延时发送最小提前量 */
#define UWB_PHY_HW_ERROR_THRESHOLD 5U    /* 连续硬件错误上限 */

/* ---- 多槽接收 / Anchor 延迟回复参数 ---- */
#define DISC_SLOT_WIDTH_US         2000U /* 每槽 2ms */
#define DISC_RX_SLOT_COUNT         4U    /* RX 槽数量 */
#define DISC_FIRST_SLOT_TIMEOUT_US 4000U /* 第一槽 4ms (GUARD + TX 上电时间) */
#define ANCHOR_ADDR_BASE           0x30U /* Anchor 地址起始 */
#define ANCHOR_REPLY_GUARD_US      3000U /* Anchor 回复保护延迟 (3ms: 处理500us+TX上电2ms+余量500us) */

/* ================================================================
 *  状态机枚举
 * ================================================================ */

typedef enum {
    UWB_PHY_ST_IDLE = 0,
    UWB_PHY_ST_TX,
    UWB_PHY_ST_RX_SLOT,
} uwb_phy_state_t;

typedef enum {
    UWB_PHY_STEP_PREPARE = 0,
    UWB_PHY_STEP_WAIT,
    UWB_PHY_STEP_FINISH,
} uwb_phy_step_t;

/* ================================================================
 *  API
 * ================================================================ */

/** 初始化 PHY 上下文, 不启动线程 */
bool UwbPhy_Init(uint16_t pan_id, uint16_t short_addr);

/** 初始化 PHY (含角色和协议栈配置, 用于 Anchor 快速应答) */
bool UwbPhy_InitWithConfig(uint16_t pan_id, uint16_t short_addr,
                           AppDeviceRole role,
                           const UwbStackConfig *stack_cfg);

/** 启动 PHY 线程 */
bool UwbPhy_StartThread(const osThreadAttr_t *attr);

/** IRQ → 通知 PHY 线程 */
void UwbPhy_NotifyIrqFromISR(void);

/** LINK → PHY: 通知有命令待处理 */
void UwbPhy_NotifyCmd(void);

/** PHY 线程入口 */
void UwbPhy_Task(void *argument);

/* ================================================================
 *  工具函数 (PHY 内部 + 其他层可调用)
 * ================================================================ */

uint64_t UwbPhy_UsToDwTime(uint32_t us);
uint64_t UwbPhy_ReadRxTimestamp(void);
uint64_t UwbPhy_ReadTxTimestamp(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_UWB_UWB_PHY_H_ */
