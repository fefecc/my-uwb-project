/**
 * @file uwb_slots.h
 * @brief UWB 共享内存池 - 零拷贝 slot 设计
 *
 * 8 个固定 slot，帧数据直接在 slot 中读写，消息队列只传 slot 索引。
 * 唯一的必要拷贝是 slot ↔ DW1000 SPI。
 *
 * 所有权规则:
 *   SLOT_FREE     → 可被任意线程分配
 *   SLOT_LINK_OWN → LINK 持有（正在写入待发送帧）
 *   SLOT_PHY_OWN  → PHY 持有（正在接收 / 已完成接收）
 */

#ifndef APP_UWB_UWB_SLOTS_H_
#define APP_UWB_UWB_SLOTS_H_

#include <stdbool.h>
#include <stdint.h>
#include "uwb_stack_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_SLOT_NUM       8
#define UWB_SLOT_DATA_SIZE UWB_STACK_MAX_FRAME_LEN

typedef enum {
    UWB_SLOT_FREE = 0,
    UWB_SLOT_LINK_OWN,
    UWB_SLOT_PHY_OWN,
} uwb_slot_owner_t;

typedef struct {
    volatile uwb_slot_owner_t owner;

    /* 帧数据 (发送/接收共用) */
    uint8_t  data[UWB_SLOT_DATA_SIZE];
    uint16_t data_len;

    /* 元数据 */
    uint16_t window_id;
    uint8_t  frame_type;
    uint16_t src_short;
    uint64_t rx_ts;
    uint64_t tx_ts;

    /* 快速应答信息 */
    uint8_t  reply_type;   /* 0 = 没有快速应答 */
    uint64_t reply_tx_ts;

    /* RX 质量 */
    UwbRxQuality quality;
} uwb_slot_t;

void     UwbSlots_Init(void);
int8_t   UwbSlots_Alloc(uwb_slot_owner_t owner);
void     UwbSlots_Free(int8_t idx);
uwb_slot_t *UwbSlots_Get(int8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* APP_UWB_UWB_SLOTS_H_ */
