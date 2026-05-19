#ifndef APP_UWB_UWB_STACK_TYPES_H_
#define APP_UWB_UWB_STACK_TYPES_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../common/app_types.h"
#include "../service/time_service.h"
#include "uwb_timestamp.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_STACK_MAX_FRAME_LEN      (127U)
#define UWB_STACK_PHY_SLOT_COUNT     (4U)
#define UWB_STACK_APP_SLOT_COUNT     (2U)
#define UWB_STACK_BROADCAST_SHORT_ID (0xFFFFU)

typedef struct {
    uint16_t pan_id;
    uint16_t short_addr;
    AppDeviceRole role;
    uint8_t frame_ctrl[2];
} UwbStackConfig;

typedef struct {
    uint16_t rx_pacc;
    uint16_t fp_index;
    uint16_t fp_ampl1;
    uint16_t fp_ampl2;
    uint16_t fp_ampl3;
    int16_t rx_power_dbm_x100;
    int16_t fp_power_dbm_x100;
    uint16_t std_noise;
    uint16_t max_noise;
    uint16_t lde_status;
    uint32_t rx_error_flags;
} UwbRxQuality;

typedef struct {
    uint16_t anchor_id;
    uint16_t exchange_seq;
    uint16_t response_slot_id;
    uint16_t status_flags;
    uint64_t tag_tx_ts;
    uint64_t anchor_rx_ts;
    uint64_t anchor_tx_ts;
    uint64_t tag_rx_ts;
    uint64_t tag_tx_local_tick_20k;
    uint64_t tag_rx_local_tick_20k;
    UwbRxQuality quality;
    uint8_t retry_count;
} UwbTwrExchange;

typedef struct {
    uint16_t anchor_id;
    uint16_t tag_id;
    uint16_t exchange_seq;
    uint16_t response_slot_id;
    uint16_t status_flags;
    double distance_m;
    UwbRxQuality quality;
    uint8_t retry_count;
    uint64_t tag_tx_ts;
    uint64_t anchor_rx_ts;
    uint64_t anchor_tx_ts;
    uint64_t tag_rx_ts;
    uint64_t frame_local_tick_20k;
} UwbRangeResult;

/* ====== plan-v4 数据交互帧类型 ====== */

/* DATA_CTRL 帧事件 (Anchor APP 收到) */
typedef struct {
    uint16_t src_id;
    uint16_t session_id;
    uint8_t  ctrl_type;    /* GET_INFO / PULL / DONE / STOP */
    uint8_t  frag_id;      /* PULL 时有效 */
} UwbDataCtrlEvent;

/* DATA_FRAG 帧事件 (Tag APP 收到) */
typedef struct {
    uint16_t src_id;
    uint16_t session_id;
    uint8_t  frag_id;
    uint8_t  total_frags;
    uint8_t  flags;        /* bit0: meta */
    int8_t   slot_index;   /* slot 持有帧数据 */
    uint16_t payload_len;
} UwbDataFragEvent;

/* MAC 表条目 (Tag 侧维护) */
typedef struct {
    bool     valid;
    uint16_t anchor_id;
    uint32_t last_seen_ms;
    uint32_t last_poll_ms;
} mac_entry_t;

#define MAC_TABLE_SIZE       8
#define MAC_ENTRY_TIMEOUT_MS (3U * 60U * 1000U)

#ifdef __cplusplus
}
#endif

#endif /* APP_UWB_UWB_STACK_TYPES_H_ */
