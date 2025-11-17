#ifndef APP_UWB_UWB_FRAME_H_
#define APP_UWB_UWB_FRAME_H_

#include <stddef.h>
#include <stdint.h>

#include "uwb_timestamp.h"

#define UWB_FRAME_MAX_USER_PAYLOAD 48U
#define UWB_FRAME_MAC_HEADER_LEN   10U // 头文件加上帧type的长度
#define UWB_FRAME_FCS_LEN          2U

typedef enum {
    UWB_FRAME_TYPE_POLL   = 0x21,
    UWB_FRAME_TYPE_RESP   = 0x10,
    UWB_FRAME_TYPE_FINAL  = 0x23,
    UWB_FRAME_TYPE_RESULT = 0x32,
    UWB_FRAME_TYPE_USER   = 0x7F
} uwb_frame_type_t;

typedef struct {
    uint16_t pan_id;
    uint16_t short_addr;
} uwb_address_t;

typedef struct {
    uint8_t frame_control[2];
    uint8_t sequence_num;
    uint16_t pan_id;
    uint16_t dest_addr;
    uint16_t source_addr;
} uwb_frame_header_t;

typedef struct {
    uwb_frame_header_t header;
    uwb_frame_type_t type;
    union {
        struct {
            uint8_t reserved;
        } poll;
        struct {
            uwb_timestamp_t poll_rx;
            uwb_timestamp_t resp_tx;
        } resp;
        struct {
            uwb_timestamp_t poll_tx;
            uwb_timestamp_t poll_rx;
            uwb_timestamp_t resp_tx;
            uwb_timestamp_t resp_rx;
            uwb_timestamp_t final_tx;
        } final;
        struct {
            uwb_timestamp_t poll_tx;
            uwb_timestamp_t poll_rx;
            uwb_timestamp_t resp_tx;
            uwb_timestamp_t resp_rx;
            uwb_timestamp_t final_tx;
            uwb_timestamp_t final_rx;
            float anchor_pos_x;
            float anchor_pos_y;
        } result;
        struct {
            uint8_t data[UWB_FRAME_MAX_USER_PAYLOAD];
            uint8_t size;
        } user;
    } payload;
} uwb_frame_t;

void uwb_frame_init(uwb_frame_t *frame, uwb_frame_type_t type, const uwb_address_t *src,
                    const uwb_address_t *dst, uint8_t seq);
size_t uwb_frame_length(const uwb_frame_t *frame);
int uwb_frame_encode(const uwb_frame_t *frame, uint8_t *buffer, size_t buffer_len);
int uwb_frame_decode(uwb_frame_t *frame, const uint8_t *buffer, size_t buffer_len);

#endif /* APP_UWB_UWB_FRAME_H_ */
