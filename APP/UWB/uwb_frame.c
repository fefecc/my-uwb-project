#include "uwb_frame.h"

#include <string.h>

#define FCF_DEFAULT_BYTE0 0x41U
#define FCF_DEFAULT_BYTE1 0x88U

static size_t uwb_frame_payload_size(const uwb_frame_t *frame)
{
    if (frame == NULL) {
        return 0;
    }
    switch (frame->type) {
        case UWB_FRAME_TYPE_POLL:
            return 0;
        case UWB_FRAME_TYPE_RESP:
            return sizeof(frame->payload.resp.poll_rx.bytes) +
                   sizeof(frame->payload.resp.resp_tx.bytes);
        case UWB_FRAME_TYPE_FINAL:
            return 5 * sizeof(frame->payload.final.poll_tx.bytes);
        case UWB_FRAME_TYPE_RESULT:
            return 6 * sizeof(frame->payload.result.poll_tx.bytes) +
                   sizeof(float) * 2U;
        case UWB_FRAME_TYPE_USER:
        default:
            return frame->payload.user.size;
    }
}

static int uwb_frame_write_timestamp(uint8_t *dst, const uwb_timestamp_t *ts)
{
    if (dst == NULL || ts == NULL) {
        return -1;
    }
    memcpy(dst, ts->bytes, sizeof(ts->bytes));
    return sizeof(ts->bytes);
}

static int uwb_frame_read_timestamp(uwb_timestamp_t *ts, const uint8_t *src)
{
    if (ts == NULL || src == NULL) {
        return -1;
    }
    memcpy(ts->bytes, src, sizeof(ts->bytes));
    ts->value &= ((1ULL << 40U) - 1ULL);
    return sizeof(ts->bytes);
}

void uwb_frame_init(uwb_frame_t *frame, uwb_frame_type_t type, const uwb_address_t *src,
                    const uwb_address_t *dst, uint8_t seq)
{
    if (frame == NULL || src == NULL || dst == NULL) {
        return;
    }
    memset(frame, 0, sizeof(*frame));
    frame->header.frame_control[0] = FCF_DEFAULT_BYTE0;
    frame->header.frame_control[1] = FCF_DEFAULT_BYTE1;
    frame->header.sequence_num     = seq;
    frame->header.pan_id           = src->pan_id;
    frame->header.dest_addr        = dst->short_addr;
    frame->header.source_addr      = src->short_addr;
    frame->type                    = type;
}

size_t uwb_frame_length(const uwb_frame_t *frame)
{
    if (frame == NULL) {
        return 0;
    }
    return UWB_FRAME_MAC_HEADER_LEN + uwb_frame_payload_size(frame) + UWB_FRAME_FCS_LEN;
}

int uwb_frame_encode(const uwb_frame_t *frame, uint8_t *buffer, size_t buffer_len)
{
    if (frame == NULL || buffer == NULL) {
        return -1;
    }

    size_t need_len = uwb_frame_length(frame);
    if (buffer_len < need_len) {
        return -1;
    }

    buffer[0] = frame->header.frame_control[0];
    buffer[1] = frame->header.frame_control[1];
    buffer[2] = frame->header.sequence_num;
    buffer[3] = (uint8_t)(frame->header.pan_id & 0xFFU);
    buffer[4] = (uint8_t)((frame->header.pan_id >> 8) & 0xFFU);
    buffer[5] = (uint8_t)(frame->header.dest_addr & 0xFFU);
    buffer[6] = (uint8_t)((frame->header.dest_addr >> 8) & 0xFFU);
    buffer[7] = (uint8_t)(frame->header.source_addr & 0xFFU);
    buffer[8] = (uint8_t)((frame->header.source_addr >> 8) & 0xFFU);
    buffer[9] = (uint8_t)frame->type;

    uint8_t *payload_ptr = &buffer[10];
    switch (frame->type) {
        case UWB_FRAME_TYPE_POLL:
            break;
        case UWB_FRAME_TYPE_RESP:
            payload_ptr += uwb_frame_write_timestamp(payload_ptr, &frame->payload.resp.poll_rx);
            payload_ptr += uwb_frame_write_timestamp(payload_ptr, &frame->payload.resp.resp_tx);
            break;
        case UWB_FRAME_TYPE_FINAL:
            payload_ptr += uwb_frame_write_timestamp(payload_ptr, &frame->payload.final.poll_tx);
            payload_ptr += uwb_frame_write_timestamp(payload_ptr, &frame->payload.final.poll_rx);
            payload_ptr += uwb_frame_write_timestamp(payload_ptr, &frame->payload.final.resp_tx);
            payload_ptr += uwb_frame_write_timestamp(payload_ptr, &frame->payload.final.resp_rx);
            payload_ptr += uwb_frame_write_timestamp(payload_ptr, &frame->payload.final.final_tx);
            break;
        case UWB_FRAME_TYPE_RESULT:
            payload_ptr += uwb_frame_write_timestamp(payload_ptr, &frame->payload.result.poll_tx);
            payload_ptr += uwb_frame_write_timestamp(payload_ptr, &frame->payload.result.poll_rx);
            payload_ptr += uwb_frame_write_timestamp(payload_ptr, &frame->payload.result.resp_tx);
            payload_ptr += uwb_frame_write_timestamp(payload_ptr, &frame->payload.result.resp_rx);
            payload_ptr += uwb_frame_write_timestamp(payload_ptr, &frame->payload.result.final_tx);
            payload_ptr += uwb_frame_write_timestamp(payload_ptr, &frame->payload.result.final_rx);
            memcpy(payload_ptr, &frame->payload.result.anchor_pos_x, sizeof(float));
            payload_ptr += sizeof(float);
            memcpy(payload_ptr, &frame->payload.result.anchor_pos_y, sizeof(float));
            payload_ptr += sizeof(float);
            break;
        case UWB_FRAME_TYPE_USER:
        default:
            memcpy(payload_ptr, frame->payload.user.data, frame->payload.user.size);
            payload_ptr += frame->payload.user.size;
            break;
    }

    memset(payload_ptr, 0, UWB_FRAME_FCS_LEN);
    return (int)need_len;
}

int uwb_frame_decode(uwb_frame_t *frame, const uint8_t *buffer, size_t buffer_len)
{
    if (frame == NULL || buffer == NULL || buffer_len < UWB_FRAME_MAC_HEADER_LEN) {
        return -1;
    }

    frame->header.frame_control[0] = buffer[0];
    frame->header.frame_control[1] = buffer[1];
    frame->header.sequence_num     = buffer[2];
    frame->header.pan_id           = (uint16_t)(buffer[3] | ((uint16_t)buffer[4] << 8));
    frame->header.dest_addr        = (uint16_t)(buffer[5] | ((uint16_t)buffer[6] << 8));
    frame->header.source_addr      = (uint16_t)(buffer[7] | ((uint16_t)buffer[8] << 8));
    frame->type                    = (uwb_frame_type_t)buffer[9];

    const uint8_t *payload_ptr = &buffer[10];
    size_t payload_bytes       = buffer_len - UWB_FRAME_MAC_HEADER_LEN - UWB_FRAME_FCS_LEN;

    switch (frame->type) {
        case UWB_FRAME_TYPE_POLL:
            break;
        case UWB_FRAME_TYPE_RESP:
            payload_ptr += uwb_frame_read_timestamp(&frame->payload.resp.poll_rx, payload_ptr);
            payload_ptr += uwb_frame_read_timestamp(&frame->payload.resp.resp_tx, payload_ptr);
            break;
        case UWB_FRAME_TYPE_FINAL:
            payload_ptr += uwb_frame_read_timestamp(&frame->payload.final.poll_tx, payload_ptr);
            payload_ptr += uwb_frame_read_timestamp(&frame->payload.final.poll_rx, payload_ptr);
            payload_ptr += uwb_frame_read_timestamp(&frame->payload.final.resp_tx, payload_ptr);
            payload_ptr += uwb_frame_read_timestamp(&frame->payload.final.resp_rx, payload_ptr);
            payload_ptr += uwb_frame_read_timestamp(&frame->payload.final.final_tx, payload_ptr);
            break;
        case UWB_FRAME_TYPE_RESULT:
            payload_ptr += uwb_frame_read_timestamp(&frame->payload.result.poll_tx, payload_ptr);
            payload_ptr += uwb_frame_read_timestamp(&frame->payload.result.poll_rx, payload_ptr);
            payload_ptr += uwb_frame_read_timestamp(&frame->payload.result.resp_tx, payload_ptr);
            payload_ptr += uwb_frame_read_timestamp(&frame->payload.result.resp_rx, payload_ptr);
            payload_ptr += uwb_frame_read_timestamp(&frame->payload.result.final_tx, payload_ptr);
            payload_ptr += uwb_frame_read_timestamp(&frame->payload.result.final_rx, payload_ptr);
            memcpy(&frame->payload.result.anchor_pos_x, payload_ptr, sizeof(float));
            payload_ptr += sizeof(float);
            memcpy(&frame->payload.result.anchor_pos_y, payload_ptr, sizeof(float));
            payload_ptr += sizeof(float);
            break;
        case UWB_FRAME_TYPE_USER:
        default:
            frame->payload.user.size = (payload_bytes > UWB_FRAME_MAX_USER_PAYLOAD)
                                           ? UWB_FRAME_MAX_USER_PAYLOAD
                                           : (uint8_t)payload_bytes;
            memcpy(frame->payload.user.data, payload_ptr, frame->payload.user.size);
            break;
    }
    return 0;
}
