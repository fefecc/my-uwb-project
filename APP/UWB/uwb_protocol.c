#include "uwb_protocol.h"

#include <string.h>

void UwbProtocol_WriteLe16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

void UwbProtocol_WriteLe32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFUL);
    dst[1] = (uint8_t)((value >> 8) & 0xFFUL);
    dst[2] = (uint8_t)((value >> 16) & 0xFFUL);
    dst[3] = (uint8_t)((value >> 24) & 0xFFUL);
}

void UwbProtocol_WriteLe64(uint8_t *dst, uint64_t value)
{
    for (uint32_t i = 0; i < 8U; ++i) {
        dst[i] = (uint8_t)((value >> (8U * i)) & 0xFFULL);
    }
}

uint16_t UwbProtocol_ReadLe16(const uint8_t *src)
{
    return (uint16_t)(src[0] | ((uint16_t)src[1] << 8));
}

uint32_t UwbProtocol_ReadLe32(const uint8_t *src)
{
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

uint64_t UwbProtocol_ReadLe64(const uint8_t *src)
{
    uint64_t value = 0;
    for (uint32_t i = 0; i < 8U; ++i) {
        value |= ((uint64_t)src[i]) << (8U * i);
    }
    return value;
}

void UwbProtocol_InitFrame(UwbProtocolFrame *frame,
                           const UwbStackConfig *cfg,
                           uint16_t dst16,
                           uint8_t seq,
                           UwbFuncCode func_code)
{
    if (frame == NULL || cfg == NULL) {
        return;
    }

    memset(frame, 0, sizeof(*frame));
    frame->mac.frame_ctrl[0] = cfg->frame_ctrl[0];
    frame->mac.frame_ctrl[1] = cfg->frame_ctrl[1];
    frame->mac.seq           = seq;
    frame->mac.pan_id        = cfg->pan_id;
    frame->mac.dst16         = dst16;
    frame->mac.src16         = cfg->short_addr;
    frame->common.proto_ver  = UWB_PROTO_VERSION;
    frame->common.func_code  = (uint8_t)func_code;
}

bool UwbProtocol_Encode(const UwbProtocolFrame *frame,
                        uint8_t *out,
                        size_t out_size,
                        size_t *out_len)
{
    if (frame == NULL || out == NULL || out_len == NULL) {
        return false;
    }

    if (frame->common.ext_header_len > UWB_PROTO_MAX_EXT_LEN ||
        frame->common.payload_len > UWB_PROTO_MAX_PAYLOAD_LEN) {
        return false;
    }

    size_t need = UWB_PROTO_MAC_HEADER_LEN + UWB_PROTO_COMMON_HDR_LEN +
                  frame->common.ext_header_len + frame->common.payload_len;
    if (need > out_size || need > UWB_STACK_MAX_FRAME_LEN) {
        return false;
    }

    out[0] = frame->mac.frame_ctrl[0];
    out[1] = frame->mac.frame_ctrl[1];
    out[2] = frame->mac.seq;
    UwbProtocol_WriteLe16(&out[3], frame->mac.pan_id);
    UwbProtocol_WriteLe16(&out[5], frame->mac.dst16);
    UwbProtocol_WriteLe16(&out[7], frame->mac.src16);

    out[9]  = frame->common.proto_ver;
    out[10] = frame->common.func_code;
    out[11] = frame->common.flags;
    out[12] = frame->common.ext_header_len;
    UwbProtocol_WriteLe16(&out[13], frame->common.payload_len);

    size_t pos = UWB_PROTO_MAC_HEADER_LEN + UWB_PROTO_COMMON_HDR_LEN;
    memcpy(&out[pos], frame->ext_header, frame->common.ext_header_len);
    pos += frame->common.ext_header_len;
    memcpy(&out[pos], frame->payload, frame->common.payload_len);
    *out_len = need;
    return true;
}

bool UwbProtocol_Decode(UwbProtocolFrame *frame,
                        const uint8_t *data,
                        size_t len)
{
    if (frame == NULL || data == NULL ||
        len < (UWB_PROTO_MAC_HEADER_LEN + UWB_PROTO_COMMON_HDR_LEN)) {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->mac.frame_ctrl[0] = data[0];
    frame->mac.frame_ctrl[1] = data[1];
    frame->mac.seq           = data[2];
    frame->mac.pan_id        = UwbProtocol_ReadLe16(&data[3]);
    frame->mac.dst16         = UwbProtocol_ReadLe16(&data[5]);
    frame->mac.src16         = UwbProtocol_ReadLe16(&data[7]);

    frame->common.proto_ver       = data[9];
    frame->common.func_code       = data[10];
    frame->common.flags           = data[11];
    frame->common.ext_header_len  = data[12];
    frame->common.payload_len     = UwbProtocol_ReadLe16(&data[13]);

    if (frame->common.proto_ver != UWB_PROTO_VERSION ||
        frame->common.ext_header_len > UWB_PROTO_MAX_EXT_LEN ||
        frame->common.payload_len > UWB_PROTO_MAX_PAYLOAD_LEN) {
        return false;
    }

    size_t need = UWB_PROTO_MAC_HEADER_LEN + UWB_PROTO_COMMON_HDR_LEN +
                  frame->common.ext_header_len + frame->common.payload_len;
    if (need > len) {
        return false;
    }

    size_t pos = UWB_PROTO_MAC_HEADER_LEN + UWB_PROTO_COMMON_HDR_LEN;
    memcpy(frame->ext_header, &data[pos], frame->common.ext_header_len);
    pos += frame->common.ext_header_len;
    memcpy(frame->payload, &data[pos], frame->common.payload_len);
    return true;
}
