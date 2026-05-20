#ifndef APP_UWB_UWB_PROTOCOL_H_
#define APP_UWB_UWB_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "uwb_stack_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_PROTO_VERSION         (1U)
#define UWB_PROTO_MAC_HEADER_LEN  (9U)
#define UWB_PROTO_COMMON_HDR_LEN  (6U)
#define UWB_PROTO_MAX_EXT_LEN     (48U)
#define UWB_PROTO_MAX_PAYLOAD_LEN (64U)

typedef enum {
    UWB_FUNC_DISCOVERY_REQ  = 0x20,
    UWB_FUNC_DISCOVERY_RESP = 0x21,
    UWB_FUNC_TWR_TAG_START  = 0x40,
    UWB_FUNC_TWR_ANCHOR_RESP = 0x41,
    UWB_FUNC_APP_DATA_CFG   = 0x70,
    UWB_FUNC_APP_DATA_FRAG  = 0x71,
    UWB_FUNC_APP_DATA_CTRL  = 0x72,
    UWB_FUNC_APP_DATA_CTRL_RESP = 0x73,
    UWB_FUNC_RING_INIT_NOTIFY = 0x80,
    UWB_FUNC_RING_INIT_ACK    = 0x81,
} UwbFuncCode;

/* ====== plan-v4 数据交互帧常量 ====== */

/* DATA_CTRL 子类型 (ext_header 中 ctrl_type 字段) */
#define UWB_DATA_CTRL_GET_INFO  0x01
#define UWB_DATA_CTRL_PULL      0x02
#define UWB_DATA_CTRL_DONE      0x03
#define UWB_DATA_CTRL_STOP      0x05

/* DATA_CTRL_RESP 应答类型 */
#define UWB_DATA_RESP_ACK   0x00
#define UWB_DATA_RESP_WAIT  0x01
#define UWB_DATA_RESP_STOP  0x02
#define UWB_DATA_RESP_ERROR 0x03

/* DATA_FRAG flags 位掩码 */
#define UWB_DATA_FRAG_FLAG_META  0x01

/* 分片参数 */
#define UWB_DATA_FRAG_PAYLOAD_SIZE  62
#define UWB_DATA_MAX_FRAGS          9

typedef struct {
    uint8_t frame_ctrl[2];
    uint8_t seq;
    uint16_t pan_id;
    uint16_t dst16;
    uint16_t src16;
} UwbMacHeader;

typedef struct {
    uint8_t proto_ver;
    uint8_t func_code;
    uint8_t flags;
    uint8_t ext_header_len;
    uint16_t payload_len;
} UwbCommonHeader;

typedef struct {
    UwbMacHeader mac;
    UwbCommonHeader common;
    uint8_t ext_header[UWB_PROTO_MAX_EXT_LEN];
    uint8_t payload[UWB_PROTO_MAX_PAYLOAD_LEN];
} UwbProtocolFrame;

void UwbProtocol_InitFrame(UwbProtocolFrame *frame,
                           const UwbStackConfig *cfg,
                           uint16_t dst16,
                           uint8_t seq,
                           UwbFuncCode func_code);
bool UwbProtocol_Encode(const UwbProtocolFrame *frame,
                        uint8_t *out,
                        size_t out_size,
                        size_t *out_len);
bool UwbProtocol_Decode(UwbProtocolFrame *frame,
                        const uint8_t *data,
                        size_t len);

void UwbProtocol_WriteLe16(uint8_t *dst, uint16_t value);
void UwbProtocol_WriteLe32(uint8_t *dst, uint32_t value);
void UwbProtocol_WriteLe64(uint8_t *dst, uint64_t value);
uint16_t UwbProtocol_ReadLe16(const uint8_t *src);
uint32_t UwbProtocol_ReadLe32(const uint8_t *src);
uint64_t UwbProtocol_ReadLe64(const uint8_t *src);

#ifdef __cplusplus
}
#endif

#endif /* APP_UWB_UWB_PROTOCOL_H_ */
