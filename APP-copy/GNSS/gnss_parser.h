// gnss_parser.h

#ifndef GNSS_PARSER_H
#define GNSS_PARSER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// --- GNSS解析器相关定义 ---

typedef enum {
    PARSER_STATE_WAIT_SYNC_1,
    PARSER_STATE_WAIT_SYNC_2,
    PARSER_STATE_WAIT_SYNC_3,
    PARSER_STATE_READ_MSG_ID,
    PARSER_STATE_READ_LENGTH,
    PARSER_STATE_READ_PAYLOAD,
    PARSER_STATE_READ_CRC
} parser_state_t;

typedef void (*gnss_message_handler_t)(uint16_t msg_id, const uint8_t *payload,
                                       uint16_t length);

// UM960协议常量
#define GNSS_SYNC_BYTE_1      0xAA
#define GNSS_SYNC_BYTE_2      0x44
#define GNSS_SYNC_BYTE_3      0xB5
#define GNSS_HEADER_SIZE      3
#define GNSS_MSG_ID_SIZE      2
#define GNSS_LENGTH_SIZE      2
#define GNSS_CRC_SIZE         4
#define GNSS_MAX_PAYLOAD_SIZE 1024

#define GNSS_MSG_ID_OFFSET    4
#define GNSS_LENGTH_OFFSET    6
#define GNSS_MESSAGE_OFFSET   24
#define GNSS_CRC_OFFSET       144

typedef struct {
    parser_state_t state;
    gnss_message_handler_t handler;

    uint8_t msg_buffer[GNSS_MAX_PAYLOAD_SIZE];
    uint16_t msg_id;
    uint16_t payload_length;
    uint16_t bytes_read;
} gnss_parser_t;

// 解析器函数
void gnss_parser_init(gnss_parser_t *parser, gnss_message_handler_t handler);
void gnss_parser_process_block(gnss_parser_t *parser, const uint8_t *data,
                               size_t len);

uint32_t calculate_crc32(uint8_t *szBuf, uint16_t iSize);

#endif // GNSS_PARSER_H
