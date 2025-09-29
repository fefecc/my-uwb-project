// gnss_parser.h

#ifndef GNSS_PARSER_H
#define GNSS_PARSER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// --- 通用循环缓冲区结构体 ---
typedef struct {
  uint8_t *buffer;  // 指向实际存储数据的内存区域
  size_t size;      // 缓冲区的总大小
  size_t head;      // 写指针（下一个要写入数据的位置）
  size_t tail;      // 读指针（下一个要读取数据的位置）
} ring_buffer_t;

// --- GNSS解析器相关定义 ---

// 解析器的状态枚举
typedef enum {
  PARSER_STATE_WAIT_SYNC_1,  // 等待同步头 0xAA
  PARSER_STATE_WAIT_SYNC_2,  // 等待同步头 0x44
  PARSER_STATE_WAIT_SYNC_3,  // 等待同步头 0x12
  PARSER_STATE_READ_MSG_ID,
  PARSER_STATE_READ_LENGTH,
  PARSER_STATE_READ_PAYLOAD,
  PARSER_STATE_READ_CRC
} parser_state_t;

// 定义一个消息处理回调函数指针类型
// 当解析器成功解析出一条完整的、校验正确的消息后，会调用此函数
// 参数：消息ID, 载荷数据指针, 载荷长度
typedef void (*gnss_message_handler_t)(uint16_t msg_id, const uint8_t *payload,
                                       uint16_t length);

// UM960协议常量
#define GNSS_SYNC_BYTE_1 0xAA
#define GNSS_SYNC_BYTE_2 0x44
#define GNSS_SYNC_BYTE_3 0xB5
#define GNSS_HEADER_SIZE 3
#define GNSS_MSG_ID_SIZE 2
#define GNSS_LENGTH_SIZE 2
#define GNSS_CRC_SIZE 4
#define GNSS_MAX_PAYLOAD_SIZE 1024  // 根据需要设置，1024通常足够

#define GNSS_MSG_ID_OFFSET 4
#define GNSS_LENGTH_OFFSET 6
#define GNSS_MESSAGE_OFFSET 24
#define GNSS_CRC_OFFSET 144

// GNSS解析器结构体
typedef struct {
  ring_buffer_t *rb;               // 指向要操作的循环缓冲区
  parser_state_t state;            // 解析器当前的状态
  gnss_message_handler_t handler;  // 成功解析后的回调函数

  // 用于临时存储正在解析的消息
  uint8_t msg_buffer[GNSS_MAX_PAYLOAD_SIZE];
  uint16_t msg_id;
  uint16_t payload_length;
  uint16_t bytes_read;  // 当前已读取的字节数
} gnss_parser_t;

// --- 函数原型声明 ---

// 循环缓冲区函数
void rb_init(ring_buffer_t *rb, uint8_t *buffer, size_t size);
bool rb_write_byte(ring_buffer_t *rb, uint8_t byte);
bool rb_read_byte(ring_buffer_t *rb, uint8_t *byte);

// 解析器函数
void gnss_parser_init(gnss_parser_t *parser, ring_buffer_t *rb,
                      gnss_message_handler_t handler);
void gnss_parser_process(gnss_parser_t *parser);

uint32_t calculate_crc32(uint8_t *szBuf, uint16_t iSize);

#endif  // GNSS_PARSER_H