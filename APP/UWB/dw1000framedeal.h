#ifndef _DW1000FRAMEDEAL_H_
#define _DW1000FRAMEDEAL_H_

#include "main.h"
#include "dw1000frame.h"

/**
 * @brief 生成一个 TWR Poll 帧。
 * @param buffer           [输出] 指向用于存放生成帧的缓冲区的指针。
 * @param buffer_size      [输入] 缓冲区的总大小，用于防止溢出。
 * @param seq_num          [输入] 当前的帧序列号。
 * @param pan_id           [输入] 目标网络的 PAN ID。
 * @param dest_addr        [输入] 目标设备（Anchor）的 16 位短地址。
 * @param source_addr      [输入] 源设备（Tag）的 16 位短地址。
 * @return size_t           成功则返回生成的帧的长度（字节），失败则返回 0。
 */
int16_t create_poll_frame(uint8_t *buffer, size_t buffer_size,
                          uint8_t seq_num, uint16_t pan_id,
                          uint16_t dest_addr, uint16_t source_addr);

/**
 * @brief 解析一个 TWR Poll 帧。
 * @param buffer           [输入] 指向包含接收到的原始帧数据的缓冲区的指针。
 * @param length           [输入] 接收到的数据的长度。
 * @param parsed_msg       [输出] 指向一个 twr_poll_msg_t 结构体的指针，用于存放解析结果。
 * @return bool             成功解析并验证为 Poll 帧则返回 true，否则返回 false。
 */
int16_t parse_poll_frame(const uint8_t *buffer, uint16_t length,
                         twr_poll_msg_t *parsed_msg);

/**
 * @param buffer 输出缓冲区，用于存储生成的响应消息帧
 * @param buffer_size 缓冲区大小，确保足够存储整个帧
 * @param seq_num 序列号，用于匹配请求-响应序列，应与Poll消息保持一致
 * @param pan_id 个域网标识符(PAN ID)
 * @param dest_addr 目标设备地址（发起方地址）
 * @param source_addr 源设备地址（响应方地址）
 * @param poll_tx_ts 发起方发送Poll消息的时间戳（5字节，40位时间戳）
 * @param poll_rx_ts 响应方接收Poll消息的时间戳（5字节，40位时间戳）
 * @param resp_tx_ts 响应方发送Response消息的时间戳（5字节，40位时间戳）
 */
int16_t create_resp_frame(uint8_t *buffer, size_t buffer_size, uint8_t seq_num,
                          uint16_t pan_id, uint16_t dest_addr,
                          uint16_t source_addr, const uint8_t *poll_rx_ts,
                          const uint8_t *resp_tx_ts);

/**
 * @brief 解析一个 TWR Response 帧。
 * @param buffer           [输入] 指向包含接收到的原始帧数据的缓冲区的指针。
 * @param length           [输入] 接收到的数据的长度。
 * @param parsed_msg       [输出] 指向一个 twr_resp_msg_t 结构体的指针，用于存放解析结果。
 * @return int16_t          成功解析则返回 0，错误则返回 -1。
 */
int16_t parse_resp_frame(const uint8_t *buffer, uint16_t length,
                         twr_resp_msg_t *parsed_msg);

/**
 * @param buffer 输出缓冲区，用于存储生成的最终消息帧
 * @param buffer_size 缓冲区大小，确保足够存储整个帧
 * @param seq_num 序列号，用于匹配请求-响应序列
 * @param pan_id 个域网标识符(PAN ID)
 * @param dest_addr 目标设备地址（响应方地址）
 * @param source_addr 源设备地址（发起方地址）
 * @param poll_tx_ts 发起方发送Poll消息的时间戳（5字节，40位时间戳）
 * @param poll_rx_ts 响应方接收Poll消息的时间戳（5字节，40位时间戳）
 * @param resp_tx_ts 响应方发送Response消息的时间戳（5字节，40位时间戳）
 * @param resp_rx_ts 发起方接收Response消息的时间戳（5字节，40位时间戳）
 * @param final_tx_ts 发起方发送Final消息的时间戳（5字节，40位时间戳）
 *
 * @return 成功返回创建的帧长度(字节数)，失败返回-1
 */
int16_t create_final_frame(uint8_t *buffer, size_t buffer_size, uint8_t seq_num,
                           uint16_t pan_id, uint16_t dest_addr,
                           uint16_t source_addr, const uint8_t *poll_tx_ts,
                           const uint8_t *poll_rx_ts, const uint8_t *resp_tx_ts,
                           const uint8_t *resp_rx_ts,
                           const uint8_t *final_tx_ts);

/**
 * @brief 解析一个 TWR final 帧。
 * @param buffer           [输入] 指向包含接收到的原始帧数据的缓冲区的指针。
 * @param length           [输入] 接收到的数据的长度。
 * @param parsed_msg       [输出] 指向一个 twr_final_msg_t 结构体的指针，用于存放解析结果。
 * @return int16_t          成功解析则返回 0，错误则返回 -1。
 */
int16_t parse_final_frame(const uint8_t *buffer, uint16_t length,
                          twr_final_msg_t *parsed_msg);

#endif