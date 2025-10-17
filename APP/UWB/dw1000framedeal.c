#include "dw1000framedeal.h"
#include "dw1000frame.h"
#include "string.h"

/*************************************************************************************************/
/* poll帧生成函数实现                                                                                */
/*************************************************************************************************/
int16_t create_poll_frame(uint8_t *buffer, size_t buffer_size,
                          uint8_t seq_num, uint16_t pan_id,
                          uint16_t dest_addr, uint16_t source_addr)
{
    const size_t frame_size = sizeof(twr_poll_msg_t);

    // 1. 参数校验：检查缓冲区大小是否足够
    if (buffer == NULL || buffer_size < frame_size) {
        return -1; // 缓冲区太小或为空指针
    }

    // 2. 将缓冲区指针转换为结构体指针，方便地填充字段
    twr_poll_msg_t *poll_msg = (twr_poll_msg_t *)buffer;

    // 3. 填充 MAC 头部
    poll_msg->frame_ctrl[0] = FCF_BYTE_0;
    poll_msg->frame_ctrl[1] = FCF_BYTE_1;
    poll_msg->sequence_num  = seq_num;

    // 填充 PAN ID (小端字节序)
    poll_msg->pan_id[0] = pan_id & 0xFF;
    poll_msg->pan_id[1] = (pan_id >> 8) & 0xFF;

    // 填充目标地址
    poll_msg->dest_addr[0] = dest_addr & 0xFF;
    poll_msg->dest_addr[1] = (dest_addr >> 8) & 0xFF;

    // 填充源地址
    poll_msg->source_addr[0] = source_addr & 0xFF;
    poll_msg->source_addr[1] = (source_addr >> 8) & 0xFF;

    // 4. 填充应用层载荷
    poll_msg->function_code = FUNC_CODE_POLL;
    // 5. 返回生成的帧的实际长度
    return (int16_t)frame_size;
}

/*************************************************************************************************/
/* poll帧解析函数实现                                                                                */
/*************************************************************************************************/
int16_t parse_poll_frame(const uint8_t *buffer, uint16_t length,
                         twr_poll_msg_t *parsed_msg)
{
    // 1. 参数校验
    if (buffer == NULL || parsed_msg == NULL) {
        return -1;
    }

    // 2. 长度校验：接收到的长度是否与 Poll 帧的预期长度一致
    if (length < sizeof(twr_poll_msg_t)) {
        // 注意：实际接收长度可能包含 2 字节的 FCS，所以这里使用 '<' 而不是 '!='
        // 更严谨的检查可以是 length == sizeof(twr_poll_msg_t) + 2
        return -1;
    }

    // 3. 将缓冲区指针转换为结构体指针，方便地访问字段
    const twr_poll_msg_t *received_msg = (const twr_poll_msg_t *)buffer;

    // 4. 内容校验：检查帧的关键字段是否符合预期
    //    检查帧控制字段
    if (received_msg->frame_ctrl[0] != FCF_BYTE_0 || received_msg->frame_ctrl[1] != FCF_BYTE_1) {
        return -1; // 不是我们期望的帧格式
    }
    //    检查功能码
    if (received_msg->function_code != FUNC_CODE_POLL) {
        return -1; // 这不是一个 Poll 帧
    }

    // 5. 所有校验通过，将解析出的数据复制到输出结构体中
    memcpy(parsed_msg, received_msg, sizeof(twr_poll_msg_t));

    return 0; // 解析成功
}

/*************************************************************************************************/
/* resp帧生成函数实现                                                                                */
/*************************************************************************************************/
int16_t create_resp_frame(uint8_t *buffer, size_t buffer_size,
                          uint8_t seq_num, uint16_t pan_id,
                          uint16_t dest_addr, uint16_t source_addr,
                          const uint8_t *poll_rx_ts, const uint8_t *resp_tx_ts)
{
    const size_t frame_size = sizeof(twr_resp_msg_t); // 发送数据结构

    // 1. 参数校验
    if (buffer == NULL || buffer_size < frame_size || poll_rx_ts == NULL) {
        return -1; // 缓冲区太小或空指针，返回错误
    }

    // 2. 将缓冲区指针转换为结构体指针，方便地填充字段
    twr_resp_msg_t *resp_msg = (twr_resp_msg_t *)buffer;

    // 3. 填充 MAC 头部
    resp_msg->frame_ctrl[0] = FCF_BYTE_0;
    resp_msg->frame_ctrl[1] = FCF_BYTE_1;
    resp_msg->sequence_num  = seq_num;

    // 填充 PAN ID (小端字节序)
    resp_msg->pan_id[0] = pan_id & 0xFF;
    resp_msg->pan_id[1] = (pan_id >> 8) & 0xFF;

    // 填充目标地址
    resp_msg->dest_addr[0] = dest_addr & 0xFF;
    resp_msg->dest_addr[1] = (dest_addr >> 8) & 0xFF;

    // 填充源地址
    resp_msg->source_addr[0] = source_addr & 0xFF;
    resp_msg->source_addr[1] = (source_addr >> 8) & 0xFF;

    // 4. 填充应用层载荷
    resp_msg->function_code = FUNC_CODE_RESP;

    memcpy(resp_msg->poll_rx_ts, poll_rx_ts, 5);
    memcpy(resp_msg->resp_tx_ts, resp_tx_ts, 5);

    // 5. 返回生成的帧的实际长度
    return (int16_t)frame_size;
}

/*************************************************************************************************/
/* resp帧解析函数实现                                                                                */
/*************************************************************************************************/
int16_t parse_resp_frame(const uint8_t *buffer, uint16_t length,
                         twr_resp_msg_t *parsed_msg)
{
    // 1. 参数校验
    if (buffer == NULL || parsed_msg == NULL) {
        return -1; // 返回错误
    }

    // 2. 长度校验
    if (length < sizeof(twr_resp_msg_t)) {
        return -1; // 返回错误
    }

    // 3. 将缓冲区指针转换为结构体指针
    const twr_resp_msg_t *received_msg = (const twr_resp_msg_t *)buffer;

    // 4. 内容校验：检查帧的关键字段是否符合预期
    if (received_msg->frame_ctrl[0] != FCF_BYTE_0 || received_msg->frame_ctrl[1] != FCF_BYTE_1) {
        return -1; // 不是我们期望的帧格式，返回错误
    }
    if (received_msg->function_code != FUNC_CODE_RESP) {
        return -1; // 这不是一个 Response 帧，返回错误
    }

    // 5. 所有校验通过，将解析出的数据复制到输出结构体中
    memcpy(parsed_msg, received_msg, sizeof(twr_resp_msg_t)); // 直接将结构体完成解析

    return 0; // 解析成功，返回 0
}

/*************************************************************************************************/
/* final帧生成函数实现                                                                                */
/*************************************************************************************************/
// poll_tx_ts本机读取  poll_rx_ts poll帧读取，但是应该在之间的解析并保存了数据帧里面  resp_rx_ts 本机读取
int16_t create_final_frame(uint8_t *buffer, size_t buffer_size,
                           uint8_t seq_num, uint16_t pan_id,
                           uint16_t dest_addr, uint16_t source_addr,
                           const uint8_t *poll_tx_ts, const uint8_t *poll_rx_ts,
                           const uint8_t *resp_tx_ts, const uint8_t *resp_rx_ts,
                           const uint8_t *final_tx_ts)
{
    const size_t frame_size = sizeof(twr_final_msg_t);

    // 1. 参数校验
    if (buffer == NULL || buffer_size < frame_size || poll_tx_ts == NULL || resp_rx_ts == NULL) {
        return -1; // 缓冲区太小或空指针，返回错误
    }

    // 2. 将缓冲区指针转换为结构体指针，方便地填充字段
    twr_final_msg_t *final_msg = (twr_final_msg_t *)buffer;

    // 3. 填充 MAC 头部
    final_msg->frame_ctrl[0] = FCF_BYTE_0;
    final_msg->frame_ctrl[1] = FCF_BYTE_1;
    final_msg->sequence_num  = seq_num;

    // 填充 PAN ID (小端字节序)
    final_msg->pan_id[0] = pan_id & 0xFF;
    final_msg->pan_id[1] = (pan_id >> 8) & 0xFF;

    // 填充目标地址
    final_msg->dest_addr[0] = dest_addr & 0xFF;
    final_msg->dest_addr[1] = (dest_addr >> 8) & 0xFF;

    // 填充源地址
    final_msg->source_addr[0] = source_addr & 0xFF;
    final_msg->source_addr[1] = (source_addr >> 8) & 0xFF;

    // 4. 填充应用层载荷
    final_msg->function_code = FUNC_CODE_FINAL;
    memcpy(final_msg->poll_tx_ts, poll_tx_ts, 5);
    memcpy(final_msg->poll_rx_ts, poll_rx_ts, 5);
    memcpy(final_msg->resp_tx_ts, resp_tx_ts, 5);
    memcpy(final_msg->resp_rx_ts, resp_rx_ts, 5);
    memcpy(final_msg->final_tx_ts, final_tx_ts, 5);

    // 5. 返回生成的帧的实际长度
    return (int16_t)frame_size;
}

/*************************************************************************************************/
/* final帧解析函数实现                                                                                */
/*************************************************************************************************/
int16_t parse_final_frame(const uint8_t *buffer, uint16_t length,
                          twr_final_msg_t *parsed_msg)
{
    // 1. 参数校验
    if (buffer == NULL || parsed_msg == NULL) {
        return -1; // 返回错误
    }

    // 2. 长度校验
    if (length < sizeof(twr_final_msg_t)) {
        return -1; // 返回错误
    }

    // 3. 将缓冲区指针转换为结构体指针
    const twr_final_msg_t *received_msg = (const twr_final_msg_t *)buffer;

    // 4. 内容校验：检查帧的关键字段是否符合预期
    if (received_msg->frame_ctrl[0] != FCF_BYTE_0 || received_msg->frame_ctrl[1] != FCF_BYTE_1) {
        return -1; // 不是我们期望的帧格式，返回错误
    }
    if (received_msg->function_code != FUNC_CODE_FINAL) {
        return -1; // 这不是一个 Final 帧，返回错误
    }

    // 5. 所有校验通过，将解析出的数据复制到输出结构体中
    memcpy(parsed_msg, received_msg, sizeof(twr_final_msg_t));

    return 0; // 解析成功，返回 0
}
