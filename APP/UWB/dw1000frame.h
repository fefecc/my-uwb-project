#ifndef _DW1000FREAME_H_
#define _DW1000FREAME_H_

#include "main.h"

#ifdef __GNUC__
#define PACKED __attribute__((packed))
#else
#define PACKED
#pragma pack(push, 1)
#endif

/*************************************************************************************************/
/* 共享宏定义                                                                                    */
/*************************************************************************************************/

// 帧控制字段 (FCF) 的常用配置:
// - FCF[0]: 数据帧 (0x01), PAN ID 压缩 (0x40) -> 0x41 (如果需要ACK, 则为 0x61)
// - FCF[1]: 目标和源都使用 16 位短地址 (0x88)
#define FCF_BYTE_0 (0x41) // 数据帧, PAN ID 压缩
#define FCF_BYTE_1 (0x88) // 源/目标均为 16 位短地址

// 应用层功能码，用于区分不同的帧
typedef enum {
    FUNC_CODE_POLL  = 0x21, // 标签发起的轮询帧
    FUNC_CODE_RESP  = 0x10, // 基站回复的响应帧
    FUNC_CODE_FINAL = 0x23  // 标签发送的最终帧
} twr_function_code_e;

/*************************************************************************************************/
/* 帧结构体定义                                                                                  */
/*************************************************************************************************/

/**
 * @brief Poll (轮询) 帧结构体
 * @note  由 Tag 发起，用于启动一次测距。
 *10字节
 */
typedef struct {
    uint8_t frame_ctrl[2];  // 帧控制字段, e.g., {FCF_BYTE_0, FCF_BYTE_1}
    uint8_t sequence_num;   // 序列号
    uint8_t pan_id[2];      // 目标 PAN ID
    uint8_t dest_addr[2];   // 目标地址 (Anchor 的短地址)
    uint8_t source_addr[2]; // 源地址 (Tag 自身的短地址)
    uint8_t function_code;  // 功能码, 应为 FUNC_CODE_POLL
} PACKED twr_poll_msg_t;

/**
 * @brief Response (响应) 帧结构体
 * @note  由 Anchor 回复 Poll 帧。
 * 载荷中包含了 Anchor 记录的两个关键时间戳。
 * 20字节
 */
typedef struct {
    uint8_t frame_ctrl[2];
    uint8_t sequence_num;
    uint8_t pan_id[2];
    uint8_t dest_addr[2];   // 目标地址 (Tag 的短地址)
    uint8_t source_addr[2]; // 源地址 (Anchor 自身的短地址)
    uint8_t function_code;  // 功能码, 应为 FUNC_CODE_RESP
    uint8_t poll_rx_ts[5];  // Anchor 接收到 Poll 帧的 40 位时间戳
    uint8_t resp_tx_ts[5];  // Tag 接收 Response 帧的 40 位时间戳

} PACKED twr_resp_msg_t;

/**
 * @brief Final (最终) 帧结构体
 * @note  由 Tag 发送，用于将所有时间戳信息传递给 Anchor，以完成距离计算。
 *35字节
 */
typedef struct {
    uint8_t frame_ctrl[2];
    uint8_t sequence_num;
    uint8_t pan_id[2];
    uint8_t dest_addr[2];   // 目标地址 (Anchor 的短地址)
    uint8_t source_addr[2]; // 源地址 (Tag 自身的短地址)
    uint8_t function_code;  // 功能码, 应为 FUNC_CODE_FINAL
    uint8_t poll_tx_ts[5];  // poll发送时间戳
    uint8_t poll_rx_ts[5];  // poll接收时间戳
    uint8_t resp_tx_ts[5];  // resp发送时间戳
    uint8_t resp_rx_ts[5];  // resp接收时间戳
    uint8_t final_tx_ts[5]; // final发送时间戳
} PACKED twr_final_msg_t;

// 恢复默认的内存对齐方式
#ifndef __GNUC__
#pragma pack(pop)
#endif
#undef PACKED

#endif //_DW1000FREAME_H_ end