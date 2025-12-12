#ifndef _SDCARD_H_
#define _SDCARD_H_

#include <stdbool.h>

#include "main.h"
#include "UM960samplingtask.h"
#include "imusamplingtask.h"
#include "timestamp.h"

// 日志来源标识
#define LOG_SRC_IMU    1
#define LOG_SRC_GNSS   2
#define LOG_SRC_DW1000 3

// 日志队列阈值
#define LOG_QUEUE_DRAIN_THRESHOLD 64

// SD写入融合数据封装（可扩展 UWB 等）
typedef struct __attribute__((packed)) {
    uint8_t sync1;                    // 帧头1：0xAA
    uint8_t sync2;                    // 帧头2：0x44
    uint8_t sync3;                    // 帧头3：0xB5
    utc_global_timestamp_t timestamp; // 全局时间戳
    gnss_fusion_msg_t gnss;           // GNSS融合数据
    IMUOrigData_t imu;                // IMU原始数据
    // 预留: 后续可添加 UWB 等其他传感器数据
} sd_fusion_record_t;

#define LOG_DATA_MAX_LEN 128 // 根据实际情况调整

typedef struct {
    utc_global_timestamp_t timestamp; // 时间戳（us/ms/whatever，只要单调递增）
    uint8_t source_id;                // 来自哪个线程/模块
    uint16_t data_len;                // 有效数据长度
    uint8_t data[LOG_DATA_MAX_LEN];   // 实际内容
} LogItem;

#define QUEUE_SIZE 1024

typedef struct {
    LogItem buffer[QUEUE_SIZE];
    volatile uint32_t head; // 写入指针（生产者用）
    volatile uint32_t tail; // 读取指针（消费者用）
} LogQueue;

// 向日志队列写入一条记录（按 timestamp.tow_ms 排序），成功返回 true，队列满返回 false
bool LogQueue_Push(LogQueue *q, const void *frame, uint8_t source_id);

// 获取当前队列元素数量
uint32_t LogQueue_Count(const LogQueue *q);

// 读取并弹出一条记录，返回指向内部缓冲区的指针，空时返回 NULL
LogItem *LogQueue_Read(LogQueue *q);

void FatFs_Check(void);     // 判断FatFs是否挂载成功，若没有创建FatFs则格式化SD卡
void FatFs_GetVolume(void); // 计算设备的容量，包括总容量和剩余容量

void SDMMCTask(void *argument);
void PackResult(void);

#endif /* _SDCARD_H_ */
