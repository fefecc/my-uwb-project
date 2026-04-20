#ifndef _IMUDATADEALTASK_H_
#define _IMUDATADEALTASK_H_

#include "FreeRTOS.h"
#include "main.h"
#include "queue.h"

#pragma pack(push, 1) // 关闭字节对齐

typedef struct {
    uint8_t sync1;      // 0xAA
    uint8_t sync2;      // 0x44
    uint8_t sync3;      // 0x55
    uint8_t cpuIdle;    // CPU 占用 0-100
    uint16_t msgID;     // = 1
    uint16_t msgLen;    // 数据体（含 CRC）的长度
    uint8_t timeRef;    // 0:GPST, 1:BDST
    uint8_t timeStatus; // 0:正常, 1:接收机内部时间, 0xFF:未知
    uint64_t time;      // 整秒时间
    float sec;          // 小数时间
    uint32_t reserved;  // 0
    uint8_t version;    // 消息版本
    uint8_t leapSec;    // 闰秒
    uint16_t delayMs;   // 输出延迟
} MsgHeader_t;

/* 六轴输出（Message ID = 1）的数据体 */
typedef struct {
    uint8_t sensor;       // 0:asm330lhh, 0xFF:未定义
    uint16_t sensitivity; // [0:7] 加速度比例系数, [8:15] 陀螺仪比例系数
    int16_t gyro[3];      // X,Y,Z 轴陀螺仪
    int16_t accel[3];     // X,Y,Z 轴加速度
    uint32_t crc;         // 32 位 CRC
} MsgBodyIMU_t;

typedef struct {
    double dw1000dis;
} MsgDw1000_t;

/* 组合在一起便于一次发送 */
typedef struct {
    MsgHeader_t hdr;
    MsgDw1000_t dw1000Msg;
    MsgBodyIMU_t body; // 这里面有crc，不改了
} MsgIMU_t;

#pragma pack(pop)

extern QueueHandle_t IMUDataToSDTaskQueue; // 外部使用的诗数据队列

void imuDataDealTaskFunc(void);

#endif /* _IMUDATADEALTASK_H_ */
