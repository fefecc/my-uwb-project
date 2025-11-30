// timestamp.h
// GNSS UTC timestamp helpers

#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdint.h>
#include <stddef.h>

typedef struct __attribute__((packed)) {
    uint8_t sync1;   // 0  十六进制 0xAA
    uint8_t sync2;   // 1  十六进制 0x44
    uint8_t sync3;   // 2  十六进制 0xB5
    uint8_t cpuIdle; // 3  CPU idle 0-100

    uint16_t messageId;     // 4  Message ID (USHORT)
    uint16_t messageLength; // 6  Message Length (USHORT)

    uint8_t timeRef;    // 8  时间系统 GPST / BDST
    uint8_t timeStatus; // 9  Time Status

    uint16_t wn; // 10 时间周 (USHORT)
    uint32_t ms; // 12 周内毫秒 (ULONG)

    uint32_t version; // 16 Release version (ULONG)

    uint8_t reserved; // 20 保留
    uint8_t leapSec;  // 21 闰秒
    uint16_t delayMs; // 22 数据输出延迟 (USHORT)
} gnss_time_raw_t;

// 对齐UTC的内部时间戳
typedef struct {
    uint32_t week;    // 周数
    uint32_t tow_ms;  // 周内毫秒
    uint8_t leap_sec; // 闰秒
    uint8_t time_ref; // GPST / BDST
    uint8_t time_status;
} utc_timestamp_t;

// 全局时间戳结构体，直接记录时间戳
typedef struct {
    uint32_t week;   // 周数
    uint32_t tow_ms; // 周内毫秒
    uint32_t us;     // 1ms -> 20*50us
} utc_global_timestamp_t;

// 常量：一周的毫秒数
#define UTC_MS_PER_WEEK ((uint32_t)604800000U)

void utc_timestamp_from_gnss(const gnss_time_raw_t *raw);
void timestamp_tick_irq(void);
void PPS_IRQHandler(void); // 外部中断的同步时间戳
utc_global_timestamp_t gettimestamp(void);

#endif // TIMESTAMP_H
