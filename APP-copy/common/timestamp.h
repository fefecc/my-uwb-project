// timestamp.h
// GNSS UTC timestamp helpers

#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stddef.h>
#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t sync3;
    uint8_t cpuIdle;

    uint16_t messageId;
    uint16_t messageLength;

    uint8_t timeRef;
    uint8_t timeStatus;

    uint16_t wn;
    uint32_t ms;

    uint32_t version;

    uint8_t reserved;
    uint8_t leapSec;
    uint16_t delayMs;
} gnss_time_raw_t;

typedef struct {
    uint32_t week;
    uint32_t tow_ms;
    uint8_t leap_sec;
    uint8_t time_ref;
    uint8_t time_status;
} utc_timestamp_t;

typedef struct __attribute__((packed)) {
    uint32_t week;
    uint32_t tow_ms;
    uint32_t us;
} utc_global_timestamp_t;

#define UTC_MS_PER_WEEK ((uint32_t)604800000U)

void utc_timestamp_from_gnss(const gnss_time_raw_t *raw);
void timestamp_tick_irq(void);
uint32_t get_local_clock_counter(void);
void PPS_IRQHandler(void);
utc_global_timestamp_t gettimestamp(void);

#endif // TIMESTAMP_H
