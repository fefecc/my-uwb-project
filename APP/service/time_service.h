#ifndef APP_SERVICE_TIME_SERVICE_H_
#define APP_SERVICE_TIME_SERVICE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TIME_SERVICE_LOCAL_TICKS_PER_SECOND (20000UL)
#define TIME_SERVICE_LOCAL_MS_PER_SECOND (1000.0f)
#define TIME_SERVICE_UTC_MS_PER_WEEK     (604800000UL)

typedef enum {
    TIME_SYNC_NONE = 0,
    TIME_SYNC_LOCKED,
    TIME_SYNC_HOLDOVER,
    TIME_SYNC_LOST,
} TimeSyncState;

typedef struct {
    uint64_t sec;
    float ms;
} TimeLocalClock;

typedef struct {
    uint32_t week;
    uint32_t week_ms;
} TimeUtcClock;

typedef struct {
    bool valid;
    TimeUtcClock utc;
    uint32_t seq;
} TimeGnssUtcCache;

typedef struct {
    uint64_t local_tick_20k;
    int64_t utc_offset_tick_20k;
    bool utc_valid;
    TimeSyncState sync_state;
    uint32_t sync_seq;
} TimeCapture;

typedef struct {
    TimeLocalClock local_clock;
    TimeUtcClock local_utc;
    bool utc_valid;
    TimeSyncState sync_state;
    uint32_t sync_seq;
} TimeTimestamp;

void TimeService_Init(void);
void TimeService_OnTim16Overflow(void);
void TimeService_WriteUtcCache(const TimeUtcClock *utc);
void TimeService_OnPpsIrq(void);
bool TimeService_GetLocalTick20k(uint64_t *out);
bool TimeService_CaptureNow(TimeCapture *out);
bool TimeService_ResolveCapture(const TimeCapture *cap, TimeTimestamp *out);
bool TimeService_GetTimestamp(TimeTimestamp *out);
bool TimeService_GetLocalClock(TimeLocalClock *out);
TimeUtcClock TimeService_UtcAddMs(TimeUtcClock utc, uint32_t delta_ms);
const TimeGnssUtcCache *TimeService_GetGnssUtcCache(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_TIME_SERVICE_H_ */
