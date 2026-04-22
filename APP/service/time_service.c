#include "time_service.h"

#include <string.h>

#include "tim.h"

typedef struct {
    bool valid;
    TimeUtcClock utc_at_anchor;
    TimeLocalClock local_at_anchor;
    uint32_t sync_seq;
} TimeUtcAnchor;

static volatile uint64_t g_local_sec_count = 0;
static TimeGnssUtcCache g_gnss_cache;
static TimeUtcAnchor g_utc_anchor;
static TimeSyncState g_sync_state = TIME_SYNC_NONE;
static bool g_utc_valid = false;

static uint32_t enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void exit_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static TimeLocalClock read_local_clock_unlocked(void)
{
    uint64_t sec = g_local_sec_count;
    uint32_t counter = __HAL_TIM_GET_COUNTER(&htim16);
    uint32_t period_count = __HAL_TIM_GET_AUTORELOAD(&htim16) + 1U;

    if (period_count == 0U) {
        period_count = 1U;
    }

    if (__HAL_TIM_GET_FLAG(&htim16, TIM_FLAG_UPDATE) != RESET &&
        counter < (period_count / 2U)) {
        sec++;
    }

    TimeLocalClock out;
    out.sec = sec;
    out.ms = ((float)counter * TIME_SERVICE_LOCAL_MS_PER_SECOND) /
             (float)period_count;
    return out;
}

static uint32_t local_clock_delta_ms(TimeLocalClock now, TimeLocalClock base)
{
    if (now.sec < base.sec ||
        (now.sec == base.sec && now.ms <= base.ms)) {
        return 0U;
    }

    double delta_ms = ((double)(now.sec - base.sec) * 1000.0) +
                      (double)now.ms -
                      (double)base.ms;

    return (uint32_t)(delta_ms + 0.5);
}

TimeUtcClock TimeService_UtcAddMs(TimeUtcClock utc, uint32_t delta_ms)
{
    uint64_t total_ms = (uint64_t)utc.week_ms + (uint64_t)delta_ms;
    utc.week += (uint32_t)(total_ms / TIME_SERVICE_UTC_MS_PER_WEEK);
    utc.week_ms = (uint32_t)(total_ms % TIME_SERVICE_UTC_MS_PER_WEEK);
    return utc;
}

void TimeService_Init(void)
{
    uint32_t lock = enter_critical();

    g_local_sec_count = 0;
    memset(&g_gnss_cache, 0, sizeof(g_gnss_cache));
    memset(&g_utc_anchor, 0, sizeof(g_utc_anchor));
    g_sync_state = TIME_SYNC_NONE;
    g_utc_valid = false;

    __HAL_TIM_SET_COUNTER(&htim16, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);

    exit_critical(lock);

    (void)HAL_TIM_Base_Start_IT(&htim16);
}

void TimeService_OnTim16Overflow(void)
{
    uint32_t lock = enter_critical();
    g_local_sec_count++;
    exit_critical(lock);
}

void TimeService_WriteUtcCache(const TimeUtcClock *utc)
{
    if (utc == NULL || utc->week_ms >= TIME_SERVICE_UTC_MS_PER_WEEK) {
        return;
    }

    uint32_t lock = enter_critical();
    g_gnss_cache.utc = *utc;
    g_gnss_cache.valid = true;
    g_gnss_cache.seq++;
    exit_critical(lock);
}

void TimeService_OnPpsIrq(void)
{
    uint32_t lock = enter_critical();

    if (g_gnss_cache.valid) {
        g_utc_anchor.utc_at_anchor = TimeService_UtcAddMs(g_gnss_cache.utc, 1000U);
        g_utc_anchor.local_at_anchor = read_local_clock_unlocked();
        g_utc_anchor.valid = true;
        g_utc_anchor.sync_seq++;
        g_sync_state = TIME_SYNC_LOCKED;
        g_utc_valid = true;
    }

    exit_critical(lock);
}

bool TimeService_GetTimestamp(TimeTimestamp *out)
{
    if (out == NULL) {
        return false;
    }

    uint32_t lock = enter_critical();

    TimeUtcAnchor anchor = g_utc_anchor;
    TimeSyncState sync_state = g_sync_state;
    bool utc_valid = g_utc_valid;
    out->local_clock = read_local_clock_unlocked();

    exit_critical(lock);

    if (anchor.valid) {
        uint32_t delta_ms = local_clock_delta_ms(out->local_clock,
                                                anchor.local_at_anchor);
        out->local_utc = TimeService_UtcAddMs(anchor.utc_at_anchor, delta_ms);
        out->utc_valid = utc_valid;
        out->sync_state = sync_state;
        out->sync_seq = anchor.sync_seq;
    } else {
        memset(&out->local_utc, 0, sizeof(out->local_utc));
        out->utc_valid = false;
        out->sync_state = TIME_SYNC_NONE;
        out->sync_seq = 0;
    }

    return true;
}

bool TimeService_GetLocalClock(TimeLocalClock *out)
{
    if (out == NULL) {
        return false;
    }

    uint32_t lock = enter_critical();
    *out = read_local_clock_unlocked();
    exit_critical(lock);

    return true;
}

const TimeGnssUtcCache *TimeService_GetGnssUtcCache(void)
{
    return &g_gnss_cache;
}
