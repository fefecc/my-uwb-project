#include "time_service.h"

#include <string.h>

#include "tim.h"

static volatile uint64_t g_local_sec_count = 0;
static TimeGnssUtcCache g_gnss_cache;
static int64_t g_utc_offset_tick_20k = 0;
static uint32_t g_sync_seq = 0;
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

static uint32_t local_tick_period_unlocked(void)
{
    uint32_t period_count = __HAL_TIM_GET_AUTORELOAD(&htim16) + 1U;
    if (period_count == 0U) {
        period_count = TIME_SERVICE_LOCAL_TICKS_PER_SECOND;
    }
    return period_count;
}

static uint64_t read_local_tick_unlocked(void)
{
    uint64_t sec = g_local_sec_count;
    uint32_t counter = __HAL_TIM_GET_COUNTER(&htim16);
    uint32_t period_count = local_tick_period_unlocked();

    if (__HAL_TIM_GET_FLAG(&htim16, TIM_FLAG_UPDATE) != RESET &&
        counter < (period_count / 2U)) {
        sec++;
    }

    return (sec * (uint64_t)period_count) + (uint64_t)counter;
}

static TimeLocalClock local_tick_to_clock(uint64_t local_tick_20k)
{
    uint32_t period_count = local_tick_period_unlocked();
    TimeLocalClock out;

    out.sec = local_tick_20k / (uint64_t)period_count;
    out.ms = ((float)(local_tick_20k % (uint64_t)period_count) *
              TIME_SERVICE_LOCAL_MS_PER_SECOND) /
             (float)period_count;
    return out;
}

static uint64_t utc_to_tick_20k(TimeUtcClock utc)
{
    const uint64_t ticks_per_week =
        (uint64_t)TIME_SERVICE_UTC_MS_PER_WEEK * (TIME_SERVICE_LOCAL_TICKS_PER_SECOND / 1000UL);
    const uint64_t ticks_per_ms =
        (uint64_t)TIME_SERVICE_LOCAL_TICKS_PER_SECOND / 1000UL;

    return ((uint64_t)utc.week * ticks_per_week) +
           ((uint64_t)utc.week_ms * ticks_per_ms);
}

static bool tick_20k_to_utc(int64_t utc_tick_20k, TimeUtcClock *out)
{
    const uint64_t ticks_per_week =
        (uint64_t)TIME_SERVICE_UTC_MS_PER_WEEK * (TIME_SERVICE_LOCAL_TICKS_PER_SECOND / 1000UL);
    const uint64_t ticks_per_ms =
        (uint64_t)TIME_SERVICE_LOCAL_TICKS_PER_SECOND / 1000UL;

    if (out == NULL || utc_tick_20k < 0) {
        return false;
    }

    uint64_t total_tick = (uint64_t)utc_tick_20k;
    uint64_t week = total_tick / ticks_per_week;
    uint64_t week_tick = total_tick % ticks_per_week;
    uint64_t week_ms = (week_tick + (ticks_per_ms / 2ULL)) / ticks_per_ms;

    if (week_ms >= TIME_SERVICE_UTC_MS_PER_WEEK) {
        week++;
        week_ms -= TIME_SERVICE_UTC_MS_PER_WEEK;
    }

    out->week = (uint32_t)week;
    out->week_ms = (uint32_t)week_ms;
    return true;
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
    g_utc_offset_tick_20k = 0;
    g_sync_seq = 0;
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
        uint64_t local_tick_20k = read_local_tick_unlocked();
        TimeUtcClock next_utc = TimeService_UtcAddMs(g_gnss_cache.utc, 1000U);

        g_utc_offset_tick_20k =
            (int64_t)utc_to_tick_20k(next_utc) - (int64_t)local_tick_20k;
        g_sync_seq++;
        g_sync_state = TIME_SYNC_LOCKED;
        g_utc_valid = true;
    }

    exit_critical(lock);
}

bool TimeService_GetLocalTick20k(uint64_t *out)
{
    if (out == NULL) {
        return false;
    }

    uint32_t lock = enter_critical();
    *out = read_local_tick_unlocked();
    exit_critical(lock);

    return true;
}

bool TimeService_CaptureNow(TimeCapture *out)
{
    if (out == NULL) {
        return false;
    }

    uint32_t lock = enter_critical();

    out->local_tick_20k = read_local_tick_unlocked();
    out->utc_offset_tick_20k = g_utc_offset_tick_20k;
    out->utc_valid = g_utc_valid;
    out->sync_state = g_sync_state;
    out->sync_seq = g_sync_seq;

    exit_critical(lock);

    return true;
}

bool TimeService_ResolveCapture(const TimeCapture *cap, TimeTimestamp *out)
{
    if (cap == NULL || out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->local_clock = local_tick_to_clock(cap->local_tick_20k);
    out->sync_state = cap->sync_state;
    out->sync_seq = cap->sync_seq;

    if (cap->utc_valid &&
        tick_20k_to_utc((int64_t)cap->local_tick_20k + cap->utc_offset_tick_20k,
                        &out->local_utc)) {
        out->utc_valid = true;
    }

    return true;
}

bool TimeService_GetTimestamp(TimeTimestamp *out)
{
    TimeCapture capture;
    if (!TimeService_CaptureNow(&capture)) {
        return false;
    }

    return TimeService_ResolveCapture(&capture, out);
}

bool TimeService_GetLocalClock(TimeLocalClock *out)
{
    if (out == NULL) {
        return false;
    }

    uint64_t local_tick_20k = 0;
    if (!TimeService_GetLocalTick20k(&local_tick_20k)) {
        return false;
    }

    *out = local_tick_to_clock(local_tick_20k);

    return true;
}

const TimeGnssUtcCache *TimeService_GetGnssUtcCache(void)
{
    return &g_gnss_cache;
}
