#include "app_time.h"

#include "main.h"

static app_timepoint_t g_timepoint = {0};

void AppTime_Init(void)
{
    g_timepoint.utc_ms  = 0;
    g_timepoint.tick_ms = HAL_GetTick();
}

void AppTime_UpdateUtc(uint64_t utc_ms)
{
    g_timepoint.utc_ms  = utc_ms;
    g_timepoint.tick_ms = HAL_GetTick();
}

app_timepoint_t AppTime_Now(void)
{
    app_timepoint_t now;
    uint32_t tick_now = HAL_GetTick();
    now.tick_ms       = tick_now;
    uint32_t delta    = tick_now - g_timepoint.tick_ms;
    now.utc_ms        = g_timepoint.utc_ms + (uint64_t)delta;
    return now;
}
