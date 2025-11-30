// timestamp.c
// GNSS UTC timestamp helpers

#include "UM960samplingtask.h"
#include "timestamp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "main.h"

extern TIM_HandleTypeDef htim16;

static volatile utc_global_timestamp_t g_timestamp = {0};
volatile utc_timestamp_t ts                        = {0};
static volatile uint32_t timestamp_s               = 0;

void utc_timestamp_from_gnss(const gnss_time_raw_t *raw)
{
    if (!raw) {
        return;
    }
    ts.week        = raw->wn;
    ts.tow_ms      = raw->ms;
    ts.leap_sec    = raw->leapSec;
    ts.time_status = raw->timeStatus;
    ts.time_ref    = raw->timeRef;
}

void timestamp_tick_irq(void) // 本地时钟
{
    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();

    timestamp_s++;

    taskEXIT_CRITICAL_FROM_ISR(saved);
}

utc_global_timestamp_t gettimestamp(void)
{
    utc_global_timestamp_t ts_out = {0};

    ts_out.week = g_timestamp.week;

    uint32_t cnt  = __HAL_TIM_GET_COUNTER(&htim16);
    ts_out.tow_ms = timestamp_s * 1000U + cnt / 20U;
    ts_out.us     = cnt % 20U * 50U;

    return ts_out;
}

void PPS_IRQHandler(void) // 外部中断的同步时钟
{
    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();

    uint32_t nowtimestamp_ms = ts.tow_ms;

    nowtimestamp_ms = ((uint32_t)nowtimestamp_ms / 1000) * 1000 + 1000;

    g_timestamp.tow_ms = nowtimestamp_ms;

    timestamp_s = g_timestamp.tow_ms / 1000;

    g_timestamp.week = ts.week;

    taskEXIT_CRITICAL_FROM_ISR(saved);
}
