#include "log_service.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "../task/app_tasks.h"

#define LOG_SERVICE_UWB_ONLY      (1U)
#define LOG_SERVICE_USART_ENABLED (1U)
#define LOG_SERVICE_SD_ENABLED  (0U)  /* 屏蔽 SD 卡写入, 防止影响 UWB 实时性 */

#if LOG_SERVICE_USART_ENABLED || LOG_SERVICE_SD_ENABLED
static bool is_uwb_task(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return false;
    }
    const char *name = pcTaskGetName(NULL);
    return (name != NULL && strncmp(name, "uwb", 3U) == 0);
}


static const char *level_name(AppLogLevel level)
{
    switch (level) {
        case APP_LOG_TRACE:
            return "TRACE";
        case APP_LOG_DEBUG:
            return "DEBUG";
        case APP_LOG_INFO:
            return "INFO";
        case APP_LOG_WARN:
            return "WARN";
        case APP_LOG_ERROR:
            return "ERROR";
        default:
            return "LOG";
    }
}

static size_t bounded_strlen(const char *text, size_t max_len)
{
    size_t len = 0;

    if (text == NULL) {
        return 0;
    }

    while (len < max_len && text[len] != '\0') {
        len++;
    }

    return len;
}
#endif

void LogService_Init(void)
{
    AppTasks_LogInit();
}

void LogService_VWrite(AppLogLevel level, const char *fmt, va_list args)
{
    if (fmt == NULL) {
        return;
    }

#if !LOG_SERVICE_USART_ENABLED && !LOG_SERVICE_SD_ENABLED
    (void)level;
    (void)args;
    return;
#else

    char msg[192];
    char line[256];

    (void)vsnprintf(msg, sizeof(msg), fmt, args);
#if LOG_SERVICE_UWB_ONLY
    if (!is_uwb_task()) {
        return;
    }
#endif

    int n = snprintf(line, sizeof(line), "%lu %-5s %s\r\n",
                     (unsigned long)HAL_GetTick(),
                     level_name(level),
                     msg);

    if (n <= 0) {
        return;
    }

#if LOG_SERVICE_USART_ENABLED
    (void)AppTasks_LogWriteText(line, bounded_strlen(line, sizeof(line)));
#endif

    /* SD 卡日志写入 (非阻塞, 写入环形缓冲区) */
#if LOG_SERVICE_SD_ENABLED
    (void)AppTasks_LogWriteSd(line, bounded_strlen(line, sizeof(line)));
#endif
#endif
}

void LogService_Write(AppLogLevel level, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    LogService_VWrite(level, fmt, args);
    va_end(args);
}
