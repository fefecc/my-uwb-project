#include "log_service.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "../task/app_tasks.h"

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

void LogService_Init(void)
{
    AppTasks_LogInit();
}

void LogService_VWrite(AppLogLevel level, const char *fmt, va_list args)
{
    if (fmt == NULL) {
        return;
    }

    char msg[192];
    char line[256];

    (void)vsnprintf(msg, sizeof(msg), fmt, args);
    int n = snprintf(line, sizeof(line), "%lu %-5s %s\r\n",
                     (unsigned long)HAL_GetTick(),
                     level_name(level),
                     msg);

    if (n <= 0) {
        return;
    }

    (void)AppTasks_LogWriteText(line, bounded_strlen(line, sizeof(line)));
}

void LogService_Write(AppLogLevel level, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    LogService_VWrite(level, fmt, args);
    va_end(args);
}
