/**
 * @file log_hal.c
 * @brief STM32 HAL 日志库实现（纯宏控制版）
 */

#include "log_hal.h"

static const char *const LOG_LEVEL_STR[] = {"TRACE", "DEBUG", "INFO",
                                            "WARN",  "ERROR", "FATAL"};

#ifdef LOG_USE_COLOR
static const char *const LOG_LEVEL_COLOR[] = {
    "\x1b[94m", "\x1b[36m", "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[35m"};
#endif

void log_write_impl(LogLevel level, const char *file, int line, const char *fmt,
                    ...) {
#ifdef LOG_USE_LOCK
  LOG_LOCK(true);
#endif

  char msg[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);

  uint32_t ms = HAL_GetTick() % 1000;
  uint32_t sec = HAL_GetTick() / 1000;
  uint32_t s = sec % 60;
  uint32_t m = (sec / 60) % 60;
  uint32_t h = (sec / 3600) % 24;

  char buf[320];
#ifdef LOG_USE_COLOR
  snprintf(buf, sizeof(buf),
           "%s%02lu:%02lu:%02lu.%03lu %-5s\x1b[0m %s:%d: %s\r\n",
           LOG_LEVEL_COLOR[level], (unsigned long)h, (unsigned long)m,
           (unsigned long)s, (unsigned long)ms, LOG_LEVEL_STR[level], file,
           line, msg);
#else
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu.%03lu %-5s %s:%d: %s\r\n",
           (unsigned long)h, (unsigned long)m, (unsigned long)s,
           (unsigned long)ms, LOG_LEVEL_STR[level], file, line, msg);
#endif

  HAL_UART_Transmit(LOG_UART_HANDLE, (uint8_t *)buf, strlen(buf), 100);

#ifdef LOG_USE_LOCK
  LOG_LOCK(false);
#endif
}
