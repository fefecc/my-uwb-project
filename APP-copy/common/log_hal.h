/**
 * @file log_hal.h
 * @brief 超轻量 STM32 HAL 日志库（编译期宏控制 + 可选互斥锁）
 *
 * 特点：
 *  - 所有日志等级由宏决定是否编译
 *  - 无运行时逻辑，体积极小
 *  - 可选 ANSI 颜色
 *  - 可选互斥锁（线程安全）
 *
 * 使用示例：
 *   #define LOG_UART_HANDLE &huart2
 *   #define LOG_LEVEL_CUTOFF LOG_LEVEL_INFO
 *   #include "log_hal.h"
 *
 *   log_info("System started, tick=%lu", HAL_GetTick());
 */

#ifndef __LOG_HAL_H__
#define __LOG_HAL_H__

#include "main.h" // 按芯片修改
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "app_log.h"

/* ===== 用户可配置宏 ===== */

/** 编译期日志等级裁剪阈值 */
#ifndef LOG_LEVEL_CUTOFF
#define LOG_LEVEL_CUTOFF LOG_LEVEL_TRACE
#endif

/** 启用颜色输出 */
// #define LOG_USE_COLOR

/** 启用互斥锁（定义后需自行实现 void LOG_LOCK(bool lock)） */
// #define LOG_USE_LOCK

/* ===== 日志等级定义 ===== */
typedef enum {
    LOG_LEVEL_TRACE = 0,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_FATAL
} LogLevel;

/* ===== 函数声明 ===== */
void log_write_impl(LogLevel level, const char *file, int line, const char *fmt,
                    ...);

/* ===== 日志宏（编译期裁剪） ===== */
#if (LOG_LEVEL_CUTOFF <= LOG_LEVEL_TRACE)
#define log_trace(...) \
    log_write_impl(LOG_LEVEL_TRACE, __FILE__, __LINE__, __VA_ARGS__)
#else
#define log_trace(...) ((void)0)
#endif

#if (LOG_LEVEL_CUTOFF <= LOG_LEVEL_DEBUG)
#define log_debug(...) \
    log_write_impl(LOG_LEVEL_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#else
#define log_debug(...) ((void)0)
#endif

#if (LOG_LEVEL_CUTOFF <= LOG_LEVEL_INFO)
#define log_info(...) \
    log_write_impl(LOG_LEVEL_INFO, __FILE__, __LINE__, __VA_ARGS__)
#else
#define log_info(...) ((void)0)
#endif

#if (LOG_LEVEL_CUTOFF <= LOG_LEVEL_WARN)
#define log_warn(...) \
    log_write_impl(LOG_LEVEL_WARN, __FILE__, __LINE__, __VA_ARGS__)
#else
#define log_warn(...) ((void)0)
#endif

#if (LOG_LEVEL_CUTOFF <= LOG_LEVEL_ERROR)
#define log_error(...) \
    log_write_impl(LOG_LEVEL_ERROR, __FILE__, __LINE__, __VA_ARGS__)
#else
#define log_error(...) ((void)0)
#endif

#if (LOG_LEVEL_CUTOFF <= LOG_LEVEL_FATAL)
#define log_fatal(...) \
    log_write_impl(LOG_LEVEL_FATAL, __FILE__, __LINE__, __VA_ARGS__)
#else
#define log_fatal(...) ((void)0)
#endif

#endif /* __LOG_HAL_H__ */
