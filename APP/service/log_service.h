#ifndef APP_SERVICE_LOG_SERVICE_H_
#define APP_SERVICE_LOG_SERVICE_H_

#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_LOG_TRACE = 0,
    APP_LOG_DEBUG,
    APP_LOG_INFO,
    APP_LOG_WARN,
    APP_LOG_ERROR,
} AppLogLevel;

void LogService_Init(void);
void LogService_Write(AppLogLevel level, const char *fmt, ...);
void LogService_VWrite(AppLogLevel level, const char *fmt, va_list args);

#define app_log_info(...)  LogService_Write(APP_LOG_INFO, __VA_ARGS__)
#define app_log_warn(...)  LogService_Write(APP_LOG_WARN, __VA_ARGS__)
#define app_log_error(...) LogService_Write(APP_LOG_ERROR, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_LOG_SERVICE_H_ */
