#ifndef APP_TASK_APP_TASKS_H_
#define APP_TASK_APP_TASKS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cmsis_os2.h"
#include "../common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool AppTasks_CreateAll(AppMode mode);
void AppTasks_LogInit(void);
bool AppTasks_LogWriteText(const char *text, size_t len);
bool AppTasks_LogWriteSd(const char *text, size_t len);

typedef enum {
    LED_LOSS_MODE_OFF = 0,
    LED_LOSS_MODE_CONFIRM,
    LED_LOSS_MODE_RATE,
} LedLossMode;

typedef struct {
    LedLossMode mode;
    uint8_t valid;
    uint8_t rate;
    uint8_t led0;
    uint8_t led1;
    uint8_t led2;
} LedLossCmd;

bool AppTasks_SendLedLossCmd(const LedLossCmd *cmd);

void AppTasks_NotifyImuIrqFromISR(void);
void AppTasks_NotifyKeyIrqFromISR(void);
void AppTasks_NotifyGnssDmaBlockFromISR(uint32_t len);
void AppTasks_NotifyUsartCmdDmaBlockFromISR(uint32_t len);
void USART1IdleHandler(void);
void GNSSIdleHandler(void);

void AppDefaultTask(void *argument);
void AppGnssTask(void *argument);
void AppImuTask(void *argument);
void AppDataSortTask(void *argument);
void AppSdWriterTask(void *argument);
void AppKeyTask(void *argument);
void AppLedTask(void *argument);
void AppUsartCmdTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_APP_TASKS_H_ */
