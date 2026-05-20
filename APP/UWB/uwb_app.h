#ifndef APP_UWB_UWB_APP_H_
#define APP_UWB_UWB_APP_H_

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"
#include "uwb_stack_types.h"
#include "uwb_link.h"

#ifdef __cplusplus
extern "C" {
#endif

bool UwbApp_Init(const UwbStackConfig *cfg);
bool UwbApp_StartThread(const osThreadAttr_t *attr);
bool UwbApp_RequestProxBuild(void);
void UwbApp_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* APP_UWB_UWB_APP_H_ */
