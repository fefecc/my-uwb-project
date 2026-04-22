#ifndef APP_UWB_UWB_STACK_H_
#define APP_UWB_UWB_STACK_H_

#include <stdbool.h>

#include "uwb_stack_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool UwbStack_StartFromConfig(void);
void UwbStack_NotifyIrqFromISR(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_UWB_UWB_STACK_H_ */
