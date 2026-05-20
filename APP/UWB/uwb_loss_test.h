#ifndef APP_UWB_UWB_LOSS_TEST_H_
#define APP_UWB_UWB_LOSS_TEST_H_

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

bool UwbLossTest_Init(void);
bool UwbLossTest_StartThread(const osThreadAttr_t *attr);

bool UwbLossTest_PostRangeTx(uint8_t seq);
bool UwbLossTest_PostRangeRx(uint8_t seq,
                             uint16_t anchor_id,
                             uint8_t response_slot_id);

void UwbLossTest_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* APP_UWB_UWB_LOSS_TEST_H_ */
