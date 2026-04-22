#ifndef APP_SERVICE_DATA_SERVICE_H_
#define APP_SERVICE_DATA_SERVICE_H_

#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "../common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DATA_SERVICE_QUEUE_LENGTH (64U)

bool DataService_Init(void);
bool DataService_Send(const AppDataNode *node, TickType_t timeout);
bool DataService_Receive(AppDataNode *node, TickType_t timeout);
QueueHandle_t DataService_Queue(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_DATA_SERVICE_H_ */
