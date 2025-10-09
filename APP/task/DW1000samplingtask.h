#ifndef _DW1000SAMPLING_H_
#define _DW1000SAMPLING_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// #define TX_Main
#define RX_Main

typedef struct
{
    uint16_t ANCHOR_TAG; //{1 anchor 0 tag}
    uint16_t ID;         // 设备编号

} UserSet;

extern TaskHandle_t dw1000samplingTaskNotifyHandle;

void DW1000samplingtask(void *argument);
void Tx_Simple_Rx_Callback(void);
void Tx_Simple_Rx_Callback(void);
#endif