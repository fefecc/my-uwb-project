#include "DW1000samplingtask.h"
#include "stdio.h"
#include "bphero_uwb.h"
#include "deca_device_api.h"
#include "trilateration.h"
#include "deca_regs.h"
#include "dwm1000_timestamp.h"

// static void Handle_TimeStamp(void);

TaskHandle_t dw1000samplingTaskNotifyHandle = NULL; // 创建imu采样线程句柄

extern int rx_main(void);
extern int tx_main(void);

void DW1000samplingtask(void *argument)
{
    dw1000samplingTaskNotifyHandle = xTaskGetCurrentTaskHandle(); // 获取当前的线程句柄
    BPhero_UWB_Message_Init();
    BPhero_UWB_Init(); // dwm1000 init related

    rx_main();
    // tx_main();
}
