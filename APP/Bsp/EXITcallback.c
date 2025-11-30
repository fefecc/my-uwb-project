#include "EXITcallback.h"

#include "FreeRTOS.h"
#include "imusamplingtask.h"
#include "main.h"
#include "task.h"
#include "DW1000samplingtask.h"
#include "bphero_uwb.h"
#include "deca_device_api.h"
#include "deca_spi.h"
#include "deca_regs.h"
#include "stdio.h"
#include "timestamp.h"

extern void uwb_isr_handler(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_4) {
        if (imusamplingTaskNotifyHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(imusamplingTaskNotifyHandle,
                                   &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }

    else if (GPIO_Pin == USER_KEY_Pin) {
        if (dw1000samplingTaskNotifyHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(dw1000samplingTaskNotifyHandle,
                                   &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }

    else if (GPIO_Pin == GPIO_PIN_8) {
        // 这个是dw1000的外部中断
        uwb_isr_handler();
    }

    else if (GPIO_Pin == GPIO_PIN_0) {
        // um960进行时间同步的操作
        PPS_IRQHandler();

    }

    else {
    }
}
