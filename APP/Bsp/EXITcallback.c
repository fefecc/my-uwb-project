#include "EXITcallback.h"

#include "FreeRTOS.h"
#include "imusamplingtask.h"
#include "main.h"
#include "task.h"
#include "DW1000samplingtask.h"
#include "bphero_uwb.h"

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

    else if (GPIO_Pin == GPIO_PIN_2) {
    }

    else if (GPIO_Pin == GPIO_PIN_8) {
        uwb_isr_handler();
    }

    else {
    }
}
