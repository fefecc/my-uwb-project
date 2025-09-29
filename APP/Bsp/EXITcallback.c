#include "EXITcallback.h"

#include "FreeRTOS.h"
#include "imusamplingtask.h"
#include "main.h"
#include "task.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
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

  else if (GPIO_Pin == GPIO_PIN_0) {
  }

  else {
  }
}
