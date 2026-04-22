#include "app_tasks.h"

#include "main.h"
#include "../service/time_service.h"
#include "../UWB/uwb_stack.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case GPIO_PIN_0:
            TimeService_OnPpsIrq();
            break;

        case GPIO_PIN_4:
            AppTasks_NotifyImuIrqFromISR();
            break;

        case USER_KEY_Pin:
            AppTasks_NotifyKeyIrqFromISR();
            break;

        case GPIO_PIN_8:
            UwbStack_NotifyIrqFromISR();
            break;

        default:
            break;
    }
}
