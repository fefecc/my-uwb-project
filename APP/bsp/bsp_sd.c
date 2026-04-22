#include "bsp_sd.h"

#include "main.h"

#define BSP_SD_DET_GPIO_PORT GPIOB
#define BSP_SD_DET_PIN       GPIO_PIN_5

bool BspSd_IsPresent(void)
{
    return HAL_GPIO_ReadPin(BSP_SD_DET_GPIO_PORT, BSP_SD_DET_PIN) == GPIO_PIN_RESET;
}
