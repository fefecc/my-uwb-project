#include "dw1000port.h"

void reset_DW1000(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(DWRSTnGPIOx, DWRSTnGPIOPINx, GPIO_PIN_RESET);

    /*Configure GPIO pin : PD9 */
    GPIO_InitStruct.Pin   = DWRSTnGPIOPINx;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DWRSTnGPIOx, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = DWRSTnGPIOPINx;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DWRSTnGPIOx, &GPIO_InitStruct);
    HAL_Delay(2);
}

extern SPI_HandleTypeDef hspi2;

#define LOW_SPEED_PRESCALER  SPI_BAUDRATEPRESCALER_64 // 表示64分频 1MHz
#define HIGH_SPEED_PRESCALER SPI_BAUDRATEPRESCALER_4  // 表示4分频 16MHz

/**
 * @brief  将SPI2设置为低速模式，用于DW1000初始化。
 */
void spi_set_rate_low(void)
{
    // 等待SPI总线空闲
    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);

    // 取消初始化SPI
    HAL_SPI_DeInit(&hspi2);

    // 修改SPI句柄中的分频系数为低速配置
    hspi2.Init.BaudRatePrescaler = LOW_SPEED_PRESCALER;

    // 使用新的配置重新初始化SPI
    HAL_SPI_Init(&hspi2);
}

/**
 * @brief  将SPI2设置为高速模式，用于DW1000正常数据通信。
 */
void spi_set_rate_high(void)
{
    // 等待SPI总线空闲
    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);

    // 取消初始化SPI
    HAL_SPI_DeInit(&hspi2);

    // 修改SPI句柄中的分频系数为高速配置
    hspi2.Init.BaudRatePrescaler = HIGH_SPEED_PRESCALER;

    // 使用新的配置重新初始化SPI
    HAL_SPI_Init(&hspi2);
}