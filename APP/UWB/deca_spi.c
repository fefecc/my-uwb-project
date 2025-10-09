/*! ----------------------------------------------------------------------------
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "deca_spi.h"
#include "deca_device_api.h"
#include "deca_sleep.h"
#include "spi.h"

#define DW1000_CS_GPIO_Port GPIOB
#define DW1000_CS_Pin       GPIO_PIN_12
#define DW1000_SPI_HANDLE   &hspi2
#define SPI_TIMEOUT         0xffff
#define SPI_ERROR           -1

extern decaIrqStatus_t decamutexon(void);
extern void decamutexoff(decaIrqStatus_t s);

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(void)
{
    // done by port.c, default SPI used is SPI1

    return 0;

} // end openspi()

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
    // while (port_SPIx_busy_sending());  // wait for tx buffer to empty

    // port_SPIx_disable();

    return 0;

} // end closespi()

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi_serial(uint16 headerLength, const uint8 *headerBuffer,
                      uint32 bodylength, const uint8 *bodyBuffer)
{

    decaIrqStatus_t stat;

    stat = decamutexon();

    // 1. 开始SPI事务：将CS引脚拉低
    HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_RESET);

    // 2. 发送命令头
    // 使用(uint8_t*)进行强制类型转换以匹配HAL函数的参数类型
    if (HAL_SPI_Transmit(DW1000_SPI_HANDLE, (uint8_t *)headerBuffer, headerLength, SPI_TIMEOUT) != HAL_OK) {
        // 如果发送失败，立即结束事务并返回错误
        HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);
        return SPI_ERROR;
    }

    // 3. 发送数据体
    if (HAL_SPI_Transmit(DW1000_SPI_HANDLE, (uint8_t *)bodyBuffer, bodylength, SPI_TIMEOUT) != HAL_OK) {
        // 如果发送失败，立即结束事务并返回错误
        HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);
        return SPI_ERROR;
    }

    // 4. 结束SPI事务：将CS引脚拉高
    HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);

    decamutexoff(stat);

    return 0;
} // end writetospi()

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be
 * found, or returns -1 if there was an error
 */

int readfromspi_serial(uint16 headerLength, const uint8 *headerBuffer,
                       uint32 readlength, uint8 *readBuffer)
{

    decaIrqStatus_t stat;

    stat = decamutexon();

    HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_RESET);

    // 2. 发送命令头。在发送的同时，DW1000也会在MISO线上返回数据，但我们忽略这些数据。
    if (HAL_SPI_Transmit(DW1000_SPI_HANDLE, (uint8_t *)headerBuffer, headerLength, SPI_TIMEOUT) != HAL_OK) {
        // 如果发送失败，立即结束事务并返回错误
        HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);
        return SPI_ERROR;
    }

    // 3. 接收数据。为了接收数据，主机必须发送等量的虚拟数据(dummy bytes)来产生时钟信号。
    // HAL_SPI_Receive函数会自动处理发送虚拟数据的过程。
    if (HAL_SPI_Receive(DW1000_SPI_HANDLE, readBuffer, readlength, SPI_TIMEOUT) != HAL_OK) {
        // 如果接收失败，立即结束事务并返回错误
        HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);
        return SPI_ERROR;
    }

    // 4. 结束SPI事务：将CS引脚拉高
    HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);

    decamutexoff(stat);

    return 0;
} // end readfromspi()
