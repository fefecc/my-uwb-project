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
#include <string.h> //
#include "FreeRTOS.h"
#include "task.h"

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

    // 1. 创建一个足够大的临时缓冲区来合并数据
    //    最好使用动态计算或一个足够大的固定值
    uint8_t spi_buffer[headerLength + bodylength];

    // 2. 将 header 和 body 复制到临时缓冲区中
    memcpy(spi_buffer, headerBuffer, headerLength);
    memcpy(spi_buffer + headerLength, bodyBuffer, bodylength);

    stat = decamutexon();

    // 3. 开始SPI事务：将CS引脚拉低
    HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_RESET);

    // 4. 只调用一次 HAL_SPI_Transmit，发送合并后的完整数据
    if (HAL_SPI_Transmit(DW1000_SPI_HANDLE, spi_buffer, headerLength + bodylength, SPI_TIMEOUT) != HAL_OK) {
        // 如果发送失败，立即结束事务并返回错误
        HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);
        decamutexoff(stat); // 不要忘记在错误路径中也释放互斥锁
        return SPI_ERROR;
    }

    // 5. 结束SPI事务：将CS引脚拉高
    HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);

    decamutexoff(stat);

    return 0; // 返回成功
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

    // 1. 准备临时的发送和接收缓冲区，大小为“命令头 + 数据体”的总长度
    uint8_t tx_buffer[headerLength + readlength];
    uint8_t rx_buffer[headerLength + readlength];

    // 2. 构建完整的发送缓冲区
    //    首先复制命令头
    memcpy(tx_buffer, headerBuffer, headerLength);
    //    然后在后面填充用于产生时钟的虚拟字节 (通常为0)
    memset(tx_buffer + headerLength, 0x00, readlength);

    stat = decamutexon();

    // 3. 开始SPI事务：将CS引脚拉低
    HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_RESET);

    // 4. 只调用一次 HAL_SPI_TransmitReceive() 来完成整个“发送命令+接收数据”的过程
    //    这保证了整个事务是连续且原子的
    if (HAL_SPI_TransmitReceive(DW1000_SPI_HANDLE, tx_buffer, rx_buffer, headerLength + readlength, SPI_TIMEOUT) != HAL_OK) {
        // 如果失败，立即结束事务并返回错误
        HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);
        decamutexoff(stat); // 不要忘记在错误路径中也释放互斥锁
        return SPI_ERROR;
    }

    // 5. 结束SPI事务：将CS引脚拉高
    HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);

    decamutexoff(stat);

    // 6. 从完整的接收缓冲区中，只复制出我们需要的数据体部分
    //    因为在发送命令头的 headerLength 期间，接收到的数据是无用的
    memcpy(readBuffer, rx_buffer + headerLength, readlength);

    return 0; // 返回成功
} // end readfromspi()

//***************************************************************************************************** */
//***************************************************************************************************** */
//***************************************************************************************************** */
//***************************************************************************************************** */
//***************************************************************************************************** */
// 中断安全版
int readfromspifromisr(uint16 headerLength, const uint8 *headerBuffer,
                       uint32 readlength, uint8 *readBuffer)
{

    // 1. 准备临时的发送和接收缓冲区，大小为“命令头 + 数据体”的总长度
    uint8_t tx_buffer[headerLength + readlength];
    uint8_t rx_buffer[headerLength + readlength];

    // 2. 构建完整的发送缓冲区
    //    首先复制命令头
    memcpy(tx_buffer, headerBuffer, headerLength);
    //    然后在后面填充用于产生时钟的虚拟字节 (通常为0)
    memset(tx_buffer + headerLength, 0x00, readlength);

    UBaseType_t uxSavedInterruptStatus;

    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    // 3. 开始SPI事务：将CS引脚拉低
    HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_RESET);

    // 4. 只调用一次 HAL_SPI_TransmitReceive() 来完成整个“发送命令+接收数据”的过程
    //    这保证了整个事务是连续且原子的
    if (HAL_SPI_TransmitReceive(DW1000_SPI_HANDLE, tx_buffer, rx_buffer, headerLength + readlength, SPI_TIMEOUT) != HAL_OK) {
        // 如果失败，立即结束事务并返回错误
        HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return SPI_ERROR;
    }

    // 5. 结束SPI事务：将CS引脚拉高
    HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);

    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

    // 6. 从完整的接收缓冲区中，只复制出我们需要的数据体部分
    //    因为在发送命令头的 headerLength 期间，接收到的数据是无用的
    memcpy(readBuffer, rx_buffer + headerLength, readlength);

    return 0; // 返回成功
} // end readfromspifromisr()

int writetospifromisr(uint16 headerLength, const uint8 *headerBuffer,
                      uint32 bodylength, const uint8 *bodyBuffer)
{

    // 1. 创建一个足够大的临时缓冲区来合并数据
    //    最好使用动态计算或一个足够大的固定值
    uint8_t spi_buffer[headerLength + bodylength];

    // 2. 将 header 和 body 复制到临时缓冲区中
    memcpy(spi_buffer, headerBuffer, headerLength);
    memcpy(spi_buffer + headerLength, bodyBuffer, bodylength);

    UBaseType_t uxSavedInterruptStatus;

    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    // 3. 开始SPI事务：将CS引脚拉低
    HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_RESET);

    // 4. 只调用一次 HAL_SPI_Transmit，发送合并后的完整数据
    if (HAL_SPI_Transmit(DW1000_SPI_HANDLE, spi_buffer, headerLength + bodylength, SPI_TIMEOUT) != HAL_OK) {
        // 如果发送失败，立即结束事务并返回错误
        HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return SPI_ERROR;
    }

    // 5. 结束SPI事务：将CS引脚拉高
    HAL_GPIO_WritePin(DW1000_CS_GPIO_Port, DW1000_CS_Pin, GPIO_PIN_SET);

    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

    return 0; // 返回成功
} // end writetospifromisr()