/*! ----------------------------------------------------------------------------
 * @file	deca_spi.h
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

#ifndef _DECA_SPI_H_
#define _DECA_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "deca_types.h"
#include "main.h"

#define DECA_MAX_SPI_HEADER_LENGTH (3) // max number of bytes in header (for formating & sizing)

#define writetospi_serial          writetospi
#define readfromspi_serial         readfromspi

int writetospi_serial(uint16 headerLength, const uint8 *headerBuffer,
                      uint32 bodylength, const uint8 *bodyBuffer);

int readfromspi_serial(uint16 headerLength, const uint8 *headerBuffer,
                       uint32 readlength, uint8 *readBuffer);
int readfromspifromisr(uint16 headerLength, const uint8 *headerBuffer,
                       uint32 readlength, uint8 *readBuffer);
int writetospifromisr(uint16 headerLength, const uint8 *headerBuffer,
                      uint32 bodylength, const uint8 *bodyBuffer);

#ifdef __cplusplus
}
#endif

#endif /* _DECA_SPI_H_ */
