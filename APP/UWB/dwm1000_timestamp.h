/*! ----------------------------------------------------------------------------
 * @file	dwm1000_timestamp.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#ifndef DWM1000_TIMESTAMP_H
#define DWM1000_TIMESTAMP_H
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"
#include "deca_sleep.h"

#include "common_header.h"

uint64 get_tx_timestamp_u64(void);
uint64 get_rx_timestamp_u64(void);
void final_msg_set_ts(uint8 *ts_field, uint64 ts);
void resp_msg_get_ts(uint8 *ts_field, uint64 ts);
void final_msg_get_ts(const uint8 *ts_field, uint64 *ts);

#endif
