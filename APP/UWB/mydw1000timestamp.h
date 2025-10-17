#ifndef _MYDW1000TIMESTAMP_H_
#define _MYDW1000TIMESTAMP_H_

#include "main.h"

int16_t my_get_tx_timestamp_u8_5byte(uint8_t *timestamp);
int16_t my_get_rx_timestamp_u8_5byte(uint8_t *timestamp);
uint64_t u8_5byte_TO_u64(const uint8_t *timestamp);
uint64_t get_rx_timestamp_u64_fromisr(void);
uint64_t get_tx_timestamp_u64_fromisr(void);
int16_t mymemcopytimestamp(uint8_t *dest, volatile uint8_t *source);
uint64_t transmitDelayTime(uint64_t last_tx_timestamp, uint16_t DELAY_MS);
int16_t u64_TO_u8_5byte(uint64_t timestamp, uint8_t *buffer);
uint64_t get_timestamp_difference_u64(uint64_t end_ts, uint64_t start_ts);

#endif