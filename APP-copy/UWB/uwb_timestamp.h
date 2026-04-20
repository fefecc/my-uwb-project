#ifndef APP_UWB_UWB_TIMESTAMP_H_
#define APP_UWB_UWB_TIMESTAMP_H_

#include <stdint.h>
#include "main.h"

typedef union __attribute__((packed)) {
    uint8_t bytes[5];
    uint64_t value;
} uwb_timestamp_t;

int uwb_timestamp_read_rx_isr(volatile uwb_timestamp_t *ts);
int uwb_timestamp_read_tx_isr(volatile uwb_timestamp_t *ts);
uint64_t uwb_timestamp_to_u64(const uwb_timestamp_t *ts);
void uwb_timestamp_from_u64(uint64_t value, uwb_timestamp_t *ts);
uwb_timestamp_t uwb_timestamp_add_delay_ms(const uwb_timestamp_t *base, uint16_t delay_ms);
uint64_t uwb_timestamp_diff(const uwb_timestamp_t *end, const uwb_timestamp_t *start);
uint64_t get_timestamp_difference_u64(uint64_t end_ts, uint64_t start_ts);

#endif /* APP_UWB_UWB_TIMESTAMP_H_ */
