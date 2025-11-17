#include "uwb_timestamp.h"

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include <string.h>

#define UWB_TIMESTAMP_LEN 5U
#define UWB_DWT_TIME_UNITS (1.0 / (499.2e6 * 128.0))

static int uwb_timestamp_read_reg_isr(uint16_t reg_id, uint16_t offset, volatile uwb_timestamp_t *ts)
{
    if (ts == NULL) {
        return -1;
    }
    uint8_t raw[UWB_TIMESTAMP_LEN] = {0};
    int status                     = dwt_readfromdeviceFromISR(reg_id, offset, UWB_TIMESTAMP_LEN, raw);
    if (status == 0) {
        for (uint32_t i = 0; i < UWB_TIMESTAMP_LEN; ++i) {
            ts->bytes[i] = raw[i];
        }
    }
    return status;
}

int uwb_timestamp_read_rx_isr(volatile uwb_timestamp_t *ts)
{
    return uwb_timestamp_read_reg_isr(RX_TIME_ID, 0, ts);
}

int uwb_timestamp_read_tx_isr(volatile uwb_timestamp_t *ts)
{
    return uwb_timestamp_read_reg_isr(TX_TIME_ID, 0, ts);
}

uint64_t uwb_timestamp_to_u64(const uwb_timestamp_t *ts)
{
    return ts ? (ts->value & ((1ULL << 40) - 1ULL)) : 0;
}

void uwb_timestamp_from_u64(uint64_t value, uwb_timestamp_t *ts)
{
    if (ts) {
        ts->value       = 0;
        uint64_t masked = value & ((1ULL << 40) - 1ULL);
        for (uint32_t i = 0; i < UWB_TIMESTAMP_LEN; ++i) {
            ts->bytes[i] = (uint8_t)(masked & 0xFFU);
            masked >>= 8;
        }
    }
}

uwb_timestamp_t uwb_timestamp_add_delay_ms(const uwb_timestamp_t *base, uint16_t delay_ms)
{
    uwb_timestamp_t result;
    uwb_timestamp_from_u64(0, &result);

    uint64_t base_val = uwb_timestamp_to_u64(base);
    double delay_units = ((double)delay_ms / 1000.0) / UWB_DWT_TIME_UNITS;
    uint64_t delay_ticks = (uint64_t)(delay_units);
    uint64_t future      = (base_val + delay_ticks) & ((1ULL << 40) - 1ULL);

    uwb_timestamp_from_u64(future, &result);
    return result;
}

uint64_t uwb_timestamp_diff(const uwb_timestamp_t *end, const uwb_timestamp_t *start)
{
    const uint64_t mask = (1ULL << 40) - 1ULL;
    uint64_t end_val   = uwb_timestamp_to_u64(end);
    uint64_t start_val = uwb_timestamp_to_u64(start);
    return (end_val - start_val) & mask;
}
