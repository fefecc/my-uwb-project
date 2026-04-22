#ifndef APP_UWB_UWB_DEVICE_H_
#define APP_UWB_UWB_DEVICE_H_

#include "deca_device_api.h"
#include "main.h"

typedef struct {
    uint8_t frameCtrl[2];
    uint8_t seqNum;
    uint16_t pan_id;
    uint16_t short_addr;
} dw1000_local_device_t;

extern dw1000_local_device_t local_device;

void UWB_DeviceInitFromConfig(void);
int configure_manual_max_tx_power(uint8_t channel, uint8_t prf);

#endif /* APP_UWB_UWB_DEVICE_H_ */
