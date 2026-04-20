#ifndef APP_UWB_UWB_DEVICE_H_
#define APP_UWB_UWB_DEVICE_H_

#include "deca_device_api.h"
#include "main.h"

void UWB_DeviceInitFromConfig(void);
int configure_manual_max_tx_power(uint8_t channel, uint8_t prf);

#endif /* APP_UWB_UWB_DEVICE_H_ */
