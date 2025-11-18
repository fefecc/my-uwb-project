#ifndef APP_UWB_UWB_DEVICE_H_
#define APP_UWB_UWB_DEVICE_H_

#include "deca_device_api.h"
#include "main.h"

#ifndef UWB_CFG_DEFAULT_CHANNEL
#define UWB_CFG_DEFAULT_CHANNEL 2
#endif

#ifndef UWB_CFG_DEFAULT_PRF
#define UWB_CFG_DEFAULT_PRF DWT_PRF_64M
#endif

#ifndef UWB_CFG_DEFAULT_PLEN
#define UWB_CFG_DEFAULT_PLEN DWT_PLEN_1024
#endif

#ifndef UWB_CFG_DEFAULT_PAC
#define UWB_CFG_DEFAULT_PAC DWT_PAC32
#endif

#ifndef UWB_CFG_DEFAULT_PREAMBLE_CODE
#define UWB_CFG_DEFAULT_PREAMBLE_CODE 9
#endif

#ifndef UWB_CFG_DEFAULT_DATA_RATE
#define UWB_CFG_DEFAULT_DATA_RATE DWT_BR_110K
#endif

#ifndef UWB_CFG_DEFAULT_PHR_MODE
#define UWB_CFG_DEFAULT_PHR_MODE DWT_PHRMODE_STD
#endif

#ifndef UWB_CFG_DEFAULT_SFD_TIMEOUT
#define UWB_CFG_DEFAULT_SFD_TIMEOUT (1025 + 64 - 32)
#endif

#ifndef UWB_CFG_DEFAULT_PAN_ID
#define UWB_CFG_DEFAULT_PAN_ID 0xF0F0
#endif

#ifndef UWB_CFG_DEFAULT_SHORT_ADDR
#define UWB_CFG_DEFAULT_SHORT_ADDR 0x0032
#endif

void UWB_DeviceInitFromConfig(void);
int configure_manual_max_tx_power(uint8_t channel, uint8_t prf);

#endif /* APP_UWB_UWB_DEVICE_H_ */
