#ifndef BPHERO_UWB_H
#define BPHERO_UWB_H

#include "deca_device_api.h"
#include "main.h"

#define ADDR1
// #define ADDR2

#define SHORT_ADDR  0x0032
#define SHORT_ADDR1 0x0032 // 表示一台地址 ，1 ，2分别进行通信，完成测距
#define SHORT_ADDR2 0x0033

#ifndef SPEED_OF_LIGHT
#define SPEED_OF_LIGHT (299702547.0) // in m/s in air
#endif

/* Buffer to store received frame. See NOTE 1 below. */
#ifndef FRAME_LEN_MAX
#define FRAME_LEN_MAX 127
#endif

#ifndef TX_ANT_DLY
#define TX_ANT_DLY 0
#endif

#ifndef RX_ANT_DLY
#define RX_ANT_DLY 32985 // 32950
#endif

void BPhero_UWB_InitWithProfile(const dwt_config_t *config, uint16_t pan_id, uint16_t short_addr);

#endif
