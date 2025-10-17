#ifndef BPHERO_UWB_H
#define BPHERO_UWB_H

#include "frame_header.h"
#include "common_header.h"

#define ADDR1
// #define ADDR2

#define SHORT_ADDR  0x0032
#define SHORT_ADDR1 0x0032 // 表示一台地址 ，1 ，2分别进行通信，完成测距
#define SHORT_ADDR2 0x0033

extern int psduLength;
extern srd_msg_dsss msg_f_send; // ranging message frame with 16-bit addresses

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
#define RX_ANT_DLY 32950
#endif

extern uint8 rx_buffer[FRAME_LEN_MAX];
/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
/* Hold copy of frame length of frame received (if good), so reader can examine it at a breakpoint. */
extern uint16 frame_len;
void BPhero_UWB_Message_Init(void);
void BPhero_UWB_Init(void); // dwm1000 init related
void bphero_setcallbacks(void (*rxcallback)(void));

#endif
