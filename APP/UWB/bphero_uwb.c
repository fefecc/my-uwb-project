#include "bphero_uwb.h"
#include "dw1000port.h"
#include "DW1000samplingtask.h"

int psduLength = 0;
srd_msg_dsss msg_f_send; // ranging message frame with 16-bit addresses
uint8 rx_buffer[FRAME_LEN_MAX];
/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
/* Hold copy of frame length of frame received (if good), so reader can examine it at a breakpoint. */
uint16 frame_len = 0;

// 设置这个代码的协议帧
void BPhero_UWB_Message_Init(void)
{
    // set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
    msg_f_send.frameCtrl[0] = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp,启用pan压缩*/ | 0x20 /* ACK request*/;
    // source/dest addressing modes and frame version
    // msg_f.frameCtrl[0] = 0x41;
    msg_f_send.frameCtrl[1] = 0x8 /*dest extended address (16bits)*/ | 0x80 /*src extended address (16bits)*/;
    msg_f_send.panID[0]     = 0xF0;
    msg_f_send.panID[1]     = 0xF0;

    msg_f_send.seqNum                 = 0;                                                         // 接收方可以根据序列号来判断是否收到了重复的数据包（例如，由于 ACK 丢失导致的重传），从而丢弃重复包。
    msg_f_send.messageData[POLL_RNUM] = 3;                                                         // copy new range number
    msg_f_send.messageData[FCODE]     = RTLS_DEMO_MSG_ANCH_POLL;                                   // message function code (specifies if message is a poll, response or other...)
    psduLength                        = (TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC); // 帧总长度： 25

    msg_f_send.seqNum = 0; // copy sequence number and then increment

    // 设置地址  源地址表示发送方设备的地址   目标地址表示接收方设备的地址

    msg_f_send.sourceAddr[0] = SHORT_ADDR & 0xFF;        // copy the address
    msg_f_send.sourceAddr[1] = (SHORT_ADDR >> 8) & 0xFF; // copy the address

    msg_f_send.destAddr[0] = 0x01; // set the destination address (broadcast == 0xffff)
    msg_f_send.destAddr[1] = 0x01; // set the destination address (broadcast == 0xffff)
}
/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config =
    {
        2,               /* Channel number. */
        DWT_PRF_64M,     /* Pulse repetition frequency. */
        DWT_PLEN_1024,   /* Preamble length. */
        DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
        9,               /* TX preamble code. Used in TX only. */
        9,               /* RX preamble code. Used in RX only. */
        1,               /* Use non-standard SFD (Boolean) */
        DWT_BR_110K,     /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
                         //    5,               /* Channel number. */
                         //    DWT_PRF_64M,     /* Pulse repetition frequency. */
                         //    DWT_PLEN_128,    /* Preamble length. */
                         //    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
                         //    9,               /* TX preamble code. Used in TX only. */
                         //    9,               /* RX preamble code. Used in RX only. */
                         //    0,               /* Use non-standard SFD (Boolean) */
                         //    DWT_BR_6M8,      /* Data rate. */
                         //    DWT_PHRMODE_STD, /* PHY header mode. */
                         //    (129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

void BPhero_UWB_Init(void) // dwm1000 init related
{
    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */

    spi_set_rate_low();

    dwt_rxreset();

    if (dwt_initialise(DWT_LOADUCODE) == -1) {
        while (1) {
        }
    }
    // dwt_configuresleepcnt(2);
    spi_set_rate_high();
    dwt_configure(&config);
    dwt_setleds(1);

    dwt_setpanid(0xF0F0); // 设置0xf0f0的网络 pan_id

#ifdef RX_Main
    dwt_setaddress16(SHORT_ADDR + 1); // 设置uwb接受测试 16位短地址
#endif
#ifdef TX_Main
    dwt_setaddress16(SHORT_ADDR); // 设置uwb发送 16位短地址
#endif

    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
}

// 不太需要 ，函数指针
void (*bphero_rxcallback)(void) = NULL; // 初始化为NULL

void bphero_setcallbacks(void (*rxcallback)(void))
{
    bphero_rxcallback = rxcallback;
}
