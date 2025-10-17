#include "bphero_uwb.h"
#include "dw1000port.h"
#include "DW1000samplingtask.h"
#include "cmsis_os.h"

int psduLength          = 0;
srd_msg_dsss msg_f_send = {0}; // ranging message frame with 16-bit addresses

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

    psduLength = 27; // 数据帧长度为27个字节

    // msg_f_send.messageData[POLL_RNUM] = 3;                       // copy new range number
    // msg_f_send.messageData[FCODE]     = RTLS_DEMO_MSG_ANCH_POLL; // message function code (specifies if message is a poll, response or other...)
    msg_f_send.seqNum = 0; // copy sequence number and then increment
}
/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config =
    {
        2,               /* Channel number. */
        DWT_PRF_64M,     /* Pulse repetition frequency. */
        DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
        DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
        9,               /* TX preamble code. Used in TX only. */
        9,               /* RX preamble code. Used in RX only. */
        1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_110K,     /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC*/
};
extern dw1000_local_device_t local_device;
extern void apply_dw1000_optimizations(const dwt_config_t *config);
void BPhero_UWB_Init(void) // dwm1000 init related
{
    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */

    spi_set_rate_low();

    if (dwt_initialise(DWT_LOADUCODE) == -1) {
        while (1) {
            osDelay(1);
        }
    }
    // dwt_configuresleepcnt(2);
    spi_set_rate_high();

    dwt_configure(&config);
    apply_dw1000_optimizations(&config);

    dwt_setrxantennadelay(RX_ANT_DLY);

    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setpanid(0xF0F0); // 设置0xf0f0的网络 pan_id

    dwt_setaddress16(local_device.short_addr);

    // #ifdef RX_Main
    //     dwt_setaddress16(SHORT_ADDR + 1); // 设置uwb接受测试 16位短地址
    // #endif
    // #ifdef TX_Main
    //     dwt_setaddress16(SHORT_ADDR); // 设置uwb发送 16位短地址
    // #endif

    /* Apply default antenna delay value. See NOTE 1 below. */

    // #define POLL_TX_TO_RESP_RX_DLY_UUS 150
    // #define RESP_RX_TIMEOUT_UUS        2700
    // #define PRE_TIMEOUT                8

    // dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    // dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    // dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    uint32_t interrupt_mask = DWT_INT_TFRS   // 发送完成
                              | DWT_INT_RFCG // 接收成功 (CRC OK)
                              | DWT_INT_RFTO // 接收帧等待超时
                              | DWT_INT_RFCE // 接收 CRC 错误
        // | DWT_INT_RXPTO  // 前导码超时 (可选)
        // | DWT_INT_SFDT   // SFD 超时 (可选)
        // | DWT_INT_RPHE   // PHY 头错误 (可选)
        ;

    dwt_setinterrupt(interrupt_mask, 1); // 中断类型设置
}
