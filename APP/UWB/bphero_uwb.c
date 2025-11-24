#include "bphero_uwb.h"
#include "dw1000port.h"
#include "DW1000samplingtask.h"
#include "cmsis_os.h"
#include "app_log.h"
#include "uwb_device.h"
#include "deca_regs.h"

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

void BPhero_UWB_InitWithProfile(const dwt_config_t *user_cfg, uint16_t pan_id, uint16_t short_addr)
{
    dwt_config_t active_cfg = (user_cfg != NULL) ? *user_cfg : config;

    reset_DW1000();
    spi_set_rate_low();

    if (dwt_initialise(DWT_LOADUCODE) == -1) {
        while (1) {
            osDelay(1);
        }
    }
    spi_set_rate_high();

    dwt_configure(&active_cfg);

    // apply_dw1000_optimizations(&active_cfg);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setpanid(pan_id);
    dwt_setaddress16(short_addr);

    // configure_manual_max_tx_power(active_cfg.chan, active_cfg.prf);

    uint32_t interrupt_mask = DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RFCE |
                              DWT_INT_RXPTO | DWT_INT_SFDT; // enable interrupt
    dwt_setinterrupt(interrupt_mask, 1);
}
