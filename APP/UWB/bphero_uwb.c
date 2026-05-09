#include "bphero_uwb.h"
#include "dw1000port.h"
#include "cmsis_os.h"
#include "uwb_device.h"
#include "deca_regs.h"

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

void pa_init_config(void)
{
    uint32_t reg;
    reg = dwt_read32bitreg(GPIO_CTRL_ID);
    reg |= 0x00014000;
    reg |= 0x00050000;
    dwt_write32bitreg(GPIO_CTRL_ID, reg);
}

static void apply_dw1000_optimizations(const dwt_config_t *config)
{
    if (config == NULL) {
        return;
    }

    dwt_writetodevice(AGC_CTRL_ID, AGC_TUNE1_OFFSET, 2, (uint8_t[]){0x9B, 0x88});
    dwt_write32bitoffsetreg(AGC_CTRL_ID, AGC_TUNE2_OFFSET, 0x2502A907);
    dwt_writetodevice(DRX_CONF_ID, DRX_TUNE2_OFFSET, 4, (uint8_t[]){0x5E, 0x01, 0x3B, 0x35});
    dwt_writetodevice(LDE_IF_ID, LDE_CFG2_OFFSET, 2, (uint8_t[]){0x07, 0x06});

    if (config->chan == 5) {
        dwt_writetodevice(RF_CONF_ID, RF_TXCTRL_OFFSET, 3, (uint8_t[]){0xE0, 0x3F, 0x1E});
        dwt_writetodevice(TX_CAL_ID, TC_PGDELAY_OFFSET, 1, (uint8_t[]){0xC0});
        dwt_writetodevice(FS_CTRL_ID, FS_PLLTUNE_OFFSET, 1, (uint8_t[]){0xBE});
    } else if (config->chan == 2) {
        dwt_writetodevice(RF_CONF_ID, RF_TXCTRL_OFFSET, 3, (uint8_t[]){0xA0, 0x5C, 0x04});
        dwt_writetodevice(TX_CAL_ID, TC_PGDELAY_OFFSET, 1, (uint8_t[]){0xC2});
        dwt_writetodevice(FS_CTRL_ID, FS_PLLTUNE_OFFSET, 1, (uint8_t[]){0x26});
    }
}

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

    pa_init_config();

    spi_set_rate_high();

    dwt_configure(&active_cfg);

    apply_dw1000_optimizations(&active_cfg);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setpanid(pan_id);
    dwt_setaddress16(short_addr);

    // dwt_enableframefilter(DWT_FF_DATA_EN);

    /* 使能完整中断掩码: TX完成 + RX好帧 + RX超时 + 所有RX错误类型 */
    uint32_t interrupt_mask = DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO |
                              DWT_INT_RFCE | DWT_INT_RPHE | DWT_INT_RFSL |
                              DWT_INT_RXOVRR | DWT_INT_SFDT | DWT_INT_ARFE;
    dwt_setinterrupt(interrupt_mask, 1);
}
