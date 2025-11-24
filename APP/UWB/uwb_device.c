#include "uwb_device.h"

#include "app_config.h"
#include "app_log.h"
#include "bphero_uwb.h"
#include "DW1000samplingtask.h"
#include "deca_device_api.h"

// Default DW1000 configuration (explicit values from library)
static const dwt_config_t kDefaultDwtConfig = {
    .chan           = 5,
    .prf            = DWT_PRF_64M,
    .txPreambLength = DWT_PLEN_1024,
    .rxPAC          = DWT_PAC32,
    .txCode         = 9,
    .rxCode         = 9,
    .nsSFD          = 1,
    .dataRate       = DWT_BR_110K,
    .phrMode        = DWT_PHRMODE_STD,
    .sfdTO          = (1025 + 64 - 32),
};

extern dw1000_local_device_t local_device;

void UWB_DeviceInitFromConfig(void)
{
    dwt_config_t hw_cfg = kDefaultDwtConfig;

    // from app_config.c
    uint16_t pan_id     = 0;
    uint16_t short_addr = 0;

    const app_config_t *cfg = AppConfig_Get();
    if (cfg == NULL) {
        log_warn("AppConfig unavailable, cannot init UWB device");
        return;
    }

    local_device.frameCtrl[0] = cfg->frame_ctrl[0];
    local_device.frameCtrl[1] = cfg->frame_ctrl[1];
    local_device.pan_id       = cfg->pan_id;
    local_device.short_addr   = cfg->short_addr;
    pan_id                    = cfg->pan_id;
    short_addr                = cfg->short_addr;

    BPhero_UWB_InitWithProfile(&hw_cfg, pan_id, short_addr);
}

#define SYS_CFG_ID        0x04U
#define TX_POWER_ID       0x1EU
#define SYS_CFG_OFFSET    0x00U
#define TX_POWER_OFFSET   0x00U
#define DIS_STXP_BIT_MASK (1UL << 18)

#define PRF_16_MHZ        1
#define PRF_64_MHZ        2

int configure_manual_max_tx_power(uint8_t channel, uint8_t prf)
{
    uint32_t tx_power_value = 0;
    uint32_t sys_cfg_val    = 0;
    int status              = 0;

    if (prf == PRF_16_MHZ) {
        switch (channel) {
            case 2:
                tx_power_value = 0x75757575;
                break;
            case 3:
                tx_power_value = 0x6F6F6F6F;
                break;
            case 4:
                tx_power_value = 0x5F5F5F5F;
                break;
            case 5:
                tx_power_value = 0x48484848;
                break;
            case 7:
                tx_power_value = 0x92929292;
                break;
            default:
                return -1;
        }
    } else if (prf == PRF_64_MHZ) {
        switch (channel) {
            case 2:
                tx_power_value = 0x67676767;
                break;
            case 3:
                tx_power_value = 0x8B8B8B8B;
                break;
            case 4:
                tx_power_value = 0x9A9A9A9A;
                break;
            case 5:
                tx_power_value = 0x85858585;
                break;
            case 7:
                tx_power_value = 0xD1D1D1D1;
                break;
            default:
                return -1;
        }
    } else {
        return -2;
    }

    sys_cfg_val = dwt_read32bitreg(SYS_CFG_ID);
    sys_cfg_val |= DIS_STXP_BIT_MASK;
    status = dwt_writetodevice(SYS_CFG_ID, SYS_CFG_OFFSET, 4, (uint8_t *)&sys_cfg_val);
    if (status != 0) {
        return -3;
    }

    status = dwt_writetodevice(TX_POWER_ID, TX_POWER_OFFSET, 4, (uint8_t *)&tx_power_value);
    if (status != 0) {
        return -4;
    }

    return 0;
}
