#include "uwb_stack.h"

#include <string.h>

#include "cmsis_os2.h"
#include "uwb_app.h"
#include "uwb_buffers.h"
#include "uwb_device.h"
#include "uwb_link.h"
#include "uwb_phy.h"
#include "uwb_slots.h"
#include "../service/config_service.h"
#include "../service/log_service.h"

static bool g_uwb_started;

static bool load_stack_config(UwbStackConfig *out)
{
    if (out == NULL) {
        return false;
    }

    const AppConfig *cfg = ConfigService_Get();
    if (cfg == NULL || !ConfigService_IsValid(cfg)) {
        cfg = ConfigService_GetDefaults();
    }

    if (cfg == NULL || !ConfigService_IsValid(cfg)) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->pan_id = cfg->pan_id;
    out->short_addr = cfg->short_addr;
    out->role = (AppDeviceRole)cfg->role;
    out->frame_ctrl[0] = cfg->frame_ctrl[0];
    out->frame_ctrl[1] = cfg->frame_ctrl[1];
    return true;
}

bool UwbStack_StartFromConfig(void)
{
    if (g_uwb_started) {
        return true;
    }

    UwbStackConfig cfg;
    if (!load_stack_config(&cfg)) {
        app_log_error("UWB config unavailable");
        return false;
    }

    /* 初始化共享资源: 队列 + slot 池 (必须在各层 Init 之前) */
    UwbBuffers_Init();
    UwbSlots_Init();

    if (!UwbPhy_InitWithConfig(cfg.pan_id, cfg.short_addr,
                               cfg.role, &cfg) ||
        !UwbLink_Init(&cfg) || !UwbApp_Init(&cfg)) {
        app_log_error("UWB stack init failed");
        return false;
    }

    UWB_DeviceInitFromConfig();

    const osThreadAttr_t phy_attr = {
        .name = "uwbPhy",
        .stack_size = 1024U * 4U,
        .priority = osPriorityAboveNormal,
    };
    const osThreadAttr_t link_attr = {
        .name = "uwbLink",
        .stack_size = 1536U * 4U,
        .priority = osPriorityNormal,
    };
    const osThreadAttr_t app_attr = {
        .name = "uwbApp",
        .stack_size = 1024U * 4U,
        .priority = osPriorityNormal,
    };

    if (!UwbPhy_StartThread(&phy_attr) ||
        !UwbLink_StartThread(&link_attr) ||
        !UwbApp_StartThread(&app_attr)) {
        app_log_error("UWB thread start failed");
        return false;
    }

    g_uwb_started = true;
    app_log_info("UWB stack started pan=0x%04X short=0x%04X role=%u",
                 cfg.pan_id,
                 cfg.short_addr,
                 (unsigned)cfg.role);
    return true;
}

void UwbStack_NotifyIrqFromISR(void)
{
    UwbPhy_NotifyIrqFromISR();
}
