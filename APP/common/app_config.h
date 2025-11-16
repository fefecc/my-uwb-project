#ifndef APP_COMMON_APP_CONFIG_H_
#define APP_COMMON_APP_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

typedef enum {
    APP_DEVICE_ROLE_TAG    = 0, // 移动端 (Tag)
    APP_DEVICE_ROLE_ANCHOR = 1, // 基站 (Anchor)
} app_device_role_t;

typedef struct {
    uint32_t magic;        // Signature to validate flash contents
    uint32_t version;      // Increment when layout changes
    uint32_t length;       // Size of this structure for sanity checking
    uint16_t pan_id;       // UWB PAN identifier
    uint16_t short_addr;   // UWB short address
    uint8_t frame_ctrl[2]; // IEEE 802.15.4 frame control bytes
    uint8_t device_role;   // 0 = Tag, 1 = Anchor
    uint8_t reserved0[1];  // Alignment padding / future use
    uint32_t log_level;    // Preferred runtime log verbosity
    uint32_t reserved[6];  // Reserved for future extensions
} app_config_t;

HAL_StatusTypeDef AppConfig_LoadFromFlash(void);
HAL_StatusTypeDef AppConfig_SaveDefaults(void);
HAL_StatusTypeDef AppConfig_Save(const app_config_t *cfg);
const app_config_t *AppConfig_Get(void);
const app_config_t *AppConfig_GetDefaults(void);
void AppConfig_InitDefaults(app_config_t *cfg);
void AppConfig_Print(const app_config_t *cfg);

#endif /* APP_COMMON_APP_CONFIG_H_ */
