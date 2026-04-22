#ifndef APP_SERVICE_CONFIG_SERVICE_H_
#define APP_SERVICE_CONFIG_SERVICE_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"
#include "../common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_SERVICE_MAGIC   (0x55435731UL)
#define CONFIG_SERVICE_VERSION (0x00010000UL)

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t length;
    uint16_t pan_id;
    uint16_t short_addr;
    uint8_t frame_ctrl[2];
    uint8_t role;
    uint8_t reserved0[1];
    uint32_t log_level;
    uint32_t reserved[8];
} AppConfig;

void ConfigService_InitDefaults(AppConfig *cfg);
HAL_StatusTypeDef ConfigService_Load(void);
HAL_StatusTypeDef ConfigService_Save(const AppConfig *cfg);
HAL_StatusTypeDef ConfigService_SaveDefaults(void);
const AppConfig *ConfigService_Get(void);
const AppConfig *ConfigService_GetDefaults(void);
bool ConfigService_IsValid(const AppConfig *cfg);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_CONFIG_SERVICE_H_ */
