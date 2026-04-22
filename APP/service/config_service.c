#include "config_service.h"

#include <string.h>

#include "stm32h7xx_hal_flash_ex.h"

#define CONFIG_FLASH_BANK       FLASH_BANK_1
#define CONFIG_FLASH_SECTOR     FLASH_SECTOR_7
#define CONFIG_FLASH_ADDR       (0x080E0000UL)
#define CONFIG_FLASH_WORD_BYTES (32U)

static const AppConfig g_default_config = {
    .magic = CONFIG_SERVICE_MAGIC,
    .version = CONFIG_SERVICE_VERSION,
    .length = sizeof(AppConfig),
    .pan_id = 0xF0F0U,
    .short_addr = 0x0034U,
    .frame_ctrl = {0x41U, 0x88U},
    .role = APP_ROLE_ANCHOR,
    .reserved0 = {0},
    .log_level = 2U,
    .reserved = {0},
};

static AppConfig g_config;
static bool g_config_loaded = false;

void ConfigService_InitDefaults(AppConfig *cfg)
{
    if (cfg != NULL) {
        memcpy(cfg, &g_default_config, sizeof(*cfg));
    }
}

bool ConfigService_IsValid(const AppConfig *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    if (cfg->magic != CONFIG_SERVICE_MAGIC ||
        cfg->version != CONFIG_SERVICE_VERSION ||
        cfg->length != sizeof(AppConfig)) {
        return false;
    }

    if (cfg->role != APP_ROLE_TAG && cfg->role != APP_ROLE_ANCHOR) {
        return false;
    }

    return true;
}

const AppConfig *ConfigService_Get(void)
{
    return g_config_loaded ? &g_config : NULL;
}

const AppConfig *ConfigService_GetDefaults(void)
{
    return &g_default_config;
}

HAL_StatusTypeDef ConfigService_Load(void)
{
    const AppConfig *flash_cfg = (const AppConfig *)CONFIG_FLASH_ADDR;

    if (ConfigService_IsValid(flash_cfg)) {
        memcpy(&g_config, flash_cfg, sizeof(g_config));
        g_config_loaded = true;
        return HAL_OK;
    }

    ConfigService_InitDefaults(&g_config);
    g_config_loaded = true;
    return HAL_ERROR;
}

static HAL_StatusTypeDef erase_config_sector(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = CONFIG_FLASH_BANK;
    erase.Sector = CONFIG_FLASH_SECTOR;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    return HAL_FLASHEx_Erase(&erase, &sector_error);
}

HAL_StatusTypeDef ConfigService_Save(const AppConfig *cfg)
{
    if (!ConfigService_IsValid(cfg)) {
        return HAL_ERROR;
    }

    AppConfig temp;
    memcpy(&temp, cfg, sizeof(temp));

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }

    status = erase_config_sector();
    if (status != HAL_OK) {
        (void)HAL_FLASH_Lock();
        return status;
    }

    uint32_t address = CONFIG_FLASH_ADDR;
    const uint8_t *src = (const uint8_t *)&temp;
    size_t remaining = sizeof(temp);
    uint32_t flash_word[CONFIG_FLASH_WORD_BYTES / sizeof(uint32_t)];

    while (remaining > 0U && status == HAL_OK) {
        size_t chunk = remaining > CONFIG_FLASH_WORD_BYTES ?
                       CONFIG_FLASH_WORD_BYTES : remaining;

        memset(flash_word, 0xFF, sizeof(flash_word));
        memcpy(flash_word, src, chunk);

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                   address,
                                   (uint32_t)flash_word);
        address += CONFIG_FLASH_WORD_BYTES;
        src += chunk;
        remaining -= chunk;
    }

    (void)HAL_FLASH_Lock();

    if (status == HAL_OK) {
        memcpy(&g_config, &temp, sizeof(g_config));
        g_config_loaded = true;
    }

    return status;
}

HAL_StatusTypeDef ConfigService_SaveDefaults(void)
{
    AppConfig cfg;
    ConfigService_InitDefaults(&cfg);
    return ConfigService_Save(&cfg);
}
