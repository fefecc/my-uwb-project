#include "app_config.h"

#include <string.h>

#include "app_log.h"
#include "stm32h7xx_hal_flash_ex.h"

#define APP_CONFIG_MAGIC        (0x43464731UL)
#define APP_CONFIG_VERSION      (0x00010003UL)

#define APP_CONFIG_FLASH_BANK   FLASH_BANK_1
#define APP_CONFIG_FLASH_SECTOR FLASH_SECTOR_7
#define APP_CONFIG_FLASH_ADDR   (0x080E0000UL)
#define APP_CONFIG_FLASH_WORD   (32U)

static const app_config_t g_app_config_defaults = {
    .magic        = APP_CONFIG_MAGIC,
    .version      = APP_CONFIG_VERSION,
    .length       = sizeof(app_config_t),
    .pan_id       = 0xF0F0,
    .short_addr   = 0x0032,
    .frame_ctrl   = {0x41, 0x88},
    .device_role  = APP_DEVICE_ROLE_ANCHOR,
    .reserved0    = {0},
    .log_level    = LOG_LEVEL_INFO,
    .anchor_pos_x = 0.0f,
    .anchor_pos_y = 0.0f,
    .reserved     = {0},
};

static app_config_t g_cached_config;
static bool g_config_ready = false;

static void AppConfig_Copy(app_config_t *dest, const app_config_t *src)
{
    if (dest == NULL || src == NULL) {
        return;
    }
    memcpy(dest, src, sizeof(app_config_t));
}

void AppConfig_InitDefaults(app_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }

    // 直接复制编译期内置的默认配置，方便集中维护参数
    AppConfig_Copy(cfg, &g_app_config_defaults);
}

static bool AppConfig_IsValid(const app_config_t *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    if ((cfg->magic != APP_CONFIG_MAGIC) || (cfg->version != APP_CONFIG_VERSION)) {
        return false;
    }

    if (cfg->length != sizeof(app_config_t)) {
        return false;
    }

    return true;
}

const app_config_t *AppConfig_Get(void)
{
    return g_config_ready ? &g_cached_config : NULL;
}

const app_config_t *AppConfig_GetDefaults(void)
{
    return &g_app_config_defaults;
}

HAL_StatusTypeDef AppConfig_LoadFromFlash(void)
{
    // 读取flash中的参数
    const app_config_t *from_flash = (const app_config_t *)APP_CONFIG_FLASH_ADDR;
    // 先校验魔数/版本/长度，确保读到的数据可用
    if (AppConfig_IsValid(from_flash)) {
        AppConfig_Copy(&g_cached_config, from_flash);
        g_config_ready = true;
        log_info("Config loaded from flash (PAN=0x%04X, short=0x%04X)", g_cached_config.pan_id, g_cached_config.short_addr);
        return HAL_OK;
    }

    log_warn("Config flash payload invalid, falling back to defaults");
    AppConfig_InitDefaults(&g_cached_config); // 复制默认的值
    g_config_ready = true;
    return HAL_ERROR;
}

static HAL_StatusTypeDef AppConfig_EraseFlash(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error        = 0;

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Banks        = APP_CONFIG_FLASH_BANK;
    erase.Sector       = APP_CONFIG_FLASH_SECTOR;
    erase.NbSectors    = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    return HAL_FLASHEx_Erase(&erase, &sector_error);
}

HAL_StatusTypeDef AppConfig_Save(const app_config_t *cfg)
{
    if (cfg == NULL) {
        return HAL_ERROR;
    }

    app_config_t temp;
    AppConfig_Copy(&temp, cfg);

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        log_error("Failed to unlock flash (status=%lu)", status);
        return status;
    }

    status = AppConfig_EraseFlash();
    if (status != HAL_OK) {
        log_error("Flash erase failed (status=%lu)", status);
        HAL_FLASH_Lock();
        return status;
    }

    uint32_t address      = APP_CONFIG_FLASH_ADDR;
    const uint8_t *source = (const uint8_t *)&temp;
    size_t remaining      = sizeof(app_config_t);
    uint32_t flash_word[APP_CONFIG_FLASH_WORD / sizeof(uint32_t)];

    while (remaining > 0 && status == HAL_OK) {
        // H7 Flash 只支持 32 字节写入，未填满的部分要补成 0xFF
        size_t chunk = (remaining >= APP_CONFIG_FLASH_WORD) ? APP_CONFIG_FLASH_WORD : remaining;
        memset(flash_word, 0xFF, sizeof(flash_word));
        memcpy(flash_word, source, chunk);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)flash_word);
        if (status != HAL_OK) {
            log_error("Flash program failed at 0x%08lX (status=%lu)", address, status);
            break;
        }
        address += APP_CONFIG_FLASH_WORD;
        source += chunk;
        remaining -= chunk;
    }

    HAL_FLASH_Lock();

    if (status == HAL_OK) {
        AppConfig_Copy(&g_cached_config, &temp);
        g_config_ready = true;
        log_info("Config saved to flash");
    }

    return status;
}

HAL_StatusTypeDef AppConfig_SaveDefaults(void)
{
    app_config_t defaults;
    AppConfig_InitDefaults(&defaults);
    return AppConfig_Save(&defaults);
}

void AppConfig_Print(const app_config_t *cfg)
{
    if (cfg == NULL) {
        log_error("Cannot print config: NULL pointer");
        return;
    }

    // 通过串口输出全部关键配置，方便调试
    log_info("Configuration dump:");
    log_info(" - PAN ID: 0x%04X", cfg->pan_id);
    log_info(" - Short Address: 0x%04X", cfg->short_addr);
    log_info(" - Frame Control Bytes: 0x%02X 0x%02X", cfg->frame_ctrl[0], cfg->frame_ctrl[1]);
    log_info(" - Device Role: %s (%u)",
             (cfg->device_role == APP_DEVICE_ROLE_ANCHOR) ? "Anchor" : "Tag",
             cfg->device_role);
    log_info(" - Anchor Position: (%.3f, %.3f)", cfg->anchor_pos_x, cfg->anchor_pos_y);
    log_info(" - Log level: %lu", cfg->log_level);
}
