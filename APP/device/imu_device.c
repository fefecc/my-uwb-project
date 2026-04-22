#include "imu_device.h"

#include <string.h>

#include "spi.h"

#define ASM330_CS_GPIO_PORT GPIOC
#define ASM330_CS_PIN       GPIO_PIN_4

static stmdev_ctx_t g_imu_ctx;

static const ImuDeviceConfig g_default_config = {
    .xl_odr = ASM330LHH_XL_ODR_208Hz,
    .g_odr = ASM330LHH_GY_ODR_208Hz,
    .xl_fs = ASM330LHH_2g,
    .g_fs = ASM330LHH_2000dps,
    .drdy_mode = ASM330LHH_DRDY_PULSED,
};

static void cs_set(GPIO_PinState state)
{
    HAL_GPIO_WritePin(ASM330_CS_GPIO_PORT, ASM330_CS_PIN, state);
}

static int32_t imu_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    (void)handle;

    if (buf == NULL || len == 0U) {
        return -1;
    }

    uint8_t addr = reg & 0x7FU;
    cs_set(GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi1, &addr, 1, 1000U) != HAL_OK) {
        cs_set(GPIO_PIN_SET);
        return -2;
    }
    if (HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, len, 1000U) != HAL_OK) {
        cs_set(GPIO_PIN_SET);
        return -3;
    }
    cs_set(GPIO_PIN_SET);
    return 0;
}

static int32_t imu_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    (void)handle;

    if (buf == NULL || len == 0U) {
        return -1;
    }

    uint8_t addr = reg | 0x80U;
    cs_set(GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi1, &addr, 1, 1000U) != HAL_OK) {
        cs_set(GPIO_PIN_SET);
        return -2;
    }
    if (HAL_SPI_Receive(&hspi1, buf, len, 1000U) != HAL_OK) {
        cs_set(GPIO_PIN_SET);
        return -3;
    }
    cs_set(GPIO_PIN_SET);
    return 0;
}

const ImuDeviceConfig *ImuDevice_DefaultConfig(void)
{
    return &g_default_config;
}

int32_t ImuDevice_Init(const ImuDeviceConfig *config)
{
    const ImuDeviceConfig *cfg = config != NULL ? config : &g_default_config;

    cs_set(GPIO_PIN_SET);
    memset(&g_imu_ctx, 0, sizeof(g_imu_ctx));
    g_imu_ctx.write_reg = imu_write;
    g_imu_ctx.read_reg = imu_read;
    g_imu_ctx.mdelay = HAL_Delay;

    uint8_t reset = 0;
    if (asm330lhh_reset_set(&g_imu_ctx, PROPERTY_ENABLE) != 0) {
        return -1;
    }

    do {
        (void)asm330lhh_reset_get(&g_imu_ctx, &reset);
    } while (reset != 0U);

    uint8_t id = 0;
    (void)asm330lhh_device_id_get(&g_imu_ctx, &id);
    if (id != IMU_DEVICE_ASM330_ID) {
        return -2;
    }

    (void)asm330lhh_device_conf_set(&g_imu_ctx, PROPERTY_ENABLE);
    (void)asm330lhh_block_data_update_set(&g_imu_ctx, PROPERTY_ENABLE);
    (void)asm330lhh_xl_data_rate_set(&g_imu_ctx, cfg->xl_odr);
    (void)asm330lhh_gy_data_rate_set(&g_imu_ctx, cfg->g_odr);
    (void)asm330lhh_xl_full_scale_set(&g_imu_ctx, cfg->xl_fs);
    (void)asm330lhh_gy_full_scale_set(&g_imu_ctx, cfg->g_fs);
    (void)asm330lhh_data_ready_mode_set(&g_imu_ctx, cfg->drdy_mode);

    asm330lhh_pin_int1_route_t route = {0};
    route.int1_ctrl.int1_drdy_xl = 1;
    route.int1_ctrl.int1_drdy_g = 1;
    (void)asm330lhh_pin_int1_route_set(&g_imu_ctx, &route);

    return 0;
}

int32_t ImuDevice_ReadRaw(AppImuSample *out)
{
    if (out == NULL) {
        return -1;
    }

    if (asm330lhh_acceleration_raw_get(&g_imu_ctx, out->accel) != 0) {
        return -2;
    }
    if (asm330lhh_angular_rate_raw_get(&g_imu_ctx, out->gyro) != 0) {
        return -3;
    }

    return 0;
}
