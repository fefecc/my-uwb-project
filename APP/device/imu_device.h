#ifndef APP_DEVICE_IMU_DEVICE_H_
#define APP_DEVICE_IMU_DEVICE_H_

#include <stdint.h>

#include "../../Thrid/asm330/asm330lhh_reg.h"
#include "../common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_DEVICE_ASM330_ID (0x6BU)

typedef struct {
    asm330lhh_odr_xl_t xl_odr;
    asm330lhh_odr_g_t g_odr;
    asm330lhh_fs_xl_t xl_fs;
    asm330lhh_fs_g_t g_fs;
    asm330lhh_dataready_pulsed_t drdy_mode;
} ImuDeviceConfig;

int32_t ImuDevice_Init(const ImuDeviceConfig *config);
int32_t ImuDevice_ReadRaw(AppImuSample *out);
const ImuDeviceConfig *ImuDevice_DefaultConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEVICE_IMU_DEVICE_H_ */
