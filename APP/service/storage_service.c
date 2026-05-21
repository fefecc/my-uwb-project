#include "storage_service.h"

#include <stdio.h>
#include <string.h>

#include "fatfs.h"
#include "sdmmc.h"
#include "../bsp/bsp_sd.h"

static bool g_fatfs_linked = false;

bool StorageService_Mount(void)
{
    if (!BspSd_IsPresent()) {
        return false;
    }

    MX_SDMMC1_SD_Init();
    if (!g_fatfs_linked) {
        MX_FATFS_Init();
        if (retSD != 0U) {
            return false;
        }
        g_fatfs_linked = true;
    }

    return f_mount(&SDFatFS, (const TCHAR *)SDPath, 1) == FR_OK;
}

bool StorageService_OpenNextLog(FIL *file)
{
    if (file == NULL) {
        return false;
    }

    char name[48];
    for (uint32_t i = 1; i < 10000U; ++i) {
        (void)snprintf(name, sizeof(name),
                       "gnss-imu-uwb-%04lu.log",
                       (unsigned long)i);
        FILINFO info;
        if (f_stat(name, &info) == FR_NO_FILE) {
            return f_open(file, name, FA_CREATE_NEW | FA_WRITE) == FR_OK;
        }
    }

    return false;
}

bool StorageService_OpenNextUwbLog(FIL *file)
{
    if (file == NULL) {
        return false;
    }

    char name[48];
    for (uint32_t i = 1; i < 10000U; ++i) {
        (void)snprintf(name, sizeof(name),
                       "tag_uwb_log_%04lu.log",
                       (unsigned long)i);
        FILINFO info;
        if (f_stat(name, &info) == FR_NO_FILE) {
            return f_open(file, name, FA_CREATE_NEW | FA_WRITE) == FR_OK;
        }
    }

    return false;
}

bool StorageService_WriteBlock(FIL *file, const uint8_t *data, size_t len)
{
    if (file == NULL || data == NULL || len == 0U) {
        return false;
    }

    UINT written = 0;
    FRESULT fr   = f_write(file, data, (UINT)len, &written);
    return fr == FR_OK && (size_t)written == len;
}
