#include "storage_service.h"

#include <stdio.h>
#include <string.h>

#include "fatfs.h"
#include "sdmmc.h"
#include "../bsp/bsp_sd.h"

static bool g_fatfs_linked = false;

static size_t bounded_strlen(const char *text, size_t max_len)
{
    size_t len = 0;

    if (text == NULL) {
        return 0;
    }

    while (len < max_len && text[len] != '\0') {
        len++;
    }

    return len;
}

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
                       "uwb-gnss-imu-sampling-%lu.log",
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
    FRESULT fr = f_write(file, data, (UINT)len, &written);
    return fr == FR_OK && (size_t)written == len;
}

bool StorageService_WriteNodeAscii(FIL *file, const AppDataNode *node)
{
    if (file == NULL || node == NULL) {
        return false;
    }

    char line[256];
    int n = 0;
    const TimeTimestamp *ts = &node->timestamp;
    uint32_t week = ts->utc_valid ? ts->local_utc.week : 0U;
    uint32_t week_ms = ts->utc_valid ? ts->local_utc.week_ms : 0U;

    switch (node->source) {
        case APP_DATA_SRC_GNSS:
            n = snprintf(line, sizeof(line),
                         "GNSS,%lu,%lu,0x%02lX%08lX,%.3f,%u,%.9f,%.9f,%.4f,%.4f,%.4f,%.4f,%lu,%lu,%u,%u\r\n",
                         (unsigned long)week,
                         (unsigned long)week_ms,
                         (uint32_t)(ts->local_clock.sec >> 32),
                         (uint32_t)(ts->local_clock.sec & 0xFFFFFFFF),
                         (double)ts->local_clock.ms,
                         ts->utc_valid ? 1U : 0U,
                         node->payload.gnss.lat,
                         node->payload.gnss.lon,
                         node->payload.gnss.hgt,
                         node->payload.gnss.lat_std,
                         node->payload.gnss.lon_std,
                         node->payload.gnss.hgt_std,
                         (unsigned long)node->payload.gnss.pos_status,
                         (unsigned long)node->payload.gnss.pos_type,
                         node->payload.gnss.svs_tracked,
                         node->payload.gnss.svs_in_sol);
            break;

        case APP_DATA_SRC_IMU:
            n = snprintf(line, sizeof(line),
                         "IMU,%lu,%lu,0x%02lX%08lX,%.3f,%u,%d,%d,%d,%d,%d,%d\r\n",
                         (unsigned long)week,
                         (unsigned long)week_ms,
                         (uint32_t)(ts->local_clock.sec >> 32),
                         (uint32_t)(ts->local_clock.sec & 0xFFFFFFFF),
                         (double)ts->local_clock.ms,
                         ts->utc_valid ? 1U : 0U,
                         node->payload.imu.accel[0],
                         node->payload.imu.accel[1],
                         node->payload.imu.accel[2],
                         node->payload.imu.gyro[0],
                         node->payload.imu.gyro[1],
                         node->payload.imu.gyro[2]);
            break;

        default:
            return false;
    }

    if (n <= 0) {
        return false;
    }

    return StorageService_WriteBlock(file,
                                     (const uint8_t *)line,
                                     bounded_strlen(line, sizeof(line)));
}
