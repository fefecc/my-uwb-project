#ifndef APP_COMMON_APP_TYPES_H_
#define APP_COMMON_APP_TYPES_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../service/time_service.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_MODE_RUN = 0,
    APP_MODE_CONFIG,
} AppMode;

typedef enum {
    APP_ROLE_TAG    = 0,
    APP_ROLE_ANCHOR = 1,
} AppDeviceRole;

typedef enum {
    APP_DATA_SRC_NONE            = 0,
    APP_DATA_SRC_GNSS            = 1,
    APP_DATA_SRC_IMU             = 2,
    APP_DATA_SRC_UWB_TWR         = 3,
    APP_DATA_SRC_UWB_ANCHOR_DATA = 4,
} AppDataSource;

/* UWB TWR result status bits.
 * DS_SHORT and DS_LONG are mutually exclusive and only valid with TWR_DS. */
#define APP_UWB_STATUS_FLAG_TWR_DS       0x0001U
#define APP_UWB_STATUS_FLAG_TWR_DS_SHORT 0x0002U
#define APP_UWB_STATUS_FLAG_TWR_DS_LONG  0x0004U

typedef struct {
    double lat;
    double lon;
    double hgt;
    uint32_t datum_id;
    float lat_std;
    float lon_std;
    float hgt_std;
    uint32_t pos_status;
    uint32_t pos_type;
    float diff_age;
    float sol_age;
    uint8_t svs_tracked;
    uint8_t svs_in_sol;
} AppGnssSample;

typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
} AppImuSample;

#define APP_UWB_ANCHOR_DATA_MAX_ENTRIES (8U)

typedef struct {
    uint16_t anchor_id;
    uint16_t tag_id;
    uint16_t exchange_seq;
    uint16_t status_flags;
    double distance_m;
    uint16_t rx_pacc;
    uint16_t fp_index;
    uint16_t fp_ampl1;
    uint16_t fp_ampl2;
    uint16_t fp_ampl3;
    uint16_t std_noise;
    uint16_t max_noise;
} AppUwbTwrSample;

typedef struct {
    uint16_t self_anchor;
    uint16_t peer_anchor;
    uint16_t dist_cm;
    uint16_t dist_std_cm;
    uint16_t avg_pacc;
    uint16_t avg_fp_index;
    uint16_t avg_fp_ampl1;
    uint16_t avg_fp_ampl2;
    uint16_t avg_fp_ampl3;
    uint16_t avg_std_noise;
    uint16_t avg_max_noise;
    int16_t avg_rx_power_dbm_x100;
    int16_t avg_fp_power_dbm_x100;
    uint8_t samples;
    uint8_t quality;
    uint8_t flags;
    uint32_t rx_error_flags;
    uint16_t lde_status;
} AppUwbAnchorEntry;

typedef struct {
    uint16_t source_anchor_id;
    uint16_t table_seq;
    uint16_t table_crc;
    uint16_t total_len;
    uint8_t total_frags;
    uint8_t entry_count;
    AppUwbAnchorEntry entries[APP_UWB_ANCHOR_DATA_MAX_ENTRIES];
} AppUwbAnchorDataSample;

typedef struct {
    AppDataSource source;
    TimeCapture time_capture;
    uint32_t enqueue_seq;
    union {
        AppGnssSample gnss;
        AppImuSample imu;
        AppUwbTwrSample uwb_twr;
        AppUwbAnchorDataSample uwb_anchor_data;
    } payload;
} AppDataNode;

#ifdef __cplusplus
}
#endif

#endif /* APP_COMMON_APP_TYPES_H_ */
