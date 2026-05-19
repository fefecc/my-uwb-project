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
    APP_ROLE_TAG = 0,
    APP_ROLE_ANCHOR = 1,
} AppDeviceRole;

typedef enum {
    APP_DATA_SRC_NONE = 0,
    APP_DATA_SRC_GNSS = 1,
    APP_DATA_SRC_IMU = 2,
    APP_DATA_SRC_UWB = 3,
} AppDataSource;

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

typedef struct {
    uint16_t anchor_id;
    uint16_t tag_id;
    uint16_t exchange_seq;
    uint16_t response_slot_id;
    uint16_t status_flags;
    double distance_m;
    uint8_t retry_count;
    uint16_t rx_pacc;
    uint16_t fp_index;
    uint16_t fp_ampl1;
    uint16_t fp_ampl2;
    uint16_t fp_ampl3;
    uint16_t std_noise;
    uint16_t max_noise;
    uint64_t tag_tx_ts;
    uint64_t anchor_rx_ts;
    uint64_t anchor_tx_ts;
    uint64_t tag_rx_ts;
} AppUwbSample;

typedef struct {
    AppDataSource source;
    TimeCapture time_capture;
    union {
        AppGnssSample gnss;
        AppImuSample imu;
        AppUwbSample uwb;
    } payload;
} AppDataNode;

#ifdef __cplusplus
}
#endif

#endif /* APP_COMMON_APP_TYPES_H_ */
