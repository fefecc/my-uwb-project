#ifndef _GNSS_H_
#define _GNSS_H_

#include "FreeRTOS.h"
#include "main.h"
#include "queue.h"
#include "task.h"
#include "timestamp.h"

typedef struct __attribute__((packed)) {
    uint8_t sync1;   // 0  十六进制 0xAA
    uint8_t sync2;   // 1  十六进制 0x44
    uint8_t sync3;   // 2  十六进制 0xB5
    uint8_t cpuIdle; // 3  CPU idle 0-100

    uint16_t messageId;     // 4  Message ID (USHORT)
    uint16_t messageLength; // 6  Message Length (USHORT)

    uint8_t timeRef;    // 8  时间系统 GPST / BDST
    uint8_t timeStatus; // 9  Time Status

    uint16_t wn; // 10 时间周 (USHORT)
    uint32_t ms; // 12 周内毫秒 (ULONG)

    uint32_t version; // 16 Release version (ULONG)

    uint8_t reserved; // 20 保留
    uint8_t leapSec;  // 21 闰秒
    uint16_t delayMs; // 22 数据输出延迟 (USHORT)
} header_t;

typedef struct __attribute__((packed)) {
    header_t header;
    uint32_t p_sol_status;
    uint32_t pos_type;
    double lat;
    double lon;
    double hgt;
    float undulation;
    uint32_t datum_id;
    float lat_std;
    float lon_std;
    float hgt_std;
    char stn_id[4];
    float diff_age;
    float sol_age;
    uint8_t svs_tracked;
    uint8_t svs_in_sol;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t ext_sol_stat;
    uint8_t gal_bds3_mask;
    uint8_t gps_glonass_bds2_mask;
    uint32_t v_sol_status;
    uint32_t vel_type;
    float latency;
    float age;
    double hor_spd;
    double trk_gnd;
    double vert_spd;
    float ver_spd_std;
    float hor_spd_std;
} bestnav_t;

// GNSS融合数据（不含时间戳）
typedef struct __attribute__((packed)) {
    double lat;
    double lon;
    double hgt;
    uint32_t datum_id;

    float lat_std;
    float lon_std;
    float hgt_std;

    uint32_t p_sol_status;
    uint32_t pos_type;
    float diff_age;
    float sol_age;
    uint8_t svs_tracked;
    uint8_t svs_in_sol;
} gnss_fusion_msg_t;

// 带时间戳的GNSS融合记录
typedef struct __attribute__((packed)) {
    utc_global_timestamp_t ts; // 时间戳
    gnss_fusion_msg_t data;    // GNSS融合数据
} gnss_fusion_record_t;

extern QueueHandle_t xUM960SamplingQueue;
extern TaskHandle_t UM960samplingTaskNotifyHandle;

int16_t GNSSInit(void);
void my_gnss_message_handler(uint16_t msg_id, const uint8_t *payload,
                             uint16_t length);
void GNSSTask(void *argument);
void DW1000samplingtask(void *argument);
void GNSSIdleHandler(void);

#endif
