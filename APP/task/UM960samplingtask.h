#ifndef _GNSS_H_
#define _GNSS_H_

#include "FreeRTOS.h"
#include "main.h"
#include "queue.h"
#include "task.h"

typedef struct __attribute__((packed)) {
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

// 1. 定义消息的结构
typedef struct {
  uint8_t* pData;  // 指向接收到的数据的指针
  size_t length;   // 这批数据的长度
} GNSS_Message_t;

extern QueueHandle_t xUM960SamplingQueue;
extern TaskHandle_t UM960samplingTaskNotifyHandle;
extern QueueHandle_t gnss_data_queue;

int16_t GNSSInit(void);
void IRQHandlerFunc(void);
void my_gnss_message_handler(uint16_t msg_id, const uint8_t* payload,
                             uint16_t length);
void UM960SamplingTaskFunc(void);

#endif