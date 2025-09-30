#ifndef _IMU_SAMPLING_TASK_H_
#define _IMU_SAMPLING_TASK_H_

#include "FreeRTOS.h"
#include "asm330lhh_reg.h"
#include "main.h"
#include "queue.h"
#include "task.h"

#define asm330_id 0x6B

// IMU原始数据结构体
typedef struct {
  int16_t accel[3];
  int16_t gyro[3];
  int64_t sec;
  float _50us;
} IMUOrigData_t;

typedef struct {
  asm330lhh_odr_xl_t xl_odr;
  asm330lhh_odr_g_t g_odr;
  asm330lhh_fs_xl_t xl_fs;
  asm330lhh_fs_g_t g_fs;
  asm330lhh_dataready_pulsed_t dr_p;
  asm330lhh_den_mode_t den;
  asm330lhh_hp_slope_xl_en_t hp;
} asm_conf;

// 系统调用
extern QueueHandle_t xIMUDataQueue;               // 外部调用的数据
extern TaskHandle_t imusamplingTaskNotifyHandle;  // 采样线程句柄

// 配置信息
extern asm_conf asm_config;

int16_t Imu_Init(void);                       // imu初始化
int16_t IMUSamplingTaskFunc(void *argument);  // imu采样线程功能函数

#endif
