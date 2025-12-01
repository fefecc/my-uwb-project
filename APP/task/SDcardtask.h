#ifndef _SDCARD_H_
#define _SDCARD_H_

#include "main.h"
#include "UM960samplingtask.h"
#include "imusamplingtask.h"
#include "timestamp.h"

// SD写入融合数据封装（可扩展 UWB 等）
typedef struct __attribute__((packed)) {
    uint8_t sync1;                    // 帧头1：0xAA
    uint8_t sync2;                    // 帧头2：0x44
    uint8_t sync3;                    // 帧头3：0xB5
    utc_global_timestamp_t timestamp; // 全局时间戳
    gnss_fusion_msg_t gnss;           // GNSS融合数据
    IMUOrigData_t imu;                // IMU原始数据
    // 预留: 后续可添加 UWB 等其他传感器数据
} sd_fusion_record_t;

void FatFs_Check(void);     // 判断FatFs是否挂载成功，若没有创建FatFs则格式化SD卡
void FatFs_GetVolume(void); // 计算设备的容量，包括总容量和剩余容量
// uint8_t FatFs_FileTest(void);  // 进行文件写入和读取测试
// int16_t sd_wirte_IMU(void);
int16_t _512ByteFromImuDataFunc(void);
int16_t SDCardTaskFunc(void);
void SDMMCTask(void *argument);
void PackResult(void);

#endif /* _SDCARD_H_ */
