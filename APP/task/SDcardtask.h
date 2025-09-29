#ifndef _SDCARD_H_
#define _SDCARD_H_

#include "main.h"

void FatFs_Check(void);  // 判断FatFs是否挂载成功，若没有创建FatFs则格式化SD卡
void FatFs_GetVolume(void);  // 计算设备的容量，包括总容量和剩余容量
// uint8_t FatFs_FileTest(void);  // 进行文件写入和读取测试
// int16_t sd_wirte_IMU(void);
int16_t _512ByteFromImuDataFunc(void);
int16_t SDCardTaskFunc(void);

#endif /* _SDCARD_H_ */
