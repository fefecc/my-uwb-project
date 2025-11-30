#include "SDcardtask.h"

#include "DoubleRingBuffer.h"
#include "FreeRTOS.h"
#include "fatfs.h"
#include "ff.h"
#include "ffconf.h"
#include "imudatadealtask.h"
#include "imusamplingtask.h"
#include "UM960samplingtask.h"
#include "queue.h"
#include "stdio.h"
#include "string.h"
#include "task.h"
#include "sdmmc.h"

// 构造内存池的大小的构建，内存数据的分配
#define SDLength     512
#define SDPoolLength SDLength * 2

#define SDLEDGPIOx   LED0_GPIO_Port
#define SDLEDPINx    LED0_Pin

// 排序用FIFO（按时间戳升序）
#define SD_FUSION_FIFO_CAP 1024
typedef struct {
    sd_fusion_record_t *buf;
    uint16_t capacity;
    uint16_t head; // 写入位置
    uint16_t tail; // 读出位置
    uint16_t count;
} sd_fusion_fifo_t;

static sd_fusion_record_t s_fifo_storage[SD_FUSION_FIFO_CAP];

static sd_fusion_fifo_t s_fifo = {
    .buf      = s_fifo_storage,
    .capacity = SD_FUSION_FIFO_CAP,
    .head     = 0,
    .tail     = 0,
    .count    = 0,
};

static void fifo_insert(const sd_fusion_record_t *rec)
{
    if (!rec) {
        return;
    }

    if (s_fifo.count >= s_fifo.capacity) {
        // 丢弃最旧：尾指针前移一位
        s_fifo.tail = (s_fifo.tail + 1) % s_fifo.capacity;
        s_fifo.count--;
        printf("sort fifo full\r\n");
    }

    s_fifo.buf[s_fifo.head] = *rec;
    s_fifo.head             = (s_fifo.head + 1) % s_fifo.capacity;
    s_fifo.count++;
}

//	函数：FatFs_Check
//	功能：进行FatFs文件系统的挂载

void FatFs_Check(void) // 判断Sd是否挂载在fatfs总线上
{
    BYTE work[_MAX_SS];
    uint8_t MyFile_Res;
    //	FATFS_LinkDriver(&SD_Driver, SDPath);	  // 初始化驱动
    MyFile_Res = f_mount(&SDFatFS, (const TCHAR *)SDPath, 1); //	挂载SD卡

    if (MyFile_Res == FR_OK) // 判断是否挂载成功
    {
        // f_setlabel("UFO");
        printf("\r\nSD文件系统挂载成功\r\n");
    } else {
        printf("SD卡还未创建文件系统，即将格式化\r\n");

        MyFile_Res = f_mkfs("0:", FM_FAT32, 0, work,
                            sizeof work); // 格式化SD卡，FAT32，簇默认大小16K

        if (MyFile_Res == FR_OK) // 判断是否格式化成功
            printf("SD卡格式化成功！\r\n");
        else
            printf("格式化失败，请检查或更换SD卡！\r\n");
    }
}
//	函数：FatFs_GetVolume
//	功能：计算设备的容量，包括总容量和剩余容量
//

void FatFs_GetVolume(void) // 计算设备容量
{
    FATFS *fs;                    // 定义结构体指针
    uint32_t SD_CardCapacity = 0; // SD卡的总容量
    uint32_t SD_FreeCapacity = 0; // SD卡空闲容量

    DWORD fre_clust, fre_sect, tot_sect; // 空闲簇，空闲扇区数，总扇区数

    f_getfree((const TCHAR *)SDPath, &fre_clust, &fs); // 获取SD卡剩余的簇

    tot_sect = (fs->n_fatent - 2) *
               fs->csize;             // 总扇区数量 = 总的簇 * 每个簇包含的扇区数
    fre_sect = fre_clust * fs->csize; // 计算剩余的可用扇区数

    SD_CardCapacity = tot_sect / 2048; // SD卡总容量 = 总扇区数 * 512(
                                       // 每扇区的字节数 ) / 1048576(换算成MB)
    SD_FreeCapacity = fre_sect / 2048; // 计算剩余的容量，单位为M
    printf("-------------------获取设备容量信息-----------------\r\n");
    printf("SD容量：%ldMB\r\n", SD_CardCapacity);
    printf("SD剩余：%ldMB\r\n", SD_FreeCapacity);
}

int16_t SDCardTaskFunc(void)
{
    static MsgIMU_t MsgSD = {0}; // 数据包结构
    // 文件管理
    FIL MyFile;         // 文件对象
    uint8_t MyFile_Res; // 检查文件函数的检查值
    UINT MyFile_Num;    // 写入数据的长度

    // 缓冲区
    static RingBuffer rb;
    static uint8_t ToSDdataPool[SDPoolLength]; // 构造内存池，用于挂载ringbuffer
    uint8_t *FileWriteBufferPoint;

    static uint8_t FullBufferIndex = 0; // 0表示都不满，1-2分别表示两段满
    static uint8_t WriteToSdData[SDLength];
    static uint16_t bufferDataLength;

    // 初始化内存池
    if (RB_Init(&rb, ToSDdataPool, SDLength) != 0) {
        printf("RingBuffer 初始化失败！\n");
        return -1;
    }

    MyFile_Res = f_open(
        &MyFile, "test11.29.txt",
        FA_CREATE_ALWAYS | FA_WRITE); // 打开文件，若不存在,则在sd卡中，创建文件

    if (MyFile_Res == FR_OK) {
        printf("文件打开/创建成功，准备写入数据...\r\n");

        while (1) {
            // xQueueReceive(IMUDataToSDTaskQueue, &MsgSD, portMAX_DELAY);
            FileWriteBufferPoint = (uint8_t *)&MsgSD;
            RB_Write(&rb, FileWriteBufferPoint, sizeof(MsgIMU_t));

            if (RB_IsBufferFull(&rb, 0)) {
                FullBufferIndex = 1; // 1表示前端不满
            }

            else if (RB_IsBufferFull(&rb, 1)) {
                FullBufferIndex = 2; // 2表示后端不满
            }

            else {
            }

            if (FullBufferIndex) {
                bufferDataLength = RB_Read(&rb, WriteToSdData, SDLength);
                if (bufferDataLength > 0) {
                    MyFile_Res =
                        f_write(&MyFile, WriteToSdData, bufferDataLength, &MyFile_Num);

                    f_sync(&MyFile);
                    HAL_GPIO_TogglePin(SDLEDGPIOx, SDLEDPINx);
                } else {
                }

                RB_ClearBufferFlag(&rb, FullBufferIndex - 1); // 清除标志
                FullBufferIndex = 0;
                if (MyFile_Res == FR_OK) {
                } else {
                }
            }
        }

        f_close(&MyFile); // 关闭文件
    }

    else {
        printf("文件打开/创建失败...\r\n");
        return -1;
    }
    return 0;
}

void PackResult(void)
{
    // 读取GNSS队列（非阻塞）
    gnss_fusion_record_t gnss_rec;
    while (xUM960SamplingQueue &&
           xQueueReceive(xUM960SamplingQueue, &gnss_rec, 0) == pdTRUE) {
        sd_fusion_record_t rec = {0};
        rec.timestamp          = gnss_rec.ts;
        rec.gnss               = gnss_rec.data;
        memset(&rec.imu, 0, sizeof(rec.imu));
        fifo_insert(&rec);
    }

    // 读取IMU队列（非阻塞）
    imu_record_t imu_rec;
    while (xIMUDataQueue &&
           xQueueReceive(xIMUDataQueue, &imu_rec, 0) == pdTRUE) {
        sd_fusion_record_t rec = {0};
        rec.timestamp          = imu_rec.ts;
        memset(&rec.gnss, 0, sizeof(rec.gnss));
        rec.imu = imu_rec.data;
        fifo_insert(&rec);
    }
}

void SDMMCTask(void *argument)
{
    /* USER CODE BEGIN SDMMCTask */
    /* Infinite loop */
    // 直接检测sd卡是否插上，如果没有不初始化，进入死循环,如果检测到插上则直接开始初始化，执行写入的代码
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) {
        // 初始化SD卡，和数据流
        MX_SDMMC1_SD_Init();
        MX_FATFS_Init();
        osDelay(5);
        FatFs_Check();

        // 简单写入测试：创建文件 test11.29.txt 并写入 "helloworld"
        FIL file;
        UINT written    = 0;
        const char *msg = "helloworld";
        FRESULT fr      = f_open(&file, "test11.29.txt", FA_CREATE_ALWAYS | FA_WRITE);
        if (fr == FR_OK) {
            while (1) {
                PackResult();
                osDelay(1);
            }

            if (s_fifo.count > 0) {
                UINT fusion_bytes   = s_fifo.count * sizeof(sd_fusion_record_t);
                UINT fusion_written = 0;
                fr                  = f_write(&file, s_fifo.buf, fusion_bytes, &fusion_written);
                s_fifo.count        = 0;
                if (fr != FR_OK || fusion_written != fusion_bytes) {
                    printf("SD 写入融合数据失败, fr=%d, wrote=%u\r\n", fr,
                           (unsigned int)fusion_written);
                }
            }
            f_write(&file, msg, strlen(msg), &written);
            f_sync(&file);
            f_close(&file);
            printf("SD 写入测试完成: %u bytes\r\n", (unsigned int)written);
        } else {
            printf("SD 写入测试失败, f_open err=%d\r\n", fr);
        }
    }

    for (;;) {
        osDelay(1);
    }
    /* USER CODE END SDMMCTask */
}
