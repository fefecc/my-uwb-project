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

// SD写入双缓冲配置
#define SD_WRITE_CHUNK_SIZE (16 * 1024)
#define SD_WRITE_BUF_COUNT  2
#define SD_LOG_FILE         "uwb_log.bin"

/* 双缓冲区结构体 */
typedef struct
{
    uint8_t buf[SD_WRITE_BUF_COUNT][SD_WRITE_CHUNK_SIZE]; // 两个物理缓冲区
    uint32_t write_pos;                                   // 当前写入位置
    uint8_t active;                                       // 当前在写的缓冲区索引：0 或 1
    volatile uint8_t ready[2];                            // 哪个缓冲区数据已经准备好：0/1
} double_buffer_t;

static double_buffer_t dbuf   = {0};
static const size_t frame_len = sizeof(sd_fusion_record_t);

// 排序用FIFO（按时间戳升序）
#define SD_FUSION_FIFO_CAP 256
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

static int fifo_pop(sd_fusion_record_t *rec) // 取数据
{
    if (s_fifo.count == 0) {
        return 0;
    }
    if (rec) {
        *rec = s_fifo.buf[s_fifo.tail];
    }
    s_fifo.tail = (s_fifo.tail + 1) % s_fifo.capacity;
    s_fifo.count--;
    return 1;
}

static void buffer_append_frame(const uint8_t *frame)
{
    const uint8_t *src = frame;
    size_t remaining   = frame_len;

    while (remaining > 0) {
        if (dbuf.write_pos >= SD_WRITE_CHUNK_SIZE) {
            dbuf.ready[dbuf.active] = 1;
            dbuf.active ^= 1;
            dbuf.write_pos = 0;
        }

        size_t space   = SD_WRITE_CHUNK_SIZE - dbuf.write_pos;
        size_t to_copy = (remaining < space) ? remaining : space;

        memcpy(&dbuf.buf[dbuf.active][dbuf.write_pos], src, to_copy);
        dbuf.write_pos += (uint32_t)to_copy;
        src += to_copy;
        remaining -= to_copy;

        if (dbuf.write_pos >= SD_WRITE_CHUNK_SIZE) {
            dbuf.ready[dbuf.active] = 1;
            dbuf.active ^= 1;
            dbuf.write_pos = 0;
        }
    }
}

static void flush_ready_buffers(FIL *sd_file)
{
    for (uint8_t i = 0; i < SD_WRITE_BUF_COUNT; i++) {
        if (dbuf.ready[i]) {
            dbuf.ready[i] = 0;
            UINT bw       = 0;
            FRESULT fr    = f_write(sd_file, dbuf.buf[i], SD_WRITE_CHUNK_SIZE, &bw);
            if (fr == FR_OK && bw == SD_WRITE_CHUNK_SIZE) {
                HAL_GPIO_TogglePin(SDLEDGPIOx, SDLEDPINx); // 每写入一帧闪烁LED
                f_sync(sd_file);                           // 立即刷新，防热插拔丢数据
            }
        }
    }
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

void PackResult(void)
{
    // 读取GNSS队列（非阻塞）
    gnss_fusion_record_t gnss_rec;
    while (xUM960SamplingQueue &&
           xQueueReceive(xUM960SamplingQueue, &gnss_rec, 0) == pdTRUE) {
        sd_fusion_record_t rec = {0};
        rec.sync1              = 0xAA;
        rec.sync2              = 0x44;
        rec.sync3              = 0xB5;
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
        rec.sync1              = 0xAA;
        rec.sync2              = 0x44;
        rec.sync3              = 0xB5;
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
    // 直接检测sd卡是否存在，如果没检测到就不上电，不执行后续的代码，如果检测到就直接开始初始化并执行写入的代码
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) {
        MX_SDMMC1_SD_Init();
        MX_FATFS_Init();
        osDelay(5);
        FatFs_Check();

        FIL file;
        FRESULT fr = f_open(&file, SD_LOG_FILE, FA_CREATE_ALWAYS | FA_WRITE);
        if (fr == FR_OK) {
            sd_fusion_record_t rec;
            while (1) {
                PackResult(); // 填充 s_fifo_storage

                while (fifo_pop(&rec)) {
                    buffer_append_frame((const uint8_t *)&rec);
                }
                flush_ready_buffers(&file);
                osDelay(1);
            }

            f_close(&file);
        } else {
            printf("SD 写入打开失败, f_open err=%d\r\n", fr);
        }
    }

    for (;;) {
        osDelay(1);
    }
    /* USER CODE END SDMMCTask */
}
