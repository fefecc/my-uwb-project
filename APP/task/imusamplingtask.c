#include "imusamplingtask.h"

#include "bsp_asm330.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "stdio.h"
#include "string.h"
#include "task.h"
#include "timestamptask.h"

extern stmdev_ctx_t dev_ctx; // imu设备

// 设置asm330的模式
asm_conf asm_config = {.xl_odr = ASM330LHH_XL_ODR_208Hz,
                       .g_odr  = ASM330LHH_GY_ODR_208Hz,
                       .xl_fs  = ASM330LHH_2g,
                       .g_fs   = ASM330LHH_2000dps,
                       .dr_p   = ASM330LHH_DRDY_PULSED,
                       .den    = ASM330LHH_EDGE_TRIGGER,
                       .hp     = ASM330LHH_LP_ODR_DIV_100};

static uint8_t dmaBuffer[12] __ALIGNED(4); // 读入数据的缓存区

static IMUOrigData_t imuData;       // imu原始数据
static timestamp_def timestampOrig; // 时间戳原始数据

QueueHandle_t xIMUDataQueue              = NULL; // 创建队列来完成数据的传输
TaskHandle_t imusamplingTaskNotifyHandle = NULL; // 创建imu采样线程句柄

int16_t IMUSamplingTaskFunc(void *argument)
{
    imusamplingTaskNotifyHandle =
        xTaskGetCurrentTaskHandle(); // 获取当前线程句柄

    static int16_t IMUInitResult;

    IMUInitResult = Imu_Init();

    while (!IMUInitResult) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        asm330lhh_acceleration_raw_get(&dev_ctx,
                                       (int16_t *)dmaBuffer); // 读取加速度计
        asm330lhh_angular_rate_raw_get(&dev_ctx,
                                       (int16_t *)(dmaBuffer + 6)); // 读取陀螺仪

        memcpy(imuData.accel, dmaBuffer, 6);
        memcpy(imuData.gyro, dmaBuffer + 6, 6);

        timestampOrig = GetCurrentTimestamp();
        imuData.sec   = timestampOrig.sec;
        imuData._50us = timestampOrig._50us;

        //  BaseType_t Xsendresult = xQueueSend(xIMUDataQueue, &imuData, 0);
        // if (Xsendresult != pdPASS) {
        //     printf("sampling queue full\r\n");
        // }
        osDelay(1);
    }

    for (;;) {
    }

    return 0;
}

// 初始化，如果有系统的存在，请在系统初始化之前完成初始化
int16_t Imu_Init(void)
{
    Asm330_Drive_Init(); // 驱动库初始化代码

    // 软件复位
    static uint8_t rst;
    asm330lhh_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
        asm330lhh_reset_get(&dev_ctx, &rst);
    } while (rst);

    /* Start device configuration. */
    asm330lhh_device_conf_set(&dev_ctx, PROPERTY_ENABLE);
    /* Enable Block Data Update */
    asm330lhh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    asm330lhh_xl_data_rate_set(&dev_ctx, asm_config.xl_odr);
    asm330lhh_gy_data_rate_set(&dev_ctx, asm_config.g_odr);
    /* Set full scale */
    asm330lhh_xl_full_scale_set(&dev_ctx, asm_config.xl_fs);
    asm330lhh_gy_full_scale_set(&dev_ctx, asm_config.g_fs);

    asm330lhh_data_ready_mode_set(&dev_ctx, asm_config.dr_p);

    asm330lhh_pin_int1_route_t route_val = {0};
    route_val.int1_ctrl.int1_drdy_xl     = 1;
    route_val.int1_ctrl.int1_drdy_g      = 1;
    asm330lhh_pin_int1_route_set(&dev_ctx, &route_val);

    uint8_t IDdata;
    asm330lhh_device_id_get(&dev_ctx, &IDdata);

    if (IDdata != asm330_id) {
        printf("ASM330 Init error\r\n");
        return -1;
    }
    printf("ASM330 Init succeed\r\n");

    return 0;
}
