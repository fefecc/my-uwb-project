/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "app_log.h"
#include "SDcardtask.h"
#include "UM960samplingtask.h"
#include "imudatadealtask.h"
#include "imusamplingtask.h"
#include "queue.h"
#include "stdio.h"
#include "DW1000samplingtask.h"
#include "sdmmc.h"
#include "fatfs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

extern TIM_HandleTypeDef htim16;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* Definitions for Init */
osThreadId_t InitHandle;
const osThreadAttr_t Init_attributes = {
    .name       = "Init",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t)osPriorityRealtime,
};

/* Definitions for dw1000sampling */
osThreadId_t dw1000samplingtaskHandle;
const osThreadAttr_t dw1000sampling_attributes = {
    .name       = "dw1000sampling",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t)osPriorityHigh7,
};

/* Definitions for IMU */
osThreadId_t IMUHandle;
const osThreadAttr_t IMU_attributes = {
    .name       = "IMU",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t)osPriorityHigh,
};

/* Definitions for SDMMC */
osThreadId_t SDMMCHandle;
const osThreadAttr_t SDMMC_attributes = {
    .name       = "SDMMC",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t)osPriorityBelowNormal,
};

/* Definitions for GNSS */
osThreadId_t GNSSHandle;
const osThreadAttr_t GNSS_attributes = {
    .name       = "GNSS",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};

void InitTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */

    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */

    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */

    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */

    gnss_data_queue = xQueueCreate(10, sizeof(GNSS_Message_t));

    dw1000data_queue = xQueueCreate(64, sizeof(double)); // 发送距离数据

    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */

    const app_config_t *cfg = AppConfig_Get();

    if (cfg->device_role == APP_DEVICE_ROLE_TAG) {

        log_info("Sensor tasks enabled for TAG role");

        InitHandle               = osThreadNew(InitTask, NULL, &Init_attributes);
        IMUHandle                = osThreadNew(IMUTask, NULL, &IMU_attributes);
        SDMMCHandle              = osThreadNew(SDMMCTask, NULL, &SDMMC_attributes);
        GNSSHandle               = osThreadNew(GNSSTask, NULL, &GNSS_attributes);
        dw1000samplingtaskHandle = osThreadNew(DW1000samplingtask, NULL, &dw1000sampling_attributes);
    }

    else if (cfg->device_role == APP_DEVICE_ROLE_ANCHOR) {

        log_info("Sensor tasks enabled for ANCHOR role");

        InitHandle               = osThreadNew(InitTask, NULL, &Init_attributes);
        dw1000samplingtaskHandle = osThreadNew(DW1000samplingtask, NULL, &dw1000sampling_attributes);
        IMUHandle                = NULL;
        SDMMCHandle              = NULL;
        GNSSHandle               = NULL;
    } else {

        log_info("Sensor tasks enabled failed");

        InitHandle               = osThreadNew(InitTask, NULL, &Init_attributes);
        dw1000samplingtaskHandle = NULL;
        IMUHandle                = NULL;
        SDMMCHandle              = NULL;
        GNSSHandle               = NULL;
    }

    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */

    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void InitTask(void *argument)
{
    // 此线程优先级极高，在线程中完成各个中断的初始化，然后删除该线程
    //  中断初始化代码，禁止在main函数中初始化
    //  GNSSInit();
    HAL_TIM_Base_Start_IT(&htim16);

    // 3. 删除任务
    osThreadTerminate(osThreadGetId());
}
/* USER CODE END Application */
