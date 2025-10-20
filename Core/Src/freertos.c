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
#include "SDcardtask.h"
#include "UM960samplingtask.h"
#include "imudatadealtask.h"
#include "imusamplingtask.h"
#include "queue.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// 外设结构初始化
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* Definitions for IMUdeal */
osThreadId_t IMUDealHandle;
const osThreadAttr_t IMuUDeal_attributes = {
    .name       = "IMUDeal",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};

osThreadId_t InitHandle;
const osThreadAttr_t Init_attributes = {
    .name       = "Init",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t)osPriorityRealtime,
};

osThreadId_t dw1000samplingtaskHandle;
const osThreadAttr_t dw1000sampling_attributes = {
    .name       = "dw1000sampling",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t)osPriorityHigh,
};

void InitTask(void *argument);
void IMUDataDealTask(void *argument);

extern void DW1000samplingtask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void IMUTask(void *argument);
void SDMMCTask(void *argument);
void GNSSTask(void *argument);

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

    xIMUDataQueue = xQueueCreate(128, sizeof(IMUOrigData_t)); // 128 帧

    IMUDataToSDTaskQueue =
        xQueueCreate(64, sizeof(MsgIMU_t)); // 64 帧 一帧 49字节

    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of IMU */
    // IMUHandle = osThreadNew(IMUTask, NULL, &IMU_attributes);

    /* creation of SDMMC */
    // SDMMCHandle = osThreadNew(SDMMCTask, NULL, &SDMMC_attributes);

    /* creation of GNSS */
    // GNSSHandle = osThreadNew(GNSSTask, NULL, &GNSS_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    // IMUDealHandle            = osThreadNew(IMUDataDealTask, NULL, &IMuUDeal_attributes);
    InitHandle               = osThreadNew(InitTask, NULL, &Init_attributes);
    dw1000samplingtaskHandle = osThreadNew(DW1000samplingtask, NULL, &dw1000sampling_attributes);
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

/* USER CODE BEGIN Header_IMUTask */
/**
 * @brief Function implementing the IMU thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_IMUTask */
void IMUTask(void *argument)
{
    /* USER CODE BEGIN IMUTask */

    IMUSamplingTaskFunc(argument);
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END IMUTask */
}

/* USER CODE BEGIN Header_SDMMCTask */
/**
 * @brief Function implementing the SDMMC thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SDMMCTask */
void SDMMCTask(void *argument)
{
    /* USER CODE BEGIN SDMMCTask */
    /* Infinite loop */
    FatFs_Check();
    SDCardTaskFunc();
    // FatFs_FileTest();
    // sd_wirte_IMU();

    for (;;) {
        osDelay(1);
    }
    /* USER CODE END SDMMCTask */
}

/* USER CODE BEGIN Header_GNSSTask */
/**
 * @brief Function implementing the GNSS thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GNSSTask */
void GNSSTask(void *argument)
{
    /* USER CODE BEGIN GNSSTask */

    UM960SamplingTaskFunc();

    /* Infinite loop */
    for (;;) {
        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // UartRx_CopyToRB();

        // ParseFrames();

        osDelay(1);
    }
    /* USER CODE END GNSSTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void IMUDataDealTask(void *argument)
{
    imuDataDealTaskFunc();
    for (;;) {
        osDelay(1);
    }
}

void InitTask(void *argument)
{
    // 此线程优先级极高，在线程中完成各个中断的初始化，然后删除该线程
    //  中断初始化代码，禁止在main函数中初始化
    GNSSInit();
    HAL_TIM_Base_Start_IT(&htim16);

    // 2. 在所有准备工作都完成后，最后再使能中断
    HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    // 3. 删除任务
    osThreadTerminate(osThreadGetId());
}
/* USER CODE END Application */
