/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "timestamptask.h"
#include "DW1000samplingtask.h"
#include "stdio.h"
#include "app_log.h"
#include "app_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static bool App_IsBootKeyPressed(void);
static void App_RunBootloaderMode(void);
static void App_PrintCurrentConfig(void);
static void App_StreamConfigSnapshot(const char *source_tag);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool App_IsBootKeyPressed(void)
{
    // 读取用户按键状态，用于判断是否需要强制进入 Boot 配置模式
    return HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET;
}

// 打印当前的状态，显示数据
static void App_PrintCurrentConfig(void)
{
    const app_config_t *cfg = AppConfig_Get();
    if (cfg == NULL) {
        log_error("Config cache not ready");
        return;
    }
    AppConfig_Print(cfg); // 打印当前的状态
}

// 发送log信息流的函数
static void App_StreamConfigSnapshot(const char *source_tag)
{
    const app_config_t *cfg = AppConfig_Get();
    if (cfg == NULL) {
        log_error("Config cache not ready, cannot emit snapshot");
        return;
    }

    const char *role_str =
        (cfg->device_role == APP_DEVICE_ROLE_ANCHOR) ? "Anchor" : "Tag";
    log_info("[CFG][%s] role=%s(%u) PAN=0x%04X short=0x%04X frame=0x%02X%02X pos=(%.2f,%.2f) log=%lu",
             (source_tag != NULL) ? source_tag : "AUTO", role_str, cfg->device_role,
             cfg->pan_id, cfg->short_addr, cfg->frame_ctrl[0], cfg->frame_ctrl[1],
             cfg->anchor_pos_x, cfg->anchor_pos_y, cfg->log_level);
}

static void App_RunBootloaderMode(void)
{
    log_warn("Bootloader key detected, entering configuration programming mode");

    // 按下按键后先写入一份默认配置，保证后续串口交互有可用参数
    if (AppConfig_SaveDefaults() != HAL_OK) {
        log_error("Failed to persist default configuration");
    } else {
        log_info("Default configuration stored to flash");
    }
    App_PrintCurrentConfig();
    UWBMssageInit();
    App_StreamConfigSnapshot("BOOT");

    const uint32_t cfg_emit_period_ms = 1000U;
    uint32_t next_emit_tick           = HAL_GetTick() + cfg_emit_period_ms;

    while (1) {
        uint32_t now = HAL_GetTick();

        if ((int32_t)(now - next_emit_tick) >= 0) {
            App_StreamConfigSnapshot("TIMER");
            next_emit_tick = now + cfg_emit_period_ms;
        }

        if (App_IsBootKeyPressed()) {
            HAL_Delay(50);
            if (App_IsBootKeyPressed()) {
                log_info("Key press detected, sending configuration snapshot");
                App_StreamConfigSnapshot("BUTTON");
                // 阻塞在此直到松开按键，防止多次触发
                while (App_IsBootKeyPressed()) {
                    HAL_Delay(10);
                }
                next_emit_tick = HAL_GetTick() + cfg_emit_period_ms;
            }
        }
        HAL_Delay(10);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
    // 开机时优先检查按键，若按住则直接进入配置烧写循环
    if (App_IsBootKeyPressed()) {
        // 这个地方是一个死循环，如果进入这个bootloader模式下会直接一直发送配置数据
        App_RunBootloaderMode();
    }

    HAL_StatusTypeDef cfg_status = AppConfig_LoadFromFlash();
    if (cfg_status != HAL_OK) {
        // Flash 数据非法时回退到默认配置，并尝试重新写入
        log_warn("Using built-in configuration defaults (status=%lu)", cfg_status);
        if (AppConfig_SaveDefaults() != HAL_OK) {
            log_error("Unable to save default configuration to flash"); // 表示使用默认的参数
        }
    }
    App_PrintCurrentConfig();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// *** 重要: 修改这里的 huartx 以匹配您项目中使用的 UART 句柄 ***
// 例如，如果您使用 UART1，句柄通常是 huart1
extern UART_HandleTypeDef huart1;
#define PRINTF_UART_HANDLE &huart1
// *** ---------------------------------------------------- ***

// 如果使用的是 ARM Compiler (Keil MDK)，通常需要重定义 fputc
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
int fputc(int ch, FILE *f)
{
    // 发送单个字符，使用阻塞模式，设置一个合理的超时时间
    if (HAL_UART_Transmit(PRINTF_UART_HANDLE, (uint8_t *)&ch, 1, HAL_MAX_DELAY) == HAL_OK) {
        return ch;
    } else {
        return EOF; // 返回文件结束符表示错误
    }
}

// 可选：实现 ferror 以避免库警告
int ferror(FILE *f)
{
    // 简单实现，可以根据需要扩展错误处理
    return EOF;
}

#elif defined(__GNUC__) // 如果使用的是 GCC (STM32CubeIDE, Makefiles with ARM GCC)
// 重定义 _write 函数 (用于 GCC/Newlib)
int _write(int file, char *ptr, int len)
{
    // file == 1 表示标准输出 (stdout)
    // file == 2 表示标准错误 (stderr)
    // 在嵌入式系统中，通常将 stdout 和 stderr 都重定向到同一个 UART
    if (file == 1 || file == 2) {
        HAL_StatusTypeDef status = HAL_UART_Transmit(PRINTF_UART_HANDLE, (uint8_t *)ptr, len, HAL_MAX_DELAY);

        if (status == HAL_OK) {
            return len; // 返回成功写入的字节数
        } else {

            return -1; // 返回 -1 表示错误
        }
    }
    // 对于其他文件描述符，返回错误

    return -1;
}
#endif
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
    if (htim->Instance == TIM16) {
        TIM_Call_Callback();
    }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    log_error("Error_Handler invoked");
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
