#include "UM960samplingtask.h"

#include "cmsis_os.h"
#include "gnss_parser.h"
#include "stdio.h"
#include "stm32h7xx_hal_dma.h"
#include "string.h"

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;

#define GNSSUART2_RX  hdma_uart4_rx
#define GNSSUartF2    huart4
#define GNSSUSART3_RX hdma_usart3_rx
#define GNSSUartF3    huart3

#define BUFFER_SIZE   1024
static uint8_t gnss_rx_buffer[BUFFER_SIZE];
static gnss_parser_t g_gnss_parser;
static volatile uint16_t g_gnss_dma_length = 0;

QueueHandle_t xUM960SamplingQueue          = NULL;
TaskHandle_t UM960samplingTaskNotifyHandle = NULL;
QueueHandle_t gnss_data_queue              = NULL;

void my_gnss_message_handler(uint16_t msg_id, const uint8_t *payload,
                             uint16_t length)
{
    switch (msg_id) {
        case 0x0846: // BESTPOSA/BESTNAV 假定ID
            if (length == sizeof(bestnav_t)) {
                const bestnav_t *nav = (const bestnav_t *)payload;
                printf("--- BESTNAV Received ---\n");
                printf("  Position Type: %u\n", (unsigned int)nav->pos_type);
                printf("  Latitude:  %.8f\n", nav->lat);
                printf("  Longitude: %.8f\n", nav->lon);
                printf("  Height:    %.4f m\n", nav->hgt);
                printf("  SVs Tracked: %u, SVs in Solution: %u\n",
                       nav->svs_tracked, nav->svs_in_sol);
                printf("--------------------------\n\n");
            } else {
                printf("parser failed!\r\n");
            }
            break;
        default:
            break;
    }
}

int16_t GNSSInit(void)
{
    gnss_parser_init(&g_gnss_parser, my_gnss_message_handler);

    __HAL_UART_ENABLE_IT(&GNSSUartF3, UART_IT_IDLE);

    HAL_UART_Receive_DMA(&GNSSUartF3, gnss_rx_buffer, BUFFER_SIZE);

    return 0;
}

void ClearBuffer(uint8_t *buffer, size_t len)
{
    if (!buffer || len == 0) {
        return;
    }
    memset(buffer, 0, len);
}

void GNSSTask(void *argument)
{
    UM960samplingTaskNotifyHandle = xTaskGetCurrentTaskHandle();

    GNSSInit();

    while (1) {
        uint32_t dma_len = 0;
        xTaskNotifyWait(0, 0, &dma_len, portMAX_DELAY); // 等待空闲中断通知并获取长度

        if (dma_len > 0) {
            gnss_parser_process_block(&g_gnss_parser,
                                      gnss_rx_buffer, dma_len);
        }
        ClearBuffer(gnss_rx_buffer, BUFFER_SIZE); // 清空缓冲区，等待下一次读取
        HAL_UART_Receive_DMA(&GNSSUartF3, gnss_rx_buffer, BUFFER_SIZE);

        osDelay(1);
    }
}

void HAL_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    if (huart != &GNSSUartF3) {
        return;
    }

    uint32_t dma_len = (uint32_t)(BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&GNSSUartF3));

    // 空闲中断触发：停止DMA并通知解析任务
    HAL_UART_DMAStop(&GNSSUartF3);
    if (UM960samplingTaskNotifyHandle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(UM960samplingTaskNotifyHandle,
                           dma_len,
                           eSetValueWithOverwrite,
                           &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void GNSSIdleHandler(void)
{
    if (__HAL_UART_GET_FLAG(&GNSSUartF3, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&GNSSUartF3);
        uint32_t dma_len = (uint32_t)(BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&GNSSUartF3));
        HAL_UART_DMAStop(&GNSSUartF3);
        if (UM960samplingTaskNotifyHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR(UM960samplingTaskNotifyHandle,
                               dma_len,
                               eSetValueWithOverwrite,
                               &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
