#include "UM960samplingtask.h"

#include "cmsis_os.h"
#include "gnss_parser.h"
#include "stdio.h"
#include "stm32h7xx_hal_dma.h"

#define gnss_parser_LEDGPIOx GPIOE
#define gnss_parser_LEDGPINx GPIO_PIN_5

extern DMA_HandleTypeDef hdma_usart3_rx;
#define GNSSUSART_RX hdma_usart3_rx
extern UART_HandleTypeDef huart3;
#define GNSSUartFx huart3

// 定义缓冲区的长度
// 为循环缓冲区分配静态内存
#define RING_BUFFER_SIZE 2048
static uint8_t gnss_rx_buffer[RING_BUFFER_SIZE];

// 定义全局的循环缓冲区和解析器实例
ring_buffer_t g_gnss_rb;
gnss_parser_t g_gnss_parser;

QueueHandle_t xUM960SamplingQueue = NULL;
TaskHandle_t UM960samplingTaskNotifyHandle = NULL;

// 2. 定义队列句柄
QueueHandle_t gnss_data_queue = NULL;

void UM960SamplingTaskFunc(void) {
  UM960samplingTaskNotifyHandle =
      xTaskGetCurrentTaskHandle();  // 获取当前线程句柄

  rb_init(&g_gnss_rb, gnss_rx_buffer, RING_BUFFER_SIZE);
  gnss_parser_init(&g_gnss_parser, &g_gnss_rb, my_gnss_message_handler);

  // b. 任务主循环
  for (;;) {
    // 调用解析器，它会处理缓冲区中所有的新数据
    g_gnss_rb.head =
        g_gnss_rb.size - __HAL_DMA_GET_COUNTER(&GNSSUSART_RX);  // 更新头指针
    gnss_parser_process(&g_gnss_parser);

    // 让出CPU，避免空转。
    // 10ms的延时意味着任务每秒最多轮询100次。
    osDelay(10);
  }
}

// length 数据长度
void my_gnss_message_handler(uint16_t msg_id, const uint8_t* payload,
                             uint16_t length) {
  // 根据消息ID来解析不同的消息
  switch (msg_id) {
    case 0x0846:  // 假设这是BESTPOSA消息的ID
      if (length == sizeof(bestnav_t)) {
        const bestnav_t* nav = (const bestnav_t*)payload;
        // 在这里使用解析出的数据，例如打印或更新全局变量
        // 打印一些关键信息进行验证
        HAL_GPIO_TogglePin(gnss_parser_LEDGPIOx, gnss_parser_LEDGPINx);
        printf("--- BESTNAV Received ---\n");
        printf("  Position Type: %u\n", (unsigned int)nav->pos_type);
        printf("  Latitude:  %.8f\n", nav->lat);
        printf("  Longitude: %.8f\n", nav->lon);
        printf("  Height:    %.4f m\n", nav->hgt);
        printf("  SVs Tracked: %u, SVs in Solution: %u\n", nav->svs_tracked,
               nav->svs_in_sol);
        printf("--------------------------\n\n");
      }
      break;

      // case 0x...: // 处理其他您关心的消息
      //     break;

    default:
      // 不关心的消息可以忽略
      break;
  }
}

int16_t GNSSInit(void) {
  HAL_UART_Receive_DMA(&GNSSUartFx, gnss_rx_buffer,
                       RING_BUFFER_SIZE);  // 启动 DMA 循环接收
  //__HAL_UART_ENABLE_IT(&GNSSUartFx, UART_IT_IDLE);
  // 开启空闲中断,循环解析的，中断不需要了

  return 0;
}
