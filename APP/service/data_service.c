#include "data_service.h"

#include "task.h"

static StaticQueue_t g_data_queue_ctrl;
static uint8_t g_data_queue_storage[DATA_SERVICE_QUEUE_LENGTH * sizeof(AppDataNode)];
static QueueHandle_t g_data_queue;
static uint32_t g_data_enqueue_seq;

bool DataService_Init(void)
{
    if (g_data_queue == NULL) {
        g_data_queue       = xQueueCreateStatic(DATA_SERVICE_QUEUE_LENGTH,
                                                sizeof(AppDataNode),
                                                g_data_queue_storage,
                                                &g_data_queue_ctrl);
        g_data_enqueue_seq = 0U;
    }

    return g_data_queue != NULL;
}

bool DataService_Send(const AppDataNode *node, TickType_t timeout)
{
    if (g_data_queue == NULL || node == NULL) {
        return false;
    }

    AppDataNode queued = *node;

    taskENTER_CRITICAL();
    queued.enqueue_seq = ++g_data_enqueue_seq;
    taskEXIT_CRITICAL();

    return xQueueSend(g_data_queue, &queued, timeout) == pdPASS;
}

bool DataService_Receive(AppDataNode *node, TickType_t timeout)
{
    if (g_data_queue == NULL || node == NULL) {
        return false;
    }

    return xQueueReceive(g_data_queue, node, timeout) == pdPASS;
}

QueueHandle_t DataService_Queue(void)
{
    return g_data_queue;
}
