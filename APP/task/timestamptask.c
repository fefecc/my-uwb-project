#include "timestamptask.h"

#include "FreeRTOS.h"
#include "task.h"
// 定时器句柄(在Cube生成的tim.c中定义)
extern TIM_HandleTypeDef htim16;

// 时间戳变量
static volatile uint64_t system_seconds = 0;

// 定时器中断回调函数，每1s触发一次
void TIM_Call_Callback(void) { system_seconds++; }

// 获取当前时间戳
timestamp_def GetCurrentTimestamp(void) {
  timestamp_def time;

  // 关中断或使用临界区确保读取的一致性
  taskENTER_CRITICAL();

  time.sec = system_seconds;

  time._50us = __HAL_TIM_GET_COUNTER(&htim16) * 0.00005;  // 0.05ms为单位

  taskEXIT_CRITICAL();

  return time;
}
