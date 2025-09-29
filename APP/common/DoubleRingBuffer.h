#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#include "main.h"

#define size_t uint16_t
#define RB_BUFFER_COUNT 2  // 双缓冲数量固定

typedef struct {
  uint8_t *buffers[RB_BUFFER_COUNT];  // 指向内存池的两个缓冲区
  size_t bufferSize;                  // 每个缓冲区容量
  size_t writePos[RB_BUFFER_COUNT];   // 写位置
  size_t readPos[RB_BUFFER_COUNT];    // 读位置
  uint8_t activeBuf;                  // 当前活动缓冲区索引
  uint8_t bufFullFlag
      [RB_BUFFER_COUNT];  // 标记缓冲区是否写满可用（1=可用，0=不可用）
} RingBuffer;

int RB_Init(RingBuffer *rb, uint8_t *memPool, size_t bufSize);
int RB_Write(RingBuffer *rb, const uint8_t *data, size_t len);
size_t RB_Read(RingBuffer *rb, uint8_t *outBuf, size_t len);
void RB_SwitchBuffer(RingBuffer *rb);
size_t RB_GetDataLength(const RingBuffer *rb, uint8_t bufIndex);

/**
 * @brief 缓冲区是否已写满可用
 */
int RB_IsBufferFull(const RingBuffer *rb, uint8_t bufIndex);

/**
 * @brief 清除缓冲区满标志（读完数据后调用）
 */
void RB_ClearBufferFlag(RingBuffer *rb, uint8_t bufIndex);

#endif
