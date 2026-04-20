#include "DoubleRingBuffer.h"

#include "string.h"

int RB_Init(RingBuffer *rb, uint8_t *memPool, size_t bufSize) {
  if (!rb || !memPool || bufSize == 0) return -1;

  rb->bufferSize = bufSize;
  rb->activeBuf = 0;

  rb->buffers[0] = memPool;
  rb->buffers[1] = memPool + bufSize;

  for (int i = 0; i < RB_BUFFER_COUNT; i++) {
    rb->writePos[i] = 0;
    rb->readPos[i] = 0;
    rb->bufFullFlag[i] = 0;
    memset(rb->buffers[i], 0, bufSize);
  }
  return 0;
}

int RB_Write(RingBuffer *rb, const uint8_t *data, size_t len) {
  if (!rb || !data || len == 0) return -1;

  size_t remaining = len;
  size_t offset = 0;

  while (remaining > 0) {
    uint8_t bufIdx = rb->activeBuf;
    size_t spaceLeft = rb->bufferSize - rb->writePos[bufIdx];

    if (spaceLeft == 0) {
      // 当前缓冲区已满 -> 标志置位
      rb->bufFullFlag[bufIdx] = 1;
      // 切换到另一缓冲区
      RB_SwitchBuffer(rb);
      bufIdx = rb->activeBuf;
      spaceLeft = rb->bufferSize;
    }

    size_t toWrite = (remaining < spaceLeft) ? remaining : spaceLeft;
    memcpy(rb->buffers[bufIdx] + rb->writePos[bufIdx], data + offset, toWrite);
    rb->writePos[bufIdx] += toWrite;
    offset += toWrite;
    remaining -= toWrite;

    // 如果正好写满，则置标志可用
    if (rb->writePos[bufIdx] >= rb->bufferSize) {
      rb->bufFullFlag[bufIdx] = 1;
    }
  }

  return 0;
}

size_t RB_Read(RingBuffer *rb, uint8_t *outBuf, size_t len) {
  if (!rb || !outBuf || len == 0) return 0;

  uint8_t bufIdx = (rb->activeBuf + 1) % RB_BUFFER_COUNT;
  size_t available = rb->writePos[bufIdx] - rb->readPos[bufIdx];
  size_t toRead = (len < available) ? len : available;

  memcpy(outBuf, rb->buffers[bufIdx] + rb->readPos[bufIdx], toRead);
  rb->readPos[bufIdx] += toRead;

  if (rb->readPos[bufIdx] >= rb->writePos[bufIdx]) {
    rb->readPos[bufIdx] = 0;
    rb->writePos[bufIdx] = 0;
    rb->bufFullFlag[bufIdx] = 0;  // 清除满标志
  }

  return toRead;
}

void RB_SwitchBuffer(RingBuffer *rb) {
  rb->activeBuf = (rb->activeBuf + 1) % RB_BUFFER_COUNT;
  rb->writePos[rb->activeBuf] = 0;
  rb->readPos[rb->activeBuf] = 0;
  rb->bufFullFlag[rb->activeBuf] = 0;
}

size_t RB_GetDataLength(const RingBuffer *rb, uint8_t bufIndex) {
  if (!rb || bufIndex >= RB_BUFFER_COUNT) return 0;
  return rb->writePos[bufIndex] - rb->readPos[bufIndex];
}

int RB_IsBufferFull(const RingBuffer *rb, uint8_t bufIndex) {
  if (!rb || bufIndex >= RB_BUFFER_COUNT) return 0;
  return rb->bufFullFlag[bufIndex];
}

void RB_ClearBufferFlag(RingBuffer *rb, uint8_t bufIndex) {
  if (!rb || bufIndex >= RB_BUFFER_COUNT) return;
  rb->bufFullFlag[bufIndex] = 0;
}
