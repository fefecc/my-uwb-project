#ifndef _IRQ_H_
#define _IRQ_H_

// 来自中断的事件通知 (使用位掩码)
typedef enum {
    UWB_EVENT_NONE           = 0,
    UWB_EVENT_TX_DONE        = (1 << 0), // 发送完成
    UWB_EVENT_RX_DONE        = (1 << 1), // 接收成功
    UWB_EVENT_RX_TIMEOUT     = (1 << 2), // 接收超时
    UWB_EVENT_RX_ERROR       = (1 << 3), // 接收出错
    UWB_EVENT_PRE_DONE       = (1 << 4), // 前导码超时
    UWB_EVENT_SFD_DONE       = (1 << 5), // SFD超时
    UWB_EVENT_FRAME_REJECTED = (1 << 6)  // 自动帧过滤拒绝事件 ,一个硬件错规则
} UWB_Event_t;

#endif /* _IRQ_H_ */