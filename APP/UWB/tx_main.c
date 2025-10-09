/**
 ******************************************************************************
 * @file    Project/STM32F10x_StdPeriph_Template/main.c
 * @author  MCD Application Team
 * @version V3.5.0
 * @date    08-April-2011
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"

#include "frame_header.h"

#include "common_header.h"
#include "bphero_uwb.h"
#include "dwm1000_timestamp.h"
#include "cmsis_os.h"

extern TaskHandle_t dw1000samplingTaskNotifyHandle;

static struct time_timestamp tx_node[MAX_TARGET_NODE]; // 最多同时与64个设备进行通信
static unsigned char distance_seqnum = 0;

// static void Handle_TimeStamp(void);
static srd_msg_dsss *msg_f_recv;

volatile uint64_t rxtimestamptemp = 0;
volatile uint64_t txtimestamptemp = 0;

/* 1. 定义状态和事件 */
//--------------------------------------------------------------------------------
typedef enum {
    UWB_STATE_IDLE,          // 空闲，等待发送周期
    UWB_STATE_AWAIT_TX_DONE, // 等待发送完成的中断
    UWB_STATE_AWAIT_RX_DONE  // 等待接收相关的中断
} UWB_State_t;

// 来自中断的事件通知 (使用位掩码)
typedef enum {
    UWB_EVENT_NONE       = 0,
    UWB_EVENT_TX_DONE    = (1 << 0), // 发送完成
    UWB_EVENT_RX_DONE    = (1 << 1), // 接收成功
    UWB_EVENT_RX_TIMEOUT = (1 << 2), // 接收超时
    UWB_EVENT_RX_ERROR   = (1 << 3)  // 接收出错
} UWB_Event_t;

void uwb_isr_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t event_flags                = UWB_EVENT_NONE;
    uint32_t status_reg                 = dwt_read32bitreg(SYS_STATUS_ID);

    rxtimestamptemp = get_rx_timestamp_u64();
    txtimestamptemp = get_tx_timestamp_u64();

    if (status_reg & SYS_STATUS_TXFRS) {
        event_flags |= UWB_EVENT_TX_DONE;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS); // 清除中断标志
    }
    if (status_reg & SYS_STATUS_RXFCG) {
        event_flags |= UWB_EVENT_RX_DONE;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
    }
    if (status_reg & SYS_STATUS_RXRFTO) {
        event_flags |= UWB_EVENT_RX_TIMEOUT;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);
    }

    // 如果有任何事件发生，就通过任务通知发送给UWB任务
    if (event_flags != UWB_EVENT_NONE && dw1000samplingTaskNotifyHandle != NULL) {
        xTaskNotifyFromISR(dw1000samplingTaskNotifyHandle, event_flags, eSetBits, &xHigherPriorityTaskWoken);
    }

    // 如果唤醒的任务优先级更高，请求上下文切换
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static volatile UWB_State_t g_current_uwb_state = UWB_STATE_IDLE;
/* Private functions ---------------------------------------------------------*/

/**
 * @brief
 * @param
 * @retval
 */
void Tx_Simple_Rx_Callback()
{

    uint32 status_reg = 0, i = 0;
    // 1. 初始化：清空接收缓冲区
    for (i = 0; i < FRAME_LEN_MAX; i++) {
        rx_buffer[i] = '\0';
    }

    // 2. 准备接收并读取状态
    /* 开启帧过滤功能，准备接收。注意这里的 DWT_FF_RSVD_EN 比较特殊 */
    dwt_enableframefilter(DWT_FF_RSVD_EN | DWT_FF_DATA_EN); // 不确定有没有问题
    // 读取系统状态寄存器，看看发生了什么事件
    status_reg = dwt_read32bitreg(SYS_STATUS_ID);

    // 3. 判断是否成功接收到一个有效帧
    if (status_reg & SYS_STATUS_RXFCG) {
        /* SYS_STATUS_RXFCG 标志位置位，表示接收成功且 CRC 校验正确 这个由dw1000自动判断 */

        // 3.1 读取接收到的数据
        // 从 RX_FINFO 寄存器获取帧长度
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        if (frame_len <= FRAME_LEN_MAX) {
            // 将数据从 UWB 芯片的接收 FIFO 读到MCU的 rx_buffer 中
            dwt_readrxdata(rx_buffer, frame_len, 0);

            // 3.2 解析数据并准备回复
            // 将原始字节流强制转换为一个预定义的消息结构体指针，方便访问
            msg_f_recv = (srd_msg_dsss *)rx_buffer;

            // 将“要发送的消息”的目标地址，设置为“刚收到的消息”的源地址
            // 这是典型的“回复”操作
            msg_f_send.destAddr[0] = msg_f_recv->sourceAddr[0];
            msg_f_send.destAddr[1] = msg_f_recv->sourceAddr[1];

            // 复制序列号，回复时可能需要
            msg_f_send.seqNum = msg_f_recv->seqNum;

            // 3.3 根据应用层功能码执行特定操作
            switch (msg_f_recv->messageData[0]) {
                case 'd': // 'd' 代表这是一个和距离(distance)相关的消息
                    // 记录本机（响应方）的发送和接收时间戳
                    tx_node[msg_f_recv->messageData[1]].tx_ts[0] = (uint32_t)(txtimestamptemp & 0x0000ffff); // 从临时变量中读取
                    tx_node[msg_f_recv->messageData[1]].tx_ts[1] = (uint32_t)((txtimestamptemp & 0xffff0000) >> 32);
                    tx_node[msg_f_recv->messageData[1]].rx_ts[0] = (uint32_t)(rxtimestamptemp & 0x0000ffff);
                    tx_node[msg_f_recv->messageData[1]].rx_ts[1] = (uint32_t)((rxtimestamptemp & 0xffff0000) >> 32);
                    break;
                default:
                    // 其他功能码的处理
                    break;
            }
        }
    } else {
        /* 4. 接收失败或无数据分支 */
        // 如果没有收到有效帧（例如接收超时、CRC错误等）
        // 清除所有接收相关的错误标志位，让系统恢复
        dwt_write32bitreg(SYS_STATUS_ID, (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR));

        // 重新开启接收器，等待下一次通信
        dwt_enableframefilter(DWT_FF_DATA_EN);
        dwt_rxenable(0);
    }
}

void BPhero_Distance_Measure_Specail_TAG(void)
{
    // dest address  = SHORT_ADDR+1,only for test!!
    msg_f_send.destAddr[0] = (SHORT_ADDR + 1) & 0xFF; // 目标地址 即接收端地址
    msg_f_send.destAddr[1] = ((SHORT_ADDR + 1) >> 8) & 0xFF;

    /* Write all timestamps in the final message. See NOTE 10 below. */

    // 计算64位数据溢出
    uint64_t rxtempdata, txtempdata;
    txtempdata = ((uint64_t)tx_node[(SHORT_ADDR + 1) & 0xFF].tx_ts[1] << 32) | tx_node[(SHORT_ADDR + 1) & 0xFF].tx_ts[0];
    rxtempdata = ((uint64_t)tx_node[(SHORT_ADDR + 1) & 0xFF].rx_ts[1] << 32) | tx_node[(SHORT_ADDR + 1) & 0xFF].rx_ts[0];

    final_msg_set_ts(&msg_f_send.messageData[FIRST_TX], txtempdata);
    final_msg_set_ts(&msg_f_send.messageData[FIRST_RX], rxtempdata);

    msg_f_send.seqNum         = distance_seqnum;
    msg_f_send.messageData[0] = 'D';
    msg_f_send.messageData[1] = (SHORT_ADDR + 1) & 0xFF;

    dwt_writetxdata(psduLength, (uint8 *)&msg_f_send, 0); // write the frame data
    dwt_writetxfctrl(psduLength, 0);

    dwt_starttx(DWT_START_TX_IMMEDIATE); // 启动发送

    dwt_enableframefilter(DWT_FF_DATA_EN);

    /* Clear good RX frame event in the DW1000 status register. */
    if (++distance_seqnum == 255)
        distance_seqnum = 0;
}

int tx_main(void)
{
    // bphero_setcallbacks(Tx_Simple_Rx_Callback); //中断
    /* Infinite loop */
    static uint32_t notified_value = 0;

    switch (g_current_uwb_state) {

        case UWB_STATE_IDLE: {
            // 在空闲状态，等待一个周期性的延时，然后开始下一次发送
            vTaskDelay(pdMS_TO_TICKS(1000)); // 每隔 1 秒发送一次

            printf("dw1000 IDLE: transmit...\n");

            // 发送标签数据
            BPhero_Distance_Measure_Specail_TAG();
            // 切换到下一个状态，等待发送完成
            g_current_uwb_state = UWB_STATE_AWAIT_TX_DONE;
            break;
        }

        case UWB_STATE_AWAIT_TX_DONE: {
            // 等待来自 ISR 的通知，最长等待 100ms
            if (xTaskNotifyWait(0x00,       /* 进入时不清除所有通知位 */
                                UINT32_MAX, /* 退出时清除所有通知位 */
                                &notified_value,
                                pdMS_TO_TICKS(100)) == pdTRUE) {
                if (notified_value & UWB_EVENT_TX_DONE) {
                    // 发送成功！
                    printf("TX_DONE: transmit succeed...\n");

                    // 立即开启接收器，并设置一个接收超时时间，10ms
                    dwt_setrxtimeout(10000); // 单位是 1.0256 us (512/499.2MHz)  10000 * 1.0256 = 10256us = 10.256ms超时

                    dwt_rxenable(0);
                    // 切换到等待接收状态
                    g_current_uwb_state = UWB_STATE_AWAIT_RX_DONE;
                }
            }

            else {
                // 等待发送完成超时，这通常意味着硬件有问题
                printf("ERROR: time-out\n");
                // 强制关闭收发器并回到空闲状态
                dwt_forcetrxoff();
                g_current_uwb_state = UWB_STATE_IDLE;
            }
            break;
        }

        case UWB_STATE_AWAIT_RX_DONE: {
            // 等待来自 ISR 的通知
            if (xTaskNotifyWait(0x00, UINT32_MAX, &notified_value, portMAX_DELAY) == pdTRUE) {
                if (notified_value & UWB_EVENT_RX_DONE) {
                    // 成功接收到数据！
                    Tx_Simple_Rx_Callback();
                    // 在这里处理接收到的 g_rx_buffer 数据...
                } else if (notified_value & UWB_EVENT_RX_TIMEOUT) {
                    // 接收超时
                    printf("RX_TIMEOUT\n");
                } else if (notified_value & UWB_EVENT_RX_ERROR) {
                    // 接收出错
                    printf("RX_ERROR\n");
                }
            }
            // 无论接收成功、超时还是失败，一个完整的周期都结束了
            // 回到空闲状态，准备下一次发送
            g_current_uwb_state = UWB_STATE_IDLE;
            break;
        }
    }

    return 0;
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
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1) {
    }
}
#endif

/**
 * @}
 */
/**
 * @}
 */
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
