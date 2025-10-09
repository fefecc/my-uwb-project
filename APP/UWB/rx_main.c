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

#include <stdio.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"

#include "frame_header.h"

#include "common_header.h"
#include "bphero_uwb.h"
#include "dwm1000_timestamp.h"
#include "cmsis_os.h"

static void Handle_TimeStamp(void);
#define MAX_ANTHOR_NODE 40
static struct distance_struct {
    double rx_distance;
    struct time_timestamp tx_node;
    struct time_timestamp rx_node;
    bool present;
    int count;
} bphero_distance[MAX_ANTHOR_NODE];

static srd_msg_dsss *msg_f;

/* Private functions ---------------------------------------------------------*/
void Simple_Rx_Callback()
{
    uint32 status_reg = 0, i = 0;

    for (i = 0; i < FRAME_LEN_MAX; i++) {
        rx_buffer[i] = '\0';
    }
    /* Activate reception immediately. See NOTE 2 below. */
    dwt_enableframefilter(DWT_FF_RSVD_EN); // disable recevie
    status_reg = dwt_read32bitreg(SYS_STATUS_ID);

    if (status_reg & SYS_STATUS_RXFCG) // good message
    {
        /* A frame has been received, copy it to our local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        if (frame_len <= FRAME_LEN_MAX) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            msg_f = (srd_msg_dsss *)rx_buffer;
            // copy source address as dest address
            msg_f_send.destAddr[0] = msg_f->sourceAddr[0];
            msg_f_send.destAddr[1] = msg_f->sourceAddr[1];
            // copy source seqNum
            msg_f_send.seqNum = msg_f->seqNum;

            switch (msg_f->messageData[0]) {
                case 'D': // distance
                    msg_f_send.messageData[0] = 'd';
                    msg_f_send.messageData[1] = msg_f->messageData[1];
                    if (bphero_distance[msg_f->sourceAddr[0]].count > 0) {
                        msg_f_send.messageData[2] = 'V';
                        int distance0             = (int)(bphero_distance[msg_f->sourceAddr[0]].rx_distance * 100); // distance 0
                        msg_f_send.messageData[3] = (uint8)(distance0 / 100);                                       // ����m
                        msg_f_send.messageData[4] = (uint8)(distance0 % 100);                                       // С��cm
                    } else {
                        msg_f_send.messageData[2] = 'N';
                        msg_f_send.messageData[3] = 0xFF;
                        msg_f_send.messageData[4] = 0xFF;
                    }
                    dwt_writetxdata(psduLength, (uint8 *)&msg_f_send, 0); // write the frame data
                    dwt_writetxfctrl(psduLength, 0);
                    /* Start transmission. */
                    dwt_starttx(DWT_START_TX_IMMEDIATE);
                    // MUST WAIT!!!!!
                    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_TXFRS))) {};
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS | SYS_STATUS_RXFCG);

                    Handle_TimeStamp();
                    break;
                default:
                    break;
            }
        }
        // enable recive again
        dwt_enableframefilter(DWT_FF_DATA_EN);
        dwt_setrxtimeout(0);
        dwt_rxenable(0);
    } else {
        // clear error flag
        dwt_write32bitreg(SYS_STATUS_ID, (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR));
        // enable recive again
        dwt_enableframefilter(DWT_FF_DATA_EN);
        dwt_rxenable(0);
    }
}

static void Handle_TimeStamp(void)
{
    bphero_distance[msg_f->sourceAddr[0]].present          = true;
    bphero_distance[msg_f->sourceAddr[0]].rx_node.tx_ts[0] = bphero_distance[msg_f->sourceAddr[0]].rx_node.tx_ts[1];
    bphero_distance[msg_f->sourceAddr[0]].rx_node.rx_ts[0] = bphero_distance[msg_f->sourceAddr[0]].rx_node.rx_ts[1];
    bphero_distance[msg_f->sourceAddr[0]].rx_node.tx_ts[1] = get_tx_timestamp_u64();
    bphero_distance[msg_f->sourceAddr[0]].rx_node.rx_ts[1] = get_rx_timestamp_u64();

    final_msg_get_ts((uint8 *)&(msg_f->messageData[FIRST_TX]), &bphero_distance[msg_f->sourceAddr[0]].tx_node.tx_ts[0]);
    final_msg_get_ts((uint8 *)&(msg_f->messageData[FIRST_RX]), &bphero_distance[msg_f->sourceAddr[0]].tx_node.rx_ts[0]);
    {
        bphero_distance[msg_f->sourceAddr[0]].rx_distance = (((bphero_distance[msg_f->sourceAddr[0]].tx_node.rx_ts[0] - bphero_distance[msg_f->sourceAddr[0]].tx_node.tx_ts[0]) - (bphero_distance[msg_f->sourceAddr[0]].rx_node.tx_ts[0] - bphero_distance[msg_f->sourceAddr[0]].rx_node.rx_ts[0])) / 2.0) * DWT_TIME_UNITS * SPEED_OF_LIGHT;
        bphero_distance[msg_f->sourceAddr[0]].count++;
    }
}

int rx_main(void)
{
    bphero_distance[0].rx_distance = 0;
    // Enable RX
    dwt_setrxtimeout(0);
    dwt_enableframefilter(DWT_FF_DATA_EN);
    dwt_rxenable(0);
    // Set Rx callback
    // bphero_setcallbacks(Simple_Rx_Callback);
    while (1) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) > 0) {
            // 被 ISR 唤醒，在这里处理 UWB 协议状态机
            // 例如：检查接收到的数据，准备下一帧发送...
            Simple_Rx_Callback();
        } else {
            // 等待超时，处理超时逻辑
            // handle_uwb_timeout();
        }
    }
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
