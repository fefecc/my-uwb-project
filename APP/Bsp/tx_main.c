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

static struct time_timestamp tx_node[MAX_TARGET_NODE];
static unsigned char distance_seqnum = 0;

// static void Handle_TimeStamp(void);
static srd_msg_dsss *msg_f_recv;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief
 * @param
 * @retval
 */
void Tx_Simple_Rx_Callback()
{
    uint32 status_reg = 0, i = 0;
    for (i = 0; i < FRAME_LEN_MAX; i++) {
        rx_buffer[i] = '\0';
    }
    /* Activate reception immediately. See NOTE 2 below. */
    dwt_enableframefilter(DWT_FF_RSVD_EN); //  recevie
    status_reg = dwt_read32bitreg(SYS_STATUS_ID);

    if (status_reg & SYS_STATUS_RXFCG) {
        /* A frame has been received, copy it to our local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        if (frame_len <= FRAME_LEN_MAX) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            msg_f_recv             = (srd_msg_dsss *)rx_buffer;
            msg_f_send.destAddr[0] = msg_f_recv->sourceAddr[0];
            msg_f_send.destAddr[1] = msg_f_recv->sourceAddr[1];

            msg_f_send.seqNum = msg_f_recv->seqNum;

            switch (msg_f_recv->messageData[0]) {
                case 'd': // distance
                    tx_node[msg_f_recv->messageData[1]].tx_ts[0] = get_tx_timestamp_u64();
                    tx_node[msg_f_recv->messageData[1]].rx_ts[0] = get_rx_timestamp_u64();
                    break;
                default:
                    break;
            }
        }
    } else {
        dwt_write32bitreg(SYS_STATUS_ID, (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR));
        // enable recive again
        dwt_enableframefilter(DWT_FF_DATA_EN);
        dwt_rxenable(0);
    }
}

void BPhero_Distance_Measure_Specail_TAG(void)
{
    // dest address  = SHORT_ADDR+1,only for test!!
    msg_f_send.destAddr[0] = (SHORT_ADDR + 1) & 0xFF;
    msg_f_send.destAddr[1] = ((SHORT_ADDR + 1) >> 8) & 0xFF;

    /* Write all timestamps in the final message. See NOTE 10 below. */
    final_msg_set_ts(&msg_f_send.messageData[FIRST_TX], tx_node[(SHORT_ADDR + 1) & 0xFF].tx_ts[0]);
    final_msg_set_ts(&msg_f_send.messageData[FIRST_RX], tx_node[(SHORT_ADDR + 1) & 0xFF].rx_ts[0]);

    msg_f_send.seqNum         = distance_seqnum;
    msg_f_send.messageData[0] = 'D';
    msg_f_send.messageData[1] = (SHORT_ADDR + 1) & 0xFF;

    dwt_writetxdata(psduLength, (uint8 *)&msg_f_send, 0); // write the frame data
    dwt_writetxfctrl(psduLength, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    dwt_enableframefilter(DWT_FF_DATA_EN);
    dwt_rxenable(0);

    // add delay for receive
    osDelay(5); // 5ms

    dwt_forcetrxoff();
    /* Clear good RX frame event in the DW1000 status register. */
    if (++distance_seqnum == 255)
        distance_seqnum = 0;
}

int tx_main(void)
{
    bphero_setcallbacks(Tx_Simple_Rx_Callback);
    /* Infinite loop */
    while (1) {
        BPhero_Distance_Measure_Specail_TAG();
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
