#include "DW1000samplingtask.h"
#include "stdio.h"
#include "bphero_uwb.h"
#include "deca_device_api.h"
#include "trilateration.h"
#include "deca_regs.h"
#include "mydw1000timestamp.h"
#include "EXITcallback.h"
#include "cmsis_os.h"
#include "dw1000frame.h"
#include "dw1000framedeal.h"
#include "string.h"
#include "stdint.h"
#include "inttypes.h"

// 这个用来判断帧的类型
typedef enum {
    Frame_Type_NULL = 0, // 空初始化
    Frame_Type_POLL,     // 起始帧，这一帧中没有任何时间戳
    Frame_Type_Response, // 应答帧
    Frame_Type_Final     // 等待接收相关的中断，完成所有应答之后，最终计算的帧
} Frame_Type_t;

TaskHandle_t dw1000samplingTaskNotifyHandle                 = NULL; // 创建dw1000采样线程句柄
static volatile isr_timestamp_packet_t isr_timestamp_packet = {0};  // 用来记录时间戳的临时变量

// TAG_STATE
typedef enum {
    TAG_STATE_IDLE,                  // 空闲状态，等待发起下一次测距
    TAG_STATE_AWAIT_POLL_TX_CONFIRM, // 已发送Poll，等待硬件发送完成的确认
    TAG_STATE_AWAIT_RESPONSE_RX,     // 已开启接收，等待来自基站(Anchor)的Response消息
    TAG_STATE_AWAIT_FINAL_TX_CONFIRM // 已发送Final消息，等待硬件发送完成的确认
} Tag_State_t;

// ANCHOR_STATE
typedef enum {
    /* * 核心 resting state，不是 IDLE。Anchor 的默认职责就是持续监听。
     * 如果使用低功耗方案，可能会有一个 IDLE 状态，然后周期性进入 AWAIT_POLL_RX。
     * 这里为了简化，我们假设 Anchor 始终在监听。
     */
    ANCHOR_STATE_AWAIT_POLL_RX, // 开启接收，等待来自任何 Tag 的 Poll 消息

    ANCHOR_STATE_AWAIT_RESPONSE_TX_CONFIRM, // 已收到 Poll，并已发送 Response，等待硬件发送完成的确认

    ANCHOR_STATE_AWAIT_FINAL_RX // 已发送 Response，并已开启接收，等待来自 Tag 的 Final 消息

} Anchor_State_t;

void reset_anchor_state_machine(Anchor_State_t *current_anchor_state);
void reset_tag_state_machine(Tag_State_t *current_tag_state);

#define MaxAnchorNum 1 // 暂时使用一个节点

double twr_distance;

uwb_node_profile_u64_t AnchorNode_u64[MaxAnchorNum] = {0};

uwb_node_profile_t AnchorNode[MaxAnchorNum] = {
    {
        .short_addr  = 0x0033,
        .poll_tx_ts  = {0},
        .poll_rx_ts  = {0},
        .resp_tx_ts  = {0},
        .resp_rx_ts  = {0},
        .final_tx_ts = {0},
        .final_rx_ts = {0},
    }};

// dw1000_local_device_t local_device =
//     {.frameCtrl[0] = 0x41,
//      .frameCtrl[1] = 0x88,
//      .seqNum       = 0,
//      .pan_id       = 0xf0f0,
//      .short_addr   = 0x0032};

dw1000_local_device_t local_device =
    {.frameCtrl[0] = 0x41,
     .frameCtrl[1] = 0x88,
     .seqNum       = 0,
     .pan_id       = 0xf0f0,
     .short_addr   = 0x0033};

static uint8_t dw1000tx_buffer[FRAME_LEN_MAX]; // 发送数据
static uint8_t dw1000rx_buffer[FRAME_LEN_MAX]; // 接收数据

extern srd_msg_dsss msg_f_send;

static Tag_State_t g_current_tag_state = TAG_STATE_IDLE;
static uint32_t notified_value         = 0;
void dw1000TagMain(void)
{

    while (1) {
        switch (g_current_tag_state) {
            case TAG_STATE_IDLE: {
                osDelay(1000); // 定时1s左右发一次数据
                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_6);
                clear_node_profile(&AnchorNode[0]);

                static int16_t frame_size = 0;
                // 填充poll帧数据
                frame_size = create_poll_frame(dw1000tx_buffer, sizeof(dw1000tx_buffer), local_device.seqNum,
                                               local_device.pan_id, AnchorNode[0].short_addr, local_device.short_addr);

                dwt_writetxdata(frame_size, dw1000tx_buffer, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(frame_size, 0);
                dwt_starttx(DWT_START_TX_IMMEDIATE);

                g_current_tag_state = TAG_STATE_AWAIT_POLL_TX_CONFIRM;
                break;
            }

            case TAG_STATE_AWAIT_POLL_TX_CONFIRM: {
                // 等待 ISR 的通知（最长等待 100ms 超时）
                if (xTaskNotifyWait(0x00,       /* 进入时不清除所有通知位 */
                                    UINT32_MAX, /* 退出时清除所有通知位 */
                                    &notified_value, pdMS_TO_TICKS(100)) == pdTRUE) {

                    if (notified_value & UWB_EVENT_TX_DONE) {

                        // 在这个状态时间戳
                        mymemcopytimestamp(AnchorNode[0].poll_tx_ts, isr_timestamp_packet.txtimestamp); // 设置poll发送这个时间点的时间戳

                        dwt_setrxtimeout(65535); // 单位是 1.0256 us (512/499.2MHz) 大概
                                                 // 65毫秒超时

                        dwt_rxenable(0);
                        // 切换到等待接收状态
                        g_current_tag_state = TAG_STATE_AWAIT_RESPONSE_RX;
                    } else {
                        printf("ERROR:POLL frame interrupt error\n");  // 中断错误主要是硬件模块报错
                        reset_tag_state_machine(&g_current_tag_state); // 复位
                    }
                } else {
                    // 等待发送确认超时，流程失败，回到 IDLE
                    printf("ERROR:AWAIT POLL_TX timeout\n");       // 这个是状态机超时，这个超时和模块本身没有关系
                    reset_tag_state_machine(&g_current_tag_state); // 复位
                }
                break;
            }

            case TAG_STATE_AWAIT_RESPONSE_RX: {
                // 等待response这个函数的接收
                // 等待 ISR 的通知（由 dwt_setrxtimeout 控制超时）
                if (xTaskNotifyWait(0x00,       /* 进入时不清除所有通知位 */
                                    UINT32_MAX, /* 退出时清除所有通知位 */
                                    &notified_value, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_RX_DONE) {
                        // 成功收到 Response 消息！

                        uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK; // 发送过来的数据长度

                        dwt_readrxdata(dw1000rx_buffer, frame_len, 0); // 读取的数据

                        twr_resp_msg_t received_response_msg;

                        if (!parse_resp_frame(dw1000rx_buffer, frame_len, &received_response_msg)) {

                            // 解析成功
                            mymemcopytimestamp(AnchorNode[0].resp_rx_ts, isr_timestamp_packet.rxtimestamp); // 这个接收的数据是response帧

                            uint64_t transmitDelayTimetick; // 计算延时之后的基础时间

                            transmitDelayTimetick = transmitDelayTime(u8_5byte_TO_u64(AnchorNode[0].resp_rx_ts), 5);

                            dwt_setdelayedtrxtime((uint32_t)(transmitDelayTimetick >> 8)); // 低位直接忽略

                            mymemcopytimestamp(AnchorNode[0].poll_rx_ts,
                                               received_response_msg.poll_rx_ts);
                            mymemcopytimestamp(AnchorNode[0].resp_tx_ts,
                                               received_response_msg.resp_tx_ts);

                            u64_TO_u8_5byte(transmitDelayTimetick, AnchorNode[0].final_tx_ts);
                            static int16_t frame_size = 0;
                            // 填充poll帧数据
                            // 构造一个final函数

                            frame_size = create_final_frame(
                                dw1000tx_buffer, sizeof(dw1000tx_buffer),
                                local_device.seqNum, local_device.pan_id,
                                AnchorNode[0].short_addr, local_device.short_addr,
                                AnchorNode[0].poll_tx_ts, AnchorNode[0].poll_rx_ts,
                                AnchorNode[0].resp_tx_ts, AnchorNode[0].resp_rx_ts,
                                AnchorNode[0].final_tx_ts);

                            dwt_writetxdata(frame_size, dw1000tx_buffer, 0);
                            dwt_writetxfctrl(frame_size, 0);

                            dwt_starttx(DWT_START_TX_DELAYED);

                            g_current_tag_state = TAG_STATE_AWAIT_FINAL_TX_CONFIRM;

                        } else {
                            printf("ERROR: RESPONSE frame error\n"); // 帧错误
                            reset_tag_state_machine(&g_current_tag_state);
                        }

                    } else {
                        printf("ERROR: RESPONSE frame interrupt error\n"); // 中断类型错误
                        reset_tag_state_machine(&g_current_tag_state);
                    }
                } else {
                    // 接收 Response 超时或出错，流程失败，回到 IDLE
                    printf("ERROR:AWAIT RESPONSE_RX timeout\n"); // 状态机超时
                    reset_tag_state_machine(&g_current_tag_state);
                }
                break;
            }

            case TAG_STATE_AWAIT_FINAL_TX_CONFIRM: {
                if (xTaskNotifyWait(0x00,       /* 进入时不清除所有通知位 */
                                    UINT32_MAX, /* 退出时清除所有通知位 */
                                    &notified_value, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_TX_DONE) {
                        // Final 消息发送成功，一次完整的测距流程结束！
                        printf("SUCCESS: twr \n");
                        local_device.seqNum++;                         // 递增序列号，为下一次做准备
                        reset_tag_state_machine(&g_current_tag_state); // 成功接收，复位
                    } else {
                        printf("ERROR:AWAIT FINAL_TX timeout\n");
                        reset_tag_state_machine(&g_current_tag_state);
                    }

                } else {
                    printf("ERROR:AWAIT FINAL_TX timeout\n");
                    reset_tag_state_machine(&g_current_tag_state);
                }

                break;
            }
        }
    }
}

static Anchor_State_t g_current_anchor_state = ANCHOR_STATE_AWAIT_POLL_RX;
// static uint32_t notified_value               = 0;
void dw1000AnchorMain(void)
{
    reset_anchor_state_machine(&g_current_anchor_state); // 开启接收
    while (1) {
        switch (g_current_anchor_state) {
            // --- 状态 1: 等待 Poll 消息 ---
            case ANCHOR_STATE_AWAIT_POLL_RX: {
                // 无限期等待 ISR 的通知
                if (xTaskNotifyWait(0x00,       /* 进入时不清除所有通知位 */
                                    UINT32_MAX, /* 退出时清除所有通知位 */
                                    &notified_value, pdMS_TO_TICKS(3000)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_RX_DONE) {
                        uint16_t frame_len =
                            dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

                        //  uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) &
                        //  RX_FINFO_RXFLEN_MASK; // 发送过来的数据长度

                        dwt_readrxdata(dw1000rx_buffer, frame_len, 0); // 读取的数据

                        //  dwt_readrxdata(rx_buffer, frame_len, 0);
                        twr_poll_msg_t received_poll_msg;

                        if (!parse_poll_frame(dw1000rx_buffer, frame_len,
                                              &received_poll_msg)) {
                            clear_node_profile(&AnchorNode[0]);

                            mymemcopytimestamp(
                                AnchorNode[0].poll_rx_ts,
                                isr_timestamp_packet
                                    .rxtimestamp); // 这个接收的数据是poll帧

                            uint64_t transmitDelayTimetick; // 计算延时之后的基础时间

                            transmitDelayTimetick = transmitDelayTime(
                                u8_5byte_TO_u64(AnchorNode[0].poll_rx_ts), 5); // 5ms

                            u64_TO_u8_5byte(transmitDelayTimetick,
                                            AnchorNode[0].resp_tx_ts);

                            dwt_setdelayedtrxtime((uint32_t)(transmitDelayTimetick >>
                                                             8)); // 低位直接忽略

                            uint16_t dest_addrtemp =
                                received_poll_msg.source_addr[0] |
                                ((uint16_t)received_poll_msg.source_addr[1] << 8);

                            static int16_t frame_size = 0; // 计算

                            frame_size = create_resp_frame(
                                dw1000tx_buffer, sizeof(dw1000tx_buffer),
                                received_poll_msg.sequence_num, local_device.pan_id,
                                dest_addrtemp, local_device.short_addr,
                                AnchorNode[0].poll_rx_ts, AnchorNode[0].resp_tx_ts);

                            dwt_writetxdata(frame_size, dw1000tx_buffer, 0);
                            dwt_writetxfctrl(frame_size, 0);

                            // dwt_writetxdata(sizeof(rx_resp_msg), rx_resp_msg, 0);
                            // dwt_writetxfctrl(sizeof(rx_resp_msg) , 0);

                            dwt_starttx(DWT_START_TX_DELAYED); // 延时发送方式

                            // 切换到下一个状态
                            g_current_anchor_state =
                                ANCHOR_STATE_AWAIT_RESPONSE_TX_CONFIRM;
                        } else {
                            // 如果不是 Poll 消息，忽略并重新开启接收
                            printf("ERROR: POLL frame error\n"); // 帧解析错误
                            reset_anchor_state_machine(&g_current_anchor_state);
                        }
                    } else {
                        // 收到其他非 RX_DONE 事件，重新开启接收
                        printf(
                            "ERROR: POLL frame interrupt error notified_value:%lu\n",
                            notified_value); // 中断类型错误
                        reset_anchor_state_machine(&g_current_anchor_state);
                    }
                } else {
                    // 这个等待函数是永久等待，不会到这里的
                    reset_anchor_state_machine(&g_current_anchor_state);
                }
            } break;

            // --- 状态 2: 等待 Response 发送完成 ---
            case ANCHOR_STATE_AWAIT_RESPONSE_TX_CONFIRM: {
                // 等待发送完成通知，设置一个短暂的超时
                if (xTaskNotifyWait(0x00,       /* 进入时不清除所有通知位 */
                                    UINT32_MAX, /* 退出时清除所有通知位 */
                                    &notified_value, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_TX_DONE) {
                        // 立即开启接收器，等待 Tag 的 Final 消息 (设置硬件超时)
                        dwt_setrxtimeout(65535);
                        dwt_rxenable(0);

                        // 切换到下一个状态
                        g_current_anchor_state = ANCHOR_STATE_AWAIT_FINAL_RX;
                    } else {
                        printf("ERROR: RESPONSE frame interrupt error\n");
                        reset_anchor_state_machine(&g_current_anchor_state);
                    }
                } else {
                    // 发送确认超时，流程失败，回到监听 Poll 的状态
                    printf("ERROR: AWAIT RESPONSE_TX timeout\n");
                    reset_anchor_state_machine(&g_current_anchor_state);
                }
                break;
            }

            // --- 状态 3: 等待 Final 消息 ---
            case ANCHOR_STATE_AWAIT_FINAL_RX: {
                if (xTaskNotifyWait(0x00,       /* 进入时不清除所有通知位 */
                                    UINT32_MAX, /* 退出时清除所有通知位 */
                                    &notified_value, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (notified_value & UWB_EVENT_RX_DONE) {
                        uint16_t frame_len =
                            dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

                        dwt_readrxdata(dw1000rx_buffer, frame_len, 0);

                        twr_final_msg_t received_final_msg;

                        if (!parse_final_frame(dw1000rx_buffer, frame_len,
                                               &received_final_msg)) {
                            mymemcopytimestamp(
                                AnchorNode[0].final_rx_ts,
                                isr_timestamp_packet.rxtimestamp); // 这个接收的数据是poll帧

                            mymemcopytimestamp(AnchorNode[0].poll_tx_ts,
                                               received_final_msg.poll_tx_ts);
                            mymemcopytimestamp(AnchorNode[0].poll_rx_ts,
                                               received_final_msg.poll_rx_ts);
                            mymemcopytimestamp(AnchorNode[0].resp_tx_ts,
                                               received_final_msg.resp_tx_ts);
                            mymemcopytimestamp(AnchorNode[0].resp_rx_ts,
                                               received_final_msg.resp_rx_ts);
                            mymemcopytimestamp(AnchorNode[0].final_tx_ts,
                                               received_final_msg.final_tx_ts);

                            AnchorNode_u64->short_addr = local_device.short_addr;
                            AnchorNode_u64->poll_tx_ts =
                                u8_5byte_TO_u64(AnchorNode[0].poll_tx_ts);
                            AnchorNode_u64->poll_rx_ts =
                                u8_5byte_TO_u64(AnchorNode[0].poll_rx_ts);
                            AnchorNode_u64->resp_tx_ts =
                                u8_5byte_TO_u64(AnchorNode[0].resp_tx_ts);
                            AnchorNode_u64->resp_rx_ts =
                                u8_5byte_TO_u64(AnchorNode[0].resp_rx_ts);
                            AnchorNode_u64->final_tx_ts =
                                u8_5byte_TO_u64(AnchorNode[0].final_tx_ts);
                            AnchorNode_u64->final_rx_ts =
                                u8_5byte_TO_u64(AnchorNode[0].final_rx_ts);

                            twr_distance = calculate_distance_from_timestamps(
                                AnchorNode_u64->poll_tx_ts, AnchorNode_u64->resp_rx_ts,
                                AnchorNode_u64->final_tx_ts, AnchorNode_u64->poll_rx_ts,
                                AnchorNode_u64->resp_tx_ts, AnchorNode_u64->final_rx_ts);

                            printf("distance : %f\r\n", twr_distance);
                            reset_anchor_state_machine(
                                &g_current_anchor_state); // 成功解析返回等待模式

                        } else {
                            printf("ERROR: FINAL frame error\n"); // 帧解析错误
                            reset_anchor_state_machine(&g_current_anchor_state);
                        }

                    } else {
                        printf("ERROR: FINAL frame interrupt error:%lu\n", notified_value); // 中断类型错误
                        reset_anchor_state_machine(&g_current_anchor_state);
                    }
                } else {
                    printf("ERROR: AWAIT FINAL_TX timeout\n"); // 硬件超时
                    reset_anchor_state_machine(&g_current_anchor_state);
                }
                break;
            }
        }
    }
}

void DW1000samplingtask(void *argument)
{
    dw1000samplingTaskNotifyHandle =
        xTaskGetCurrentTaskHandle(); // 获取当前的线程句柄
    // BPhero_UWB_Message_Init();       // 消息结构体初始化
    BPhero_UWB_Init(); // dwm1000 init related

    // dw1000TagMain();
    dw1000AnchorMain();
}

// dw1000中断处理句柄
void uwb_isr_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    UWB_Event_t event_to_notify = UWB_EVENT_NONE;

    static uint32_t status_reg;

    status_reg = dwt_read32bitoffsetregFromISR(SYS_STATUS_ID, 0); // 读取状态

    uint32_t taskEXIT_CRITICAL_temp;
    // 用来记录时间戳
    taskEXIT_CRITICAL_temp = taskENTER_CRITICAL_FROM_ISR();
    static uint8_t temp_timestamp_buffer[5];
    my_get_rx_timestamp_u8_5byte(temp_timestamp_buffer);
    memcpy((void *)isr_timestamp_packet.rxtimestamp, temp_timestamp_buffer, 5);
    my_get_tx_timestamp_u8_5byte(temp_timestamp_buffer);
    memcpy((void *)isr_timestamp_packet.txtimestamp, temp_timestamp_buffer, 5);
    taskEXIT_CRITICAL_FROM_ISR(taskEXIT_CRITICAL_temp);

    //  根据状态寄存器的值，判断发生了什么事件，并映射到您的 UWB_Event_t 枚举
    if (status_reg & SYS_STATUS_TXFRS) {
        // 发送完成 [cite: 1102]
        event_to_notify |= UWB_EVENT_TX_DONE;
    }

    if (status_reg & SYS_STATUS_RXFCG) {
        // 接收成功 (CRC校验正确) [cite: 1103]
        event_to_notify |= UWB_EVENT_RX_DONE;
    }

    if (status_reg & SYS_STATUS_RXRFTO) {
        // 接收帧等待超时 [cite: 1104]
        event_to_notify |= UWB_EVENT_RX_TIMEOUT;
    }
    if (status_reg & SYS_STATUS_RXSFDTO) {
        // SFD超时
        event_to_notify |= UWB_EVENT_SFD_DONE;
    }
    if (status_reg & SYS_STATUS_RXPTO) {
        // PRE超时
        event_to_notify |= UWB_EVENT_PRE_DONE;
    }

    if (status_reg & SYS_STATUS_AFFREJ) {
        // 帧被自动过滤功能拒绝 [cite: 1093]
        event_to_notify |= UWB_EVENT_FRAME_REJECTED;
    }

    // 将多个硬件错误统一归为 UWB_EVENT_RX_ERROR
    if (status_reg & (SYS_STATUS_RXFCE | SYS_STATUS_RXPHE | SYS_STATUS_RXRFSL)) {
        // RXFCE: CRC 错误 [cite: 1103]
        // RXPHE: PHY Header 错误 [cite: 1103]
        // RXRFSL: Reed Solomon 解码错误 [cite: 1104]
        event_to_notify |= UWB_EVENT_RX_ERROR;
    }

    dwt_write32bitoffsetregFromISR(SYS_STATUS_ID, 0, status_reg); // 清除中断标志

    // 如果有任何事件发生，就通过任务通知发送给UWB任务
    if (event_to_notify != UWB_EVENT_NONE &&
        dw1000samplingTaskNotifyHandle != NULL) {
        xTaskNotifyFromISR(dw1000samplingTaskNotifyHandle, event_to_notify,
                           eSetBits, &xHigherPriorityTaskWoken);
    }

    // 如果唤醒的任务优先级更高，请求上下文切换
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief  清空（重置）一个 UWB 节点档案的所有数据。
 * @note   本函数通过将结构体的所有字节设置为 0 来实现清空。
 * @param  node_profile [输出] 指向要被清空的 uwb_node_profile_t 结构体的指针。
 * @return int16_t      成功则返回 0，如果传入的指针为空，则返回 -1 表示错误。
 */
int16_t clear_node_profile(uwb_node_profile_t *node_profile)
{
    // 1. 参数校验：检查传入的指针是否有效
    if (node_profile == NULL) {
        return -1; // 如果是空指针，则返回错误
    }
    uint16_t short_addrtemp = node_profile->short_addr;

    // 2. 核心操作：使用 memset 将整个结构体占用的内存区域清零
    //    - 第一个参数: 目标内存区域的起始地址
    //    - 第二个参数: 要填充的值 (0)
    //    - 第三个参数: 要填充的字节数 (整个结构体的大小)
    memset(node_profile, 0, sizeof(uwb_node_profile_t));

    node_profile->short_addr = short_addrtemp;

    // 3. 返回成功
    return 0;
}

void apply_dw1000_optimizations(const dwt_config_t *config)
{
    // 注意：以下值部分来源于官方API示例，因为用户手册中的值主要针对16MHz PRF。
    // 您应该根据您的最终配置，从官方驱动源码中确认最匹配的优化值。

    // 2.5.5.1: AGC_TUNE1 (地址 0x23, 子地址 0x04)
    // 手册推荐值为 0x8870 (针对 16MHz PRF)。
    // 对于 64MHz PRF，通常也使用此值或根据API推荐调整。
    dwt_writetodevice(AGC_CTRL_ID, AGC_TUNE1_OFFSET, 2, (uint8_t[]){0x9B, 0x88});

    // 2.5.5.2: AGC_TUNE2 (地址 0x23, 子地址 0x0C)
    // 手册要求必须配置。一个常用的安全值是 0x2502A907。
    dwt_write32bitoffsetreg(AGC_CTRL_ID, AGC_TUNE2_OFFSET, 0x2502A907);

    // 2.5.5.3: DRX_TUNE2 (地址 0x27, 子地址 0x08)
    // 手册推荐值为 0x311A002D (针对 16MHz PRF)。
    // 对于 64MHz PRF 和 110kbps 速率，一个常用的推荐值是 0x3332003D。0x353B015E
    dwt_writetodevice(DRX_CONF_ID, DRX_TUNE2_OFFSET, 4, (uint8_t[]){0x5E, 0x01, 0x3B, 0x35});

    // 2.5.5.5: LDE_CFG2 (地址 0x2E, 子地址 0x1806)
    // 对于 64MHz PRF，推荐值为 0x0607。

    dwt_writetodevice(LDE_IF_ID, LDE_CFG2_OFFSET, 2, (uint8_t[]){0x07, 0x06});

    // 2.5.5.7: RF_TXCTRL (地址 0x28, 子地址 0x0C) - 根据信道设置
    if (config->chan == 5) {
        // 手册指出通道5的默认值非最优，需要根据表格36配置。
        // 一个常用的推荐值是 0x1E3FE0。
        dwt_writetodevice(RF_CONF_ID, RF_TXCTRL_OFFSET, 3, (uint8_t[]){0xE0, 0x3F, 0x1E}); // 0x001E3FE0
        dwt_writetodevice(TX_CAL_ID, TC_PGDELAY_OFFSET, 1, (uint8_t[]){0xC0});
        dwt_writetodevice(FS_CTRL_ID, FS_PLLTUNE_OFFSET, 1, (uint8_t[]){0xBE}); // 0xBE
    }

    if (config->chan == 2) {
        // 手册指出通道5的默认值非最优，需要根据表格36配置。
        // 一个常用的推荐值是 0x1E3FE0。
        dwt_writetodevice(RF_CONF_ID, RF_TXCTRL_OFFSET, 3, (uint8_t[]){0xA0, 0x5C, 0x04}); // 0x00045CA0
        dwt_writetodevice(TX_CAL_ID, TC_PGDELAY_OFFSET, 1, (uint8_t[]){0xC2});
        dwt_writetodevice(FS_CTRL_ID, FS_PLLTUNE_OFFSET, 1, (uint8_t[]){0x26}); // 0x26
    }
    // TODO: 在此为您的其他信道添加推荐值
}

/**
 * @brief  根据 DS-TWR 协议收集的所有时间戳，计算出距离。
 * @param  tag_poll_tx_ts      Tag 发送 Poll 的时间
 * @param  tag_resp_rx_ts      Tag 接收 Response 的时间
 * @param  tag_final_tx_ts     Tag 发送 Final 的时间
 * @param  anchor_poll_rx_ts   Anchor 接收 Poll 的时间
 * @param  anchor_resp_tx_ts   Anchor 发送 Response 的时间
 * @param  anchor_final_rx_ts  Anchor 接收 Final 的时间
 * @return double              计算出的距离（单位：米）。如果时间戳无效则返回负值。
 */
double calculate_distance_from_timestamps(uint64_t tag_poll_tx_ts, uint64_t tag_resp_rx_ts, uint64_t tag_final_tx_ts,
                                          uint64_t anchor_poll_rx_ts, uint64_t anchor_resp_tx_ts, uint64_t anchor_final_rx_ts)
{
    // 1. 计算四个基本时间差，并处理时间戳回绕
    double T_round_tag    = (double)get_timestamp_difference_u64(tag_resp_rx_ts, tag_poll_tx_ts);
    double T_reply_anchor = (double)get_timestamp_difference_u64(anchor_resp_tx_ts, anchor_poll_rx_ts);
    double T_round_anchor = (double)get_timestamp_difference_u64(anchor_final_rx_ts, anchor_resp_tx_ts);
    double T_reply_tag    = (double)get_timestamp_difference_u64(tag_final_tx_ts, tag_resp_rx_ts);

    // 2. 使用 DS-TWR 公式计算飞行时间 (单位：DW1000 时间单位)
    double Tprop_ticks;
    double numerator   = (T_round_tag * T_round_anchor) - (T_reply_tag * T_reply_anchor);
    double denominator = T_round_tag + T_round_anchor + T_reply_tag + T_reply_anchor;

    if (denominator == 0) {
        return -1.0; // 防止除以零错误
    }

    Tprop_ticks = numerator / denominator;

    // 3. 将飞行时间从 DWT 时间单位转换为秒
    double Tprop_seconds = Tprop_ticks * DWT_TIME_UNITS;

    // 4. 将飞行时间转换为距离（米）
    double distance_meters = Tprop_seconds * SPEED_OF_LIGHT;

    return distance_meters;
}

void reset_tag_state_machine(Tag_State_t *current_tag_state)
{
    // 1. 禁用 DW1000 中断，防止 ISR 在复位过程中干扰
    //    参数可能需要根据您的具体实现调整
    uint32_t interrupt_mask = DWT_INT_TFRS   // 发送完成
                              | DWT_INT_RFCG // 接收成功 (CRC OK)
                              | DWT_INT_RFTO // 接收帧等待超时
                              | DWT_INT_RFCE // 接收 CRC 错误
        // | DWT_INT_RXPTO  // 前导码超时 (可选)
        // | DWT_INT_SFDT   // SFD 超时 (可选)
        // | DWT_INT_RPHE   // PHY 头错误 (可选)
        ;

    dwt_setinterrupt(interrupt_mask, 0); // 禁用所有 DW1000 中断源

    // 2. 强制 DW1000 进入 IDLE 状态，中止任何当前操作
    //    这是确保 DW1000 状态干净的关键步骤
    dwt_forcetrxoff();

    // 3. 清除可能残留的 DW1000 状态寄存器标志位
    //    读取并写回 SYS_STATUS 可以清除所有已置位的标志位
    uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    dwt_write32bitreg(SYS_STATUS_ID, status_reg); // 写 1 清除置位的标志

    // 4. 清除 RTOS 任务通知状态
    //    防止旧的通知影响复位后的状态机行为
    if (dw1000samplingTaskNotifyHandle != NULL) {
        xTaskNotifyStateClear(dw1000samplingTaskNotifyHandle);
    } else {
        // 如果在任务内部调用，可以传入 NULL
        xTaskNotifyStateClear(NULL);
    }

    // 5. 将状态机变量重置为初始状态
    *current_tag_state = TAG_STATE_IDLE;

    // 6. （可选）清除与特定 TWR 交换相关的内部数据
    //    例如，清除存储的时间戳等 (如果 clear_node_profile 做这个事情)
    // clear_node_profile(&AnchorNode[0]); // 根据需要在 IDLE 状态开始时调用，或此处

    // 7. 重新启用必要的 DW1000 中断
    //    需要根据您的应用启用哪些中断源
    //    示例：启用发送完成、接收完成、接收超时、接收错误中断
    dwt_setinterrupt(interrupt_mask, 1);
}

#define RX_SOFTRESET_BIT_MASK   (1UL << 28)   // Bit 28 for RX reset
#define ALL_SOFTRESET_BITS_MASK (0xFUL << 28) // Bits 31-28, 正常操作时应为 1

/**
 * @brief 复位 Anchor 状态机及其相关资源到初始状态。
 * @note 应该在初始化、严重错误恢复或看门狗复位后调用。
 */
void reset_anchor_state_machine(Anchor_State_t *current_anchor_state)
{

    // 1. 禁用 DW1000 中断，防止 ISR 在复位过程中干扰
    //    参数可能需要根据您的具体实现调整
    uint32_t interrupt_mask = DWT_INT_TFRS   // 发送完成
                              | DWT_INT_RFCG // 接收成功 (CRC OK)
                              | DWT_INT_RFTO // 接收帧等待超时
                              | DWT_INT_RFCE // 接收 CRC 错误

                              | DWT_INT_RXPTO // 前导码超时 (可选)
                              | DWT_INT_SFDT  // SFD 超时 (可选)
        // | DWT_INT_RPHE   // PHY 头错误 (可选)
        ;

    dwt_setinterrupt(interrupt_mask, 0); // 禁用所有 DW1000 中断源

    // 2. 强制 DW1000 进入 IDLE 状态
    dwt_forcetrxoff();
    uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    dwt_write32bitreg(SYS_STATUS_ID, status_reg); // 写 1 清除置位的标志

    uint32_t pmsc_ctrl0_value;

    pmsc_ctrl0_value = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);

    dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl0_value & ~RX_SOFTRESET_BIT_MASK);
    osDelay(1);
    dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, (pmsc_ctrl0_value & ~ALL_SOFTRESET_BITS_MASK) | ALL_SOFTRESET_BITS_MASK);
    // 4. 清除 RTOS 任务通知状态
    if (dw1000samplingTaskNotifyHandle != NULL) {
        xTaskNotifyStateClear(dw1000samplingTaskNotifyHandle);
    } else {
        xTaskNotifyStateClear(NULL);
    }

    // 5. 将状态机变量重置为初始状态
    *current_anchor_state = ANCHOR_STATE_AWAIT_POLL_RX;

    // 6. 重新启用接收器，因为 Anchor 的初始状态就是等待接收
    //    注意：如果之前设置了接收超时，可能需要在这里重新设置，
    //    但对于 Anchor 等待 Poll，通常不需要超时或设置很长的超时。
    //    dwt_setrxtimeout(0); // 禁用超时，或设置为合适的超时值

    // 7. 重新启用必要的 DW1000 中断
    dwt_setinterrupt(interrupt_mask, 1);
    dwt_setrxtimeout(0);
    dwt_rxenable(0); // 立即启用接收
}
