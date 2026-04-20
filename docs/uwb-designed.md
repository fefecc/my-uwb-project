# 新 UWB 协议栈简化大纲

## 1. 总体目标

协议栈分三层：

- 物理层：`phy_thread` 单线程控制 DW1000，负责 IRQ 事件、时间戳、RX/TX buffer、RX/TX 启动和质量参数读取。
- 数据链路层：`uwb_link_thread` 单线程负责调度、状态机、发现、测距交换、MAC/公共头/扩展头打包解析、重传、时间槽、共享槽和应用数据分包。
- 应用层：负责业务 payload 语义、测距归并、距离计算、质量归并、拓扑/业务计算和上报。

应用层不直接打 UWB 空口帧。链路层可以解析和改写 header，但除发现、测距、应用数据控制等特殊功能帧外，不解析普通业务 payload。

## 2. 总体结构

```text
+------------------------------+
| 应用层                       |
| 业务 payload / 距离 / 拓扑 / 上报 |
+-------------^----------------+
              | 规整事件 / 数据块 / 结果
+-------------+----------------+
| 数据链路层                   |
| uwb_link_thread              |
| 头部打包解析 / 调度 / 重传 / 时间槽 |
+-------------^----------------+
              | 物理命令 / 物理事件 / 共享区
+-------------+----------------+
| 物理层                       |
| phy_thread 控制 DW1000        |
| IRQ / RX/TX / 时间戳 / 质量参数 |
+-------------^----------------+
              |
| DW1000                       |
+------------------------------+
```

## 3. 线程模型

| 入口 | 职责 |
| --- | --- |
| `dw1000_irq_handler` | 只做硬件中断捕获和轻量记录，唤醒 `phy_thread` |
| `phy_thread` | 唯一直接访问 DW1000 的线程 |
| `uwb_link_thread` | 唯一链路线程，不拆 `link_rx_thread` / `link_tx_thread` |
| `app_thread` | 应用计算和业务处理线程 |

`phy_thread` 主循环：

```text
等待 IRQ 或物理命令
  -> 读取 DW1000 状态和时间戳
  -> 处理 RX/TX/错误事件
  -> 读写 RX/TX buffer
  -> 执行 RX_ENABLE / TX_NOW / TX_DELAYED
  -> 写物理共享区并通知链路线程
```

`uwb_link_thread` 主循环：

```text
读取物理事件
  -> 扫描时间槽
  -> 处理 RX/TX/错误事件
  -> 处理应用命令和共享槽
  -> 生成物理控制命令
  -> 等待下一事件或最近时间槽
```

等待 `TX_DONE` 时不能死等。进入 `LINK_TX_WAIT_DONE` 后可以扫描时间槽、搬运事件和预打包下一帧，但不能生成新的 TX 物理命令，也不能回收当前发送槽。

## 4. 物理层

物理层只封装 DW1000 硬件能力：

- 读取本地时间、DW1000 40-bit 时间戳、`SYS_STATUS`、`RX_TIME`、`TX_TIME`。
- 读写 RX/TX buffer，启动 immediate TX / delayed TX，打开 RX 和设置 timeout。
- 读取 CIR、首径功率、接收功率、噪声、LDE 状态等质量参数。
- 向链路层提供轻量物理事件。

物理层不解析功能码、MAC 头、公共协议头、扩展头或业务 payload，不维护发现表，不做重传和距离计算。

物理事件：

```c
typedef enum {
    PHY_EVT_NONE = 0,
    PHY_EVT_RX_OK,
    PHY_EVT_TX_DONE,
    PHY_EVT_RX_TIMEOUT,
    PHY_EVT_RX_ERROR,
    PHY_EVT_TX_ERROR
} phy_event_type_t;
```

## 5. 物理层和链路层通信

链路层通过“物理命令队列 + 物理共享区 + 轻量通知”使用物理层，不直接调用 DW1000 驱动。

物理命令至少包含：

```text
cmd_type
tx_delay_time
rx_slot_interval / rx_slot_width / rx_slot_count / rx_slot_base_id
tx_buf / tx_len
```

接收槽规则：

```text
slot_id = rx_slot_base_id + slot_index
slot_start = base_time + slot_index * rx_slot_interval
slot_end = slot_start + rx_slot_width
```

物理到链路共享槽至少包含：

```text
slot_id / generation / owner / state / valid
rx_len / rx_data / rx_ts / irq_local_time
quality / error_flags
```

所有权规则：

1. 初始为 `FREE/EMPTY`。
2. `phy_thread` 写入前切到 `PHY/WRITING`。
3. 写完后递增 `generation`，切到 `LINK/READY`，通知链路层。
4. 通知只传 `event_type / slot_id / generation / status / timestamp`。
5. 链路层校验 `generation`，消费后释放为 `FREE/EMPTY`。
6. 槽未释放时物理层不能覆盖，只能统计 overflow 或上报错误。

## 6. 数据链路层

链路层负责：

- 消费物理事件和物理共享区。
- 生成物理控制命令。
- 解析/打包 MAC 头、公共协议头和扩展头。
- 快速改写 `SEQ`、`flags`、`retry_count`、分片序号、控制类型和长度字段。
- 处理特殊功能帧：发现、测距、应用数据控制。
- 对普通业务 payload 只做透传、分片、重组、校验和重传。
- 维护 `link_neighbor_table`、`range_candidate_list`、时间槽、共享槽和调度。

链路层不计算距离，不做测距归并，不解析普通业务 payload 的业务语义，不暴露原始中断给应用层。

链路状态：

| 状态 | 含义 |
| --- | --- |
| `LINK_IDLE` | 空闲，可生成 TX/RX 物理命令 |
| `LINK_RX_ON` | RX 命令已下发，等待物理事件 |
| `LINK_RX_PROCESS` | 处理收到的数据包 |
| `LINK_TX_PREPARE` | 打包或准备发送 |
| `LINK_TX_WAIT_DONE` | TX 命令已下发，等待 `PHY_EVT_TX_DONE` |
| `LINK_RECOVER` | 错误恢复 |

## 7. 帧结构

基础 MAC 头：

```text
FC | SEQ | PAN ID | DST16 | SRC16
```

公共协议头：

```text
proto_ver | func_code | flags | ext_header_len | payload_len
```

公共头只放稳定字段；功能参数放扩展头；业务数据放 payload。新增参数优先用 `flags` 标记能力，并用 `ext_header_len` 跳过未知扩展字段。

帧类型：

| 帧类型 | 扩展头 |
| --- | --- |
| `DISCOVERY_REQ` | `discovery_epoch + resp_slot_base + resp_slot_width + resp_slot_count` |
| `DISCOVERY_RESP` | `discovery_epoch + short_id + capability` |
| `TWR_TAG_START` | `exchange_seq + target_short_id + resp_slot_id + resp_slot_width` |
| `TWR_ANCHOR_RESP` | `exchange_seq + anchor_rx_ts + anchor_tx_ts` |
| `APP_DATA_CFG` | `app_msg_id + session_id + total_len + block_size + frag_size + frag_cnt + block_crc32 + slice_policy` |
| `APP_DATA_FRAG` | `app_msg_id + session_id + block_seq + frag_seq + retry_count + frag_payload_len + frag_payload` |
| `APP_DATA_CTRL` | `app_msg_id + session_id + ctrl_type + ctrl_payload_len + ctrl_payload` |

功能码分组：

| 分组 | 范围 |
| --- | --- |
| 链路控制 | `0x00 - 0x1F` |
| 设备发现 | `0x20 - 0x3F` |
| 测距流程 | `0x40 - 0x6F` |
| 应用数据 | `0x70 - 0x8F` |
| 调试诊断 | `0xB0 - 0xCF` |

## 8. 发现流程

- 每 1s 发送 `DISCOVERY_REQ`。
- `DISCOVERY_REQ` 声明本轮发现应答窗口。
- Anchor 按 `short_id` 映射到应答槽，回复 `DISCOVERY_RESP`。
- `LINK_TIMEOUT_DISCOVERY_RESP` 关闭本轮发现窗口。
- 链路层维护 `link_neighbor_table`，并生成 `range_candidate_list`。
- 应用层只接收发现表快照或设备变化事件。

## 9. 测距流程

空口只保留一次交换：

```text
TWR_TAG_START -> TWR_ANCHOR_RESP
```

链路层每次只上报一次完整交换事件。应用层归并同一 Anchor 的相邻两次交换后计算距离。

响应窗口由 Tag 指定：

- 单 Anchor：`TWR_TAG_START` 携带 `target_short_id` 和响应窗口。
- 多 Anchor：Tag 声明一组窗口，Anchor 按 `short_id` 映射。
- Anchor 只在被指定或映射到的窗口内发送 `TWR_ANCHOR_RESP`。

单次交换事件：

```c
typedef struct {
    uint16_t anchor_id;
    uint16_t exchange_seq;
    uint16_t response_slot_id;
    uint16_t status_flags;
    uint64_t tag_tx_ts;
    uint64_t anchor_rx_ts;
    uint64_t anchor_tx_ts;
    uint64_t tag_rx_ts;
    uwb_rx_quality_t quality;
    uint8_t retry_count;
} app_twr_exchange_t;
```

`anchor_rx_ts/anchor_tx_ts` 来自 `TWR_ANCHOR_RESP`，`tag_tx_ts/tag_rx_ts` 由链路层绑定到同一 `exchange_seq`。

相邻两次交换的计算输入：

```text
t3 = 上一条 anchor_tx_ts
t4 = 上一条 tag_rx_ts
t5 = 当前 tag_tx_ts
t6 = 当前 anchor_rx_ts
t7 = 当前 anchor_tx_ts
t8 = 当前 tag_rx_ts

Ra = t6 - t3
Da = t5 - t4
Rb = t8 - t5
Db = t7 - t6
tof = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db)
distance = tof * DW_TIME_UNIT * SPEED_OF_LIGHT - antenna_delay_comp
```

时间戳差值必须处理 DW1000 40-bit 回绕；乘法中间值要避免 64-bit 溢出。

## 10. 应用数据交互

应用层提交原始业务 payload 和控制命令；链路层负责头部、分片、重传和控制帧。

基础框架：

1. `APP_DATA_CFG` 声明会话、长度、块大小、分片大小、校验和切片策略。
2. `APP_DATA_FRAG` 承载 payload 分片。
3. 接收方按 `app_msg_id + session_id + block_seq` 建立重组上下文。
4. 确认、缺片反馈和重传策略挂在 `APP_DATA_CTRL` 上。
5. 策略字段、bitmap、窗口大小、最大重传次数等参数后续按能力扩展。

重传时链路层只快速改写 `retry_count`、`flags` 和必要的 MAC `SEQ`，不重新请求应用层打包。`TX_DONE` 不释放应用数据槽，当前块槽只在确认策略判定成功后释放。

## 11. 应用层和链路层共享槽

应用层和链路层之间共享内存放数据，队列只传槽号和轻量事件：

```text
app_to_link_slots[N]  -> app_to_link_queue(slot_id, generation, event_type)
link_to_app_slots[M]  -> link_to_app_queue(slot_id, generation, event_type)
```

应用到链路槽关键字段：

```text
slot_id / generation / owner / state
cmd_type / target_id / send_policy
session_id / block_seq / frag_seq
payload_len / payload
retry_count_offset / mutable_flags_offset
```

双槽规则：

- `current_slot` 保存当前发送或等待确认的数据块。
- `backup_slot` 保存下一块或重传备份。
- `current_slot` 不因 `TX_DONE` 释放。
- 两个槽都占用时，应用层新数据等待或返回 busy。

## 12. 时间槽和调度

时间槽：

```text
LINK_TIMEOUT_RX_RANGE_RESP
LINK_TIMEOUT_DISCOVERY_RESP
LINK_TIMEOUT_APP_LINK_ACK
LINK_TIMEOUT_RETRY_BACKOFF
LINK_TIMEOUT_TX_DONE_GUARD
```

规则：

- 等待应答、ACK、TX_DONE 保护时申请时间槽。
- 收到对应事件后取消。
- 到期只产生轻量超时事件，由链路状态机处理。
- 重传时取消旧槽并申请新槽。

发送调度优先级：

1. 等待 `TX_DONE` 时不生成新的 TX 命令。
2. 必要应答优先，例如 `DISCOVERY_RESP`、`TWR_ANCHOR_RESP`、`CTRL_ACK/CTRL_NACK`。
3. 到期重传优先于新发送。
4. 发现帧按 1s 周期发送。
5. 无应用数据时持续测距。
6. 有应用数据时按“测距交换 -> 数据分片”交替。
7. 无可发送内容时下发 RX 命令。

## 13. 质量参数

`phy_thread` 读取质量参数，链路层随 RX 事件上报应用层。建议包含：

```text
rx_pacc / fp_index / fp_ampl1 / fp_ampl2 / fp_ampl3
cir_pwr / rx_power / fp_power
std_noise / max_noise / lde_status / rx_error_flags
```

应用层对相邻两次测距交换的质量做归并，输出 `range_quality`。

## 14. 应用层

应用层负责：

- 发现表快照、应用侧 Anchor 视图。
- `APP_EVT_TWR_EXCHANGE` 消费、相邻交换归并、距离计算和质量归并。
- 普通业务 payload 语义解析、拓扑、外部命令和结果上报。
- 提交待发送原始应用数据给链路层。

应用层不直接操作 DW1000，不发送发现帧，不打包空口帧，不修改 MAC/公共头/链路扩展头、`retry_count` 或重传状态。

## 15. 落地顺序

1. 建立 `phy_thread`，跑通 RX/TX、中断、时间戳和质量参数。
2. 建立 `uwb_link_thread`、物理命令队列、物理共享区和事件通知。
3. 实现 MAC 头、公共协议头和扩展头的打包、解析和快速改写。
4. 实现发现流程和 `link_neighbor_table`。
5. 实现 `TWR_TAG_START -> TWR_ANCHOR_RESP`。
6. 应用层实现相邻测距交换归并和距离计算。
7. 数据链路层实现 `APP_DATA_CFG`、`APP_DATA_FRAG` 和 `APP_DATA_CTRL` 框架。
8. 补充确认、缺片反馈、重传和 `retry_count` 快速改写策略。
9. 应用层做业务 payload 解析、质量归并、拓扑计算和结果上报。

## 16. 最终边界

```text
物理层：
  phy_thread 单线程控制 DW1000，负责事件、时间戳、RX/TX 和质量参数。

数据链路层：
  uwb_link_thread 单线程完成物理命令、共享区消费、发现、测距、头部解析打包、
  header 快速改写、分包、时间槽、重传、共享槽、邻居表和调度。
  除特殊功能帧外，不解析普通业务 payload 的语义。

应用层：
  负责普通业务 payload 语义、测距归并、距离计算、质量归并、拓扑、业务命令和上报。
```
