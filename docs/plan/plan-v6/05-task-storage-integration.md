# 任务与存储集成

## 目标

把 plan-v6 的新增功能按任务和数据通道拆开，避免初始化、Tag 拉取、丢包测试和 SD 写入互相阻塞。

## 任务拆分

| 任务             | 职责                                                                               |
| ---------------- | ---------------------------------------------------------------------------------- |
| `uwbAppTask`   | TWR 结果计算、Anchor 距离表缓存、DATA 会话业务处理                                 |
| `initTask`     | Anchor 初始化 token 状态机、3s 本机测距窗口、LED_LOCAL/LED_GLOBAL                  |
| `proxPullTask` | Tag 距离表拉取策略，40m 门限和 1s 全局节流                                         |
| `lossTestTask` | 按键进入持续丢包检测，1s 预热后按最近 3s 时间桶统计输出 LED 状态                 |
| `sdWriterTask` | 统一处理 SD block 写入                                                             |
| `keyTask`      | 工作阶段短按向 `lossTestTask` 发送启动命令，初始化阶段按键触发 Anchor 初始化     |
| `ledTask`      | 正常工作闪烁，测试/初始化状态下按状态机接管部分 LED                                |

初期可以把 `initTask` 和 `proxPullTask` 逻辑放在 `uwbAppTask` 中轮询实现，等状态稳定后再独立成文件。

## APP 数据流

当前 `UwbApp` 会把 UWB 样本写入 `DataService`。`DataService` 是单消费者队列，不能同时被排序线程和丢包检测线程消费。

测试阶段推荐：

```text
UwbApp  -> DataService -> dataSortTask
UwbLink -> RANGE_TX/RANGE_RX lossTestQueue -> lossTestTask -> ledCmdQueue -> ledTask
```

丢包检测使用独立队列，不抢占 `DataService`，因此可以和排序线程解耦。当前统计不再依赖 seq gap，也不依赖 APP 层测距结果，而是在最近 3s 时间桶内直接统计 LINK 层发布的 `RANGE_TX` 和 `RANGE_RX` 数。

Tag 默认可以继续运行 APP DATA 拉取。按键进入丢包检测时，Key 线程只启动 `lossTestTask`；不要求 `UwbApp_Task` 关闭 DATA 拉取或清当前 DATA 会话。DATA 帧不投递为 `RANGE_TX/RANGE_RX`，不会进入丢包统计。

后续正式版本推荐：

```text
UwbApp  -> UwbSampleBus -> dataSortTask
UwbLink -> LinkEventBus -> lossTestTask
                       -> realtime monitor
```

即把 APP 样本分发和 LINK 层事件分发拆开，避免多个消费者抢同一队列，同时保持丢包统计不依赖 APP 层测距结果。

## SD 16KB 双缓存

现有规划使用 16KB 双缓存：

```c
#define APP_SD_BLOCK_SIZE  (16U * 1024U)
```

该双缓存只用于普通 GNSS/IMU 采样数据。丢包检测不写 SD 日志，不使用独立 UWB SD FIFO。

## 丢包检测日志

当前测试阶段丢包检测只通过 LED 展示结果，不写 SD，也不输出以下日志：

```text
LOSS_TX
LOSS_FRAME
LOSS_SUMMARY
```

因此不存在 `uwb_sd_fifo_flush()` 要求。后续如果需要恢复离线分析，再单独增加编译开关和日志通道。

## 排序线程处理

功能测试阶段可以按需求屏蔽排序线程：

```text
AppTasks_CreateAll()
  -> 可不创建 AppDataSortTask
  -> UWB 丢包检测仍从 lossTestQueue 读取
```

当前测试阶段排序线程可以继续消费 `DataService`；丢包检测不写 SD，也不占用 `sdWriterTask`。正式采集阶段再打开排序线程写入，或改造为多消费者分发。

## 文件拆分建议

| 文件                           | 内容                                   |
| ------------------------------ | -------------------------------------- |
| `APP/UWB/uwb_init.c/h`       | Anchor 初始化 token 状态机             |
| `APP/UWB/uwb_prox_table.c/h` | 本地距离表结构、序列化、CRC            |
| `APP/UWB/uwb_prox_pull.c/h`  | Tag 拉取策略                           |
| `APP/UWB/uwb_loss_test.c/h`  | 丢包检测统计                           |
| `APP/task/app_tasks.c`       | 任务创建、按键通知、LED 状态接口       |
| `APP/UWB/uwb_protocol.h`     | 新增 INIT_TOKEN 和 PULL_PROX 控制类型  |

## 关键约束

- 距离表拉取和丢包测试的模式切换由后续实现确认；当前丢包统计本身不依赖 APP DATA 是否运行。
- 任意两次距离表拉取之间至少间隔 1s。
- Tag 侧 APP DATA 默认开启；进入丢包测试模式时不需要关闭 DATA 拉取，因为丢包统计只消费 LINK 层测距 TX/RX 事件。
- Anchor token 转发必须等普通 ACK；未收到 ACK 不结束本机初始化。
- 丢包测试按 `anchor_id + response_slot_id` 分开统计，LED 显示最近 3s 内收包率最高的一组。
- 丢包统计使用每个 Anchor/slot 的 `rx / tx` 计算收包率，不再用测距 seq gap 估算，也不做全局 seq 去重。
- 丢包检测前 1s 只接收 LINK 层事件不统计；之后使用最近 3s 时间桶驱动 LED。
- 丢包检测运行期间不写 SD 日志，统计结果只驱动 LED；DATA 帧不参与丢包统计。
- 当前测试阶段普通排序线程是否写 SD 按采集需求配置，丢包检测不占用 SD 双缓存。
