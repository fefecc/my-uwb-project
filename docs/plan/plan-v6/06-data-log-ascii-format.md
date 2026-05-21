# GNSS/IMU/UWB ASCII 数据日志方案

## 目标

统一 SD 卡日志和调试串口数据输出。所有采集数据先进入 `dataSort` 排序线程，按本地单调时钟排序后，使用同一个 ASCII formatter 生成数据行，再同时写入 SD FIFO 和当前调试串口 `USART1`。

串口输出和 SD 文件内容必须一致，不能维护两套格式字符串。普通调试日志可以继续走 `LogService`，数据日志行由排序线程统一输出。

## 输出文件

SD 卡数据文件使用递增编号，避免多次实验覆盖旧数据：

```text
gnss-imu-uwb-0001.log
gnss-imu-uwb-0002.log
...
gnss-imu-uwb-9999.log
```

打开文件时从 `0001` 开始查找第一个不存在的文件，使用 `FA_CREATE_NEW | FA_WRITE` 创建。当前 `StorageService_OpenNextLog()` 的命名建议改为这个格式；原独立 UWB debug 日志可以保留为调试用途，但正式采样数据只写入上面的统一文件。

## 数据流

```text
gnssTask   -> DataService queue
imuTask    -> DataService queue
uwbApp     -> DataService queue
             |
             v
dataSort    -> 按 TimeCapture.local_tick_20k 排序
             -> format_data_ascii()
             -> USART1 data output
             -> SD ASCII FIFO
             |
             v
sdWriter    -> gnss-imu-uwb-xxxx.log
```

排序键建议使用：

```text
(time_capture.local_tick_20k, enqueue_seq)
```

`local_tick_20k` 是主排序键；`enqueue_seq` 只用于同一 tick 内保持稳定顺序。`UWB_ANCHOR_DATA` 先作为一个完整表事件进入排序线程，排序完成后再按条目拆成多行输出。

## 通用 ASCII 行格式

每条数据一行，CSV，`\r\n` 结尾。字段顺序固定为：

```text
utc_week,utc_ms,utc_valid,mono_ms,msg_type,<payload...>\r\n
```

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `utc_week` | `uint32` | GNSS/UTC week；无 UTC 同步时为 `0` |
| `utc_ms` | `uint32` | week 内毫秒；无 UTC 同步时为 `0` |
| `utc_valid` | `uint8` | `1` 表示 UTC 有效，`0` 表示无效 |
| `mono_ms` | decimal string | 本地单调时间，单位 ms，保留 3 位小数；由 20 kHz tick 换算，不带单位 |
| `msg_type` | string | `GNSS`、`IMU`、`UWB_TWR`、`UWB_ANCHOR_DATA` |

当前 `TimeService` 已保存 `week/week_ms` 和 20 kHz 本地 tick。日志输出时把本地 tick 换算成 `mono_ms = local_tick_20k / 20.0`。这里不做日历字符串转换，离线分析时再把 `utc_week + utc_ms` 转换成需要的 UTC 表达。

数值约定：

| 类型 | 格式 |
| --- | --- |
| 地址/ID | `0x%04X` |
| DW1000 40-bit timestamp | `0x%010llX` |
| 普通整数 | 十进制 |
| 浮点距离/经纬度 | 十进制，保留足够精度 |

## 消息类型

### `IMU`

来源：`imuTask`

建议 payload：

```text
accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
```

完整行：

```text
utc_week,utc_ms,utc_valid,mono_ms,IMU,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
```

数据结构：`AppImuSample`

| 字段 | C 类型 | 原始大小 |
| --- | --- | --- |
| `accel[3]` | `int16_t[3]` | 6 B |
| `gyro[3]` | `int16_t[3]` | 6 B |
| 合计 |  | 12 B |

### `GNSS`

来源：`gnssTask`，当前使用 BESTNAV。

建议 payload：

```text
lat,lon,hgt,datum_id,lat_std,lon_std,hgt_std,pos_status,pos_type,diff_age,sol_age,svs_tracked,svs_in_sol
```

完整行：

```text
utc_week,utc_ms,utc_valid,mono_ms,GNSS,lat,lon,hgt,datum_id,lat_std,lon_std,hgt_std,pos_status,pos_type,diff_age,sol_age,svs_tracked,svs_in_sol
```

数据结构：`AppGnssSample`

| 字段 | C 类型 | 原始大小 |
| --- | --- | --- |
| `lat/lon/hgt` | `double * 3` | 24 B |
| `datum_id` | `uint32_t` | 4 B |
| `lat_std/lon_std/hgt_std` | `float * 3` | 12 B |
| `pos_status/pos_type` | `uint32_t * 2` | 8 B |
| `diff_age/sol_age` | `float * 2` | 8 B |
| `svs_tracked/svs_in_sol` | `uint8_t * 2` | 2 B |
| 合计 |  | 58 B，实际结构体会因 `double` 对齐约为 64 B |

### `UWB_TWR`

来源：`uwbApp` 的 TWR 测距结果。代码中使用 `APP_DATA_SRC_UWB_TWR`。

建议 payload：

```text
anchor_id,tag_id,exchange_seq,status_flags,distance_m,rx_pacc,fp_index,fp_ampl1,fp_ampl2,fp_ampl3,std_noise,max_noise
```

完整行：

```text
utc_week,utc_ms,utc_valid,mono_ms,UWB_TWR,anchor_id,tag_id,exchange_seq,status_flags,distance_m,rx_pacc,fp_index,fp_ampl1,fp_ampl2,fp_ampl3,std_noise,max_noise
```

时间戳使用测距结果里的 `frame_local_tick_20k`，即当前 `publish_range_result()` 里已经覆盖到 `node.time_capture.local_tick_20k` 的帧时间，而不是 formatter 运行时的时间。

数据结构：`AppUwbSample`

| 字段 | C 类型 | 原始大小 |
| --- | --- | --- |
| `anchor_id/tag_id/exchange_seq/status_flags` | `uint16_t * 4` | 8 B |
| `distance_m` | `double` | 8 B |
| `rx_pacc/fp_index/fp_ampl1/fp_ampl2/fp_ampl3/std_noise/max_noise` | `uint16_t * 7` | 14 B |
| 合计 |  | 30 B，实际结构体会因对齐约为 32 B |

### `UWB_ANCHOR_DATA`

来源：Tag 从 Anchor 拉取到的 Anchor 邻近表，也就是当前 prox table。代码中使用 `APP_DATA_SRC_UWB_ANCHOR_DATA`。

这个消息按“发送该表的基站 ID + 表内条目”拆行。一个 Anchor 表有 `entry_count` 条，就输出 `entry_count` 行；所有行使用同一个 DATA 会话启动时间，不再为每个分片或每个条目单独生成时间戳。

时间捕获建议：

```text
tag_start_session(anchor_id) 时保存 TimeCapture，字段名可用 start_time_capture。
DATA 表验证成功后，用 start_time_capture 构造 UWB_ANCHOR_DATA 节点进入 DataService。
```

如果 `entry_count == 0`，建议输出一条空表行，保留“该 Anchor 已返回空表”的事件。

建议 payload：

```text
source_anchor_id,table_seq,entry_index,entry_count,peer_anchor_id,dist_cm,dist_std_cm,avg_pacc,avg_fp_index,avg_fp_ampl1,avg_fp_ampl2,avg_fp_ampl3,avg_std_noise,avg_max_noise,avg_rx_power_dbm_x100,avg_fp_power_dbm_x100,samples,quality,flags,rx_error_flags,lde_status,total_len,total_frags,table_crc
```

完整行：

```text
utc_week,utc_ms,utc_valid,mono_ms,UWB_ANCHOR_DATA,source_anchor_id,table_seq,entry_index,entry_count,peer_anchor_id,dist_cm,dist_std_cm,avg_pacc,avg_fp_index,avg_fp_ampl1,avg_fp_ampl2,avg_fp_ampl3,avg_std_noise,avg_max_noise,avg_rx_power_dbm_x100,avg_fp_power_dbm_x100,samples,quality,flags,rx_error_flags,lde_status,total_len,total_frags,table_crc
```

当前 prox table 原始格式：

| 区域 | 大小 | 内容 |
| --- | --- | --- |
| Header | 8 B | `version(1), entry_count(1), self_anchor(2), table_seq(2), table_crc(2)` |
| Entry | 35 B/条 | 一个 peer anchor 的统计结果 |
| 最大表 | 288 B | `8 + 8 * 35`，当前 `PROX_TABLE_MAX_ENTRIES = 8` |

Entry 字段拆分：

| 偏移 | 字段 | 类型 | 大小 |
| --- | --- | --- | --- |
| 0 | `self_anchor` | `uint16_t` | 2 B |
| 2 | `peer_anchor` | `uint16_t` | 2 B |
| 4 | `dist_cm` | `uint16_t` | 2 B |
| 6 | `dist_std_cm` | `uint16_t` | 2 B |
| 8 | `avg_pacc` | `uint16_t` | 2 B |
| 10 | `avg_fp_index` | `uint16_t` | 2 B |
| 12 | `avg_fp_ampl1` | `uint16_t` | 2 B |
| 14 | `avg_fp_ampl2` | `uint16_t` | 2 B |
| 16 | `avg_fp_ampl3` | `uint16_t` | 2 B |
| 18 | `avg_std_noise` | `uint16_t` | 2 B |
| 20 | `avg_max_noise` | `uint16_t` | 2 B |
| 22 | `avg_rx_power_dbm_x100` | `int16_t` | 2 B |
| 24 | `avg_fp_power_dbm_x100` | `int16_t` | 2 B |
| 26 | `samples` | `uint8_t` | 1 B |
| 27 | `quality` | `uint8_t` | 1 B |
| 28 | `flags` | `uint8_t` | 1 B |
| 29 | `rx_error_flags` | `uint32_t` | 4 B |
| 33 | `lde_status` | `uint16_t` | 2 B |
| 合计 |  |  | 35 B |

建议运行时结构：

```c
#define APP_UWB_ANCHOR_DATA_MAX_ENTRIES  (8U)

typedef struct {
    uint16_t self_anchor;
    uint16_t peer_anchor;
    uint16_t dist_cm;
    uint16_t dist_std_cm;
    uint16_t avg_pacc;
    uint16_t avg_fp_index;
    uint16_t avg_fp_ampl1;
    uint16_t avg_fp_ampl2;
    uint16_t avg_fp_ampl3;
    uint16_t avg_std_noise;
    uint16_t avg_max_noise;
    int16_t avg_rx_power_dbm_x100;
    int16_t avg_fp_power_dbm_x100;
    uint8_t samples;
    uint8_t quality;
    uint8_t flags;
    uint32_t rx_error_flags;
    uint16_t lde_status;
} AppUwbAnchorEntry;

typedef struct {
    uint16_t source_anchor_id;
    uint16_t table_seq;
    uint16_t table_crc;
    uint16_t total_len;
    uint8_t total_frags;
    uint8_t entry_count;
    AppUwbAnchorEntry entries[APP_UWB_ANCHOR_DATA_MAX_ENTRIES];
} AppUwbAnchorDataSample;
```

`AppUwbAnchorEntry` 原始字段为 35 B，C 结构体实际会因对齐约为 36 B；8 条约 288 B，加表头后整个 `AppUwbAnchorDataSample` 约 300 B。加入 `AppDataNode` union 后，`DataService` 队列节点会明显变大，需要重新确认静态队列内存。

## 建议的 `AppDataSource`

```c
typedef enum {
    APP_DATA_SRC_NONE = 0,
    APP_DATA_SRC_GNSS,
    APP_DATA_SRC_IMU,
    APP_DATA_SRC_UWB_TWR,
    APP_DATA_SRC_UWB_ANCHOR_DATA,
} AppDataSource;
```

建议 `AppDataNode` 保持“一条事件一个节点”：

```c
typedef struct {
    AppDataSource source;
    TimeCapture time_capture;
    uint32_t enqueue_seq;
    union {
        AppGnssSample gnss;
        AppImuSample imu;
        AppUwbSample uwb_twr;
        AppUwbAnchorDataSample uwb_anchor_data;
    } payload;
} AppDataNode;
```

`UWB_ANCHOR_DATA` 不要在 UWB 线程里提前拆成多条节点；否则排序窗口会被单个表占满，也不利于保证整张表使用同一启动时间。

## 串口输出

使用当前调试串口 `USART1`，参数保持现有配置：

```text
460800, 8N1
```

数据行由 `dataSort` 输出，避免生产线程直接打印导致串口顺序和 SD 顺序不一致。实现上可以新增 `AppTasks_DataOutputWrite(line, len)`，内部同时写 `APP_USART_LOG_SLOT_DATA` 和 SD FIFO。

`APP_USART_LOG_SLOT_DATA` 调整为 4096 B，单条 `UWB_ANCHOR_DATA` 表最多拆出 8 行时不会立即挤掉整组数据。

## SD 写入

当前 SD FIFO 是 16 KB 双缓冲，保持满块写入即可：

```text
满 16 KB：立即通知 sdWriter 写块
未满 16 KB：暂存在 RAM 中，等待后续数据填满
```

实验结束前最后不足 16 KB 的尾部数据可以接受丢失，暂不增加 partial flush，减少 SD 写入路径复杂度。

## 示例

```text
2420,345678901,1,6172839.450,IMU,12,-8,16384,31,-22,4
2420,345678920,1,6172858.000,GNSS,31.230000000,121.470000000,12.3450,61,0.0120,0.0130,0.0210,0,56,0.000,0.000,28,21
2420,345678940,1,6172877.000,UWB_TWR,0x0021,0x0001,1024,0x0007,3.422,812,71,2230,2198,2251,121,311
2420,345679000,1,6172900.000,UWB_ANCHOR_DATA,0x0021,7,0,2,0x0022,342,12,900,64,2300,2210,2280,120,300,-6500,-6900,18,92,0x01,0x00000000,0x0040,78,2,0xA55C
2420,345679000,1,6172900.000,UWB_ANCHOR_DATA,0x0021,7,1,2,0x0023,527,18,850,69,2100,2080,2160,130,330,-6800,-7200,15,88,0x01,0x00000000,0x0040,78,2,0xA55C
```

## 实现顺序

1. 重命名/扩展 `AppDataSource`，把 UWB TWR 和 UWB Anchor Data 分开。
2. 新增 `AppUwbAnchorDataSample`，在 DATA 表 CRC 验证成功后发布一个完整表事件。
3. 给 `AppDataNode` 增加 `enqueue_seq`，排序键改为 `(local_tick_20k, enqueue_seq)`。
4. 把 formatter 收敛成一个函数，SD 和 USART1 共用同一行。
5. `dataSort` 排序后同时写串口和 SD FIFO。
6. `StorageService_OpenNextLog()` 改为 `gnss-imu-uwb-%04lu.log`。
