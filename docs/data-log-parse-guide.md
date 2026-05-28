# SD 数据日志解析说明

本文档描述当前固件写入 SD 卡的正式数据日志格式，用于离线解析 `gnss-imu-uwb-xxxx.log` 文件。

## 文件

日志文件名由固件自动递增创建：

```text
gnss-imu-uwb-0001.log
gnss-imu-uwb-0002.log
...
gnss-imu-uwb-9999.log
```

文件内容是文本格式，每条数据一行，字段用英文逗号 `,` 分隔，行尾为 `\r\n`。解析时可按 CSV 的简单逗号分隔处理；当前字段中没有带逗号的字符串字段。

## 通用字段

每一行前 5 个字段固定：

```text
utc_week,utc_ms,utc_valid,mono_ms,msg_type,<payload...>
```

| 序号 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | `utc_week` | uint32 | UTC/GNSS 周。没有 UTC 同步时为 `0`。 |
| 1 | `utc_ms` | uint32 | 当前周内毫秒。没有 UTC 同步时为 `0`。 |
| 2 | `utc_valid` | uint8 | `1` 表示 UTC 有效，`0` 表示 UTC 无效。 |
| 3 | `mono_ms` | decimal | 本地单调时间，单位 ms，来自 20 kHz 本地 tick，保留 3 位小数。 |
| 4 | `msg_type` | string | 数据类型名。见下方各类型。 |

当前支持的 `msg_type`：

```text
GNSS
IMU
UWB_TWR
UWB_ANCHOR_DATA
```

解析建议：先读取第 4 个字段 `msg_type`，再按对应字段表解析后续 payload。

## IMU

示例：

```text
0,0,0,27.800,IMU,171,-578,16722,44,-2161,2324
```

完整格式：

```text
utc_week,utc_ms,utc_valid,mono_ms,IMU,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
```

| 序号 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 5 | `accel_x` | int16 | 加速度 X 原始值。 |
| 6 | `accel_y` | int16 | 加速度 Y 原始值。 |
| 7 | `accel_z` | int16 | 加速度 Z 原始值。 |
| 8 | `gyro_x` | int16 | 陀螺仪 X 原始值。 |
| 9 | `gyro_y` | int16 | 陀螺仪 Y 原始值。 |
| 10 | `gyro_z` | int16 | 陀螺仪 Z 原始值。 |

## GNSS

完整格式：

```text
utc_week,utc_ms,utc_valid,mono_ms,GNSS,lat,lon,hgt,datum_id,lat_std,lon_std,hgt_std,pos_status,pos_type,diff_age,sol_age,svs_tracked,svs_in_sol
```

| 序号 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 5 | `lat` | double | 纬度，单位 degree，输出 9 位小数。 |
| 6 | `lon` | double | 经度，单位 degree，输出 9 位小数。 |
| 7 | `hgt` | double | 高程，输出 4 位小数。 |
| 8 | `datum_id` | uint32 | 坐标基准 ID。 |
| 9 | `lat_std` | float | 纬度标准差，输出 4 位小数。 |
| 10 | `lon_std` | float | 经度标准差，输出 4 位小数。 |
| 11 | `hgt_std` | float | 高程标准差，输出 4 位小数。 |
| 12 | `pos_status` | uint32 | GNSS 位置状态。 |
| 13 | `pos_type` | uint32 | GNSS 解类型。 |
| 14 | `diff_age` | float | 差分龄期，输出 3 位小数。 |
| 15 | `sol_age` | float | 解龄期，输出 3 位小数。 |
| 16 | `svs_tracked` | uint8 | 跟踪卫星数。 |
| 17 | `svs_in_sol` | uint8 | 参与解算卫星数。 |

## UWB_TWR

这是 Tag 和 Anchor 的 UWB 测距结果。

完整格式：

```text
utc_week,utc_ms,utc_valid,mono_ms,UWB_TWR,anchor_id,tag_id,exchange_seq,status_flags,distance_m,rx_pacc,fp_index,fp_ampl1,fp_ampl2,fp_ampl3,std_noise,max_noise
```

| 序号 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 5 | `anchor_id` | hex uint16 | Anchor 短地址，格式 `0x%04X`。 |
| 6 | `tag_id` | hex uint16 | Tag 短地址，格式 `0x%04X`。 |
| 7 | `exchange_seq` | uint16 | 测距交换序号。 |
| 8 | `status_flags` | hex uint16 | 测距状态标志，格式 `0x%04X`。 |
| 9 | `distance_m` | double | 距离，单位 m，输出 3 位小数。 |
| 10 | `rx_pacc` | uint16 | DW1000 接收前导累加计数。 |
| 11 | `fp_index` | uint16 | First Path index。 |
| 12 | `fp_ampl1` | uint16 | First Path amplitude 1。 |
| 13 | `fp_ampl2` | uint16 | First Path amplitude 2。 |
| 14 | `fp_ampl3` | uint16 | First Path amplitude 3。 |
| 15 | `std_noise` | uint16 | 标准噪声。 |
| 16 | `max_noise` | uint16 | 最大噪声。 |

`status_flags` 当前定义：

| bit | 宏 | 说明 |
| --- | --- | --- |
| 0 | `APP_UWB_STATUS_FLAG_TWR_DS` | 使用 DS-TWR 结果。 |
| 1 | `APP_UWB_STATUS_FLAG_TWR_DS_SHORT` | DS-TWR 短间隔结果。 |
| 2 | `APP_UWB_STATUS_FLAG_TWR_DS_LONG` | DS-TWR 长间隔结果。 |

## UWB_ANCHOR_DATA

这是感知表/邻近表数据。代码中叫 `prox table`，最终日志类型名是：

```text
UWB_ANCHOR_DATA
```

一张表可能包含多条 entry。固件会把一张表拆成多行输出：

- `entry_count > 0` 时输出 `entry_count` 行。
- `entry_count == 0` 时也输出 1 行空表记录，peer 相关字段为 0。
- 同一张表的多行具有相同的 `source_anchor_id`、`table_seq`、`entry_count`、`total_len`、`total_frags`、`table_crc` 和时间字段。
- `entry_index` 从 `0` 开始。

完整格式：

```text
utc_week,utc_ms,utc_valid,mono_ms,UWB_ANCHOR_DATA,source_anchor_id,table_seq,entry_index,entry_count,peer_anchor_id,dist_cm,dist_std_cm,avg_pacc,avg_fp_index,avg_fp_ampl1,avg_fp_ampl2,avg_fp_ampl3,avg_std_noise,avg_max_noise,avg_rx_power_dbm_x100,avg_fp_power_dbm_x100,samples,quality,flags,rx_error_flags,lde_status,total_len,total_frags,table_crc
```

| 序号 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 5 | `source_anchor_id` | hex uint16 | 发送这张表的 Anchor ID，格式 `0x%04X`。 |
| 6 | `table_seq` | uint16 | 感知表序号。 |
| 7 | `entry_index` | uint8 | 当前行在表内的 entry 下标，从 `0` 开始。 |
| 8 | `entry_count` | uint8 | 这张表内 entry 总数，最大 8。 |
| 9 | `peer_anchor_id` | hex uint16 | 当前 entry 对应的邻近 Anchor ID，格式 `0x%04X`。空表时为 `0x0000`。 |
| 10 | `dist_cm` | uint16 | 到邻近 Anchor 的平均距离，单位 cm。 |
| 11 | `dist_std_cm` | uint16 | 距离标准差，单位 cm。 |
| 12 | `avg_pacc` | uint16 | 平均接收前导累加计数。 |
| 13 | `avg_fp_index` | uint16 | 平均 First Path index。 |
| 14 | `avg_fp_ampl1` | uint16 | 平均 First Path amplitude 1。 |
| 15 | `avg_fp_ampl2` | uint16 | 平均 First Path amplitude 2。 |
| 16 | `avg_fp_ampl3` | uint16 | 平均 First Path amplitude 3。 |
| 17 | `avg_std_noise` | uint16 | 平均标准噪声。 |
| 18 | `avg_max_noise` | uint16 | 平均最大噪声。 |
| 19 | `avg_rx_power_dbm_x100` | int16 | 平均 RX power，单位 dBm * 100。例：`-6500` 表示 `-65.00 dBm`。 |
| 20 | `avg_fp_power_dbm_x100` | int16 | 平均 FP power，单位 dBm * 100。 |
| 21 | `samples` | uint8 | 参与统计的有效样本数。 |
| 22 | `quality` | uint8 | 质量评分，范围通常为 0 到 100。 |
| 23 | `flags` | hex uint8 | entry 状态标志，格式 `0x%02X`。 |
| 24 | `rx_error_flags` | hex uint32 | 接收错误标志 OR 汇总，格式 `0x%08lX`。 |
| 25 | `lde_status` | hex uint16 | LDE 状态 OR 汇总，格式 `0x%04X`。 |
| 26 | `total_len` | uint16 | 原始 prox table 总长度，单位 byte。 |
| 27 | `total_frags` | uint8 | UWB DATA 传输时的总分片数。 |
| 28 | `table_crc` | hex uint16 | 原始 prox table CRC，格式 `0x%04X`。 |

`flags` 当前定义：

| bit | 值 | 说明 |
| --- | --- | --- |
| 0 | `0x01` | entry 有效。 |
| 1 | `0x02` | 样本数低于阈值。 |
| 2 | `0x04` | 统计过程中出现过 RX error。 |

感知表原始二进制布局，仅用于理解 `total_len` 和 CRC，不是 SD 文件落盘格式：

```text
Header: 8 bytes
  version       uint8
  entry_count   uint8
  self_anchor   uint16 little-endian
  table_seq     uint16 little-endian
  table_crc     uint16 little-endian

Entry: 35 bytes each
  self_anchor              uint16 little-endian
  peer_anchor              uint16 little-endian
  dist_cm                  uint16 little-endian
  dist_std_cm              uint16 little-endian
  avg_pacc                 uint16 little-endian
  avg_fp_index             uint16 little-endian
  avg_fp_ampl1             uint16 little-endian
  avg_fp_ampl2             uint16 little-endian
  avg_fp_ampl3             uint16 little-endian
  avg_std_noise            uint16 little-endian
  avg_max_noise            uint16 little-endian
  avg_rx_power_dbm_x100    int16 little-endian
  avg_fp_power_dbm_x100    int16 little-endian
  samples                  uint8
  quality                  uint8
  flags                    uint8
  rx_error_flags           uint32 little-endian
  lde_status               uint16 little-endian
```

因此：

```text
total_len = 8 + entry_count * 35
```

## 解析注意事项

1. 十六进制字段带 `0x` 前缀，解析时按 base 16 转整数。
2. `mono_ms` 是本地单调时间，不是 UTC 时间；做传感器对齐时优先用它。
3. 当 `utc_valid == 0` 时，`utc_week` 和 `utc_ms` 不可用于绝对时间。
4. `UWB_ANCHOR_DATA` 的多行表示同一张表的多个 entry，可用 `(source_anchor_id, table_seq, table_crc)` 分组。
5. SD 写入按 16 KB 块写入，实验结束时最后不足 16 KB 的尾部数据可能不会落盘。

## Python 解析示例

```python
def parse_int(text):
    return int(text, 16) if text.startswith(("0x", "0X")) else int(text)


def parse_line(line):
    fields = line.strip().split(",")
    if len(fields) < 5:
        return None

    record = {
        "utc_week": int(fields[0]),
        "utc_ms": int(fields[1]),
        "utc_valid": int(fields[2]),
        "mono_ms": float(fields[3]),
        "msg_type": fields[4],
    }

    t = record["msg_type"]
    p = fields[5:]

    if t == "IMU":
        record.update({
            "accel_x": int(p[0]),
            "accel_y": int(p[1]),
            "accel_z": int(p[2]),
            "gyro_x": int(p[3]),
            "gyro_y": int(p[4]),
            "gyro_z": int(p[5]),
        })
    elif t == "UWB_ANCHOR_DATA":
        record.update({
            "source_anchor_id": parse_int(p[0]),
            "table_seq": int(p[1]),
            "entry_index": int(p[2]),
            "entry_count": int(p[3]),
            "peer_anchor_id": parse_int(p[4]),
            "dist_cm": int(p[5]),
            "dist_std_cm": int(p[6]),
            "avg_pacc": int(p[7]),
            "avg_fp_index": int(p[8]),
            "avg_fp_ampl1": int(p[9]),
            "avg_fp_ampl2": int(p[10]),
            "avg_fp_ampl3": int(p[11]),
            "avg_std_noise": int(p[12]),
            "avg_max_noise": int(p[13]),
            "avg_rx_power_dbm_x100": int(p[14]),
            "avg_fp_power_dbm_x100": int(p[15]),
            "samples": int(p[16]),
            "quality": int(p[17]),
            "flags": parse_int(p[18]),
            "rx_error_flags": parse_int(p[19]),
            "lde_status": parse_int(p[20]),
            "total_len": int(p[21]),
            "total_frags": int(p[22]),
            "table_crc": parse_int(p[23]),
        })

    return record
```
