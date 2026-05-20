# Anchor 本地距离表

## 目标

每个 Anchor 只保存自己与周围 3 个 Anchor 的距离关系。Tag 后续分别向每个 Anchor 拉取本地表，再由上位机或 SD 后处理重建完整环境关系。

不在 Anchor 之间传递全局距离表。

## 测距窗口

每个 Anchor 收到初始化 token 后，对其他 3 个 Anchor 测距约 3s。

```c
#define INIT_ANCHOR_COUNT       4U
#define INIT_PEER_COUNT         3U
#define INIT_RANGING_MS         3000U
#define INIT_MIN_VALID_SAMPLES  8U
```

3s 到时无论样本是否满额，都生成当前可用的本地表。样本不足的 peer 记录为低质量或无效。

## 样本累积

不保存原始样本，只保存累积量：

```c
typedef struct {
    bool     valid;
    uint16_t peer_anchor;
    uint16_t sample_count;
    uint32_t pacc_sum;
    uint32_t valid_count;
    double   distance_sum_m;
    double   distance_sq_sum_m;
} init_peer_accum_t;
```

每次 TWR 成功后更新对应 peer：

```c
accum->distance_sum_m    += distance_m;
accum->distance_sq_sum_m += distance_m * distance_m;
accum->pacc_sum          += rx_pacc;
accum->valid_count++;
accum->sample_count++;
```

## 表结构

表大小远小于 512B，但仍按显式序列化处理，避免 C struct 对齐、float 和端序问题。

```c
#define PROX_TABLE_VERSION      1U
#define PROX_TABLE_MAX_ENTRIES  3U

typedef struct __attribute__((packed)) {
    uint16_t self_anchor;
    uint16_t peer_anchor;
    uint16_t dist_cm;
    uint16_t avg_pacc;
    uint8_t  sample_count; /* 超过 255 时截断 */
    uint8_t  quality;      /* 0-100 */
} prox_entry_t;

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  entry_count;
    uint16_t source_anchor;
    uint16_t init_seq;
    uint16_t payload_crc16;
    prox_entry_t entries[PROX_TABLE_MAX_ENTRIES];
} prox_table_t;
```

当前结构大小约 38B，远低于 DATA 512B 缓存限制。

## 质量评估

`quality` 使用 0-100 分。推荐先用简单规则：

```text
sample_score = min(valid_count, 64) * 100 / 64
pacc_score   = clamp(avg_pacc, 0..255) * 100 / 255
std_score    = distance_std_cm <= 20 ? 100 :
               distance_std_cm >= 100 ? 0 :
               100 - (distance_std_cm - 20) * 100 / 80

quality = 50% sample_score + 30% pacc_score + 20% std_score
```

样本数小于 `INIT_MIN_VALID_SAMPLES` 时，该条记录仍可保留，但 `quality` 置低，并设置状态标记为不可靠。

## 本地缓存

Anchor 初始化完成后把 `prox_table_t` 缓存在本机业务缓存中，等待 Tag 拉取。

推荐维护：

```c
typedef struct {
    bool         valid;
    uint16_t     init_seq;
    uint32_t     updated_ms;
    prox_table_t table;
} anchor_prox_cache_t;
```

缓存更新必须是原子语义：先构建临时表，CRC 校验字段填完后再替换全局缓存。

## Tag 拉取兼容

Tag 拉取时 Anchor 直接把本地 `prox_table_t` 作为 DATA payload 发送。即使后续扩展字段，也保持 `version` 和 `entry_count` 可解析。
