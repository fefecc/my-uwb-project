# Tag 距离表拉取策略

## 目标

Tag 在正常工作阶段分别向每个 Anchor 拉取一次本地距离表。拉取必须有节流策略，避免进场瞬间多个 DATA 会话连续触发影响 TWR。

## 触发条件

Tag 对 Anchor 的距离表拉取需要同时满足：

```text
1. 已经发现 Anchor，并有有效 TWR 距离
2. Anchor 距离小于 40m
3. 连续若干次测距满足距离条件
4. 该 Anchor 对应 init_seq/table_version 尚未拉取
5. 距离上一次任意拉取已经 >= 1s
6. 当前没有正在进行的 DATA 会话
7. 当前没有正在运行丢包测试
```

建议距离防抖：

```c
#define TAG_PROX_PULL_DISTANCE_CM      4000U
#define TAG_PROX_PULL_STABLE_COUNT     3U
#define TAG_PROX_PULL_MIN_INTERVAL_MS  1000U
```

1s 间隔是全局间隔，不是每个 Anchor 独立间隔。

## 拉取记录

Tag 维护每个 Anchor 的拉取状态：

```c
typedef struct {
    bool     valid;
    uint16_t anchor_id;
    uint16_t pulled_init_seq;
    uint8_t  stable_near_count;
    bool     pulled;
    uint32_t last_seen_ms;
} tag_prox_pull_record_t;
```

如果 Anchor 的 `init_seq` 更新，允许重新拉取。

## 拉取流程

复用 DATA 会话，但新增业务类型为“拉取 Anchor 本地距离表”：

```text
Tag -> Anchor: DATA_CFG / PULL_PROX
Anchor -> Tag: ACK
Tag -> Anchor: GET_INFO
Anchor -> Tag: META
Tag -> Anchor: PULL frag=1...
Anchor -> Tag: prox_table_t fragments
Tag -> Anchor: DONE
```

当前表约 38B，通常 1 个 data frag 即可完成；仍保留分片机制，兼容后续扩展。

## 和 TWR 的关系

拉取距离表是低频后台动作，不能持续抢占 TWR：

- 任意两次拉取至少间隔 1s。
- DATA 会话期间允许 TWR 频率下降，但不能连续排队多个拉取。
- 丢包测试期间禁止触发拉取，避免测试结果被 DATA 会话干扰。

## SD 记录

Tag 收到 `prox_table_t` 后写入 SD。建议文件名包含 Anchor ID 和 init_seq：

```text
uwb-prox-0031-seq-0012.bin
uwb-prox-0032-seq-0012.bin
```

也可以同时写一条 ASCII 摘要日志：

```text
PROX,anchor=0x0031,init_seq=0x0012,entries=3,crc=0xA1B2
```
