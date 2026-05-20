# Anchor 初始化令牌

## 目标

Anchor 初始化通过按键触发。触发设备作为 origin，负责启动一轮 `INIT_TOKEN` 环形传递，并在 token 回到自己时判断整体初始化完成。

其他 Anchor 只关心本机初始化：收到 token 后开始本机测距，生成本地距离表，通知下一个 Anchor 并收到 ACK 后结束本机初始化。

## LED 语义

| LED | 工作阶段含义 |
|-----|--------------|
| LED_LOCAL | 本机初始化建立中 |
| LED_GLOBAL | 整体初始化未闭环，仅 origin 使用 |
| LED_WORK | 正常工作闪烁 |
| LED_LOSS_* | 工作阶段丢包测试阈值灯，见丢包测试文档 |

建议映射时保留 `LED_WORK` 给当前已有工作闪烁灯；其余 LED 由初始化或丢包测试状态机临时接管。

## 起始 Anchor 流程

```text
按键短按
  -> 分配 init_seq
  -> LED_LOCAL  常亮
  -> LED_GLOBAL 常亮
  -> 本机对其他 3 个 Anchor 测距约 3s
  -> 生成本地距离表
  -> 单播 INIT_TOKEN 给下一个 Anchor
  -> 等待普通 ACK，未收到则持续重发
  -> 收到 ACK 后 LED_LOCAL 熄灭
  -> 等待 INIT_TOKEN 回到自己
  -> 收到回环 token 后 LED_GLOBAL 熄灭
  -> 进入 READY，等待 Tag 进场
```

起始 Anchor 收到回环 token 时不再次执行本机初始化，只做 `init_seq/origin/hop` 校验并结束整体状态。

## 非起始 Anchor 流程

```text
收到 INIT_TOKEN
  -> 立即回普通 ACK
  -> 如果是重复 token，只 ACK，不重复初始化
  -> LED_LOCAL 常亮
  -> 本机对其他 3 个 Anchor 测距约 3s
  -> 生成本地距离表
  -> 单播 INIT_TOKEN 给下一个 Anchor
  -> 等待普通 ACK，未收到则持续重发
  -> 收到 ACK 后 LED_LOCAL 熄灭
  -> 进入 READY，等待 Tag 进场
```

非起始 Anchor 不需要知道整体初始化是否完成，也不需要等待 `INIT_DONE`。

## 控制命令

`INIT_TOKEN` 使用单播控制命令。建议作为独立初始化控制帧，或作为 DATA_CTRL 的新增子类型，但不要复用 Tag 拉取状态机语义。

```c
typedef struct __attribute__((packed)) {
    uint8_t  cmd_type;       /* INIT_TOKEN */
    uint8_t  version;
    uint16_t init_seq;
    uint16_t origin_anchor;
    uint16_t sender_anchor;
    uint16_t target_anchor;
    uint8_t  hop_count;
    uint8_t  visited_mask;   /* 4 个 Anchor 的访问标记 */
    uint16_t crc16;
} init_token_t;
```

## ACK 与重发

ACK 使用普通 ACK，含义是“目标 Anchor 已收到并接受该 token”。ACK 不表示目标 Anchor 已完成 3s 测距。

发送侧状态机：

```text
SEND_TOKEN
  -> WAIT_ACK
  -> 收到 ACK：转入本机结束或等待回环
  -> ACK 超时：按固定间隔重发 INIT_TOKEN
```

现场要求 4 个 Anchor 全部在线，因此允许无限重发，避免误结束。建议超过 5s 后让 `LED_LOCAL` 快闪表示卡在转发阶段，但仍继续重发。

## 重复 token 处理

ACK 丢失时发送侧会重发。接收侧必须保证幂等：

- 相同 `init_seq + origin_anchor + sender_anchor + target_anchor` 的 token 再次到达时，必须再次 ACK。
- 如果本机已经开始或完成该轮初始化，不得重复启动测距。
- 如果 token 回到 origin，origin 不 ACK 后继续转发，而是结束整体初始化。

## 下一个 Anchor 选择

固定 4 个 Anchor，按短地址升序形成环：

```text
0x0030 -> 0x0031 -> 0x0032 -> 0x0033 -> 0x0030
```

如果 origin 不是最小 ID，也从 origin 的下一个 ID 开始继续环形传递。
