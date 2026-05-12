# UWB 多源传感器数据采集系统 — 汇报

---

## Slide 1: 标题页

**UWB 多源传感器数据采集系统**

基于 STM32H743VITX 的嵌入式多源传感器融合平台

汇报人：___

日期：2026年5月

---

## Slide 2: 项目目标

| 目标 | 说明 |
|------|------|
| **UWB 测距** | Tag 主动发起，Anchor 被动响应，支持多 Anchor 同时测距 |
| **GNSS 定位** | 接收解算结果，提供 UTC 时间基准 |
| **IMU 采样** | 三轴加速度 + 三轴陀螺仪，~208Hz |
| **多源融合** | 统一时间戳，按时间排序写入 SD 卡 |
| **配置管理** | 串口配置 PAN ID / 短地址 / 角色，Flash 持久化 |

---

## Slide 3: 硬件平台

```
┌─────────────────────────────────────────────┐
│              STM32H743VITX                  │
│              (Cortex-M7, 480MHz)            │
├─────────────────────────────────────────────┤
│  DW1000 (UWB)     ◄── SPI2, IRQ=PD8        │
│  UM960 (GNSS)     ◄── USART3+DMA, PPS=PC0  │
│  ASM330 (IMU)     ◄── SPI1, IRQ=PA4        │
│  SD Card          ◄── SDMMC1 4-bit         │
│  Debug UART       ◄── USART1 460800        │
├─────────────────────────────────────────────┤
│  FreeRTOS, 128KB Heap, 9 Tasks             │
└─────────────────────────────────────────────┘
```

---

## Slide 4: 系统架构

```
┌──────────┐  ┌──────────┐  ┌──────────┐
│  GNSS    │  │   IMU    │  │   UWB    │
│ Task     │  │  Task    │  │  Stack   │
└────┬─────┘  └────┬─────┘  └────┬─────┘
     │             │             │
     └─────────────┼─────────────┘
                   ▼
         ┌─────────────────┐
         │  DataService    │  ← 统一数据队列 (64 deep)
         │  (TimeService)  │  ← 统一时间戳
         └────────┬────────┘
                  ▼
         ┌─────────────────┐
         │   DataSort      │  ← 排序 + ASCII 转换
         └────────┬────────┘
                  ▼
         ┌─────────────────┐
         │   SD Writer     │  ← 16KB 双缓冲
         └─────────────────┘
```

---

## Slide 5: 已完成功能

### 系统框架
- ✅ FreeRTOS 9 任务调度
- ✅ 分层架构 (bsp/device/service/task/UWB)
- ✅ 配置模式 + 运行模式 (按键切换)
- ✅ Flash 配置持久化
- ✅ 双缓冲 SD 写入

### GNSS / IMU / 时间同步
- ✅ UM960 协议解析，UTC 时间提取
- ✅ PPS 整秒对齐，本地连续时钟 (TIM2)
- ✅ ASM330 六轴采样，EXTI 触发

### UWB 协议栈
- ✅ 三层架构 (PHY/LINK/APP)
- ✅ Discovery 多槽接收 (4 槽 × 2ms)
- ✅ Anchor 延迟应答 (统一 DELAYED TX)
- ✅ DS-TWR 测距计算

---

## Slide 6: UWB 协议栈亮点

### 三层架构
| 层 | 职责 | 优先级 |
|----|------|--------|
| **PHY** | DW1000 硬件控制、中断处理、快速应答 | AboveNormal |
| **LINK** | 帧调度、状态机、多槽管理 | AboveNormal |
| **APP** | 测距计算、数据发布 | Normal |

### 多槽接收 (plan-v2)
```
Tag TX ──→ Slot 0 (2ms) ──→ Slot 1 (2ms) ──→ Slot 2 (2ms) ──→ Slot 3 (2ms)
              ↑                 ↑                 ↑                 ↑
           Anchor 0          Anchor 1          Anchor 2          Anchor 3
         (delay 1ms)       (delay 3ms)       (delay 5ms)       (delay 7ms)
```

### DS-TWR 算法
```
ToF = (ra × rb - da × db) / (ra + rb + da + db)

ra = Anchor 第1次回复延迟    da = Tag 第2次请求延迟
rb = Anchor 第2次回复延迟    db = Tag 第1次往返
```

---

## Slide 7: 时间同步机制

```
┌─────────────────────────────────────────────────────┐
│                    TimeService                       │
├─────────────────────────────────────────────────────┤
│  GNSS UTC ──→ UTC 缓存 ──→ PPS 锚点 ──→ 本地 UTC    │
│                                                      │
│  TIM2 ────→ 本地连续时钟 (单调递增，不受校时影响)    │
├─────────────────────────────────────────────────────┤
│  输出: week + week_ms + local_sec + local_ms        │
│        + utc_valid + sync_state                     │
└─────────────────────────────────────────────────────┘
```

**同步状态机**: `LOCKED → HOLDOVER → LOST`

---

## Slide 8: 测试数据 (参考)

| 指标 | 数值 | 说明 |
|------|------|------|
| Discovery 成功率 | > 99% | 室内 20m 测试 |
| 测距频率 | ~5 Hz | 当前 200ms 周期 |
| 单次交换耗时 | ~11ms | DISC_REQ → DISC_RESP |
| DS-TWR 频率 (计划) | ~90 Hz | plan-v3 连续测距 |
| IMU 采样率 | 208 Hz | EXTI 触发 |
| SD 写入块 | 16 KB | 双缓冲 |

---

## Slide 9: 后续计划

### Phase 1: DISC_RESP 携带时间戳
- Anchor 在应答帧中嵌入 `rx_ts` + `tx_ts`
- 供 Tag 侧精确 TWR 计算

### Phase 2: 连续测距
- 去掉 200ms 固定周期
- 事件驱动，目标 ~90 Hz

### Phase 3: 滑窗 DS-TWR
- 复用相邻两次交换时间戳
- 丢包自动退化 SS-TWR

### Phase 4: DATA 帧传输
- Tag 从 Anchor 拉取数据
- 三缓冲 + 重发机制

---

## Slide 10: 总结

### 已完成
- 多源传感器数据采集统一框架
- UWB Discovery + DS-TWR 测距
- 多槽接收 + 延迟应答
- 统一时间戳 + SD 存储

### 技术亮点
- 三层协议栈解耦设计
- 零拷贝共享内存池
- 事件驱动状态机
- 时间同步锚点机制

### 下一步
- 实现 plan-v3 连续测距
- 提升 UWB 测距频率至 ~90 Hz

---

## Slide 11: Q&A

**感谢聆听！**

---

# 附录：关键文件

| 模块 | 文件 |
|------|------|
| 系统入口 | `APP/app/app.c` |
| UWB 协议栈 | `APP/UWB/uwb_stack.c`, `uwb_phy.c`, `uwb_link.c`, `uwb_app.c` |
| 时间服务 | `APP/service/time_service.c` |
| 数据服务 | `APP/service/data_service.c` |
| 配置服务 | `APP/service/config_service.c` |
| 任务定义 | `APP/task/app_tasks.c` |
