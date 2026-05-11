# UWB PHY 中断丢失问题调试总结

> 日期: 2026-05-10
> 设备: STM32H743VI (Tag) + DW1000
> 状态: 部分修复，待优化

---

## 一、问题现象

Tag 上电后发送 DISC_REQ，**EXTI 中断从未触发**，所有 DW1000 事件仅靠 50ms 软件轮询（POLL_CATCH）捕获。由于 RX_SLOT 超时仅 10ms，50ms 轮询周期内 RX 窗口已关闭，导致 **100% 丢包**。

### 典型日志

```
9652  [PHY] TX_STARTED imm=1
9702  [PHY] POLL_CATCH st=0xFFFFFFFF    ← 50ms 后轮询捕获，含 reserved bit
9703  [LINK] TX_DONE (slot recycled by PHY)
```

### 关键特征

- `0xFFFFFFFF` 包含 bit 19 (`SYS_STATUS_reserved`)，DW1000 永远不会置位此位
- 上电后 **必定出现**
- 有时等待一段时间后自动恢复，有时无法恢复
- Anchor 侧工作正常，问题仅在 Tag 侧

---

## 二、排查过程

### 2.1 尝试一：CPLOCK/MCPLOCK 理论 ❌ 失败

**假设**: `dwt_initialise()` 在 SYS_MASK 中遗留了 MCPLOCK 位，`dwt_setinterrupt()` 的 OR 操作保留了它，导致 CPLOCK (PLL锁定) 永久匹配 → IRQ 引脚钉死 HIGH → EXTI 上升沿无法触发。

**修改**:
- `bphero_uwb.c`: 用 `dwt_write32bitreg` 直接写入 SYS_MASK，替代 `dwt_setinterrupt` 的 OR 操作
- `uwb_phy.c`: `PHY_STATUS_CLEAR_MASK` 补充 CPLOCK/SLP2INIT 等位

**结果**: 烧录后仍然出现 `st=0xFFFFFFFF`，问题未解决。已 git 回退。

**结论**: CPLOCK 不是根因。`0xFFFFFFFF` 说明 SPI 通信本身就不通。

### 2.2 尝试二：SPI 校验 + 跳过垃圾数据 ✅ 有效

**假设**: `0xFFFFFFFF` 是 SPI 读取失败（MISO 全高），不是真实的 DW1000 状态。旧代码将垃圾 `0xFFFFFFFF` 当作合法 status 处理，向 SPI 不通的 DW1000 写入各种命令，导致状态彻底混乱。

**修改**:
- 在 POLL_CATCH 中先读 `dwt_readdevid()` 验证 SPI 链路
- 如果 DEV_ID ≠ `0xDECA0130`，跳过本轮处理

**结果**: 上电后不再出现 `0xFFFFFFFF`，中断正常工作，TX_DONE 在 4ms 内出现，RX 正常收到 Anchor 回复。

### 2.3 根因确认

**真正的修复不是 DEV_ID 校验，而是 `continue` 跳过了对垃圾 status 的处理。**

旧代码处理 `0xFFFFFFFF` 的破坏链：

```
poll_st = 0xFFFFFFFF
  → TXFRS=1 且 RXFCG=1 → 进入组合处理路径
    → irq_tx_done() → run_state_machine() → TX FINISH
      → dwt_read TX timestamp (SPI 坏 → 垃圾)
      → UwbSlots_Free → 进入 RX_SLOT
        → dwt_setrxtimeout (SPI 坏 → 写入失败)
        → dwt_rxenable (SPI 坏 → 写入失败)
    → irq_rx_ok(0xFFFFFFFF) → 读帧数据 (SPI 坏 → 垃圾)
  → DW1000 内部状态彻底混乱，后续即使 SPI 恢复也无法正常工作
```

---

## 三、当前代码状态

### 已修改的文件

| 文件 | 修改内容 | 状态 |
|------|----------|------|
| `uwb_phy.c` | POLL_CATCH 中增加 DEV_ID 校验，SPI 异常时跳过处理 | ✅ 已生效 |

### 当前代码的已知问题

1. **每轮多读一次 DEV_ID**: `dwt_readdevid()` 是额外的 SPI 事务，增加了轮询开销
2. **`continue` 跳过了看门狗**: SPI 持续异常时，看门狗代码永远执行不到，无法触发 `enter_listening()` 恢复

---

## 四、待办事项

### 4.1 优化 SPI 校验方式（优先级高）

用 reserved bit 检测替代 DEV_ID 读取，零额外 SPI 开销：

```c
uint32_t poll_st = dwt_read32bitreg(SYS_STATUS_ID);
if (poll_st & SYS_STATUS_reserved) {  // bit 19, DW1000 永远不会置位
    // SPI 垃圾数据，不要 continue，让它落到看门狗处理
    // 这样既过滤了垃圾，又保留了看门狗恢复能力
}
```

### 4.2 修复 POLL_CATCH 中垃圾数据的恢复路径（优先级高）

当前 `continue` 会跳过看门狗，导致 SPI 持续异常时无法恢复。应改为：
- 检测到垃圾数据时 **不处理**，但 **不跳过** 看门狗
- 让看门狗正常超时后调用 `enter_listening()` 恢复

### 4.3 调查 SPI 启动阶段为什么返回 0xFFFFFFFF（优先级中）

可能原因：
- DW1000 上电后 PLL/时钟未完全稳定，SPI 暂时不可用
- `reset_DW1000()` 后延时不够（当前仅 `osDelay(5)` = 5ms）
- SPI2 高速模式 (16MHz) 在启动阶段可能过快

### 4.4 评估是否需要 CPLOCK 修复（优先级低）

虽然 CPLOCK/MCPLOCK 不是导致 `0xFFFFFFFF` 的原因，但 SPI 恢复后仍可能存在 CPLOCK 导致 IRQ 钉高的问题。待 SPI 问题彻底解决后，需要验证中断是否在所有场景下都能正常触发。

### 4.5 Tag-Anchor 通信调试（优先级中）

当前 Anchor 侧能正常收发，但 Tag 侧 RX 仍可能存在超时问题，待中断问题彻底解决后进一步调试。

---

## 五、硬件/配置参考

| 参数 | 值 |
|------|-----|
| MCU | STM32H743VI |
| SYSCLK | 480 MHz (HSI 64MHz × PLL) |
| SPI2 时钟源 | CLKP = HSI = 64 MHz |
| SPI2 高速分频 | /4 = 16 MHz |
| SPI2 低速分频 | /64 = 1 MHz |
| DW1000 Channel | 2 |
| DW1000 Data Rate | 110 kbps |
| DW1000 Preamble | 1024 symbols |
| DW1000 PRF | 64 MHz |
| EXTI IRQ Pin | PD8 (EXTI9_5, 上升沿) |
| EXTI 优先级 | 5 (= FreeRTOS MAX_SYSCALL) |
| SPI2 CS Pin | PB12 (软件控制) |
