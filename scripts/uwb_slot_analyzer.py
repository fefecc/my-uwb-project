#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UWB 时隙优化分析工具

用 MCU tick (ms) 统计 Tag 往返延迟和 Anchor 处理时间, 输出最优时隙参数建议。

用法:
  python uwb_slot_analyzer.py              # 默认 30s
  python uwb_slot_analyzer.py -d 60        # 采集 60s
  python uwb_slot_analyzer.py -d 0         # 无限, Ctrl+C 停止
  python uwb_slot_analyzer.py --raw        # 显示原始串口行
"""

import argparse
import math
import re
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

try:
    import serial
except ImportError:
    print("错误: pip install pyserial")
    sys.exit(1)

try:
    import yaml
except ImportError:
    yaml = None

try:
    from colorama import Fore, Style, init as colorama_init
    colorama_init()
except ImportError:
    class _D:
        def __getattr__(self, _): return ""
    Fore = Style = _D()

SCRIPT_DIR = Path(__file__).resolve().parent
CFG_PATH = SCRIPT_DIR / "config" / "serial_test.yaml"

# ================================================================
#  正则 (提取行首 MCU tick)
# ================================================================

# Tag: 20803 INFO  [PHY] TX_STARTED imm=1
RE_TAG_TX = re.compile(r"(\d+)\s+\S+\s+\[PHY\]\s+TX_STARTED")

# Tag: 20853 INFO  [LINK] RX type=33 src=0x0030
RE_TAG_RX = re.compile(r"(\d+)\s+\S+\s+\[LINK\]\s+RX\s+type=(\d+)\s+src=0x([0-9A-Fa-f]+)")

# Tag: 20853 INFO  [LINK] RX_TIMEOUT
RE_TAG_TIMEOUT = re.compile(r"(\d+)\s+\S+\s+\[LINK\]\s+RX_TIMEOUT")

# Tag: [LINK] preparing DISC_REQ tick=20803
RE_TAG_PREPARE = re.compile(r"(\d+)\s+\S+\s+\[LINK\]\s+preparing\s+DISC_REQ")

# Tag: [LINK] DISC_REQ win=104 seq=103
RE_TAG_SENT = re.compile(r"(\d+)\s+\S+\s+\[LINK\]\s+DISC_REQ\s+win=(\d+)\s+seq=(\d+)")

# Anchor: [PHY] TX_STARTED fast_reply
RE_ANC_REPLY_TX = re.compile(r"(\d+)\s+\S+\s+\[PHY\]\s+TX_STARTED\s+fast_reply")

# Anchor: [LINK] RX type=32 src=0x0101
RE_ANC_RX = re.compile(r"(\d+)\s+\S+\s+\[LINK\]\s+RX\s+type=(\d+)\s+src=0x([0-9A-Fa-f]+)")

# Anchor: [LINK] fast_reply=1 tx=0x...
RE_ANC_REPLY_DONE = re.compile(r"(\d+)\s+\S+\s+\[LINK\]\s+fast_reply=")

# ================================================================
#  串口读取
# ================================================================

class SerialReader(threading.Thread):
    def __init__(self, port, baudrate, name, callback, log_file=None):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.dev_name = name
        self.callback = callback
        self.log_file = log_file
        self._stop = threading.Event()
        self.connected = False

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.connected = True
            print(f"{Fore.GREEN}[{self.dev_name}] 已连接 {self.port}{Style.RESET_ALL}")
        except serial.SerialException as e:
            print(f"{Fore.RED}[{self.dev_name}] 无法打开 {self.port}: {e}{Style.RESET_ALL}")
            return

        buf = ""
        while not self._stop.is_set():
            try:
                raw = ser.read(ser.in_waiting or 1)
                if not raw:
                    continue
                buf += raw.decode("utf-8", errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line:
                        if self.log_file:
                            self.log_file.write(f"{line}\n")
                            self.log_file.flush()
                        self.callback(self.dev_name, line)
            except serial.SerialException:
                if not self._stop.is_set():
                    print(f"{Fore.RED}[{self.dev_name}] 串口断开{Style.RESET_ALL}")
                break

    def stop(self):
        self._stop.set()

# ================================================================
#  分析引擎
# ================================================================

class SlotAnalyzer:
    def __init__(self, show_raw=False):
        self.lock = threading.Lock()
        self.show_raw = show_raw
        self.start_time = time.time()

        # Tag 侧
        self.tag_req_count = 0
        self.tag_ack_count = 0
        self.tag_timeout_count = 0
        self._tag_tx_tick = 0          # PHY TX_STARTED 的 MCU tick
        self._tag_prepare_tick = 0
        self.tag_rtt_ms: list[int] = []       # prepare → RX (ms)
        self.tag_period_ms: list[int] = []    # prepare → prepare (ms)
        self._tag_last_prepare = 0

        # Anchor 侧
        self.anchor_rx_count = 0
        self.anchor_reply_count = 0
        self._anc_reply_tx_tick = 0
        self.anc_reply_to_rx_ms: list[int] = []  # PHY TX_STARTED → LINK RX (ms)

    def on_line(self, dev: str, line: str):
        if self.show_raw:
            c = Fore.CYAN if dev == "TAG" else Fore.YELLOW
            print(f"{c}[{dev}] {line}{Style.RESET_ALL}")
        with self.lock:
            if dev == "TAG":
                self._parse_tag(line)
            else:
                self._parse_anchor(line)

    def _parse_tag(self, line: str):
        m = RE_TAG_PREPARE.search(line)
        if m:
            tick = int(m.group(1))
            if self._tag_last_prepare > 0:
                self.tag_period_ms.append(tick - self._tag_last_prepare)
            self._tag_last_prepare = tick
            self._tag_prepare_tick = tick
            return

        m = RE_TAG_SENT.search(line)
        if m:
            self.tag_req_count += 1
            return

        m = RE_TAG_TX.search(line)
        if m:
            self._tag_tx_tick = int(m.group(1))
            return

        m = RE_TAG_RX.search(line)
        if m:
            tick = int(m.group(1))
            ftype = int(m.group(2))
            if ftype == 33 and self._tag_prepare_tick > 0:  # DISC_RSP
                rtt = tick - self._tag_prepare_tick
                self.tag_rtt_ms.append(rtt)
                self.tag_ack_count += 1
                self._tag_prepare_tick = 0
            return

        m = RE_TAG_TIMEOUT.search(line)
        if m:
            self.tag_timeout_count += 1
            self._tag_prepare_tick = 0
            return

    def _parse_anchor(self, line: str):
        m = RE_ANC_REPLY_TX.search(line)
        if m:
            self._anc_reply_tx_tick = int(m.group(1))
            self.anchor_reply_count += 1
            return

        m = RE_ANC_RX.search(line)
        if m:
            tick = int(m.group(1))
            ftype = int(m.group(2))
            if ftype == 32:  # DISC_REQ
                self.anchor_rx_count += 1
                # Anchor: 从 fast_reply TX 到 LINK 看到的 RX (顺序: PHY先, LINK后)
                if self._anc_reply_tx_tick > 0:
                    delta = tick - self._anc_reply_tx_tick
                    if 0 < delta < 500:
                        self.anc_reply_to_rx_ms.append(delta)
                    self._anc_reply_tx_tick = 0
            return

    # ---- 报告 ----

    def print_report(self):
        print(f"\n{self.get_report_text()}\n")

    def get_report_text(self) -> str:
        with self.lock:
            return self._build_report()

    def _build_report(self) -> str:
        elapsed = time.time() - self.start_time
        L = []
        L.append(f"{'='*60}")
        L.append(f"  UWB 时隙分析  |  {elapsed:.0f}s")
        L.append(f"{'='*60}")

        # 丢包
        L.append(f"\n【丢包统计】")
        L.append(f"  Tag  REQ: {self.tag_req_count}  ACK: {self.tag_ack_count}  TIMEOUT: {self.tag_timeout_count}")
        if self.tag_req_count > 0:
            loss = (1 - self.tag_ack_count / self.tag_req_count) * 100
            L.append(f"  丢包率: {loss:.1f}%")
        L.append(f"  Anchor  RX: {self.anchor_rx_count}  Reply: {self.anchor_reply_count}")

        # Tag RTT (prepare → RX)
        L.append(f"\n【Tag 往返延迟 (prepare→RX, MCU tick ms)】")
        self._append_stats(L, self.tag_rtt_ms, "ms")

        # Discovery 周期
        L.append(f"\n【Discovery 周期 (prepare→prepare)】")
        self._append_stats(L, self.tag_period_ms, "ms")

        # Anchor 处理
        L.append(f"\n【Anchor 处理 (PHY fast_reply→LINK RX)】")
        self._append_stats(L, self.anc_reply_to_rx_ms, "ms")

        # 建议
        if self.tag_rtt_ms:
            s = sorted(self.tag_rtt_ms)
            n = len(s)
            avg = sum(s) / n
            std = (sum((x - avg) ** 2 for x in s) / max(n - 1, 1)) ** 0.5
            p95 = s[min(int(n * 0.95), n - 1)]
            p99 = s[min(int(n * 0.99), n - 1)]

            slot_start = (p95 + 2 * std) * 1000  # ms → µs
            slot_start_r = math.ceil(slot_start / 500) * 500
            slot_width = max(6 * std * 1000, 500)  # ms → µs
            slot_width_r = math.ceil(slot_width / 500) * 500
            rx_timeout = min(math.ceil((p99 * 1000 + 4 * std * 1000) / 1000) * 1000, 65000)

            L.append(f"\n{'─'*60}")
            L.append(f"  ★ 最优时隙建议 (基于 {n} 个样本)")
            L.append(f"{'─'*60}")
            L.append(f"  首槽延迟 (P95+2σ):  {slot_start_r:>6} µs  ({slot_start_r/1000:.1f} ms)")
            L.append(f"  槽宽度   (6σ):      {slot_width_r:>6} µs  ({slot_width_r/1000:.1f} ms)")
            L.append(f"  RX超时   (P99+4σ):  {rx_timeout:>6} µs  ({rx_timeout/1000:.1f} ms)")
            L.append(f"\n  固件宏:")
            L.append(f"    #define PHY_DISC_REPLY_DELAY_US    {slot_start_r}U")
            L.append(f"    #define DISC_SLOT_WIDTH_US         {slot_width_r}U")
            L.append(f"    #define UWB_PHY_RX_SLOT_TIMEOUT_US {rx_timeout}U")
            L.append(f"\n  说明: MCU tick 精度 1ms, LINK 轮询 50ms")
            L.append(f"        实际值含轮询开销, 真实 RTT 更小")

        L.append(f"{'='*60}")
        return "\n".join(L)

    @staticmethod
    def _append_stats(L: list, data: list, unit: str):
        if not data:
            L.append(f"  无数据")
            return
        n = len(data)
        avg = sum(data) / n
        std = (sum((x - avg) ** 2 for x in data) / max(n - 1, 1)) ** 0.5
        s = sorted(data)
        mn, mx = s[0], s[-1]
        p50 = s[int(n * 0.5)]
        p95 = s[min(int(n * 0.95), n - 1)]
        p99 = s[min(int(n * 0.99), n - 1)]
        L.append(f"  N={n}  均值={avg:.1f}{unit}  σ={std:.1f}")
        L.append(f"  最小={mn}  最大={mx}")
        L.append(f"  P50={p50}  P95={p95}  P99={p99}")


# ================================================================
#  主流程
# ================================================================

def load_cfg():
    if yaml and CFG_PATH.exists():
        with open(CFG_PATH, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    return {}


def main():
    cfg = load_cfg()
    serial_cfg = cfg.get("serial", {})
    default_tag = serial_cfg.get("tag", "COM6")
    default_anchor = serial_cfg.get("anchor", "COM7")
    default_baud = serial_cfg.get("baudrate", 460800)

    p = argparse.ArgumentParser(description="UWB 时隙优化分析")
    p.add_argument("-d", "--duration", type=int, default=30,
                   help="采集时间 (秒), 0=无限 (默认 30)")
    p.add_argument("--tag", default=default_tag)
    p.add_argument("--anchor", default=default_anchor)
    p.add_argument("--baud", type=int, default=default_baud)
    p.add_argument("--raw", action="store_true", help="显示原始串口行")
    args = p.parse_args()

    analyzer = SlotAnalyzer(show_raw=args.raw)

    now = datetime.now()
    ts_str = now.strftime("%Y%m%d_%H%M%S")
    run_dir = SCRIPT_DIR / "logs" / f"slot_analysis_{ts_str}"
    run_dir.mkdir(parents=True, exist_ok=True)

    tag_log = open(run_dir / f"tag_{ts_str}.log", "w", encoding="utf-8")
    anc_log = open(run_dir / f"anchor_{ts_str}.log", "w", encoding="utf-8")

    tag_r = SerialReader(args.tag, args.baud, "TAG", analyzer.on_line, tag_log)
    anc_r = SerialReader(args.anchor, args.baud, "ANCHOR", analyzer.on_line, anc_log)

    dur_text = f"{args.duration}s" if args.duration > 0 else "无限 (Ctrl+C)"
    print(f"\n{Fore.GREEN}{'='*60}")
    print(f"  UWB 时隙分析工具")
    print(f"  Tag: {args.tag}  Anchor: {args.anchor}  @ {args.baud}")
    print(f"  持续: {dur_text}  日志: {run_dir}")
    print(f"{'='*60}{Style.RESET_ALL}\n")

    tag_r.start()
    anc_r.start()
    time.sleep(1)

    if not tag_r.connected and not anc_r.connected:
        print(f"{Fore.RED}两个串口都无法连接{Style.RESET_ALL}")
        return

    t0 = time.time()
    last_report = t0

    try:
        while True:
            time.sleep(0.5)
            now_t = time.time()
            if now_t - last_report >= 10:
                analyzer.print_report()
                last_report = now_t
            if args.duration > 0 and now_t - t0 >= args.duration:
                print(f"\n{Fore.YELLOW}采集完成{Style.RESET_ALL}")
                break
    except KeyboardInterrupt:
        print(f"\n{Fore.YELLOW}用户中断{Style.RESET_ALL}")

    tag_r.stop()
    anc_r.stop()

    print(f"\n{Fore.GREEN}>>> 最终报告 <<<{Style.RESET_ALL}")
    analyzer.print_report()

    report_text = analyzer.get_report_text()
    report_path = run_dir / f"report_{ts_str}.txt"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(report_text)

    tag_log.close()
    anc_log.close()
    print(f"日志: {run_dir}")
    print(f"报告: {report_path}")


if __name__ == "__main__":
    main()
