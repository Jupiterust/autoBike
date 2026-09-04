#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_tuning_bridge.py  ——  飞轮平衡自行车 串口遥测桥接层 / PID 调参助手

作用:
  1) 实时读取 STM32 通过串口发来的二进制遥测帧(见下方线协议),做帧同步 + CRC 校验 + 解码
  2) 持续把每一帧落成带列名的 CSV 日志(供 Claude Code 事后读取分析)
  3) 支持"抓取一段时间窗"并自动算出调参关键指标(真实环频/丢帧/饱和/各环误差/PID分项主导/D项噪声等),
     输出人类可读摘要 + 机器可读 JSON(供 Claude Code 直接读)

这是"上位机"侧。固件侧必须按同一份 SCHEMA 发送二进制帧(交给工程目录里的 Claude Code 实现)。
上位机与固件唯一需要对齐的就是 PROTO_VERSION + SCHEMA + CRC 算法,三者一致即可。

依赖:  pip install pyserial
用法示例:
  # 列出可用串口
  python pid_tuning_bridge.py --list

  # 持续记录到 CSV(Ctrl-C 停止)。Mac 上口名通常形如 /dev/tty.usbmodemXXXX 或 /dev/tty.usbserial-XXXX
  python pid_tuning_bridge.py --port /dev/tty.usbmodem14203 --baud 921600 --log

  # 抓取 6 秒窗口 -> 存 CSV + 打印摘要 + 存 summary JSON(最适合让 Claude Code 分析一次摔倒/一次受扰)
  python pid_tuning_bridge.py --port /dev/tty.usbmodem14203 --baud 921600 --capture 6

  # 对已存在的日志 CSV 重新算指标(不接硬件也能用)
  python pid_tuning_bridge.py --analyze logs/telem_20260904_153000.csv
"""

import argparse
import json
import os
import struct
import sys
import time
from collections import deque
from datetime import datetime

# ----------------------------------------------------------------------------
# 线协议 (STM32 -> PC)  ——  固件与上位机的共同契约
# ----------------------------------------------------------------------------
# 一帧结构 (小端 little-endian):
#   [0xAA][0x55]          帧头 magic, 2 字节
#   [LEN]                 payload 长度, 1 字节 uint8 (= 下面 payload 的字节数)
#   [payload ...]         按 SCHEMA 顺序打包的字段, LEN 字节
#   [CRC16_L][CRC16_H]    CRC16-CCITT(0x1021, init=0xFFFF), 覆盖 LEN + payload, 2 字节 小端
#
# payload 第 0 个字段固定是 uint8 版本号, 上位机据此判断固件 schema 是否匹配。
# 波特率建议 >= 921600(USB CDC 虚拟串口可无视波特率)。发送频率建议按 TELEM_DECIM 降采样,
# 例如控制环 400Hz、每 2 次发一帧 = 200Hz,足够调参且省带宽。
#
# 若要增删字段: 改 SCHEMA + 递增 PROTO_VERSION,固件侧同步修改即可。
PROTO_VERSION = 1
MAGIC = b"\xAA\x55"

# (字段名, struct 格式符)  —— struct 格式符: I=uint32, i=int32, f=float32, H=uint16, h=int16, B=uint8
# 顺序 == 固件打包顺序 == CSV 列顺序。第一个必须是 ver:B。
SCHEMA = [
    ("ver",                 "B"),   # 协议版本, 必须 == PROTO_VERSION
    # ---- P0 最小可用集 ----
    ("loop_cnt",            "I"),   # balance() 自增序号, 用于精确丢帧检测
    ("t_ms",                "I"),   # 车上 HAL_GetTick() 绝对时间(ms), 用于算真实环频
    ("imu_rol",             "f"),   # deg      角度环实际值
    ("angle_target",        "f"),   # deg      角度环设定值(合成后)
    ("imu_vx",              "f"),   # deg/s    角速度环实际值
    ("pwm_x",               "f"),   # deg/s    角速度环设定值
    ("set_speed0_raw",      "f"),   # turn/s   飞轮指令(限幅前)
    ("set_speed0",          "f"),   # turn/s   飞轮指令(限幅后)
    ("now_speed0",          "f"),   # enc/2.5ms 速度环实际值
    # ---- P1 强烈建议 ----
    ("pwm_accel",           "f"),   # deg      速度环输出
    ("fly_gain",            "f"),   # deg      饱和解脱项(看是否长期贴 ±0.3)
    ("servo_zhongzhi_gain", "f"),   # deg      转向补偿
    ("servo_ctl",           "f"),   # -        打舵量(限幅后)
    ("servo_lim",           "f"),   # -        当前允许打舵斜率
    ("inner_p",             "f"),   # -        内环 P 分项 = kp*Bias
    ("inner_i",             "f"),   # -        内环 I 分项 = ki*∫Bias
    ("inner_d",             "f"),   # -        内环 D 分项 = kd*ΔBias
    ("crc_err_cnt",         "H"),   # -        IMU 链路累计 CRC 错误数(健康度)
    ("flags",               "H"),   # bitfield M0/M1/restart/upper/饱和/CRC错 等
]

# flags 位定义(与固件约定;仅示例,按你的实际含义调整)
FLAG_BITS = {
    0: "M0_Flag",
    1: "M1_Flag",
    2: "restart_flag",
    3: "upper_Flag",
    4: "sat_flag",       # 飞轮指令被限幅
    5: "imu_crc_err",    # 本帧 IMU 数据 CRC 异常
}

FIELD_NAMES = [name for name, _ in SCHEMA]
PAYLOAD_FMT = "<" + "".join(fmt for _, fmt in SCHEMA)
PAYLOAD_LEN = struct.calcsize(PAYLOAD_FMT)


# ----------------------------------------------------------------------------
# CRC16-CCITT (0x1021, init 0xFFFF)  —— 固件侧用同一算法
# ----------------------------------------------------------------------------
def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# 参考 C 实现(贴进固件即可,与上面完全等价):
#   uint16_t crc16_ccitt(const uint8_t *d, uint32_t n){
#     uint16_t crc=0xFFFF;
#     for(uint32_t i=0;i<n;i++){ crc^=(uint16_t)d[i]<<8;
#       for(int k=0;k<8;k++) crc = (crc&0x8000)?(crc<<1)^0x1021:(crc<<1); }
#     return crc; }


# ----------------------------------------------------------------------------
# 帧解析器: 喂入任意字节流, 吐出解码后的帧 dict
# ----------------------------------------------------------------------------
class FrameParser:
    def __init__(self):
        self.buf = bytearray()
        self.frames_ok = 0
        self.crc_bad = 0
        self.ver_bad = 0

    def feed(self, chunk: bytes):
        out = []
        self.buf.extend(chunk)
        while True:
            # 找帧头
            i = self.buf.find(MAGIC)
            if i < 0:
                # 保留最后 1 字节(可能是半个 magic)
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break
            if i > 0:
                del self.buf[:i]  # 丢弃 magic 之前的垃圾
            if len(self.buf) < 3:
                break  # 还没收到 LEN
            length = self.buf[2]
            frame_len = 3 + length + 2  # magic+len + payload + crc
            if len(self.buf) < frame_len:
                break  # 帧还没收全
            payload = bytes(self.buf[3:3 + length])
            crc_rx = self.buf[3 + length] | (self.buf[4 + length] << 8)
            crc_calc = crc16_ccitt(bytes(self.buf[2:3 + length]))  # 覆盖 LEN+payload
            del self.buf[:frame_len]
            if crc_rx != crc_calc:
                self.crc_bad += 1
                continue
            if length != PAYLOAD_LEN:
                self.ver_bad += 1
                continue
            vals = struct.unpack(PAYLOAD_FMT, payload)
            rec = dict(zip(FIELD_NAMES, vals))
            if rec.get("ver") != PROTO_VERSION:
                self.ver_bad += 1
                continue
            self.frames_ok += 1
            out.append(rec)
        return out


# ----------------------------------------------------------------------------
# CSV 记录器
# ----------------------------------------------------------------------------
class CsvLogger:
    def __init__(self, path):
        self.path = path
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.f = open(path, "w", buffering=1)
        # 额外记 PC 接收时刻(仅用于对照;时序基准以车上 t_ms/loop_cnt 为准)
        self.f.write("pc_ts," + ",".join(FIELD_NAMES) + "\n")

    def write(self, rec):
        self.f.write(f"{time.time():.6f}," + ",".join(str(rec[n]) for n in FIELD_NAMES) + "\n")

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass


# ----------------------------------------------------------------------------
# 指标分析: 输入一段帧列表 -> 调参摘要
# ----------------------------------------------------------------------------
def _stats(xs):
    if not xs:
        return {"n": 0}
    n = len(xs)
    mean = sum(xs) / n
    var = sum((x - mean) ** 2 for x in xs) / n
    return {"n": n, "min": min(xs), "max": max(xs), "mean": mean, "std": var ** 0.5}


def _warnings(r, m0_active, col):
    """把常见"数据不可用作基线"的情况显式报出来。"""
    w = []
    n = len(r)
    if m0_active == 0:
        w.append("整段 M0_Flag=0:平衡未使能,控制环没闭合,这不是有效的调参基线")
    elif m0_active < n:
        w.append(f"仅 {m0_active}/{n} 帧平衡使能,饱和/误差指标只在这些帧上有意义")
    ns = col("now_speed0")
    if all(abs(x) < 1e-9 for x in ns):
        w.append("now_speed0 全程为 0:飞轮未转,或编码器反馈未接入 now_speed0;平衡使能后请确认它变非零")
    at = col("angle_target")
    if (max(at) - min(at)) < 1e-6 and m0_active > 0:
        w.append("angle_target 全程恒定:确认 tap 取的是合成后的零点而非基础零点")
    return w


def analyze(records):
    """records: list[dict],按接收顺序。返回可读+可序列化的指标字典。"""
    if len(records) < 2:
        return {"error": "样本太少,至少需要 2 帧"}

    r = records
    col = lambda k: [x[k] for x in r]

    # --- 环路频率 / 丢帧(以车上时间与序号为准)---
    t = col("t_ms")
    lc = col("loop_cnt")
    dt_ms = [t[i + 1] - t[i] for i in range(len(t) - 1) if t[i + 1] >= t[i]]
    dloop = [lc[i + 1] - lc[i] for i in range(len(lc) - 1)]
    span_ms = t[-1] - t[0]
    n_loops = lc[-1] - lc[0]
    # 期望每帧 loop_cnt 增量应恒定(=TELEM_DECIM)。众数之外的都算丢帧/抖动。
    inc_mode = max(set(dloop), key=dloop.count) if dloop else 0
    dropped = sum(d - inc_mode for d in dloop if d > inc_mode) if inc_mode else 0
    real_hz = (n_loops / span_ms * 1000.0) if span_ms > 0 else 0.0
    frame_hz = (len(r) / span_ms * 1000.0) if span_ms > 0 else 0.0

    # --- 各环跟踪误差 ---
    ang_err = [r[i]["angle_target"] - r[i]["imu_rol"] for i in range(len(r))]
    rate_err = [r[i]["pwm_x"] - r[i]["imu_vx"] for i in range(len(r))]

    # --- 平衡是否使能 / flags ---
    FLAG_M0_BIT = 1 << 0
    FLAG_SAT_BIT = 1 << 4
    flags_series = col("flags")
    m0_mask = [bool(f & FLAG_M0_BIT) for f in flags_series]
    m0_active = sum(1 for x in m0_mask if x)
    balance_active_frac = m0_active / len(r)

    # --- 飞轮饱和(平衡车头号失效模式);只在平衡使能(M0)期间才有意义 ---
    # 用固件已带 M0 保护的 SAT 标志为准;M0 未使能时不计入,避免把断电清零误判成饱和。
    raw = col("set_speed0_raw")
    post = col("set_speed0")
    if m0_active > 0:
        sat_hits = sum(1 for i in range(len(r)) if m0_mask[i] and (flags_series[i] & FLAG_SAT_BIT))
        sat_depth = [abs(raw[i] - post[i]) for i in range(len(r)) if m0_mask[i]]
        sat_duty = sat_hits / m0_active
    else:
        sat_hits, sat_depth, sat_duty = 0, [], 0.0

    # --- 内环 PID 分项:谁在主导 + D 项噪声 ---
    p_abs = [abs(x) for x in col("inner_p")]
    i_abs = [abs(x) for x in col("inner_i")]
    d_abs = [abs(x) for x in col("inner_d")]
    mean_p, mean_i, mean_d = (sum(p_abs) / len(r), sum(i_abs) / len(r), sum(d_abs) / len(r))
    dom_total = mean_p + mean_i + mean_d + 1e-12
    d_series = col("inner_d")
    d_diff = [abs(d_series[i + 1] - d_series[i]) for i in range(len(d_series) - 1)]
    d_noise = (sum(d_diff) / len(d_diff)) if d_diff else 0.0  # 逐帧抖动,越大越像在放大噪声

    # --- fly_gain 是否长期贴限(±0.3 为示例阈值,按实际改)---
    fg = col("fly_gain")
    fg_pinned = sum(1 for v in fg if abs(v) >= 0.3 - 1e-6) / len(fg)

    summary = {
        "samples": len(r),
        "timing": {
            "span_ms": span_ms,
            "real_loop_hz": round(real_hz, 1),
            "frame_hz": round(frame_hz, 1),
            "loop_inc_per_frame_mode": inc_mode,
            "dropped_loops_est": dropped,
            "dt_ms_stats": _stats(dt_ms),
        },
        "angle_loop":  {"actual_deg": _stats(col("imu_rol")),  "error_deg":   _stats(ang_err)},
        "rate_loop":   {"actual_dps": _stats(col("imu_vx")),   "error_dps":   _stats(rate_err)},
        "speed_loop":  {"now_speed0": _stats(post),            "cmd_raw":     _stats(raw)},
        "flywheel_saturation": {
            "duty_fraction": round(sat_duty, 3),
            "depth_stats": _stats(sat_depth),
            "note": "duty 长期 >0.2 说明飞轮经常憋在限幅里,先降增益/加大飞轮能力,别急着调其它环",
        },
        "inner_pid_terms": {
            "mean_abs_P": round(mean_p, 4),
            "mean_abs_I": round(mean_i, 4),
            "mean_abs_D": round(mean_d, 4),
            "dominant": max([("P", mean_p), ("I", mean_i), ("D", mean_d)], key=lambda x: x[1])[0],
            "share_pct": {"P": round(100 * mean_p / dom_total, 1),
                          "I": round(100 * mean_i / dom_total, 1),
                          "D": round(100 * mean_d / dom_total, 1)},
            "d_frame_to_frame_jitter": round(d_noise, 4),
            "note": "D 抖动远大于 |D| 均值 => D 在放大噪声,考虑降 kd 或加微分低通",
        },
        "fly_gain_pinned_fraction": round(fg_pinned, 3),
        "balance_active_fraction": round(balance_active_frac, 3),
        "health": {"imu_crc_err_last": col("crc_err_cnt")[-1],
                   "imu_crc_err_during": col("crc_err_cnt")[-1] - col("crc_err_cnt")[0]},
        "warnings": _warnings(r, m0_active, col),
    }
    return summary


def print_summary(s):
    print("\n" + "=" * 60)
    print("  调参摘要 (以车上时间/序号为基准)")
    print("=" * 60)
    if "error" in s:
        print("  ", s["error"]); return
    tm = s["timing"]
    print(f"  样本帧数        : {s['samples']}   窗口 {tm['span_ms']} ms")
    print(f"  真实环路频率    : {tm['real_loop_hz']} Hz   (发帧 {tm['frame_hz']} Hz)")
    print(f"  估计丢环数      : {tm['dropped_loops_est']}   每帧序号增量众数 {tm['loop_inc_per_frame_mode']}")
    ae = s["angle_loop"]["error_deg"]; re = s["rate_loop"]["error_dps"]
    print(f"  角度环误差(deg) : mean {ae['mean']:+.3f}  std {ae['std']:.3f}  |max| {max(abs(ae['min']),abs(ae['max'])):.3f}")
    print(f"  角速度环误差    : mean {re['mean']:+.3f}  std {re['std']:.3f}")
    fs = s["flywheel_saturation"]
    print(f"  飞轮饱和占比    : {fs['duty_fraction']*100:.1f}%   平均饱和深度 {fs['depth_stats'].get('mean',0):.3f}")
    ip = s["inner_pid_terms"]
    print(f"  内环分项主导    : {ip['dominant']}  (P {ip['share_pct']['P']}% / I {ip['share_pct']['I']}% / D {ip['share_pct']['D']}%)")
    print(f"  D 项逐帧抖动    : {ip['d_frame_to_frame_jitter']}   (|D|均值 {ip['mean_abs_D']})")
    print(f"  fly_gain 贴限   : {s['fly_gain_pinned_fraction']*100:.1f}%")
    print(f"  平衡使能占比    : {s.get('balance_active_fraction', 0)*100:.0f}%")
    h = s["health"]
    print(f"  IMU CRC 错      : 累计 {h['imu_crc_err_last']}  本段新增 {h.get('imu_crc_err_during', '?')}")
    for msg in s.get("warnings", []):
        print(f"  ! {msg}")
    print("=" * 60 + "\n")


# ----------------------------------------------------------------------------
# 串口读取主循环
# ----------------------------------------------------------------------------
def open_serial(port, baud):
    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("缺少 pyserial,请先: pip install pyserial")
    return serial.Serial(port, baud, timeout=0.05)


def list_ports():
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        sys.exit("缺少 pyserial,请先: pip install pyserial")
    ports = list(lp.comports())
    if not ports:
        print("未发现串口设备。")
        return
    print("可用串口:")
    for p in ports:
        print(f"  {p.device:25s}  {p.description}")


def run_stream(port, baud, capture_s=None, do_log=True, log_path=None):
    ser = open_serial(port, baud)
    parser = FrameParser()
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    if do_log and not log_path:
        log_path = os.path.join("logs", f"telem_{ts}.csv")
    logger = CsvLogger(log_path) if do_log else None
    captured = deque()
    t_start = time.time()
    last_report = t_start
    mode = f"抓取 {capture_s}s 窗口" if capture_s else "持续记录 (Ctrl-C 停止)"
    print(f"[bridge] {port} @ {baud}  {mode}")
    if logger:
        print(f"[bridge] 日志: {log_path}")
    try:
        while True:
            data = ser.read(4096)
            if data:
                for rec in parser.feed(data):
                    if logger:
                        logger.write(rec)
                    if capture_s is not None:
                        captured.append(rec)
            now = time.time()
            if now - last_report >= 1.0:
                print(f"[bridge] ok={parser.frames_ok} crc_bad={parser.crc_bad} "
                      f"ver/len_bad={parser.ver_bad} buf={len(parser.buf)}B", end="\r")
                last_report = now
            if capture_s is not None and (now - t_start) >= capture_s:
                break
    except KeyboardInterrupt:
        print("\n[bridge] 已停止。")
    finally:
        ser.close()
        if logger:
            logger.close()

    if capture_s is not None:
        recs = list(captured)
        s = analyze(recs)
        print_summary(s)
        js = os.path.join("logs", f"summary_{ts}.json")
        os.makedirs("logs", exist_ok=True)
        with open(js, "w") as f:
            json.dump(s, f, indent=2, ensure_ascii=False)
        print(f"[bridge] 摘要 JSON: {js}   数据 CSV: {log_path}")
        print("[bridge] 把上面两个文件路径告诉 Claude Code,让它读并结合固件源码分析。")


def analyze_csv(path):
    import csv
    recs = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rec = {}
            for name, fmt in SCHEMA:
                v = row.get(name)
                if v is None:
                    continue
                rec[name] = int(float(v)) if fmt in ("I", "i", "H", "h", "B") else float(v)
            recs.append(rec)
    s = analyze(recs)
    print_summary(s)
    print(json.dumps(s, indent=2, ensure_ascii=False))


def main():
    ap = argparse.ArgumentParser(description="飞轮平衡自行车 串口遥测桥接层 / PID 调参助手")
    ap.add_argument("--list", action="store_true", help="列出可用串口")
    ap.add_argument("--port", help="串口设备,如 /dev/tty.usbmodemXXXX")
    ap.add_argument("--baud", type=int, default=921600, help="波特率 (默认 921600)")
    ap.add_argument("--log", action="store_true", help="持续记录到 CSV")
    ap.add_argument("--capture", type=float, metavar="SEC", help="抓取 N 秒窗口后算摘要并退出")
    ap.add_argument("--logfile", help="指定日志 CSV 路径")
    ap.add_argument("--analyze", metavar="CSV", help="对已有 CSV 重算指标(离线)")
    args = ap.parse_args()

    print(f"[schema] PROTO_VERSION={PROTO_VERSION}  payload={PAYLOAD_LEN}B  fields={len(SCHEMA)}")

    if args.list:
        list_ports(); return
    if args.analyze:
        analyze_csv(args.analyze); return
    if not args.port:
        ap.error("需要 --port(或用 --list 查看,--analyze 离线分析)")
    if args.capture:
        run_stream(args.port, args.baud, capture_s=args.capture, do_log=True, log_path=args.logfile)
    else:
        run_stream(args.port, args.baud, capture_s=None, do_log=True, log_path=args.logfile)


if __name__ == "__main__":
    main()
