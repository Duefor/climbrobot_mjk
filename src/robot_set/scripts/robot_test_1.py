#!/home/duefor/miniconda3/envs/mathdraw/bin/python
"""
Z 方向力数据折线图（含巴特沃斯低通滤波）

读取 force_log_*.csv，绘制 Fz 随时间变化的折线图：
  1. 原始 Fz
  2. 原始 Fz + 合成震荡（可选）
  3. 滤波后 Fz

用法: python robot_test_1.py [csv_file]

依赖: numpy, matplotlib, scipy
"""

import csv
import glob
import os
import sys
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import scipy.signal

# ---- 配置 ----
CSV_PATTERN = "force_log_*.csv"
X_LIM = (0, 10)
LINE_WIDTH = 0.3
CUTOFF_HZ = 15.0                # 巴特沃斯截止频率 (Hz)
FILTER_ORDER = 2                 # 滤波器阶数

# 合成震荡 — 在 [freq_low, freq_high] 内随机叠加多个正弦波
ADD_OSCILLATION = True
OSC_FREQ_LOW = 5.0               # 震荡最低频率 (Hz)
OSC_FREQ_HIGH = 20.0              # 震荡最高频率 (Hz)
OSC_NUM_WAVES = 15                # 随机正弦波个数
OSC_AMP = 0.5                     # 震荡幅值

TIME_COL = "timestamp_s"
FZ_COL = "Fz_N"


def find_csvs(pattern: str) -> List[str]:
    candidates = glob.glob(pattern)
    if not candidates:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        os.chdir(os.path.normpath(os.path.join(script_dir, "..", "..", "..")))
        candidates = glob.glob(pattern)
    candidates.sort(key=os.path.getmtime, reverse=True)
    return candidates


def pick_csv(candidates: List[str]) -> Optional[str]:
    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0]
    print("可用 CSV 文件:")
    for i, f in enumerate(candidates):
        print(f"  [{i}] {f}")
    while True:
        try:
            idx = int(input(f"选择 [0-{len(candidates)-1}]: ").strip())
            if 0 <= idx < len(candidates):
                return candidates[idx]
        except (ValueError, EOFError):
            pass
        print("无效输入，请重试")


def load_csv(filepath: str) -> Tuple[np.ndarray, np.ndarray]:
    """返回 (times, fz) 两个一维数组。"""
    times, fz_vals = [], []
    with open(filepath, "r") as f:
        for row in csv.DictReader(f):
            try:
                times.append(float(row[TIME_COL]))
                fz_vals.append(float(row[FZ_COL]))
            except (ValueError, KeyError):
                continue
    return np.array(times), np.array(fz_vals)


def estimate_fs(t: np.ndarray) -> float:
    if len(t) < 2:
        return 250.0
    dt = float(np.median(np.diff(t)))
    return 1.0 / max(dt, 1e-9) if dt > 0 else 250.0


def apply_butterworth(
    data: np.ndarray, cutoff: float, fs: float, order: int = 2
) -> np.ndarray:
    nyq = 0.5 * fs
    w = cutoff / nyq
    if w >= 1.0:
        print(f"[WARN] cutoff={cutoff} Hz >= Nyquist={nyq} Hz, 跳过滤波")
        return data.copy()
    b, a = scipy.signal.butter(order, w, btype="low")
    return scipy.signal.filtfilt(b, a, data)


def add_oscillation(data: np.ndarray, t: np.ndarray,
                    freq_low: float, freq_high: float,
                    num_waves: int, amp: float) -> np.ndarray:
    """叠加 N 个随机频率（在 [freq_low, freq_high] 内均匀分布）的正弦波。"""
    rng = np.random.default_rng(42)
    noise = np.zeros_like(t, dtype=np.float64)
    freqs = rng.uniform(freq_low, freq_high, num_waves)
    phases = rng.uniform(0, 2 * np.pi, num_waves)
    print(f"[INFO] 震荡频率: {np.sort(freqs).round(1)}")
    for f, phi in zip(freqs, phases):
        noise += np.sin(2 * np.pi * f * t + phi)
    noise *= amp / num_waves  # 归一化幅值
    return data.astype(np.float64) + noise


def plot_fz(t: np.ndarray, fz: np.ndarray, title: str,
            lw: float = 0.3) -> plt.Figure:
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, fz, linewidth=lw, color="#1f77b4")
    ax.set_xlim(X_LIM)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Fz (N)")
    ax.set_title(title)
    ax.grid(True, linestyle="--", alpha=0.5)
    fig.tight_layout()
    return fig


def main():
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
        if not os.path.isfile(csv_path):
            print(f"[ERROR] 文件不存在: {csv_path}", file=sys.stderr)
            sys.exit(1)
    else:
        candidates = find_csvs(CSV_PATTERN)
        csv_path = pick_csv(candidates)
        if csv_path is None:
            print(f"[ERROR] 未找到匹配: {CSV_PATTERN}", file=sys.stderr)
            sys.exit(1)

    print(f"[INFO] 读取: {csv_path}")
    t, fz = load_csv(csv_path)
    print(f"[INFO] 共 {len(t)} 个数据点")
    if len(t) < 2:
        print("[ERROR] 数据点不足", file=sys.stderr)
        sys.exit(1)

    fs = estimate_fs(t)
    print(f"[INFO] 采样率: {fs:.1f} Hz, "
          f"Butterworth order={FILTER_ORDER}, cutoff={CUTOFF_HZ} Hz")

    if ADD_OSCILLATION:
        print(f"[INFO] 叠加震荡: freq=[{OSC_FREQ_LOW}, {OSC_FREQ_HIGH}] Hz, "
              f"waves={OSC_NUM_WAVES}, amp={OSC_AMP}")
        fz_noisy = add_oscillation(fz, t, OSC_FREQ_LOW, OSC_FREQ_HIGH,
                                   OSC_NUM_WAVES, OSC_AMP)
    else:
        fz_noisy = fz

    fz_filtered = apply_butterworth(fz_noisy, CUTOFF_HZ, fs, FILTER_ORDER)

    basename = os.path.basename(csv_path)
    stem = os.path.splitext(csv_path)[0]
    osc_tag = f"_osc{OSC_FREQ_LOW}-{OSC_FREQ_HIGH}Hz" if ADD_OSCILLATION else ""

    # 图1: 原始
    fig1 = plot_fz(t, fz, f"Raw Fz — {basename}", LINE_WIDTH)
    fig1.savefig(stem + "_raw.png", dpi=150)
    print(f"[INFO] 已保存: {stem}_raw.png")

    # 图2: 原始 + 震荡
    if ADD_OSCILLATION:
        fig2 = plot_fz(t, fz_noisy,
                       f"Raw",
                       LINE_WIDTH)
        fig2.savefig(stem + f"_raw{osc_tag}.png", dpi=150)
        print(f"[INFO] 已保存: {stem}_raw{osc_tag}.png")

    # 图3: 滤波后
    title = (f"Filtered")
    fig3 = plot_fz(t, fz_filtered, title, LINE_WIDTH * 1.5)
    fig3.savefig(stem + f"_filtered{osc_tag}.png", dpi=150)
    print(f"[INFO] 已保存: {stem}_filtered{osc_tag}.png")

    plt.show()


if __name__ == "__main__":
    main()
