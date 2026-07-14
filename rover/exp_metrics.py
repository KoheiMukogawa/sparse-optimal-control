# -*- coding: utf-8 -*-
"""時系列 → 評価指標（bag/sim共通）。

analyze_bag.py（bag由来）と exp_backends.SimBackend（sim由来）の両方から使う。
指標は卒論4指標（RMSE・Σ|u|・スパース性・計算負荷）＋チャタ診断
（ω符号反転・飽和率。Lturn_compare.md B節と同定義）。
"""

import math

W_ZERO = 0.05    # |ω|<これ をゼロ操舵とみなす [rad/s]
V_ACTIVE = 0.005  # |v|>これ を「走行中」とみなす [m/s]
W_SAT = 1.5      # |ω|>これ を飽和側とみなす [rad/s]（上限2.0の75%）


def _pct(sorted_vals, p):
    if not sorted_vals:
        return float('nan')
    return sorted_vals[min(len(sorted_vals) - 1, int(p * len(sorted_vals)))]


def compute_metrics(twist, perr=(), solve_ms=()):
    """時系列から指標dictを返す。

    twist   : [(t_s, v, w)] 適用された速度指令
    perr    : [(t_s, y_e)]  横偏差（odom基準）
    solve_ms: [ms]          求解時間（Kanayamaは空でよい）
    """
    if not twist:
        raise ValueError("twist が空（走行データなし）")

    # 走行区間 = |v|>V_ACTIVE（最初の発進〜最後の駆動）: analyze_bag.py と同一
    active = [(t, v, w) for t, v, w in twist if abs(v) > V_ACTIVE]
    if not active:
        active = list(twist)
    t0, t1 = active[0][0], active[-1][0]
    dt = (t1 - t0) / max(1, len(active) - 1)

    sum_u = sum((abs(v) + abs(w)) for _, v, w in active) * dt
    ws = [w for _, _, w in active]
    w_zero = sum(1 for w in ws if abs(w) < W_ZERO) / len(ws)
    sat = sum(1 for w in ws if abs(w) > W_SAT) / len(ws)

    # ω符号反転（デッドバンドW_ZERO）: sim_delay_probe.py と同一
    flips, prev = 0, 0
    for w in ws:
        s = 1 if w > W_ZERO else (-1 if w < -W_ZERO else 0)
        if s != 0:
            if prev != 0 and s != prev:
                flips += 1
            prev = s

    ye = [y for t, y in perr if t0 - 0.2 <= t <= t1 + 0.2]
    rmse_cm = (100.0 * math.sqrt(sum(y * y for y in ye) / len(ye))
               if ye else float('nan'))

    sv = sorted(solve_ms)
    return dict(
        drive_s=t1 - t0,
        steps=len(active),
        rmse_cm=rmse_cm,
        sum_u=sum_u,
        w_zero_ratio=w_zero,
        flips=flips,
        sat_ratio=sat,
        max_w=max(abs(w) for w in ws),
        solve_p50=_pct(sv, 0.50),
        solve_p95=_pct(sv, 0.95),
        solve_max=sv[-1] if sv else float('nan'),
    )
