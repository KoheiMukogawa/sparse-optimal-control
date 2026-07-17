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


def _dist_to_polyline(waypoints, x, y):
    """点 (x,y) から折れ線経路への最短距離 [m]。"""
    best = float('inf')
    for i in range(len(waypoints) - 1):
        ax, ay = waypoints[i]
        bx, by = waypoints[i + 1]
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy)
                         / (dx * dx + dy * dy)))
        d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
        best = min(best, d)
    return best


def truth_metrics(rows, waypoints, window_s=0.5):
    """カメラ真値 rows → 終点（末尾window_sの平均・θは円形平均）と横偏差RMSE。

    rows: [(t_s, x, y, theta, n_tags, quality_px)]（欠測は x=None）。
    有効行ゼロなら {}（runs.csv の truth列は空欄のまま）。

    追従指標（truth_end_dist_cm / truth_rmse_cm）は **実測した開始poseに
    固定したコース** 基準。follower はコースを自分の開始位置・向き基準で
    描くため、homing の向き残差（±2°でも1.4m先で±5cm）が追従誤差に
    混入しないようにする。タグ座標系ゴールへの絶対距離は
    truth_end_dist_abs_cm に別掲（設営・homing品質の指標）。
    """
    valid = [r for r in rows if r[1] is not None]
    if not valid:
        return {}
    t_end = valid[-1][0]
    win = [r for r in valid if r[0] >= t_end - window_s]
    ex = sum(r[1] for r in win) / len(win)
    ey = sum(r[2] for r in win) / len(win)
    eth = math.atan2(sum(math.sin(r[3]) for r in win),
                     sum(math.cos(r[3]) for r in win))
    # 開始pose（先頭window_sの平均）でコースを回転・平行移動
    t0 = valid[0][0]
    w0 = [r for r in valid if r[0] <= t0 + window_s]
    sx = sum(r[1] for r in w0) / len(w0)
    sy = sum(r[2] for r in w0) / len(w0)
    sth = math.atan2(sum(math.sin(r[3]) for r in w0),
                     sum(math.cos(r[3]) for r in w0))
    c, s = math.cos(sth), math.sin(sth)
    course = [(sx + c * px - s * py, sy + s * px + c * py)
              for px, py in waypoints]
    gx, gy = course[-1]
    agx, agy = waypoints[-1]
    rmse = math.sqrt(sum(_dist_to_polyline(course, r[1], r[2]) ** 2
                         for r in valid) / len(valid))
    return dict(truth_end_x=ex, truth_end_y=ey, truth_end_theta=eth,
                truth_end_dist_cm=math.hypot(ex - gx, ey - gy) * 100.0,
                truth_end_dist_abs_cm=math.hypot(ex - agx, ey - agy) * 100.0,
                truth_rmse_cm=rmse * 100.0)
