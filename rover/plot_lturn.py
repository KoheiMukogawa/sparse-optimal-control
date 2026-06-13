# -*- coding: utf-8 -*-
"""L字3者比較の可視化: odom軌跡（経路フレーム）＋ω(t)＋v(t)。

各 bag の /odom を「走行開始姿勢を原点・開始向きをx軸」に変換して重ね描き
（各runでodom絶対値が違うため）。/rover_twist から操舵ω・速度vの時系列。
チャタリング診断（ω符号反転回数・|ω|>1.5飽和率）も標準出力に出す。

使い方:
  uv run python rover/plot_lturn.py \
    results/2026-06-13_Lturn_kanayama results/2026-06-13_Lturn_l2 \
    results/2026-06-13_Lturn_l1 -o results/2026-06-13_Lturn_plot.png
"""

import argparse
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
V_ACTIVE = 0.005   # m/s: 走行中とみなす
W_DEAD = 0.05      # rad/s: ゼロ操舵帯


def yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def load(bagdir):
    odom, tw = [], []
    with AnyReader([Path(bagdir)], default_typestore=TYPESTORE) as r:
        for c, ts, raw in r.messages():
            t = ts * 1e-9
            if c.topic == '/odom':
                m = r.deserialize(raw, c.msgtype)
                p = m.pose.pose
                odom.append((t, p.position.x, p.position.y,
                             yaw(p.orientation.x, p.orientation.y,
                                 p.orientation.z, p.orientation.w)))
            elif c.topic == '/rover_twist':
                m = r.deserialize(raw, c.msgtype)
                tw.append((t, m.linear.x, m.angular.z))
    return odom, tw


def to_path_frame(odom):
    """走行開始姿勢を原点・開始向きをx軸にした経路フレームへ変換。"""
    # 走行はまだ無いが、起動〜発進まで静止なので先頭姿勢=開始姿勢S。
    _, x0, y0, th0 = odom[0]
    c, s = math.cos(th0), math.sin(th0)
    xs, ys = [], []
    for _, x, y, _ in odom:
        dx, dy = x - x0, y - y0
        xs.append(dx * c + dy * s)
        ys.append(-dx * s + dy * c)
    return xs, ys


def diag(name, tw):
    """チャタリング診断。"""
    active = [(t, v, w) for t, v, w in tw if abs(v) > V_ACTIVE]
    if not active:
        active = tw
    t0 = active[0][0]
    ts = [t - t0 for t, _, _ in active]
    ws = [w for _, _, w in active]
    vs = [v for _, v, _ in active]
    # ω符号反転（デッドバンド外同士）
    flips, prev = 0, 0
    for w in ws:
        sgn = 1 if w > W_DEAD else (-1 if w < -W_DEAD else 0)
        if sgn != 0:
            if prev != 0 and sgn != prev:
                flips += 1
            prev = sgn
    sat = sum(1 for w in ws if abs(w) > 1.5) / len(ws)
    zero = sum(1 for w in ws if abs(w) < W_DEAD) / len(ws)
    print(f"{name:<10} 反転{flips:>3}回  max|ω|={max(abs(w) for w in ws):.2f}  "
          f"|ω|>1.5={sat*100:4.0f}%  |ω|<0.05={zero*100:4.0f}%  "
          f"走行{ts[-1]:.1f}s")
    return ts, ws, vs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs=3, help="kanayama l2 l1 の順")
    ap.add_argument("-o", "--out", default="results/2026-06-13_Lturn_plot.png")
    args = ap.parse_args()

    labels = ["Kanayama", "L2-MPC", "L1-MPC(lam=0.3)"]
    colors = ["tab:gray", "tab:blue", "tab:red"]

    fig, ax = plt.subplots(1, 3, figsize=(16, 5.2))

    # 理想L字経路
    ideal_x = [0, 1.5, 1.5]
    ideal_y = [0, 0, 1.5]
    ax[0].plot(ideal_x, ideal_y, 'k--', lw=1.2, label="ideal path", zorder=1)

    print("=== チャタリング診断（走行区間 /rover_twist） ===")
    for bag, lab, col in zip(args.bags, labels, colors):
        odom, tw = load(bag)
        xs, ys = to_path_frame(odom)
        ax[0].plot(xs, ys, color=col, lw=1.4, label=lab, alpha=0.85)
        ts, ws, vs = diag(lab, tw)
        ax[1].plot(ts, ws, color=col, lw=1.0, label=lab, alpha=0.85)
        ax[2].plot(ts, vs, color=col, lw=1.0, label=lab, alpha=0.85)

    ax[0].set_title("odom trajectory (path frame)")
    ax[0].set_xlabel("x [m] (1st leg)")
    ax[0].set_ylabel("y [m] (2nd leg, left)")
    ax[0].set_aspect("equal", adjustable="box")
    ax[0].grid(True, alpha=0.3)
    ax[0].legend(fontsize=8)

    ax[1].axhline(2.0, color='k', ls=':', lw=0.8)
    ax[1].axhline(-2.0, color='k', ls=':', lw=0.8)
    ax[1].set_title("angular velocity omega(t)")
    ax[1].set_xlabel("t [s]")
    ax[1].set_ylabel("omega [rad/s]")
    ax[1].grid(True, alpha=0.3)
    ax[1].legend(fontsize=8)

    ax[2].set_title("linear velocity v(t)")
    ax[2].set_xlabel("t [s]")
    ax[2].set_ylabel("v [m/s]")
    ax[2].grid(True, alpha=0.3)
    ax[2].legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"\nsaved: {args.out}")


if __name__ == "__main__":
    main()
