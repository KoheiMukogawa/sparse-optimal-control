# -*- coding: utf-8 -*-
"""実機 L1 チャタの原因仮説検証: 入力遅延・odomノイズを入れた閉ループsim。

理想sim（遅延0・ノイズ0）では L1 はクリーンだが、実機 L1 は L字コーナーで
bang-bang チャタを起こした（Σ|u| 4.5倍, ω符号反転15回）。
仮説: 求解/通信遅延（~1ステップ）と odom ノイズが、L1 の bang-off を
限界振動化させる（L2 のなめらか制御は遅延に頑健）。

ここでは差動二輪プラントに「入力遅延 delay_steps」「測定ノイズ」を加え、
L字コーナーで L1/L2 が遅延に対しどう振る舞うかを比較する。

実行: uv run python rover/sim_delay_probe.py
"""

import math
import random
from collections import deque

from follower_core import goal_crossed, goal_scaled_vr, reference_pose, tracking_error
from mpc_core import MPCFollower

V_MAX, W_MAX = 0.15, 2.0
DT = 0.1
BASE_VR = 0.1
LOOKAHEAD = 0.15
GOAL_TOL = 0.05
W_DEAD = 0.05
L_TURN = [(0.0, 0.0), (1.5, 0.0), (1.5, 1.5)]


def simulate(mpc, delay_steps=0, pos_noise=0.0, yaw_noise=0.0, seed=1):
    rng = random.Random(seed)
    x, y, th = 0.0, 0.0, 0.0
    t = 0.0
    buf = deque()
    applied = []      # (v, w) 実際にプラントへ入れた指令
    cross_sq = []
    reached = False
    while t < 120.0:
        gx, gy = L_TURN[-1]
        gd = math.hypot(gx - x, gy - y)
        if gd < GOAL_TOL or goal_crossed(L_TURN, x, y):
            reached = True
            break
        # 測定（ノイズ付き）→ 制御器が見る状態
        xm = x + rng.gauss(0, pos_noise)
        ym = y + rng.gauss(0, pos_noise)
        thm = th + rng.gauss(0, yaw_noise)
        x_r, y_r, th_r = reference_pose(L_TURN, xm, ym, LOOKAHEAD)
        x_e, y_e, th_e = tracking_error(xm, ym, thm, x_r, y_r, th_r)
        v_r = goal_scaled_vr(BASE_VR, gd)
        cmd = mpc.command(x_e, y_e, th_e, v_r, w_r=0.0)
        if cmd is None:
            break
        # 入力遅延: delay_steps 前の指令を適用（足りない間は直進巡航）
        buf.append(cmd)
        if len(buf) > delay_steps:
            v, w = buf.popleft()
        else:
            v, w = v_r, 0.0
        applied.append((v, w))
        # 真の非線形プラントを1ステップ
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        th += w * DT
        cross_sq.append(y_e ** 2)
        t += DT

    ws = [w for _, w in applied] or [0.0]
    flips, prev = 0, 0
    for w in ws:
        s = 1 if w > W_DEAD else (-1 if w < -W_DEAD else 0)
        if s != 0:
            if prev != 0 and s != prev:
                flips += 1
            prev = s
    sum_u = sum(abs(v) + abs(w) for v, w in applied) * DT
    rmse = 100 * math.sqrt(sum(cross_sq) / len(cross_sq)) if cross_sq else 0.0
    sat = sum(1 for w in ws if abs(w) > 1.5) / len(ws)
    zero = sum(1 for w in ws if abs(w) < W_DEAD) / len(ws)
    return dict(ok=reached, t=t, rmse=rmse, sum_u=sum_u,
                flips=flips, maxw=max(abs(w) for w in ws), sat=sat, zero=zero)


def row(tag, r):
    ok = "到達" if r["ok"] else "未達"
    print(f"{tag:<26} {ok} t={r['t']:4.1f}s RMSE={r['rmse']:5.2f}cm "
          f"Σ|u|={r['sum_u']:6.2f} 反転{r['flips']:>3} max|ω|={r['maxw']:.2f} "
          f"|ω|>1.5={r['sat']*100:3.0f}% ω0={r['zero']*100:3.0f}%")


def main():
    print("L字（経路上スタート）。Σ|u| は実機 analyze_bag と同じ ×dt 単位。")
    print("実機実測 → L2: Σ|u|4.54/反転0  L1: Σ|u|20.71/反転15\n")
    for reg, lam in [("l2", None), ("l1", 0.3)]:
        mpc = MPCFollower(N=15, ts=DT, reg=reg, lam=(lam or 1.0),
                          v_max=V_MAX, w_max=W_MAX)
        name = "L2" if reg == "l2" else f"L1(λ={lam})"
        print(f"--- {name} ---")
        row(f"  遅延0 ノイズ0(理想)", simulate(mpc, delay_steps=0))
        row(f"  遅延1step", simulate(mpc, delay_steps=1))
        row(f"  遅延2step", simulate(mpc, delay_steps=2))
        row(f"  遅延1+odomノイズ5mm", simulate(mpc, delay_steps=1, pos_noise=0.005, yaw_noise=math.radians(1)))
        print()

    # 対策候補: 移動抑制（Δu率ペナルティ）を L1 に入れて遅延2でチャタが消えるか
    print("=== 対策: L1(λ=0.3)+移動抑制 move_suppress を遅延2stepで評価 ===")
    for ms in [0.0, 0.5, 2.0, 5.0]:
        mpc = MPCFollower(N=15, ts=DT, reg="l1", lam=0.3,
                          v_max=V_MAX, w_max=W_MAX, move_suppress=ms)
        row(f"  ms={ms} 遅延2step", simulate(mpc, delay_steps=2))
    print("（参考）理想(遅延0)でのスパース性維持を確認:")
    for ms in [0.0, 2.0]:
        mpc = MPCFollower(N=15, ts=DT, reg="l1", lam=0.3,
                          v_max=V_MAX, w_max=W_MAX, move_suppress=ms)
        row(f"  ms={ms} 遅延0", simulate(mpc, delay_steps=0))


if __name__ == "__main__":
    main()
