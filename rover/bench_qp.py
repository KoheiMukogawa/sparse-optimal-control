# -*- coding: utf-8 -*-
"""MPC-QP 求解時間ベンチ（実現可能性ゲート / 研究計画 P1.1）。

目的:
  RPi4 上で「状態3×ホライズンN」の MPC-QP が制御周期（既定0.1s）に
  収まるか、収まる N の上限はどこかを実測する。L2（二次コスト）版を主に
  測り、参考として L1（λ‖u‖₁）版も測る（cvxpy では目的関数1行差）。

設計の根拠:
  - 状態 = Kanayama 誤差 (x_e, y_e, θ_e)、入力 = (v, ω) の偏差 → follower_core と整合
  - 誤差ダイナミクスを参照軌道まわりで線形化・離散化した標準モデルを使う
    （求解時間は QP サイズで決まるので、係数の厳密さより次元構成が重要）
  - 終端は等式制約でなくソフトコスト（実行不可能回避、研究計画通り）

使い方（RPi 上で推奨）:
  python3 bench_qp.py                      # N=5..30 を L2 で掃引
  python3 bench_qp.py --horizons 10,20,30 --reg l1
  python3 bench_qp.py --period 0.2 --iters 200 --out ../results/bench

低電圧監視:
  実行中 /sys/class/hwmon/hwmon1/in0_lcrit_alarm を併監視し、
  1 が観測されたら結果に警告を残す（電源律速かどうかの切り分け用）。
"""

import argparse
import glob
import os
import platform
import statistics
import time

import numpy as np

try:
    import cvxpy as cp
except ImportError:
    raise SystemExit("cvxpy が必要です: pip install --user cvxpy osqp")


# ---- 誤差ダイナミクス（参照軌道まわりの線形化・連続時間） ----
# ẋ_e = w_r y_e - δv + v_r cosθ_e
# ẏ_e = -w_r x_e + v_r θ_e
# θ̇_e = -δω
# 状態 z=(x_e,y_e,θ_e)、入力 u=(δv,δω)。直進想定 w_r≈0 で評価。
def build_system(v_r, w_r, ts):
    A_c = np.array([[0.0, w_r, 0.0],
                    [-w_r, 0.0, v_r],
                    [0.0, 0.0, 0.0]])
    B_c = np.array([[-1.0, 0.0],
                    [0.0, 0.0],
                    [0.0, -1.0]])
    # 1次オイラー離散化（ベンチ目的には十分。求解時間はサイズ依存）
    A = np.eye(3) + ts * A_c
    B = ts * B_c
    return A, B


def build_problem(N, A, B, reg, lam, u_lim):
    """ホライズン N の MPC-QP を構築。z0 をパラメータにして再求解で使い回す。"""
    nx, nu = B.shape
    z0 = cp.Parameter(nx)
    z = cp.Variable((nx, N + 1))
    u = cp.Variable((nu, N))

    Q = np.diag([10.0, 10.0, 1.0])
    R = np.diag([1.0, 1.0])
    Qf = 10.0 * Q  # 終端ソフトコスト

    cost = 0
    constr = [z[:, 0] == z0]
    for k in range(N):
        cost += cp.quad_form(z[:, k], Q)
        if reg == "l1":
            cost += lam * cp.norm1(u[:, k])
        else:
            cost += cp.quad_form(u[:, k], R)
        constr += [z[:, k + 1] == A @ z[:, k] + B @ u[:, k]]
        constr += [cp.abs(u[:, k]) <= u_lim]
    cost += cp.quad_form(z[:, N], Qf)
    prob = cp.Problem(cp.Minimize(cost), constr)
    return prob, z0


def read_lcrit_alarm():
    """RPi の低電圧 critical アラームを読む。無ければ None。"""
    for path in glob.glob("/sys/class/hwmon/hwmon*/in0_lcrit_alarm"):
        try:
            with open(path) as f:
                return path, int(f.read().strip())
        except (OSError, ValueError):
            continue
    return None, None


def bench_one(N, A, B, reg, lam, u_lim, iters, warmup, rng):
    prob, z0 = build_problem(N, A, B, reg, lam, u_lim)
    nx = A.shape[0]
    times = []
    alarm_hits = 0
    alarm_path = None
    for i in range(iters + warmup):
        z0.value = rng.normal(scale=[0.2, 0.2, 0.3], size=nx)
        t0 = time.perf_counter()
        prob.solve(solver=cp.OSQP, warm_start=True)
        dt = time.perf_counter() - t0
        if i >= warmup:
            times.append(dt)
        ap, av = read_lcrit_alarm()
        if av:
            alarm_hits += 1
            alarm_path = ap
    return times, alarm_hits, alarm_path


def summarize(times):
    s = sorted(times)
    pct = lambda p: s[min(len(s) - 1, int(p * len(s)))]
    return {
        "n": len(s),
        "mean_ms": 1e3 * statistics.mean(s),
        "p50_ms": 1e3 * pct(0.50),
        "p95_ms": 1e3 * pct(0.95),
        "max_ms": 1e3 * s[-1],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--horizons", default="5,10,15,20,25,30")
    ap.add_argument("--reg", choices=["l2", "l1"], default="l2")
    ap.add_argument("--lam", type=float, default=1.0, help="L1 正則化係数")
    ap.add_argument("--period", type=float, default=0.1, help="制御周期[s]（判定閾値）")
    ap.add_argument("--iters", type=int, default=200)
    ap.add_argument("--warmup", type=int, default=20)
    ap.add_argument("--v_r", type=float, default=0.1)
    ap.add_argument("--w_r", type=float, default=0.0)
    ap.add_argument("--u_lim", type=float, default=0.5)
    ap.add_argument("--out", default=None, help="結果 md の出力先（拡張子なしのパス）")
    args = ap.parse_args()

    horizons = [int(x) for x in args.horizons.split(",")]
    A, B = build_system(args.v_r, args.w_r, args.period)
    rng = np.random.default_rng(0)

    ap_path, _ = read_lcrit_alarm()
    lines = []
    lines.append(f"# MPC-QP 求解時間ベンチ ({args.reg.upper()})")
    lines.append("")
    lines.append(f"- 機種: {platform.node()} / {platform.machine()}")
    lines.append(f"- 制御周期: {args.period*1e3:.0f} ms（判定閾値）, "
                 f"v_r={args.v_r}, w_r={args.w_r}, u_lim={args.u_lim}")
    lines.append(f"- iters={args.iters}, warmup={args.warmup}, solver=OSQP(warm_start)")
    lines.append(f"- 低電圧アラーム監視: {ap_path or '見つからず(非RPi?)'}")
    lines.append("")
    lines.append("| N | mean | p50 | p95 | max | 周期内? | lcrit |")
    lines.append("|---|------|-----|-----|-----|--------|-------|")

    print(lines[0])
    thr_ms = args.period * 1e3
    for N in horizons:
        times, alarm_hits, _ = bench_one(
            N, A, B, args.reg, args.lam, args.u_lim,
            args.iters, args.warmup, rng)
        s = summarize(times)
        ok = "OK" if s["p95_ms"] < thr_ms else "NG"
        row = (f"| {N} | {s['mean_ms']:.1f} | {s['p50_ms']:.1f} | "
               f"{s['p95_ms']:.1f} | {s['max_ms']:.1f} | {ok} | {alarm_hits} |")
        lines.append(row)
        print(f"N={N:2d}  p50={s['p50_ms']:6.1f}ms  p95={s['p95_ms']:6.1f}ms  "
              f"max={s['max_ms']:6.1f}ms  {ok}  lcrit={alarm_hits}")

    lines.append("")
    lines.append("判定: p95 < 周期 を満たす最大 N が設計上限。lcrit>0 なら電源律速の疑い。")
    report = "\n".join(lines) + "\n"

    if args.out:
        path = args.out + f"_{args.reg}.md"
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as f:
            f.write(report)
        print(f"\n結果を書き出し: {path}")


if __name__ == "__main__":
    main()
