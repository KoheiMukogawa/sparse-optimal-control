# -*- coding: utf-8 -*-
"""オープンループ最大ハンズオフ制御（卒論 2.6節・図2.5）。

永原(2017)の離散時間版を差動二輪の線形化モデルに適用したもの。参照速度 V で
直進中の動作点まわりで線形化し、n ステップで終端状態を厳密にゼロへ落とす
入力列のうち L1 ノルム最小のものを求める。

    min ||z||_1  s.t.  Φz = ζ,  ||z||_inf <= u_max

L1 最小化の解は多くの成分が厳密にゼロになる（bang-off-bang）。これが
「できるだけ手を放す」= Maximum Hands-off の離散時間版の実体である。

背景: 2026-04-28 に同じ実験を行った記録（δω ゼロ 39/50 = 78%・L1 30.65）が
`docs/作業記録/2026-04-28_sparse_control_sim.md` にあるが、当時の実装コードと
図はリポジトリに残っていない（`sparse_rover.py` は拡張子が .py の報告書）。
本モジュールは定式化から再実装し、記録値の再現性も確認する。

実行: uv run python rover/openloop_sparse.py
出力: results/2026-07-26_openloop_bangoffbang/{summary.md, series.csv, bangoffbang.png}
"""

import argparse
import csv
import math
import os

import cvxpy as cp
import numpy as np

ZERO_TOL = 1e-6  # |δω| <これ を「厳密にゼロ」とみなす


def discretize(h=0.1, V=1.0):
    """動作点まわりの線形モデルを離散化した (Ad, Bd) を返す。

    A = [[0,0,0],[0,0,V],[0,0,0]], B = [[1,0],[0,0],[0,1]] に対し
    Ad = I + A h, Bd = [[h,0],[0,V h^2/2],[0,h]]。
    """
    Ad = np.array([[1.0, 0.0, 0.0],
                   [0.0, 1.0, V * h],
                   [0.0, 0.0, 1.0]])
    Bd = np.array([[h, 0.0],
                   [0.0, V * h * h / 2.0],
                   [0.0, h]])
    return Ad, Bd


def reachability(Ad, Bd, n):
    """Φ = [Ad^(n-1)Bd, Ad^(n-2)Bd, ..., Bd]（3 × 2n）。"""
    blocks = [np.linalg.matrix_power(Ad, n - 1 - k) @ Bd for k in range(n)]
    return np.hstack(blocks)


def solve_sparse(x0, n=50, h=0.1, V=1.0, u_max=1.0):
    """L1 最小の入力列を求め、入力・状態軌道・スパース性の指標を返す。"""
    Ad, Bd = discretize(h, V)
    Phi = reachability(Ad, Bd, n)
    zeta = -np.linalg.matrix_power(Ad, n) @ np.array(x0, dtype=float)

    z = cp.Variable(2 * n)
    prob = cp.Problem(cp.Minimize(cp.norm1(z)),
                      [Phi @ z == zeta, cp.norm_inf(z) <= u_max])
    prob.solve()
    if z.value is None:
        raise RuntimeError(f"求解失敗: status={prob.status}")

    zv = np.asarray(z.value).ravel()
    dv = zv[0::2]
    dw = zv[1::2]

    x = np.array(x0, dtype=float)
    traj = [x.copy()]
    for k in range(n):
        x = Ad @ x + Bd @ zv[2 * k:2 * k + 2]
        traj.append(x.copy())

    return dict(
        z=zv.tolist(),
        dv=dv.tolist(),
        dw=dw.tolist(),
        x_traj=[t.tolist() for t in traj],
        l1=float(np.abs(zv).sum()),
        dw_zero=int(np.sum(np.abs(dw) < ZERO_TOL)),
        dv_zero=int(np.sum(np.abs(dv) < ZERO_TOL)),
        n=n,
        h=h,
    )


def write_outputs(sol, outdir, x0):
    os.makedirs(outdir, exist_ok=True)
    n, h = sol["n"], sol["h"]

    with open(os.path.join(outdir, "series.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["k", "t", "dx", "dy", "dtheta_deg", "dv", "dw"])
        for k in range(n):
            x = sol["x_traj"][k]
            w.writerow([k, f"{k*h:.2f}", f"{x[0]:.6f}", f"{x[1]:.6f}",
                        f"{math.degrees(x[2]):.4f}",
                        f"{sol['dv'][k]:.6f}", f"{sol['dw'][k]:.6f}"])

    xf = sol["x_traj"][-1]
    ratio = sol["dw_zero"] / n
    lines = [
        "# オープンループ最大ハンズオフ制御（再実装）",
        "",
        f"条件: h={h}s, V=1.0m/s, n={n}（{n*h:.0f}秒）, "
        f"x0=({x0[0]}, {x0[1]}, {math.degrees(x0[2]):.0f}deg), |z|inf<=1",
        "",
        "| 項目 | 値 |",
        "|------|-----|",
        f"| 終端 δx | {xf[0]:.2e} m |",
        f"| 終端 δy | {xf[1]:.2e} m |",
        f"| 終端 δθ | {math.degrees(xf[2]):.2e} deg |",
        f"| **δω ゼロ入力ステップ数** | **{sol['dw_zero']} / {n}（{ratio*100:.0f}%）** |",
        f"| δv ゼロ入力ステップ数 | {sol['dv_zero']} / {n}（{sol['dv_zero']/n*100:.0f}%） |",
        f"| L1ノルム（目的関数値） | {sol['l1']:.2f} |",
        "",
        "## 2026-04-28 の記録との対比",
        "",
        "| 項目 | 2026-04-28 の記録 | 本再実装 |",
        "|------|-------------------|----------|",
        f"| δω ゼロ入力 | 39 / 50（78%） | {sol['dw_zero']} / {n}（{ratio*100:.0f}%） |",
        f"| L1ノルム | 30.65 | {sol['l1']:.2f} |",
        "",
        "当時の実装コードと図はリポジトリに残っていないため、",
        "`docs/作業記録/2026-04-28_sparse_control_sim.md` の定式化から再実装した。",
    ]
    with open(os.path.join(outdir, "summary.md"), "w") as f:
        f.write("\n".join(lines) + "\n")

    _plot(sol, os.path.join(outdir, "bangoffbang.png"))


def _plot(sol, png_path):
    """図2.5: 誤差の収束と、3値をとる操舵入力 δω。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return

    n, h = sol["n"], sol["h"]
    t_state = [k * h for k in range(n + 1)]
    t_input = [k * h for k in range(n)]
    traj = sol["x_traj"]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.5, 6), sharex=True)

    ax1.plot(t_state, [x[0] for x in traj], label=r"$\delta x$ [m]")
    ax1.plot(t_state, [x[1] for x in traj], label=r"$\delta y$ [m]")
    ax1.plot(t_state, [math.degrees(x[2]) / 30.0 for x in traj],
             label=r"$\delta\theta$ [deg] $\div$ 30")
    ax1.axhline(0.0, color="0.6", lw=0.8)
    ax1.set_ylabel("tracking error")
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)

    zero = [abs(w) < ZERO_TOL for w in sol["dw"]]
    ax2.step(t_input, sol["dw"], where="post", color="0.25", lw=1.2)
    ax2.plot([t for t, z in zip(t_input, zero) if z], [0.0] * sum(zero),
             "o", ms=4, color="tab:green",
             label=f"exactly zero ({sol['dw_zero']}/{n})")
    ax2.plot([t for t, z in zip(t_input, zero) if not z],
             [w for w, z in zip(sol["dw"], zero) if not z],
             "o", ms=4, color="tab:red",
             label=f"active ({n - sol['dw_zero']}/{n})")
    ax2.axhline(0.0, color="0.6", lw=0.8)
    ax2.set_ylabel(r"steering correction  $\delta\omega$")
    ax2.set_xlabel("time [s]")
    ax2.legend(fontsize=9, loc="lower right")
    ax2.set_ylim(-1.35, 1.35)
    ax2.grid(True, alpha=0.3)

    fig.suptitle("Maximum hands-off (open loop): bang-off-bang steering")
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="オープンループ最大ハンズオフ制御")
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--h", type=float, default=0.1)
    ap.add_argument("--vel", type=float, default=1.0)
    ap.add_argument("--outdir", default="results/2026-07-26_openloop_bangoffbang")
    args = ap.parse_args()

    x0 = (2.0, 1.0, math.radians(30))
    sol = solve_sparse(x0, n=args.n, h=args.h, V=args.vel)
    xf = sol["x_traj"][-1]
    print(f"終端: δx={xf[0]:.2e} δy={xf[1]:.2e} δθ={math.degrees(xf[2]):.2e}deg")
    print(f"δω ゼロ入力: {sol['dw_zero']}/{args.n} "
          f"（{sol['dw_zero']/args.n*100:.0f}%）  記録値: 39/50（78%）")
    print(f"L1ノルム: {sol['l1']:.2f}  記録値: 30.65")

    write_outputs(sol, args.outdir, x0)
    print(f"\n出力: {args.outdir}/{{summary.md, series.csv, bangoffbang.png}}")


if __name__ == "__main__":
    main()
