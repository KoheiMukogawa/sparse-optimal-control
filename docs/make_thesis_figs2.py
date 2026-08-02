# -*- coding: utf-8 -*-
"""卒論の残りの図を作る（S9 で作れなかった分）。

`docs/make_thesis_figs.py` が実機バッチ由来の図（図5.6・5.7・6.2〜6.8）を作るのに対し、
本スクリプトは **(a) 別データ由来の図** と **(b) 模式図** を作る。

作る図:
  図1.3   本研究の論理の流れ                （模式）
  図3.1   コスト関数の対比                   （模式）
  図3.2   移動抑制項の作用                   （模式）
  図3.3   制御ノードの構成                   （模式）
  図4.1   システム構成                       （模式）
  図4.3   ソフトウェアの3層構成              （模式）
  図4.5   カメラ真値計測系の幾何             （模式）
  図4.6   俯瞰画像上のタグ検出例             （実画像・未追跡）
  図4.7   床タグ配置の良否                   （模式＋実測値）
  図4.8   タグ自動サーベイの処理             （模式）
  図4.9   自動原点復帰の状態遷移             （模式）
  図4.10  バッチ1サイクルのシーケンス        （模式）
  図5.1   N に対する求解時間                 （results/2026-06-13_rpi_bench/）
  図5.8   第5章の総括                        （模式）
  図6.1   経路形状と座標定義                 （configs/path_L_turn.yaml）
  図7.1   遅延による切替制御の限界振動       （数値計算）
  図7.2   λ=2 の見かけの最適                 （results/2026-07-26_delay_sweep/delay.csv）

実行: uv run python docs/make_thesis_figs2.py
出力: results/2026-08-02_thesis_figs2/
"""

import csv
import math
import os
import sys

ROVER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rover")
sys.path.insert(0, os.path.abspath(ROVER))

import numpy as np

import fig_style as FS
from fig_style import plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

OUT = "results/2026-08-2_thesis_figs2".replace("2026-08-2", "2026-08-02")
DELAY_CSV = "results/2026-07-26_delay_sweep/delay.csv"

# ---------------------------------------------------------------- 模式図の部品


def box(ax, x, y, w, h, text, fill=None, edge=None, fontsize=9, weight=None):
    """角丸の箱＋中央揃えテキスト。座標は箱の左下。"""
    fill = FS.FILL if fill is None else fill
    edge = FS.INK if edge is None else edge
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0.012,rounding_size=0.02",
        linewidth=1.2, facecolor=fill, edgecolor=edge, zorder=2))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fontsize, color=FS.INK, zorder=3, linespacing=1.5,
            fontweight=weight)
    return (x + w / 2, y + h / 2)


def arrow(ax, p0, p1, text=None, color=None, style="-|>", dx=0.0, dy=0.0,
          fontsize=8, rad=0.0):
    color = FS.INK if color is None else color
    ax.add_patch(FancyArrowPatch(
        p0, p1, arrowstyle=style, mutation_scale=12, linewidth=1.2,
        color=color, zorder=1,
        connectionstyle=f"arc3,rad={rad}"))
    if text:
        ax.text((p0[0] + p1[0]) / 2 + dx, (p0[1] + p1[1]) / 2 + dy, text,
                ha="center", va="center", fontsize=fontsize, color=color,
                zorder=3)


def blank(figsize=(9, 5)):
    """枠なしの模式図用キャンバス（0..1 の正規化座標）。"""
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    return fig, ax


def save(fig, name):
    path = os.path.join(OUT, name)
    fig.savefig(path)
    plt.close(fig)
    print(f"  {name}")


# ---------------------------------------------------------------- データ由来

# results/2026-06-13_rpi_bench/2026-06-13_rpi_bench_{l1,l2}.md の表を転記。
# 元は Markdown の表のみで機械可読な CSV が無いため、ここが唯一の機械可読な写し。
BENCH_N = [5, 10, 15, 20, 25, 30]
BENCH = {
    "l2": dict(mean=[11.4, 13.1, 13.6, 14.1, 16.0, 17.6],
               p95=[17.5, 19.0, 16.6, 14.3, 16.5, 17.7],
               max=[20.7, 23.8, 80.4, 14.5, 20.9, 18.7]),
    "l1": dict(mean=[9.8, 12.7, 16.2, 19.2, 22.5, 26.1],
               p95=[10.6, 14.1, 18.0, 21.6, 25.0, 29.2],
               max=[11.0, 14.6, 19.5, 23.9, 29.9, 34.8]),
}


def fig5_1():
    """N に対する求解時間。左=制御周期に対する余裕、右=L1/L2 の差の拡大。"""
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 4.2))
    for ax, zoom in zip(axes, (False, True)):
        for cond in ("l2", "l1"):
            st = FS.style(cond)
            ax.plot(BENCH_N, BENCH[cond]["p95"], marker="o", markersize=5,
                    linewidth=2, color=st["color"], linestyle=st["linestyle"],
                    label=f"{FS.SHORT[cond]}　p95")
            ax.plot(BENCH_N, BENCH[cond]["mean"], marker=".", markersize=4,
                    linewidth=1, color=st["color"], alpha=0.55,
                    linestyle=":", label=f"{FS.SHORT[cond]}　平均")
        ax.set_xlabel("予測ホライズン $N$")
        ax.set_ylabel("求解時間 [ms]")
        ax.set_xticks(BENCH_N)
        if zoom:
            ax.set_ylim(0, 34)
            ax.set_title("(b) 拡大（L1 と L2 の差）")
        else:
            ax.axhline(FS.CONTROL_PERIOD_MS, color=FS.COLOR["l1"],
                       linestyle="--", linewidth=1.5)
            ax.text(5.2, FS.CONTROL_PERIOD_MS - 4, "制御周期 100 ms（判定線）",
                    fontsize=8.5, color=FS.COLOR["l1"], va="top")
            ax.set_ylim(0, 112)
            ax.set_title("(a) 制御周期に対する余裕")
    axes[1].annotate("L1 は $N$ に対しほぼ線形に増える",
                     xy=(27.5, 26.2), xytext=(9.5, 28.5), fontsize=8.5,
                     color=FS.INK, va="center",
                     arrowprops=dict(arrowstyle="->", color=FS.INK_MUTED,
                                     linewidth=1,
                                     connectionstyle="arc3,rad=-0.15"))
    axes[1].annotate("L2 は $N$ にほぼ依らない", xy=(22, 14.8), xytext=(11, 5.5),
                     fontsize=8.5, color=FS.INK,
                     arrowprops=dict(arrowstyle="->", color=FS.INK_MUTED,
                                     linewidth=1))
    axes[0].legend(frameon=False, fontsize=8.5, loc="center left")
    fig.suptitle("図5.1　Raspberry Pi 4 上の QP 求解時間（単体ベンチ・各 200 回）")
    fig.text(0.5, 0.005,
             "注: L2・$N$=15 の最大値 80.4 ms は cvxpy の初回コールド求解（発進前1回のみ）。"
             "本図は p95 と平均を示す。",
             ha="center", fontsize=7.5, color=FS.INK_MUTED)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    save(fig, "fig5_1_solve_vs_N.png")


def fig6_1():
    """経路形状と、本論文で用いる2つの座標系の定義。"""
    wps = [(0.0, 0.0), (1.5, 0.0), (1.5, 1.5)]
    fig, ax = plt.subplots(figsize=(6.4, 5.4))
    xs, ys = zip(*wps)
    ax.plot(xs, ys, "-", linewidth=3, color=FS.COLOR["l2"], zorder=2,
            label="参照経路（L字）")
    ax.plot([0], [0], marker="o", markersize=10, color=FS.INK, zorder=3)
    ax.plot([1.5], [1.5], marker="*", markersize=17, color=FS.COLOR["l1_ms2"],
            zorder=3)
    ax.annotate("走行原点\n（開始姿勢）", (0, 0), textcoords="offset points",
                xytext=(10, -6), fontsize=9, va="top", color=FS.INK)
    ax.annotate("ゴール", (1.5, 1.5), textcoords="offset points",
                xytext=(12, 0), fontsize=9, color=FS.INK)
    ax.annotate("", xy=(1.5, 0), xytext=(0.0, 0),
                arrowprops=dict(arrowstyle="-|>", color=FS.INK_MUTED, lw=1))
    ax.text(0.75, 0.09, "第1直線 1.5 m", ha="center", fontsize=9,
            color=FS.INK_MUTED)
    ax.text(1.63, 0.75, "第2直線 1.5 m", rotation=90, va="center", fontsize=9,
            color=FS.INK_MUTED)
    ax.annotate("左 90°", (1.5, 0), textcoords="offset points",
                xytext=(-52, 12), fontsize=9, color=FS.INK_MUTED)
    # 終点計測の座標系（第6.5節）
    ax.annotate("", xy=(2.05, 1.5), xytext=(1.5, 1.5),
                arrowprops=dict(arrowstyle="-|>", color=FS.COLOR["l1"], lw=1.6))
    ax.annotate("", xy=(1.5, 2.05), xytext=(1.5, 1.5),
                arrowprops=dict(arrowstyle="-|>", color=FS.COLOR["l1"], lw=1.6))
    ax.text(2.09, 1.5, "$x$：コーナー外側（＋）", fontsize=8.5, va="center",
            color=FS.COLOR["l1"])
    ax.text(1.5, 2.12, "$y$：行き過ぎ（＋）", fontsize=8.5, ha="center",
            color=FS.COLOR["l1"])
    ax.text(1.5, 2.35, "終点計測の座標系（ゴールを原点）", fontsize=8.5,
            ha="center", color=FS.COLOR["l1"], fontweight="bold")
    ax.set_xlabel("$x$ [m]（第1直線の進行方向）")
    ax.set_ylabel("$y$ [m]")
    ax.set_aspect("equal")
    ax.set_xlim(-0.45, 2.6)
    ax.set_ylim(-0.45, 2.6)
    ax.set_title("図6.1　L字経路の形状と座標定義")
    fig.tight_layout()
    save(fig, "fig6_1_course.png")


def _load_delay():
    with open(DELAY_CSV) as f:
        return [dict(name=r["name"], d=int(r["delay_steps"]),
                     rmse=float(r["rmse"]), sum_u=float(r["sum_u"]),
                     flips=float(r["flips"])) for r in csv.DictReader(f)]


def fig7_2():
    """λ=2 の RMSE が遅延2stepでのみ落ち込む＝見かけの最適（第7.4.1節）。"""
    rows = _load_delay()
    fig, ax = plt.subplots(figsize=(7.0, 4.4))
    show = [("l1_lam2_ms0.5", "λ=2・$w_{ms}$=0.5（単一条件では最良に見えた点）"),
            ("l1_lam0.3_ms2", "λ=0.3・$w_{ms}$=2.0（採用値）")]
    for name, label in show:
        s = sorted((r for r in rows if r["name"] == name), key=lambda r: r["d"])
        ax.plot([r["d"] for r in s], [r["rmse"] for r in s],
                marker=FS.DELAY_MARKER[name], markersize=7, linewidth=2.2,
                color=FS.DELAY_COLOR[name], linestyle=FS.DELAY_LINESTYLE[name],
                label=label)
    ax.axvline(2, color=FS.INK_MUTED, linestyle=":", linewidth=1.2, zorder=0)
    ax.text(1.92, 6.2, "設計に使った\n単一条件（遅延2ステップ）", fontsize=8.5,
            ha="right", va="center", color=FS.INK_MUTED)
    ax.annotate("この1点だけ 1.19 cm", xy=(2, 1.19), xytext=(2.7, 4.6),
                fontsize=9, color=FS.INK,
                arrowprops=dict(arrowstyle="->", color=FS.INK_MUTED, lw=1.1))
    ax.annotate("両隣は 11〜14 cm", xy=(3, 12.86), xytext=(3.1, 8.4),
                fontsize=9, color=FS.INK,
                arrowprops=dict(arrowstyle="->", color=FS.INK_MUTED, lw=1.1))
    ax.set_xlabel("入力遅延 [ステップ（1 step = 0.1 s）]")
    ax.set_ylabel("横偏差 RMSE [cm]")
    ax.set_xticks([0, 1, 2, 3, 4])
    ax.set_ylim(0, 16)
    ax.legend(frameon=False, fontsize=9, loc="upper center",
              bbox_to_anchor=(0.5, -0.15), ncol=1)
    ax.set_title("図7.2　単一条件での最適化が掴む「見かけの最適」")
    fig.tight_layout()
    save(fig, "fig7_2_apparent_optimum.png")


def fig7_1():
    """切替制御＋むだ時間が限界振動を生む機序（数値計算による概念図）。

    最も単純な切替系 dx/dt = -K·sign(x(t-τ)) を陽解法で解く。
    τ=0 では原点へ収束し、τ>0 では振幅が τ に比例する極限閉軌道に落ち着く。
    L1 最適制御の bang-off-bang 入力が遅延下で限界振動化する機序と同型である。
    """
    dt, T, K = 0.002, 6.0, 1.0
    n = int(T / dt)

    def run(tau):
        lag = int(round(tau / dt))
        x = np.zeros(n)
        x[0] = 1.0
        for i in range(n - 1):
            xd = x[max(0, i - lag)]
            u = -K * (1.0 if xd > 0 else (-1.0 if xd < 0 else 0.0))
            x[i + 1] = x[i] + dt * u
        return x

    t = np.arange(n) * dt
    fig, ax = plt.subplots(figsize=(7.4, 4.4))
    for tau, color, ls, lab in [
            (0.0, FS.COLOR["l1_ms2"], "-", "むだ時間 τ = 0（理想）"),
            (0.10, FS.COLOR["kanayama"], (0, (4, 1.5)), "τ = 0.10 s"),
            (0.20, FS.COLOR["l1"], "-", "τ = 0.20 s（実機の推定遅延）")]:
        ax.plot(t, run(tau), color=color, linestyle=ls, linewidth=2, label=lab)
    ax.axhline(0, color=FS.INK_MUTED, linewidth=0.8, zorder=0)
    ax.set_xlabel("時間 [s]")
    ax.set_ylabel("状態 $x$")
    ax.set_ylim(-0.42, 1.05)
    ax.legend(frameon=False, fontsize=9)
    ax.set_title("図7.1　切替入力とむだ時間が生む限界振動\n"
                 "（$\\dot{x} = -\\mathrm{sign}\\,x(t-\\tau)$・振幅は τ に比例）")
    ax.annotate("遅延がないと原点で止まる", xy=(3.0, 0.0), xytext=(3.2, 0.42),
                fontsize=8.5, color=FS.INK,
                arrowprops=dict(arrowstyle="->", color=FS.INK_MUTED, lw=1))
    ax.annotate("行き過ぎ→戻し を繰り返す\n＝チャタリング", xy=(4.6, -0.2),
                xytext=(3.4, -0.38), fontsize=8.5, color=FS.INK,
                arrowprops=dict(arrowstyle="->", color=FS.INK_MUTED, lw=1))
    fig.tight_layout()
    save(fig, "fig7_1_limit_cycle.png")


def fig3_2():
    """移動抑制項が入力系列に及ぼす作用（模式）。"""
    k = np.arange(14)
    naive = np.array([0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 0, 0, 0, 0], float)
    fixed = np.array([0, 0, 1, 1, 1, 0.6, 0.2, 0, 0, 0, 0, 0, 0, 0], float)
    fig, axes = plt.subplots(2, 1, figsize=(7.0, 4.6), sharex=True)
    for ax, y, color, title in [
            (axes[0], naive, FS.COLOR["l1"],
             "$w_{ms}=0$：符号反転が続く（振幅は上限に張り付く）"),
            (axes[1], fixed, FS.COLOR["l1_ms2"],
             "$w_{ms}>0$：上限までは動くが、反転が抑えられる")]:
        ax.step(k, y, where="mid", color=color, linewidth=2)
        ax.fill_between(k, 0, y, step="mid", color=color, alpha=0.18)
        ax.axhline(0, color=FS.INK_MUTED, linewidth=0.8)
        ax.set_ylim(-1.45, 1.45)
        ax.set_yticks([-1, 0, 1])
        ax.set_yticklabels(["$-\\delta u_{\\max}$", "0", "$+\\delta u_{\\max}$"])
        ax.set_title(title, fontsize=10)
        ax.set_ylabel("補正 $\\delta\\omega$")
    axes[1].set_xlabel("予測ステップ $k$")
    fig.suptitle("図3.2　移動抑制項が抑えるのは「大きさ」ではなく「変化率」")
    fig.tight_layout()
    save(fig, "fig3_2_move_suppress.png")


# ---------------------------------------------------------------- 模式図


def fig1_3():
    fig, ax = blank((9.6, 2.9))
    steps = [
        ("実機で問題が発現\nL1 がコーナーで\nチャタリング（第5.4節）", FS.COLOR["l1"]),
        ("sim で機序を特定\n入力遅延2stepで\n定量再現（第5.6節）", None),
        ("対策を設計\n入力変化率への\nペナルティ（第5.7節）", None),
        ("実機で確認\n反転 18→1 回\n（第5.8節）", FS.COLOR["l1_ms2"]),
        ("sim の予測性を検証\n入力側は±7%以内\n（第5.9節）", None),
    ]
    w, h, gap = 0.167, 0.56, 0.031
    for i, (text, edge) in enumerate(steps):
        x = i * (w + gap) + 0.019
        c = box(ax, x, 0.22, w, h, text, fontsize=8.4, edge=edge,
                fill=FS.FILL_ACCENT if edge else FS.FILL)
        if i:
            arrow(ax, (x - gap + 0.004, 0.5), (x - 0.004, 0.5))
    ax.text(0.5, 0.04, "問い: スパース制御を実機で成立させる条件は何か",
            ha="center", fontsize=10, color=FS.INK, fontweight="bold")
    ax.set_title("図1.3　本研究の論理の流れ")
    fig.tight_layout()
    save(fig, "fig1_3_logic_flow.png")


def fig3_1():
    fig, ax = blank((8.6, 3.0))
    box(ax, 0.03, 0.30, 0.44, 0.42,
        "L2-MPC\n\n"
        "cost += cp.quad_form(du, R)", fill=FS.FILL, edge=FS.COLOR["l2"],
        fontsize=10)
    box(ax, 0.53, 0.30, 0.44, 0.42,
        "L1-MPC\n\n"
        "cost += lam * cp.norm1(du)", fill=FS.FILL, edge=FS.COLOR["l1_ms2"],
        fontsize=10)
    arrow(ax, (0.475, 0.51), (0.525, 0.51), style="<|-|>")
    ax.text(0.5, 0.17, "差分はこの1行のみ。骨格・制約・重み・実装は完全に共有する",
            ha="center", fontsize=9.5, color=FS.INK)
    ax.text(0.5, 0.06, "（`rover/mpc_core.py`）", ha="center", fontsize=8.5,
            color=FS.INK_MUTED)
    ax.set_title("図3.1　L2 と L1 の差はコスト関数の1行に限定される")
    fig.tight_layout()
    save(fig, "fig3_1_cost_diff.png")


def fig3_3():
    fig, ax = blank((8.6, 3.4))
    c = box(ax, 0.34, 0.35, 0.32, 0.30, "mpc_follower\n（制御ノード）",
            fill=FS.FILL_ACCENT, fontsize=9.5)
    box(ax, 0.03, 0.40, 0.22, 0.20, "/odom", fontsize=9)
    arrow(ax, (0.25, 0.50), (0.34, 0.50), "購読", dy=0.055)
    outs = [("/rover_twist\n速度指令", 0.72), ("/path_error\n追従誤差", 0.42),
            ("/mpc_solve_ms\n求解時間", 0.12)]
    for label, y in outs:
        box(ax, 0.74, y, 0.23, 0.20, label, fontsize=9)
        arrow(ax, (0.66, 0.50), (0.74, y + 0.10), rad=0.0)
    ax.text(0.70, 0.955, "配信", fontsize=9, color=FS.INK)
    box(ax, 0.34, 0.03, 0.32, 0.18, "follower_core（共通）\n参照点・誤差・ゴール判定",
        fontsize=8.6, edge=FS.INK_MUTED)
    arrow(ax, (0.50, 0.21), (0.50, 0.35), style="-|>")
    ax.set_title("図3.3　制御ノードの構成と入出力トピック")
    fig.tight_layout()
    save(fig, "fig3_3_node.png")


def fig4_1():
    fig, ax = blank((9.4, 4.0))
    box(ax, 0.06, 0.13, 0.40, 0.69, "", fill=FS.FILL_ACCENT)
    ax.text(0.26, 0.745, "LiteRover（機体）", ha="center", fontsize=10,
            fontweight="bold", color=FS.INK, zorder=3)
    ax.text(0.26, 0.645, "Raspberry Pi 4\nUbuntu 22.04 / ROS 2 Humble",
            ha="center", va="center", fontsize=9, color=FS.INK, zorder=3,
            linespacing=1.5)
    for label, y in [("モータ・オドメトリ", 0.36), ("LiDAR", 0.19)]:
        box(ax, 0.10, y, 0.32, 0.13, label, fontsize=8.6, edge=FS.INK_MUTED,
            fill="#FFFFFF")
    ax.text(0.26, 0.87, "制御ノードはすべてここで実行", ha="center", fontsize=9,
            color=FS.COLOR["l2"], fontweight="bold")
    box(ax, 0.58, 0.52, 0.38, 0.30,
        "開発用ラップトップ\nWSL2 Ubuntu 24.04 / ROS 2 Jazzy\n（起動・監視・記録のみ）",
        fontsize=9)
    box(ax, 0.58, 0.10, 0.38, 0.24, "俯瞰カメラ C270\n＋ AprilTag（真値計測）",
        fontsize=9, edge=FS.COLOR["l1_ms2"])
    arrow(ax, (0.58, 0.22), (0.46, 0.40),
          "UDP（復帰指令のみ）", dy=-0.075, color=FS.COLOR["l1_ms2"])
    arrow(ax, (0.58, 0.62), (0.46, 0.62), "SSH / bag 回収", dy=0.06)
    ax.text(0.5, 0.015,
            "制御ループに無線を含めない構成とし、DDS の不安定性を運用で回避している",
            ha="center", fontsize=8.6, color=FS.INK_MUTED)
    ax.set_title("図4.1　システム構成")
    fig.tight_layout()
    save(fig, "fig4_1_system.png")


def fig4_3():
    fig, ax = blank((9.0, 3.6))
    layers = [
        ("評価・解析層", "run_batch ・ exp_backends ・ analyze_bag", 0.70, FS.FILL),
        ("ノード層", "mpc_follower ・ path_follower ・ udp_twist_bridge", 0.42,
         FS.FILL),
        ("純ロジック層（ROS 非依存）",
         "follower_core ・ mpc_core ・ exp_metrics ・ truth_core ・ homing",
         0.14, FS.FILL_ACCENT),
    ]
    for name, mods, y, fill in layers:
        box(ax, 0.06, y, 0.88, 0.22, "", fill=fill)
        ax.text(0.10, y + 0.145, name, fontsize=9.5, va="center",
                fontweight="bold", color=FS.INK, zorder=3)
        ax.text(0.10, y + 0.065, mods, fontsize=8.6, va="center",
                color=FS.INK, zorder=3)
    for y in (0.64, 0.36):
        arrow(ax, (0.5, y + 0.06), (0.5, y))
    ax.text(0.5, 0.03,
            "純ロジック層は sim と実機で共有される＝比較に制御則以外の差分が入らない",
            ha="center", fontsize=9, color=FS.COLOR["l2"])
    ax.set_title("図4.3　ソフトウェアの3層構成")
    fig.tight_layout()
    save(fig, "fig4_3_layers.png")


def fig4_5():
    """計測系の幾何。カメラ→床タグ（PnP）と、光線と高さ z の平面の交点。"""
    fig, ax = plt.subplots(figsize=(7.4, 4.8))
    cam = np.array([0.55, 1.85])
    ax.plot(*cam, marker="v", markersize=15, color=FS.INK, zorder=4)
    ax.text(cam[0], cam[1] + 0.09, "俯瞰カメラ", ha="center", fontsize=9.5)
    # 床とロボット上面
    ax.axhline(0, color=FS.INK, linewidth=1.6)
    ax.text(-0.03, 0.03, "床（$z=0$）", fontsize=9, color=FS.INK)
    ax.plot([0.95, 1.30], [0.42, 0.42], color=FS.COLOR["l1_ms2"], linewidth=3)
    ax.text(1.33, 0.42, "ロボット上面タグ\n（$z=0.12$ m）", fontsize=9,
            va="center", color=FS.COLOR["l1_ms2"])
    ax.annotate("", xy=(0.95, 0.0), xytext=(0.95, 0.42),
                arrowprops=dict(arrowstyle="<|-|>", color=FS.INK_MUTED, lw=1))
    ax.text(0.90, 0.21, "$z$", fontsize=9, ha="right", color=FS.INK_MUTED)
    # 床タグ（PnP に使う4枚のうち3枚を描く）
    for x in (0.10, 0.62, 1.62):
        ax.plot([x - 0.06, x + 0.06], [0, 0], color=FS.COLOR["l2"],
                linewidth=5, zorder=3)
        ax.annotate("", xy=(x, 0.02), xytext=cam,
                    arrowprops=dict(arrowstyle="-", color=FS.COLOR["l2"],
                                    lw=1, alpha=0.7))
    ax.text(0.36, -0.13, "床基準タグ（中心座標が既知）", ha="center",
            fontsize=9, color=FS.COLOR["l2"])
    # ロボットタグへの光線と、床へ落ちる誤差
    ax.annotate("", xy=(1.12, 0.42), xytext=cam,
                arrowprops=dict(arrowstyle="-|>", color=FS.COLOR["l1_ms2"],
                                lw=1.8))
    ax.annotate("", xy=(1.33, 0.0), xytext=(1.12, 0.42),
                arrowprops=dict(arrowstyle="-", color=FS.COLOR["l1"], lw=1.2,
                                linestyle=":"))
    ax.annotate("", xy=(1.12, -0.06), xytext=(1.33, -0.06),
                arrowprops=dict(arrowstyle="<|-|>", color=FS.COLOR["l1"], lw=1))
    ax.annotate("床平面で解くと\nこの分ずれる", xy=(1.30, -0.06),
                xytext=(1.60, -0.24), fontsize=8.4, color=FS.COLOR["l1"],
                va="center",
                arrowprops=dict(arrowstyle="->", color=FS.COLOR["l1"], lw=0.9))
    ax.text(0.02, 1.55, "① 床タグ4枚の中心から\n　 solvePnP でカメラ姿勢を求める",
            fontsize=9, color=FS.COLOR["l2"])
    ax.annotate("② 視線と平面 $z$ の\n　 交点で厳密に解く", xy=(1.12, 0.42),
                xytext=(1.28, 0.95), fontsize=9, color=FS.COLOR["l1_ms2"],
                arrowprops=dict(arrowstyle="->", color=FS.COLOR["l1_ms2"],
                                lw=0.9))
    ax.set_xlim(-0.05, 2.20)
    ax.set_ylim(-0.42, 2.05)
    ax.axis("off")
    ax.set_title("図4.5　カメラ真値計測系の幾何")
    fig.tight_layout()
    save(fig, "fig4_5_geometry.png")


def fig4_7():
    """床タグ配置の良否（実測値つき）。"""
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 4.4))
    course = [(0, 0), (1.0, 0), (1.0, 1.0)]
    layouts = [
        ("(a) 片側に偏在（不良）", [(-0.25, -0.25), (-0.20, 0.50), (0.38, 0.45),
                                    (0.42, -0.30)], "手実測との差 約 15 cm",
         FS.COLOR["l1"]),
        ("(b) コースを囲む（良）", [(-0.2, -0.2), (-0.17, 0.55), (1.15, 1.23),
                                    (1.14, -0.20)], "手実測との差 2.5 cm",
         FS.COLOR["l1_ms2"]),
    ]
    for ax, (title, tags, note, color) in zip(axes, layouts):
        xs, ys = zip(*course)
        ax.plot(xs, ys, linewidth=3, color=FS.COLOR["l2"], zorder=2)
        tx, ty = zip(*tags)
        ax.scatter(tx, ty, s=90, marker="s", color=FS.INK, zorder=3,
                   label="床タグ")
        # 重心まわりの偏角で並べ替えて凸包を閉じる（自己交差を避ける）
        cx, cy = sum(tx) / len(tx), sum(ty) / len(ty)
        ordered = sorted(tags, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
        hull = ordered + [ordered[0]]
        hx, hy = zip(*hull)
        ax.fill(hx, hy, color=color, alpha=0.12, zorder=0)
        ax.plot(hx, hy, color=color, linewidth=1.2, linestyle="--", zorder=1)
        ax.plot([1.0], [1.0], marker="*", markersize=16,
                color=FS.COLOR["l1_ms2"], zorder=4)
        ax.text(1.0, 1.12, "ゴール", ha="center", fontsize=9)
        ax.set_title(f"{title}\n{note}", fontsize=10, color=color)
        ax.set_aspect("equal")
        ax.set_xlim(-0.5, 1.6)
        ax.set_ylim(-0.6, 1.5)
        ax.set_xticks([])
        ax.set_yticks([])
    axes[0].annotate("ゴールが\n外挿領域", xy=(1.0, 1.0), xytext=(0.55, 0.85),
                     fontsize=8.6, color=FS.COLOR["l1"],
                     arrowprops=dict(arrowstyle="->", color=FS.COLOR["l1"],
                                     lw=1.1))
    fig.suptitle("図4.7　床タグ配置が計測精度を決める（破線＝タグ4枚の凸包）")
    fig.tight_layout()
    save(fig, "fig4_7_tag_layout.png")


def fig4_8():
    fig, ax = blank((9.6, 2.6))
    steps = ["① タグごと\nIPPE で初期解",
             "② 全16隅の\n再投影誤差を\n一括最小化",
             "③ 規約へ変換\ntag0=原点\ntag0→tag3=+x",
             "④ 配置チェック\n（凸包内包）",
             "⑤ yaml を更新"]
    w, h, gap = 0.167, 0.55, 0.031
    for i, text in enumerate(steps):
        x = i * (w + gap) + 0.019
        box(ax, x, 0.28, w, h, text, fontsize=8.4)
        if i:
            arrow(ax, (x - gap + 0.004, 0.555), (x - 0.004, 0.555))
    ax.text(0.5, 0.10, "拘束: 全タグは床平面 $z=0$ 上・各タグは実寸 0.135 m の正方形",
            ha="center", fontsize=9, color=FS.INK_MUTED)
    ax.set_title("図4.8　タグ自動サーベイの処理")
    fig.tight_layout()
    save(fig, "fig4_8_survey.png")


def fig4_9():
    fig, ax = blank((9.2, 3.0))
    ph = [("TURN\n目標方向へ\nその場旋回", 0.04), ("GO\n直線追従\n（Kanayama）", 0.30),
          ("ALIGN\n最終向きを\n合わせる", 0.56)]
    for text, x in ph:
        box(ax, x, 0.32, 0.20, 0.38, text, fontsize=9, fill=FS.FILL_ACCENT)
    arrow(ax, (0.24, 0.51), (0.30, 0.51))
    ax.text(0.27, 0.755, "向き誤差\n<45°", ha="center", fontsize=8,
            color=FS.INK)
    arrow(ax, (0.50, 0.51), (0.56, 0.51))
    ax.text(0.53, 0.755, "距離\n<3 cm", ha="center", fontsize=8, color=FS.INK)
    box(ax, 0.80, 0.38, 0.17, 0.26, "完了\n（±3 cm / ±2°）", fontsize=8.6,
        edge=FS.COLOR["l1_ms2"])
    arrow(ax, (0.76, 0.51), (0.80, 0.51))
    box(ax, 0.22, 0.03, 0.34, 0.18,
        "10 cm 圏内は後退を許す\n点収束則へ切替", fontsize=8.6,
        edge=FS.COLOR["l1"], fill="#FFFFFF")
    arrow(ax, (0.38, 0.32), (0.38, 0.21), color=FS.COLOR["l1"])
    ax.text(0.585, 0.12,
            "行き過ぎたときに Kanayama が作る平衡点\n（sim で 4.8 cm 停滞）を回避するため",
            fontsize=8.4, color=FS.COLOR["l1"], va="center")
    ax.set_title("図4.9　自動原点復帰の状態遷移")
    fig.tight_layout()
    save(fig, "fig4_9_homing.png")


def fig4_10():
    fig, ax = blank((9.2, 3.2))
    steps = ["条件を設定して\n走行開始", "走行中\n真値を CSV 記録",
             "指標を算出し\nruns.csv へ追記", "自動原点復帰\n（失敗時1回再試行）"]
    w, h, gap = 0.215, 0.40, 0.035
    ys = 0.42
    for i, text in enumerate(steps):
        x = i * (w + gap) + 0.01
        box(ax, x, ys, w, h, text, fontsize=8.8)
        if i:
            arrow(ax, (x - gap + 0.004, ys + h / 2), (x - 0.004, ys + h / 2))
    # ループの戻り（箱の下を回す。箱に隠れないよう ys より下を通す）
    yb = ys - 0.11
    right = 3 * (w + gap) + 0.01 + w / 2
    left = 0.01 + w / 2
    ax.plot([right, right, left], [ys, yb, yb], color=FS.INK, linewidth=1.2,
            zorder=1)
    ax.add_patch(FancyArrowPatch((left, yb), (left, ys), arrowstyle="-|>",
                                 mutation_scale=12, linewidth=1.2,
                                 color=FS.INK, zorder=1))
    ax.text(0.5, yb - 0.055, "次の走行へ（無人で反復）", ha="center", fontsize=9,
            color=FS.INK)
    box(ax, 0.16, 0.02, 0.30, 0.15, "q + Enter で即停止", fontsize=8.6,
        edge=FS.COLOR["l1"], fill="#FFFFFF")
    box(ax, 0.54, 0.02, 0.30, 0.15, "連続2本失敗で停止", fontsize=8.6,
        edge=FS.COLOR["l1"], fill="#FFFFFF")
    ax.text(0.5, 0.955, "実験者は開始時に一括して許可し、以降は監視のみ",
            ha="center", fontsize=9, color=FS.INK_MUTED)
    ax.set_title("図4.10　全自動バッチの1サイクル")
    fig.tight_layout()
    save(fig, "fig4_10_batch.png")


def fig4_6(src="docs/tags/WIN_20260716_09_16_58_Pro.jpg"):
    """俯瞰画像上のタグ検出例。

    src は俯瞰カメラで撮影した実画像。**リポジトリには追跡されていない**ため、
    無い場合はスキップする（他の図の生成は止めない）。
    """
    if not os.path.exists(src):
        print(f"  (skip) fig4_6: {src} が無い")
        return
    import cv2
    from truth_core import detect_tags, make_detector, tag_center

    img = cv2.imread(src)
    det = detect_tags(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), make_detector())
    fig, ax = plt.subplots(figsize=(8.0, 4.6))
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    for tid, c in sorted(det.items()):
        robot = tid == 10
        col = FS.COLOR["l1_ms2"] if robot else FS.COLOR["l2"]
        poly = np.vstack([c, c[:1]])
        ax.plot(poly[:, 0], poly[:, 1], color=col, linewidth=2.2)
        cx, cy = tag_center(c)
        ax.plot(cx, cy, marker="+", markersize=9, color=col,
                markeredgewidth=2)
        # ロボットタグは tag0 と隣接しがちなので左上へ逃がす
        off = (-14, 30) if robot else (12, -16)
        ha = "right" if robot else "left"
        ax.annotate(f"id={tid}" + ("（ロボット）" if robot else ""),
                    (cx, cy), textcoords="offset points", xytext=off,
                    ha=ha, fontsize=9, color=col, fontweight="bold")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)
    ax.set_title("図4.6　俯瞰画像上のタグ検出例\n"
                 "（青＝床基準タグ4枚、緑＝ロボット上面タグ。＋は中心）")
    fig.tight_layout()
    save(fig, "fig4_6_detection.png")


def fig5_8():
    fig, ax = blank((9.6, 3.4))
    cols = [
        ("問題", "L 字コーナーで\nナイーブ L1 が\nチャタリング\n\n反転 18 回\nΣ|u| は L2 の 4.1 倍",
         FS.COLOR["l1"]),
        ("原因", "入力遅延（約 200 ms）\n下での限界振動\n\n遅延2step の sim で\n定量再現",
         None),
        ("対策", "入力変化率への\n二次ペナルティ\n$w_{ms}=2.0$\n\n遅延マージンを買う",
         None),
        ("検証", "実機 n=3 で\n反転 18→1 回\nΣ|u| 19.17→5.02\n\nゼロ率 0.88 を維持",
         FS.COLOR["l1_ms2"]),
    ]
    w, h, gap = 0.213, 0.62, 0.035
    for i, (head, body, edge) in enumerate(cols):
        x = i * (w + gap) + 0.018
        ax.text(x + w / 2, 0.90, head, ha="center", fontsize=11,
                fontweight="bold", color=edge or FS.INK)
        box(ax, x, 0.18, w, h, body, fontsize=8.6, edge=edge,
            fill=FS.FILL_ACCENT if edge else FS.FILL)
        if i:
            arrow(ax, (x - gap + 0.004, 0.49), (x - 0.004, 0.49))
    ax.text(0.5, 0.05,
            "成立条件: (a) 遅延への明示的な配慮　(b) 求解時間が遅延の一部である認識　"
            "(c) 操舵を要する経路での検証",
            ha="center", fontsize=9, color=FS.INK)
    ax.set_title("図5.8　第5章の総括")
    fig.tight_layout()
    save(fig, "fig5_8_summary.png")


def main():
    os.makedirs(OUT, exist_ok=True)
    FS.setup()
    print(f"出力: {OUT}")
    for fn in (fig1_3, fig3_1, fig3_2, fig3_3, fig4_1, fig4_3, fig4_5,
               fig4_6, fig4_7, fig4_8, fig4_9, fig4_10, fig5_1, fig5_8, fig6_1,
               fig7_1, fig7_2):
        fn()
    print("完了")


if __name__ == "__main__":
    main()
