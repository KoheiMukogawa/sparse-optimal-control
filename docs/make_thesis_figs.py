# -*- coding: utf-8 -*-
"""卒論 第5・6章の図を既存の実機データから作る（S9）。

実機は故障中だが、2026-07-15 の L字バッチ（4条件×3本＝12走行の bag と
手計測の終点）は取得済みなので、第6章の図の大半は実機なしで作れる。

作る図:
  図5.6  対策前後の ω(t)（l1 vs l1_ms2）
  図5.7  sim と実機の対応（±10% 帯つき散布図・相対差）
  図6.2  条件別の RMSE と Σ|u|（誤差棒つき）
  図6.3  4条件の ω(t) 並置
  図6.4  求解時間の分布（箱ひげ）
  図6.5  真値終点の分布（散布図・向きを矢印表示）
  図6.6  odom RMSE と真値終点誤差の乖離
  図6.8  総合レーダーチャート

実行: uv run python docs/make_thesis_figs.py
出力: results/2026-07-26_thesis_figs/
"""

import argparse
import math
import os
import sys

ROVER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rover")
sys.path.insert(0, os.path.abspath(ROVER))

import numpy as np

import fig_style as FS
from fig_style import plt
from plot_lturn import V_ACTIVE, W_DEAD, load
from thesis_data import (COMPARE_METRICS, CONDITIONS, RADAR_AXES,
                         aggregate_runs, external_by_condition, load_external,
                         load_runs, load_solve_ms, real_vs_sim)
from thesis_data import METRIC_LABEL as TD_METRIC_LABEL

REAL = "results/2026-07-15_Lturn_3way_real"
SIM = "results/2026-07-15_Lturn_3way_sim"


# ---------------------------------------------------------------- データ補助

def omega_series(bagdir):
    """走行区間の (t, ω)。開始を t=0 に揃える。"""
    _, tw = load(bagdir)
    active = [(t, v, w) for t, v, w in tw if abs(v) > V_ACTIVE] or tw
    t0 = active[0][0]
    return [t - t0 for t, _, _ in active], [w for _, _, w in active]


def representative(agg, cond, rep=1):
    return next(r for r in agg[cond]["runs"] if r["rep"] == rep)


def _save(fig, outdir, name):
    path = os.path.join(outdir, name)
    fig.savefig(path)
    plt.close(fig)
    print(f"  {name}")
    return path


# ------------------------------------------------------------------ 各図

def fig5_6(agg, outdir):
    """図5.6 対策前後の ω(t)。素の L1 のチャタが移動抑制で消えることを示す。"""
    fig, axes = plt.subplots(2, 1, figsize=(7.2, 5.0), sharex=True, sharey=True)
    for ax, cond in zip(axes, ["l1", "l1_ms2"]):
        r = representative(agg, cond)
        ts, ws = omega_series(r["bagdir"])
        ax.axhline(0, color="0.6", lw=0.8, zorder=0)
        for s in (2.0, -2.0):
            ax.axhline(s, color="0.3", ls=":", lw=0.8, zorder=0)
        ax.plot(ts, ws, color=FS.COLOR[cond], lw=1.2, zorder=3)
        ax.set_ylabel("ω [rad/s]")
        ax.set_title(
            f"{FS.LABEL[cond]}  —  符号反転 {int(r['flips'])} 回, "
            f"Σ|u| {r['sum_u']:.2f}, 飽和 {r['sat_ratio']*100:.0f}%",
            loc="left")
    axes[-1].set_xlabel("走行開始からの時間 [s]")
    fig.suptitle("図5.6  移動抑制の導入によるチャタリングの消失（実機・各代表1本）")
    fig.tight_layout()
    return _save(fig, outdir, "fig5_6_omega_before_after.png")


METRIC_MARKER = {"rmse_cm": "o", "sum_u": "s", "w_zero_ratio": "^",
                 "flips": "D"}


def fig5_7(pairs, outdir):
    """図5.7 sim が実機をどこまで当てたか。当たった指標と外れた指標を分けて示す。"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.4, 4.6))

    # 左: 対数散布図（±10% 帯つき）。反転0対0は対数軸に載らないので除く。
    plotted = [p for p in pairs if p["sim"] > 0 and p["real"] > 0]
    lo = min(min(p["sim"], p["real"]) for p in plotted) * 0.6
    hi = max(max(p["sim"], p["real"]) for p in plotted) * 1.6
    line = np.logspace(math.log10(lo), math.log10(hi), 50)
    ax1.fill_between(line, line * 0.9, line * 1.1, color="0.75", alpha=0.45,
                     label="±10% 帯", zorder=0)
    ax1.plot(line, line, "k--", lw=0.9, label="完全一致", zorder=1)

    seen_m, seen_c = set(), set()
    for p in plotted:
        ax1.plot(p["sim"], p["real"], METRIC_MARKER[p["metric"]], ms=8,
                 color=FS.COLOR[p["cond"]], markeredgecolor="0.2",
                 markeredgewidth=0.6, zorder=3)
        seen_m.add(p["metric"])
        seen_c.add(p["cond"])
    ax1.set_xscale("log")
    ax1.set_yscale("log")
    ax1.set_xlim(lo, hi)
    ax1.set_ylim(lo, hi)
    ax1.set_xlabel("シミュレーション（遅延2step）")
    ax1.set_ylabel("実機（n=3 平均）")
    ax1.set_title("sim と実機の対応（両対数）", fontsize=10)
    handles = [plt.Line2D([], [], ls="", marker=METRIC_MARKER[m], color="0.35",
                          ms=7, label=TD_METRIC_LABEL[m])
               for m in COMPARE_METRICS if m in seen_m]
    handles += [plt.Line2D([], [], ls="", marker="o", color=FS.COLOR[c], ms=7,
                           label=FS.SHORT[c]) for c in CONDITIONS if c in seen_c]
    handles += [plt.Line2D([], [], ls="--", color="k", lw=0.9, label="完全一致"),
                plt.Rectangle((0, 0), 1, 1, color="0.75", alpha=0.45,
                              label="±10% 帯")]
    ax1.legend(handles=handles, fontsize=7, loc="upper left", ncol=2)

    # 右: 相対差。RMSE だけが帯から外れることを見せる。
    xs, labels = [], []
    for i, m in enumerate(COMPARE_METRICS):
        for p in [q for q in pairs if q["metric"] == m]:
            if p["rel_diff"] is None:
                continue
            ax2.plot(i, p["rel_diff"] * 100, METRIC_MARKER[m], ms=9,
                     color=FS.COLOR[p["cond"]], markeredgecolor="0.2",
                     markeredgewidth=0.6, zorder=3)
        xs.append(i)
        labels.append(TD_METRIC_LABEL[m])
    ax2.axhspan(-10, 10, color="0.75", alpha=0.45, zorder=0, label="±10% 帯")
    ax2.axhline(0, color="k", ls="--", lw=0.9, zorder=1)
    ax2.set_xticks(xs)
    ax2.set_xticklabels(labels, fontsize=8)
    ax2.set_ylabel("相対差 (実機 − sim) / sim [%]")
    ax2.set_title("指標ごとの相対差", fontsize=10)
    ax2.legend(fontsize=8, loc="upper right")

    fig.suptitle("図5.7  シミュレーションの予測性（2026-07-15 L字バッチ）")
    fig.text(0.01, -0.03,
             "操舵量・ω0率・反転は ±7% 以内で一致するが、追従 RMSE は実機が "
             "12〜21% 悪い側にずれる（絶対差は全条件 0.4cm 以内）。"
             "反転が 0 対 0 の Kanayama・L2 は左図（対数軸）から除いている。",
             fontsize=8, va="top")
    fig.tight_layout()
    return _save(fig, outdir, "fig5_7_real_vs_sim.png")


def fig6_2(agg, outdir):
    """図6.2 追従精度と操舵量。提案手法が L2 に勝っていないことを隠さず示す。"""
    fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.8))
    for ax, key, ylabel, title in [
            (axes[0], "rmse_cm", "追従 RMSE [cm]", "追従精度（odom基準・小さいほど良い）"),
            (axes[1], "sum_u", "Σ|u|", "操舵量（消費エネルギー代理・小さいほど良い）")]:
        xs = range(len(CONDITIONS))
        means = [agg[c][key]["mean"] for c in CONDITIONS]
        stds = [agg[c][key]["std"] for c in CONDITIONS]
        ax.bar(xs, means, yerr=stds, capsize=4,
               color=[FS.COLOR[c] for c in CONDITIONS], edgecolor="0.25", lw=0.6)
        for x, m, s in zip(xs, means, stds):
            ax.text(x, m + s, f"{m:.2f}", ha="center", va="bottom", fontsize=8)
        ax.set_xticks(list(xs))
        ax.set_xticklabels([FS.SHORT[c] for c in CONDITIONS])
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=10)
        ax.margins(y=0.18)
    fig.suptitle("図6.2  実機 n=3 の平均±標準偏差")
    fig.tight_layout()
    return _save(fig, outdir, "fig6_2_rmse_sumu.png")


def fig6_3(agg, outdir):
    """図6.3 4条件の ω(t) を同一スケールで並置する。"""
    fig, axes = plt.subplots(len(CONDITIONS), 1, figsize=(7.2, 8.0),
                             sharex=True, sharey=True)
    for ax, cond in zip(axes, CONDITIONS):
        r = representative(agg, cond)
        ts, ws = omega_series(r["bagdir"])
        ax.axhspan(-W_DEAD, W_DEAD, color="0.85", zorder=0)
        ax.axhline(0, color="0.6", lw=0.8, zorder=1)
        ax.plot(ts, ws, color=FS.COLOR[cond], lw=1.2, zorder=3)
        ax.set_ylabel("ω [rad/s]")
        ax.set_title(
            f"{FS.LABEL[cond]}  —  ω0率 {r['w_zero_ratio']*100:.0f}%, "
            f"反転 {int(r['flips'])} 回", loc="left")
    axes[-1].set_xlabel("走行開始からの時間 [s]")
    fig.suptitle("図6.3  操舵入力 ω(t) の比較（灰帯＝ゼロ操舵とみなす |ω|<0.05）")
    fig.tight_layout()
    return _save(fig, outdir, "fig6_3_omega_all.png")


def fig6_4(agg, outdir):
    """図6.4 求解時間の分布。初回コールド求解は発進前の1回なので除外する。"""
    conds = [c for c in CONDITIONS if agg[c]["solve_p95"]["mean"] is not None]
    data, dropped, over = [], 0, 0
    for c in conds:
        pooled = []
        for r in agg[c]["runs"]:
            s = load_solve_ms(r["bagdir"])
            dropped += 1
            pooled += s[1:]          # 先頭＝cvxpy 初回コールド求解
        over += sum(1 for v in pooled if v > FS.CONTROL_PERIOD_MS)
        data.append(pooled)

    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    bp = ax.boxplot(data, tick_labels=[FS.SHORT[c] for c in conds],
                    patch_artist=True, widths=0.55,
                    flierprops=dict(marker=".", markersize=3, alpha=0.5))
    for patch, c in zip(bp["boxes"], conds):
        patch.set_facecolor(FS.COLOR[c])
        patch.set_alpha(0.65)
    for med in bp["medians"]:
        med.set_color("0.15")
    ax.axhline(FS.CONTROL_PERIOD_MS, color="crimson", ls="--", lw=1.0,
               label=f"制御周期 {FS.CONTROL_PERIOD_MS:.0f} ms")
    ax.set_ylabel("求解時間 [ms]")
    ax.set_title("図6.4  MPC 求解時間の分布（3本ぶんをまとめた・実機 RPi4）")
    ax.legend()
    fig.text(0.01, -0.02,
             f"初回コールド求解 {dropped} 点を除外。除外後に制御周期を超えた点: {over} 件。"
             "Kanayama は解析的フィードバックのため求解時間を持たない。",
             fontsize=8, va="top")
    fig.tight_layout()
    print(f"    （コールド除外 {dropped} 点 / 周期超過 {over} 件）")
    return _save(fig, outdir, "fig6_4_solve_time.png")


def fig6_5(ext, outdir):
    """図6.5 真値終点の散布図。矢印は最終姿勢（経路終端方向に対する向き）。"""
    fig, ax = plt.subplots(figsize=(6.2, 5.4))
    ax.axhline(0, color="0.6", lw=0.8)
    ax.axvline(0, color="0.6", lw=0.8)
    ax.plot(0, 0, "k*", ms=14, label="ゴール真値", zorder=5)

    seen = set()
    for e in ext:
        c = e["cond"]
        # θ は y 軸（経路終端方向）に対する向き。左+ なので反時計回り。
        th = math.radians(e["theta_deg"])
        dx, dy = -math.sin(th) * 6.0, math.cos(th) * 6.0
        ax.arrow(e["x_cm"], e["y_cm"], dx, dy, width=0.25, head_width=1.4,
                 color=FS.COLOR[c], alpha=0.55, length_includes_head=True,
                 zorder=2)
        ax.plot(e["x_cm"], e["y_cm"], "o", ms=7, color=FS.COLOR[c],
                markeredgecolor="0.25", markeredgewidth=0.6, zorder=3,
                label=FS.LABEL[c] if c not in seen else None)
        seen.add(c)
        ax.annotate(f"{e['rep']}", (e["x_cm"], e["y_cm"]),
                    textcoords="offset points", xytext=(6, -9), fontsize=7)

    ax.set_xlabel("x [cm]　第1直進の進行方向（コーナー外側が +）")
    ax.set_ylabel("y [cm]　旋回後の進行方向（行き過ぎが +）")
    ax.set_title("図6.5  真値終点の分布（手計測 n=3・矢印は最終姿勢）")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="upper left", fontsize=8)
    fig.text(0.01, -0.02,
             "全9走行が x>0（コーナー外側へ流れる系統誤差）。"
             "Kanayama は本バッチでは外部計測していない。", fontsize=8, va="top")
    fig.tight_layout()
    return _save(fig, outdir, "fig6_5_endpoint_scatter.png")


def fig6_6(agg, ext, outdir):
    """図6.6 odom RMSE と真値終点誤差の乖離。odom 基準は追従性能を過大評価する。"""
    extc = external_by_condition(ext)
    conds = [c for c in CONDITIONS if c in extc]

    fig, ax = plt.subplots(figsize=(6.4, 4.6))
    for c in conds:
        runs = sorted(agg[c]["runs"], key=lambda r: r["rep"])
        pts = sorted(extc[c]["points"], key=lambda e: e["rep"])
        xs = [r["rmse_cm"] for r in runs]
        ys = [e["goal_dist_cm"] for e in pts]
        ax.plot(xs, ys, "o", ms=8, color=FS.COLOR[c], markeredgecolor="0.25",
                markeredgewidth=0.6, label=FS.LABEL[c])
        ax.plot(agg[c]["rmse_cm"]["mean"], extc[c]["goal_dist_cm"]["mean"],
                "X", ms=13, color=FS.COLOR[c], markeredgecolor="k",
                markeredgewidth=0.8, zorder=4)

    lim = max(ax.get_xlim()[1], 5.0)
    xs = np.linspace(0, lim, 50)
    ax.plot(xs, xs, "k--", lw=0.9, label="1:1（odom と真値が一致）")
    ax.plot(xs, 5 * xs, color="0.5", ls=":", lw=0.9, label="5倍・10倍")
    ax.plot(xs, 10 * xs, color="0.5", ls=":", lw=0.9)
    ax.set_xlim(0, lim)
    ax.set_xlabel("odom 基準 追従 RMSE [cm]")
    ax.set_ylabel("真値によるゴール距離 [cm]")
    ax.set_title("図6.6  odom は滑りを見ていない（× は条件平均）")
    ax.legend(fontsize=8, loc="upper left")
    fig.tight_layout()
    return _save(fig, outdir, "fig6_6_odom_vs_truth.png")


def fig6_8(agg, ext, outdir):
    """図6.8 総合レーダー。3条件内の相対評価であることを明記する。"""
    from thesis_data import radar_axes

    axes_vals = radar_axes(agg, ext)
    names = [n for n, _, _ in RADAR_AXES]
    conds = [c for c in CONDITIONS if c in axes_vals]
    k = len(names)
    angles = [i / k * 2 * math.pi for i in range(k)] + [0.0]

    fig, ax = plt.subplots(figsize=(5.8, 5.6),
                           subplot_kw=dict(projection="polar"))
    for c in conds:
        vals = list(axes_vals[c]) + [axes_vals[c][0]]
        ax.plot(angles, vals, "o-", ms=4, lw=1.6, color=FS.COLOR[c],
                label=FS.LABEL[c])
        ax.fill(angles, vals, color=FS.COLOR[c], alpha=0.12)
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(names)
    ax.set_ylim(0, 1.05)
    ax.set_yticks([0.25, 0.5, 0.75, 1.0])
    ax.set_yticklabels(["0.25", "0.5", "0.75", "1.0"], fontsize=8)
    ax.set_title("図6.8  総合比較（5軸・外側ほど良い）", pad=18)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.22), fontsize=8)
    fig.text(0.01, 0.0,
             "各軸は最良条件に対する比（最良=1.0、0.5 なら最良の2倍悪い）。"
             "Kanayama は真値計測と求解時間を持たないため除外。", fontsize=8, va="top")
    fig.tight_layout()
    return _save(fig, outdir, "fig6_8_radar.png")


# ------------------------------------------------------------------ main

def main():
    ap = argparse.ArgumentParser(description="卒論 第5・6章の図を作る")
    ap.add_argument("--batch", default=REAL)
    ap.add_argument("--sim-batch", default=SIM)
    ap.add_argument("--outdir", default="results/2026-07-26_thesis_figs")
    args = ap.parse_args()

    FS.setup()
    os.makedirs(args.outdir, exist_ok=True)

    runs = load_runs(os.path.join(args.batch, "runs.csv"))
    ext = load_external(os.path.join(args.batch, "external.csv"))
    agg = aggregate_runs(runs)

    sim_agg = aggregate_runs(load_runs(os.path.join(args.sim_batch, "runs.csv")))
    pairs = real_vs_sim(agg, sim_agg)

    print(f"入力: {args.batch}（{len(runs)}走行・外部計測{len(ext)}点）")
    print(f"      {args.sim_batch}（sim 基準）")
    print(f"出力: {args.outdir}")
    fig5_6(agg, args.outdir)
    fig5_7(pairs, args.outdir)
    fig6_2(agg, args.outdir)
    fig6_3(agg, args.outdir)
    fig6_4(agg, args.outdir)
    fig6_5(ext, args.outdir)
    fig6_6(agg, ext, args.outdir)
    fig6_8(agg, ext, args.outdir)
    print("完了")


if __name__ == "__main__":
    main()
