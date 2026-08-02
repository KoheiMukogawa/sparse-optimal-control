# -*- coding: utf-8 -*-
"""遅延ステップ数に対する各制御器の耐性スイープ（S2）。

既存の sim_delay_probe.py は 0/1/2 step × {L2, L1} を print するだけだった。
本モジュールは「対策が何ステップの遅延まで耐えるか」を条件×遅延の表として残す。
遅延1step = 制御周期0.1s に相当し、実機の推定遅延は約200ms＝2step。

λ×move_suppress の設計面（固定遅延下）は sweep_grid.py を参照。

実行: uv run python rover/sweep_delay.py
"""

import argparse
import csv
import os
from collections import namedtuple

from mpc_core import MPCFollower
from sim_delay_probe import DT, V_MAX, W_MAX, simulate

Condition = namedtuple("Condition", "name reg lam move_suppress")

# 既定の比較条件。L1 は「実機で検証済みの λ=0.3 系」と
# 「S1 で sim 最良だった λ=2 系」の両方を見る。
DEFAULT_CONDITIONS = [
    Condition("l2", "l2", 1.0, 0.0),
    Condition("l1_lam0.3", "l1", 0.3, 0.0),
    Condition("l1_lam0.3_ms0.5", "l1", 0.3, 0.5),
    Condition("l1_lam0.3_ms2", "l1", 0.3, 2.0),
    Condition("l1_lam2", "l1", 2.0, 0.0),
    Condition("l1_lam2_ms0.5", "l1", 2.0, 0.5),
]

CSV_FIELDS = ["name", "reg", "lam", "move_suppress", "delay_steps", "ok",
              "t", "rmse", "sum_u", "flips", "maxw", "sat", "zero"]


def sweep_delay(conditions, delay_steps_list, horizon=15, seed=1):
    """各条件を各遅延で走らせ、1組1行で返す（条件が外側・遅延が内側）。"""
    rows = []
    for c in conditions:
        for d in delay_steps_list:
            mpc = MPCFollower(N=horizon, ts=DT, reg=c.reg, lam=c.lam,
                              v_max=V_MAX, w_max=W_MAX,
                              move_suppress=c.move_suppress)
            metrics = simulate(mpc, delay_steps=d, seed=seed)
            rows.append(dict(name=c.name, reg=c.reg, lam=c.lam,
                             move_suppress=c.move_suppress, delay_steps=d,
                             **metrics))
    return rows


def write_outputs(rows, outdir):
    """delay.csv と table.md を書く。"""
    os.makedirs(outdir, exist_ok=True)

    with open(os.path.join(outdir, "delay.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_FIELDS)
        for r in rows:
            w.writerow([r["name"], r["reg"], f"{r['lam']:g}",
                        f"{r['move_suppress']:g}", r["delay_steps"], r["ok"],
                        f"{r['t']:.1f}", f"{r['rmse']:.3f}",
                        f"{r['sum_u']:.3f}", r["flips"], f"{r['maxw']:.2f}",
                        f"{r['sat']:.4f}", f"{r['zero']:.4f}"])

    lines = ["# 遅延耐性スイープ（L字経路）", "",
             "遅延1step = 制御周期0.1s。実機の推定遅延は約200ms＝**遅延2step**。", "",
             "| 条件 | 遅延step | 到達 | RMSE | Σ\\|u\\| | ω反転 | 飽和率 | ω0率 |",
             "|------|----------|------|------|--------|-------|--------|------|"]
    for r in rows:
        lines.append(
            f"| {r['name']} | {r['delay_steps']} | "
            f"{'✓' if r['ok'] else '✗'} | {r['rmse']:.2f}cm | "
            f"{r['sum_u']:.2f} | {r['flips']} | {r['sat']*100:.0f}% | "
            f"{r['zero']*100:.0f}% |")
    with open(os.path.join(outdir, "table.md"), "w") as f:
        f.write("\n".join(lines) + "\n")

    _plot(rows, os.path.join(outdir, "delay_sweep.png"))


def _plot(rows, png_path):
    """遅延軸に対する4指標を条件ごとに描く（論文 図5.4 の元）。"""
    try:
        import fig_style as FS
        from fig_style import plt
    except ImportError:
        return
    FS.setup()

    panels = [("flips", "ω 符号反転 [回]", 1.0),
              ("sum_u", "入力積算 Σ|u|", 1.0),
              ("rmse", "横偏差 RMSE [cm]", 1.0),
              ("zero", "ω ゼロ率 [%]", 100.0)]
    names = list(dict.fromkeys(r["name"] for r in rows))
    fig, axes = plt.subplots(2, 2, figsize=(9, 6.4))
    for ax, (key, label, scale) in zip(axes.ravel(), panels):
        for name in names:
            series = sorted((r for r in rows if r["name"] == name),
                            key=lambda r: r["delay_steps"])
            ax.plot([r["delay_steps"] for r in series],
                    [r[key] * scale for r in series],
                    color=FS.DELAY_COLOR.get(name),
                    linestyle=FS.DELAY_LINESTYLE.get(name, "-"),
                    marker=FS.DELAY_MARKER.get(name, "o"),
                    markersize=5, linewidth=2,
                    label=FS.DELAY_LABEL.get(name, name))
        ax.set_xlabel("入力遅延 [ステップ（1 step = 0.1 s）]")
        ax.set_ylabel(label)
        ax.set_xticks(sorted({r["delay_steps"] for r in rows}))
    # 実機の推定遅延（約200ms=2step）を全パネルに引く
    for ax in axes.ravel():
        ax.axvline(2, color=FS.INK_MUTED, linestyle=":", linewidth=1.2,
                   zorder=0)
    # 注記は最も空いているパネル（RMSE・遅延2step付近は低い値のみ）に置く
    axes[1][0].annotate("実機の推定遅延", xy=(2, 0.97),
                        xycoords=("data", "axes fraction"),
                        ha="center", va="top", fontsize=8, color=FS.INK_MUTED)
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, fontsize=8,
               frameon=False, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle("図5.4　入力遅延に対する各条件の耐性（L字経路・sim）")
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(png_path)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="遅延耐性スイープ（L字経路）")
    ap.add_argument("--delays", default="0,1,2,3,4")
    ap.add_argument("--horizon", type=int, default=15)
    ap.add_argument("--outdir", default="results/2026-07-26_delay_sweep")
    args = ap.parse_args()

    delays = [int(x) for x in args.delays.split(",")]
    print(f"== 遅延耐性スイープ（{len(DEFAULT_CONDITIONS)}条件 × "
          f"{len(delays)}遅延 = {len(DEFAULT_CONDITIONS)*len(delays)}）==")
    rows = []
    for c in DEFAULT_CONDITIONS:
        for d in delays:
            r = sweep_delay([c], [d], horizon=args.horizon)[0]
            rows.append(r)
            print(f"  {c.name:<18} 遅延{d} {'到達' if r['ok'] else '未達'} "
                  f"RMSE={r['rmse']:5.2f}cm Σ|u|={r['sum_u']:6.2f} "
                  f"反転{r['flips']:>3} 飽和{r['sat']*100:3.0f}% "
                  f"ω0={r['zero']*100:3.0f}%")

    write_outputs(rows, args.outdir)
    print(f"\n出力: {args.outdir}/{{delay.csv, table.md, delay_sweep.png}}")


if __name__ == "__main__":
    main()
