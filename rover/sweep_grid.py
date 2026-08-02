# -*- coding: utf-8 -*-
"""λ × move_suppress の2次元スイープ（入力遅延下）。

既存の sweep_lambda.py は**遅延なし**条件の λ 1次元スイープであり、
本研究の主軸「実機遅延下でスパース制御を成立させる条件」とは前提が食い違う。
本モジュールは遅延を入れた条件で λ と move_suppress の設計面を取る。

遅延モデルは sim_delay_probe.simulate をそのまま使う（実機と定量一致した実績:
遅延2step で Σ|u| 18.49・反転19 vs 実機 20.71・15）。モデルを増やさないこと。
"""

import argparse
import csv
import os

from mpc_core import MPCFollower
from sim_delay_probe import DT, V_MAX, W_MAX, simulate


def sweep_grid(lams, move_suppresses, delay_steps=2, horizon=15, seed=1):
    """λ×move_suppress の全組み合わせを遅延下で走らせ、1組1行で返す。

    行は lam / move_suppress に simulate() の指標
    （ok, t, rmse, sum_u, flips, maxw, sat, zero）を足したもの。
    """
    rows = []
    for lam in lams:
        for ms in move_suppresses:
            mpc = MPCFollower(N=horizon, ts=DT, reg="l1", lam=lam,
                              v_max=V_MAX, w_max=W_MAX, move_suppress=ms)
            metrics = simulate(mpc, delay_steps=delay_steps, seed=seed)
            rows.append(dict(lam=lam, move_suppress=ms, **metrics))
    return rows


CSV_FIELDS = ["lam", "move_suppress", "ok", "t", "rmse", "sum_u",
              "flips", "maxw", "sat", "zero"]


def write_outputs(rows, outdir, delay_steps):
    """grid.csv と table.md を outdir に書く。遅延条件は両方に明記する。"""
    os.makedirs(outdir, exist_ok=True)

    with open(os.path.join(outdir, "grid.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_FIELDS)
        for r in rows:
            w.writerow([f"{r['lam']:g}", f"{r['move_suppress']:g}", r["ok"],
                        f"{r['t']:.1f}", f"{r['rmse']:.3f}", f"{r['sum_u']:.3f}",
                        r["flips"], f"{r['maxw']:.2f}", f"{r['sat']:.4f}",
                        f"{r['zero']:.4f}"])

    lines = [
        f"# λ × move_suppress グリッド（遅延{delay_steps}step・L字経路）",
        "",
        f"入力遅延を**遅延{delay_steps}step**（実機の約200ms相当）注入した条件での2次元スイープ。",
        "遅延なしの λ 1次元スイープは `results/2026-06-13_lambda_sweep/` を参照（前提が違う）。",
        "",
        "| λ | w_ms | 到達 | RMSE | Σ\\|u\\| | ω反転 | 飽和率 | ω0率 |",
        "|---|------|------|------|--------|-------|--------|------|",
    ]
    for r in rows:
        lines.append(
            f"| {r['lam']:g} | {r['move_suppress']:g} | "
            f"{'✓' if r['ok'] else '✗'} | {r['rmse']:.2f}cm | {r['sum_u']:.2f} | "
            f"{r['flips']} | {r['sat']*100:.0f}% | {r['zero']*100:.0f}% |")
    with open(os.path.join(outdir, "table.md"), "w") as f:
        f.write("\n".join(lines) + "\n")

    _plot_ms(rows, os.path.join(outdir, "sweep_ms.png"), delay_steps)


def _plot_ms(rows, png_path, delay_steps):
    """w_ms 軸に対する4指標の変化を λ 系列ごとに描く（論文 図5.5 の元）。"""
    try:
        import fig_style as FS
        from fig_style import plt
    except ImportError:
        return
    FS.setup()

    panels = [("flips", "ω 符号反転 [回]", 1.0),
              ("sum_u", "入力積算 Σ|u|", 1.0),
              ("zero", "ω ゼロ率 [%]", 100.0),
              ("rmse", "横偏差 RMSE [cm]", 1.0)]
    lams = sorted({r["lam"] for r in rows})
    # λ は順序量なので単一色相の濃淡で表す（カテゴリ色を割り当てない）
    ramp = FS.lam_ramp(lams)
    fig, axes = plt.subplots(2, 2, figsize=(9, 6.4))
    for ax, (key, label, scale) in zip(axes.ravel(), panels):
        for lam in lams:
            series = sorted((r for r in rows if r["lam"] == lam),
                            key=lambda r: r["move_suppress"])
            ax.plot([r["move_suppress"] for r in series],
                    [r[key] * scale for r in series],
                    color=ramp[lam], marker="o", markersize=5, linewidth=2,
                    label=f"λ={lam:g}")
        ax.set_xlabel("移動抑制重み $w_{ms}$")
        ax.set_ylabel(label)
        ax.set_xticks(sorted({r["move_suppress"] for r in rows}))
    # 採用値 w_ms=2.0 を示す
    for ax in axes.ravel():
        ax.axvline(2.0, color=FS.INK_MUTED, linestyle=":", linewidth=1.2,
                   zorder=0)
    axes[0][0].annotate("採用値", xy=(2.0, 0.97), xycoords=("data", "axes fraction"),
                        ha="center", va="top", fontsize=8, color=FS.INK_MUTED)
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=5, fontsize=9,
               frameon=False, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle(f"図5.5　λ×移動抑制のグリッド（入力遅延 {delay_steps} ステップ下・sim）")
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(png_path)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(
        description="λ × move_suppress の2次元スイープ（入力遅延下・L字経路）")
    ap.add_argument("--lams", default="0.25,0.3,0.5,1,2")
    ap.add_argument("--move-suppress", dest="mss", default="0,0.5,1,2,5")
    ap.add_argument("--delay-steps", type=int, default=2,
                    help="入力遅延ステップ数（2 = 実機の約200ms相当）")
    ap.add_argument("--horizon", type=int, default=15)
    ap.add_argument("--outdir", default="results/2026-07-26_lambda_ms_grid")
    args = ap.parse_args()

    lams = [float(x) for x in args.lams.split(",")]
    mss = [float(x) for x in args.mss.split(",")]
    print(f"== λ×w_ms グリッド（遅延{args.delay_steps}step, N={args.horizon}, "
          f"{len(lams)}×{len(mss)}={len(lams)*len(mss)}条件）==")

    rows = []
    for lam in lams:
        for ms in mss:
            r = sweep_grid([lam], [ms], delay_steps=args.delay_steps,
                           horizon=args.horizon)[0]
            rows.append(r)
            print(f"  λ={lam:<5g} w_ms={ms:<4g} {'到達' if r['ok'] else '未達'} "
                  f"RMSE={r['rmse']:5.2f}cm Σ|u|={r['sum_u']:6.2f} "
                  f"反転{r['flips']:>3} 飽和{r['sat']*100:3.0f}% "
                  f"ω0={r['zero']*100:3.0f}%")

    write_outputs(rows, args.outdir, args.delay_steps)
    print(f"\n出力: {args.outdir}/{{grid.csv, table.md, sweep_ms.png}}")


if __name__ == "__main__":
    main()
