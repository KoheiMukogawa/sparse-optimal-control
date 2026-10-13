# -*- coding: utf-8 -*-
"""研究会の卒論概要（docs/abstract/gaiyou.tex）用の図を作る。

卒論本文の図をそのまま流用すると、画像の中に「図5.6 …」という**本文の図番号つき
タイトルが焼き込まれている**ため、LaTeX 側のキャプション（Fig. 1: …）と二重になり、
しかも概要では存在しない図番号を名乗ることになる。
そこで概要用に **suptitle を外した版**を生成する。

`docs/make_thesis_figs.py` は卒論本文が依存しているので変更しない。
各軸のタイトル（符号反転・Σ|u|・飽和率）は情報量があるので残す。

実行: uv run python docs/abstract/make_abstract_figs.py
出力: docs/abstract/fig/
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "rover"))
sys.path.insert(0, os.path.join(ROOT, "docs"))

import fig_style as FS
from fig_style import plt
from thesis_data import aggregate_runs, load_runs

from make_thesis_figs import REAL, omega_series, representative

OUT = os.path.join(HERE, "fig")


def omega_before_after(agg, outdir):
    """対策前後の ω(t)。make_thesis_figs.fig5_6 から suptitle を除いたもの。

    **段幅に合わせた実寸で作る。** 本文図は figsize=(7.2, 5.0) だが、これを
    2段組みの1カラム（約73mm≒2.9in）へ縮めると図内の文字が1/2.5になって読めない。
    最終表示寸法に近い figsize で作り、フォントを相対的に大きくしてある。
    """
    fig, axes = plt.subplots(2, 1, figsize=(3.6, 2.9), sharex=True, sharey=True)
    for ax, cond in zip(axes, ["l1", "l1_ms2"]):
        r = representative(agg, cond)
        ts, ws = omega_series(r["bagdir"])
        ax.axhline(0, color="0.6", lw=0.8, zorder=0)
        for s in (2.0, -2.0):
            ax.axhline(s, color="0.3", ls=":", lw=0.8, zorder=0)
        ax.plot(ts, ws, color=FS.COLOR[cond], lw=1.0, zorder=3)
        ax.set_ylabel("ω [rad/s]", fontsize=8)
        # 段幅が狭いので2行に分ける（1行だと図幅からはみ出す）
        ax.set_title(
            f"{FS.LABEL[cond]}\n"
            f"符号反転 {int(r['flips'])} 回, Σ|u| {r['sum_u']:.2f}, "
            f"飽和 {r['sat_ratio']*100:.0f}%",
            loc="left", fontsize=8, linespacing=1.3)
        ax.tick_params(labelsize=7)
    axes[-1].set_xlabel("走行開始からの時間 [s]", fontsize=8)
    fig.tight_layout()
    path = os.path.join(outdir, "omega_before_after.png")
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def main():
    FS.setup()
    os.makedirs(OUT, exist_ok=True)
    agg = aggregate_runs(load_runs(os.path.join(ROOT, REAL, "runs.csv")))
    print("出力:", omega_before_after(agg, OUT))


if __name__ == "__main__":
    main()
