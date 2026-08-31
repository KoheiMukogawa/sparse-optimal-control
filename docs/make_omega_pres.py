# -*- coding: utf-8 -*-
"""発表用の ω(t) 波形図と、その元になる実測時系列 CSV を作る。

既存の卒論図（図5.6・図6.3）は MPC が出した**指令 ω**（/rover_twist・10Hz）だけを
描いている。発表では「指令はこう打った／機体は実際にこう回った」を1枚で見せたいので、
車輪オドメトリの**実測 ω**（/odom の twist.angular.z・20Hz）を重ねる。

入力: results/2026-07-15_Lturn_3way_real/（実機 4条件×3本の bag と runs.csv）
出力: results/2026-08-09_omega_pres/
  omega_timeseries.csv  12走行×2系統（cmd/odom）の生の時系列
  omega_stats.csv       走行ごとの ω 統計（指令ベース・実測ベースを並記）
  fig_omega_pres.png/.pdf  発表用の対策前後2段図

実行: uv run python docs/make_omega_pres.py
"""

import argparse
import csv
import math
import os
import sys

ROVER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rover")
sys.path.insert(0, os.path.abspath(ROVER))

from pathlib import Path

from rosbags.highlevel import AnyReader

import fig_style as FS
from fig_style import plt
from plot_lturn import TYPESTORE, V_ACTIVE, W_DEAD
from thesis_data import load_runs

REAL = "results/2026-07-15_Lturn_3way_real"
OUTDIR = "results/2026-08-09_omega_pres"
W_CLAMP = 2.0   # rad/s: 指令のクランプ値（mpc_core の入力上限）
W_SAT = 1.5     # rad/s: 飽和とみなす閾値（既刊の診断値と揃える）


def load_twists(bagdir):
    """bag から指令 twist と odom twist を (t, v, ω) で読む。

    plot_lturn.load は /odom の pose しか取らないので、実測 ω を使うここでは
    twist フィールドを読む専用のローダを持つ。
    """
    cmd, odom = [], []
    with AnyReader([Path(bagdir)], default_typestore=TYPESTORE) as r:
        for c, ts, raw in r.messages():
            if c.topic not in ("/rover_twist", "/odom"):
                continue
            m = r.deserialize(raw, c.msgtype)
            t = ts * 1e-9
            if c.topic == "/rover_twist":
                cmd.append((t, m.linear.x, m.angular.z))
            else:
                tw = m.twist.twist
                odom.append((t, tw.linear.x, tw.angular.z))
    return cmd, odom


def series(bagdir):
    """走行区間の指令 ω と実測 ω を、走行開始 t=0 に揃えて返す。

    t0 は既刊の図と同じ定義（/rover_twist で |v|>V_ACTIVE になった最初の点）を使う。
    実測側も同じ t0 を引き、走行終了時刻までに切ることで両者を同一時間軸に載せる。
    """
    tw, odom = load_twists(bagdir)
    active = [(t, v, w) for t, v, w in tw if abs(v) > V_ACTIVE] or tw
    t0, t1 = active[0][0], active[-1][0]
    cmd = [(t - t0, v, w) for t, v, w in active]
    meas = [(t - t0, v, w) for t, v, w in odom if t0 <= t <= t1]
    return cmd, meas


def stats(ws):
    """ω 列のチャタリング診断。plot_lturn.diag と同じ定義。"""
    flips, prev = 0, 0
    for w in ws:
        sgn = 1 if w > W_DEAD else (-1 if w < -W_DEAD else 0)
        if sgn != 0:
            if prev != 0 and sgn != prev:
                flips += 1
            prev = sgn
    n = len(ws)
    return {
        "n": n,
        "flips": flips,
        "max_abs_w": max(abs(w) for w in ws),
        "zero_ratio": sum(1 for w in ws if abs(w) < W_DEAD) / n,
        "sat_ratio": sum(1 for w in ws if abs(w) > W_SAT) / n,
        "sum_abs_w": sum(abs(w) for w in ws),
    }


# ------------------------------------------------------------------ 出力

def dump_csv(runs, outdir):
    """全走行の時系列と統計を CSV に落とす。図を使わず数値で見たい時用。"""
    ts_path = os.path.join(outdir, "omega_timeseries.csv")
    st_path = os.path.join(outdir, "omega_stats.csv")
    rows = 0
    st = []
    with open(ts_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["cond", "rep", "source", "t_s", "v_mps", "omega_radps"])
        for r in runs:
            cmd, meas = series(r["bagdir"])
            for src, data in (("cmd", cmd), ("odom", meas)):
                for t, v, om in data:
                    w.writerow([r["cond"], r["rep"], src,
                                f"{t:.4f}", f"{v:.6f}", f"{om:.6f}"])
                    rows += 1
            row = {"cond": r["cond"], "rep": r["rep"],
                   "drive_s": round(cmd[-1][0], 3)}
            for src, data in (("cmd", cmd), ("odom", meas)):
                for k, v in stats([om for _, _, om in data]).items():
                    row[f"{src}_{k}"] = round(v, 4) if isinstance(v, float) else v
            st.append(row)

    with open(st_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(st[0].keys()))
        w.writeheader()
        w.writerows(st)
    print(f"  omega_timeseries.csv ({rows} 行)")
    print(f"  omega_stats.csv ({len(st)} 走行)")
    return st


def setup_pres():
    """発表用の描画設定。投影で読めるよう論文用より一回り大きくする。"""
    FS.setup()
    plt.rcParams.update({
        "figure.dpi": 200,
        "savefig.dpi": 200,
        "font.size": 14,
        "axes.titlesize": 15,
        "axes.labelsize": 14,
        "legend.fontsize": 12,
        "xtick.labelsize": 12,
        "ytick.labelsize": 12,
    })


def fig_pres(runs, st, outdir, rep=1, conds=("l1", "l1_ms2")):
    """対策前後の ω(t)。指令（条件色・階段）に実測（灰）を重ねる。"""
    fig, axes = plt.subplots(len(conds), 1, figsize=(11.5, 6.6),
                             sharex=True, sharey=True)
    by = {(r["cond"], r["rep"]): r for r in runs}
    stat = {(s["cond"], s["rep"]): s for s in st}

    for ax, cond in zip(axes, conds):
        cmd, meas = series(by[(cond, rep)]["bagdir"])
        s = stat[(cond, rep)]

        ax.axhline(0, color="0.6", lw=1.0, zorder=0)
        for y in (W_CLAMP, -W_CLAMP):
            ax.axhline(y, color="0.35", ls=":", lw=1.2, zorder=0)

        ax.plot([t for t, _, _ in meas], [w for _, _, w in meas],
                color=FS.INK_MUTED, lw=1.5, alpha=0.9, zorder=2,
                label="実測 ω（車輪オドメトリ・20Hz）")
        ax.plot([t for t, _, _ in cmd], [w for _, _, w in cmd],
                color=FS.COLOR[cond], lw=2.2, drawstyle="steps-post", zorder=3,
                label="指令 ω（MPC出力・10Hz）")

        ax.set_ylabel("ω [rad/s]")
        ax.set_title(
            f"{FS.LABEL[cond]}  —  指令の符号反転 {s['cmd_flips']} 回／"
            f"実測 max|ω| {s['odom_max_abs_w']:.2f} rad/s", loc="left")
        # 直進区間（コーナー前）が空いているので凡例はそこに置く。指令の色は
        # 条件ごとに違うため、共通凡例にせず各段に置く。
        ax.legend(loc="upper left", framealpha=0.92)

    axes[-1].text(0.995, 0.05, f"点線 ±{W_CLAMP:.0f} rad/s ＝ 指令のクランプ値",
                  transform=axes[-1].transAxes, ha="right", va="bottom",
                  fontsize=11, color=FS.INK_MUTED)
    axes[-1].set_xlabel("走行開始からの時間 [s]")
    fig.suptitle("実機 L字走行の操舵入力 ω(t)：移動抑制の導入前後（各条件 代表1本）",
                 fontsize=16)
    fig.tight_layout()

    paths = []
    for ext in ("png", "pdf"):
        p = os.path.join(outdir, f"fig_omega_pres.{ext}")
        fig.savefig(p)
        paths.append(p)
        print(f"  {os.path.basename(p)}")
    plt.close(fig)
    return paths


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--outdir", default=OUTDIR)
    ap.add_argument("--rep", type=int, default=1, help="代表として描く走行番号")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    runs = [r for r in load_runs(os.path.join(REAL, "runs.csv")) if r["ok"]]
    print(f"実機 {len(runs)} 走行を読み込み")

    st = dump_csv(runs, args.outdir)
    setup_pres()
    fig_pres(runs, st, args.outdir, rep=args.rep)
    print(f"出力: {args.outdir}")


if __name__ == "__main__":
    main()
