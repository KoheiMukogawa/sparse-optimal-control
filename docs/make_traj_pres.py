# -*- coding: utf-8 -*-
"""発表用の走行軌跡図（経路フレーム）と、その元になる軌跡 CSV を作る。

発表スライドの ω(t) 図（docs/make_omega_pres.py）は「入力がどう振れたか」を
示すが、制御が専門でない聴衆には角速度の時系列は像を結びにくい。
「上から見るとロボットがどう走ったか」を並べて出すための図。

2面構成にしてある。理由は素の L1 のチャタが**軌跡上では小さい**ため：
指令 ω は ±2.0 rad/s を往復しているが、車体の応答で均されて経路の
うねりは 1〜2 cm しかない。1.5 m 角の全体図に等尺で描くと軸幅の 1 % で、
投影では見えない。そこで

  左（全体）: 課題の形（1.5 m ＋ 左90° ＋ 1.5 m）と、Kanayama の外膨らみ
  右（拡大）: 旋回後の横方向偏差だけを cm 目盛で拡大。L1 のジグザグはここで見える

の役割分担にした。左だけ・右だけでは「何をしている実験か」「何が壊れて
いるか」のどちらかが落ちる。

**この図はオドメトリ基準である。** 車輪の滑りは原理的に写らないので、
追従精度の議論には使えない（それは外部カメラ真値の図の役目）。
ここでは「指令の振動が経路のうねりとして現れている」ことの定性的な
可視化に用途を限る。図中にもその旨を明記してある。

入力: results/2026-07-15_Lturn_3way_real/（実機 4条件×3本の bag と runs.csv）
      ※ 2026-06-13 の単発走行（n=1）ではなくバッチ（n=3）を使う。
         発表の数値は n=3 に統一する方針のため。
出力: results/2026-08-16_traj_pres/
  traj_pathframe.csv        12走行の経路フレーム軌跡（生の点列）
  crosstrack_stats.csv      走行ごとの旋回後の横偏差の統計
  fig_traj_pres.png/.pdf    発表用の2面図

実行: uv run python docs/make_traj_pres.py
"""

import argparse
import csv
import os
import sys

ROVER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rover")
sys.path.insert(0, os.path.abspath(ROVER))

import fig_style as FS
from fig_style import plt
from plot_lturn import load, to_path_frame
from thesis_data import load_runs

REAL = "results/2026-07-15_Lturn_3way_real"
OUTDIR = "results/2026-08-16_traj_pres"

LEG = 1.5        # m: L字の各直線の長さ（理想経路）
Y_TURN = 0.20    # m: これ以上進んだら「旋回を終えて第2直線に乗った」とみなす

# cm: 経路のうねりを数えるときのヒステリシス幅。直近の極値からこれ以上
# 戻ってはじめて「折り返した」と数える。単純な差分の符号反転で数えると
# odom の量子化ノイズを拾い、滑らかなはずの Kanayama が 28〜30 回になる
# （ω を実測値で数えると 38〜51 回出てしまうのと同じ罠）。
# 0.1〜0.8 cm の範囲で結果は変わらない（素L1 16〜17 回、他は 0〜1 回）ので、
# この値は結論に効いていない。
WIGGLE_HYST = 0.2

# 左図に出す条件（4条件すべて）と、右の拡大図に出す条件。
# Kanayama は外膨らみ（13 cm）が大きすぎて、L1 のジグザグ（1〜2 cm）と
# 同じ軸に載せると後者が潰れる。破綻の様式が別物なので拡大図からは外す。
CONDS_ALL = ("kanayama", "l2", "l1", "l1_ms2")
CONDS_ZOOM = ("l2", "l1", "l1_ms2")


def second_leg(xs, ys):
    """第2直線に乗ってからの (進行距離 y, 横方向偏差 x-1.5) を返す。

    偏差の符号は既刊の図6.1・図6.5と揃える（＋がコーナー外側）。
    """
    out = []
    for x, y in zip(xs, ys):
        if y >= Y_TURN:
            out.append((y, (x - LEG) * 100.0))  # cm
    return out


def wiggles(dev, hyst=WIGGLE_HYST):
    """横偏差の折り返し回数＝経路上のうねりの数。

    直近の極値から hyst 以上戻ったところで1回と数える（ZigZag法）。
    指令 ω の符号反転とは別物で、こちらは**車体が実際に蛇行した回数**。
    """
    n, d = 0, 0          # d: +1 外向きに移動中 / -1 内向き / 0 未確定
    hi = lo = dev[0]
    for v in dev[1:]:
        if d >= 0:
            hi = max(hi, v)
        if d <= 0:
            lo = min(lo, v)
        if d >= 0 and hi - v >= hyst:
            if d == 1:   # 未確定(0)からの初回は「折り返し」ではない
                n += 1
            d, lo = -1, v
        elif d <= 0 and v - lo >= hyst:
            if d == -1:
                n += 1
            d, hi = 1, v
    return n


def collect(runs):
    """全走行の経路フレーム軌跡を読み、(cond, rep) をキーに束ねる。"""
    traj = {}
    for r in runs:
        odom, _ = load(r["bagdir"])
        xs, ys = to_path_frame(odom)
        traj[(r["cond"], r["rep"])] = (xs, ys)
    return traj


# ------------------------------------------------------------------ 出力

def dump_csv(traj, outdir):
    """軌跡の生の点列と、旋回後の横偏差の統計を CSV に落とす。"""
    tp = os.path.join(outdir, "traj_pathframe.csv")
    sp = os.path.join(outdir, "crosstrack_stats.csv")

    rows = 0
    with open(tp, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["cond", "rep", "i", "x_m", "y_m"])
        for (cond, rep), (xs, ys) in sorted(traj.items()):
            for i, (x, y) in enumerate(zip(xs, ys)):
                w.writerow([cond, rep, i, f"{x:.5f}", f"{y:.5f}"])
                rows += 1

    st = []
    for (cond, rep), (xs, ys) in sorted(traj.items()):
        seg = second_leg(xs, ys)
        dev = [d for _, d in seg]
        st.append({
            "cond": cond, "rep": rep, "n": len(dev),
            "dev_min_cm": round(min(dev), 2),
            "dev_max_cm": round(max(dev), 2),
            "dev_ptp_cm": round(max(dev) - min(dev), 2),
            "dev_final_cm": round(dev[-1], 2),
            "wiggles": wiggles(dev),
        })
    with open(sp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(st[0].keys()))
        w.writeheader()
        w.writerows(st)

    print(f"  traj_pathframe.csv ({rows} 行)")
    print(f"  crosstrack_stats.csv ({len(st)} 走行)")
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
        "legend.fontsize": 11,
        "xtick.labelsize": 12,
        "ytick.labelsize": 12,
    })


def fig_pres(traj, st, outdir, reps=(1, 2, 3)):
    """左＝全体軌跡（等尺）、右＝旋回後の横偏差の拡大。"""
    fig, (ax0, ax1) = plt.subplots(
        1, 2, figsize=(13.0, 5.6), gridspec_kw={"width_ratios": [1.0, 1.25]})
    by = {(s["cond"], s["rep"]): s for s in st}

    # --- 左: 全体 ------------------------------------------------------
    ax0.plot([0, LEG, LEG], [0, 0, LEG], color=FS.INK, ls="--", lw=1.6,
             zorder=1, label="理想経路")
    for cond in CONDS_ALL:
        for j, rep in enumerate(reps):
            xs, ys = traj[(cond, rep)]
            ax0.plot(xs, ys, color=FS.COLOR[cond], ls=FS.LINESTYLE[cond],
                     lw=1.9, alpha=0.85, zorder=3,
                     label=FS.LABEL[cond] if j == 0 else None)
    ax0.plot(0, 0, "o", color=FS.INK, ms=7, zorder=4)
    ax0.annotate("走行開始", (0, 0), textcoords="offset points",
                 xytext=(6, -18), fontsize=11, color=FS.INK)
    ax0.plot(LEG, LEG, "*", color=FS.INK, ms=15, zorder=4)
    ax0.annotate("ゴール", (LEG, LEG), textcoords="offset points",
                 xytext=(8, 2), fontsize=11, color=FS.INK)
    ax0.set_xlabel("x [m]（第1直線の進行方向）")
    ax0.set_ylabel("y [m]（左90°旋回後の進行方向）")
    ax0.set_title("走行軌跡（実機 n=3 を重ね描き）", loc="left")
    ax0.set_aspect("equal", adjustable="box")
    ax0.set_xlim(-0.15, 1.85)
    ax0.set_ylim(-0.15, 1.70)
    ax0.legend(loc="upper left", framealpha=0.92, fontsize=10)
    # この尺度では MPC 系 3 条件はほぼ重なる。差が出ているのは Kanayama だけで、
    # しかもその外れ方は走行ごとに揃わない。右図が必要な理由でもあるので明記する。
    kan = [d for r in reps for d in
           (by[("kanayama", r)]["dev_min_cm"], by[("kanayama", r)]["dev_max_cm"])]
    ax0.text(0.06, 0.60,
             f"この尺度では MPC 系 3 条件は\nほぼ重なる。Kanayama だけが外れ、\n"
             f"走行ごとのばらつきも大きい\n"
             f"（{min(kan):.0f}〜{max(kan):+.0f} cm）".replace("-", "−"),
             transform=ax0.transAxes, ha="left", va="top",
             fontsize=10.5, color=FS.INK_MUTED, linespacing=1.5)

    # --- 右: 旋回後の横偏差 --------------------------------------------
    ax1.axhline(0, color=FS.INK, ls="--", lw=1.6, zorder=1, label="理想経路")
    for cond in CONDS_ZOOM:
        for j, rep in enumerate(reps):
            seg = second_leg(*traj[(cond, rep)])
            ax1.plot([y for y, _ in seg], [d for _, d in seg],
                     color=FS.COLOR[cond], ls=FS.LINESTYLE[cond], lw=2.0,
                     alpha=0.85, zorder=3,
                     label=FS.LABEL[cond] if j == 0 else None)
    ax1.set_xlabel("旋回後の進行距離 [m]")
    ax1.set_ylabel("経路からの横方向偏差 [cm]")
    ax1.set_title("旋回後の拡大：素の L1 だけが蛇行している", loc="left")
    ax1.set_xlim(Y_TURN, LEG)
    # 上に注記を置く帯を空ける（データは -6.3〜+2.9 cm に収まる）
    lo = min(s["dev_min_cm"] for s in st if s["cond"] in CONDS_ZOOM)
    hi = max(s["dev_max_cm"] for s in st if s["cond"] in CONDS_ZOOM)
    ax1.set_ylim(lo - 0.8, hi + 3.2)
    ax1.legend(loc="lower right", framealpha=0.92, fontsize=10)

    # うねりの回数は指令の符号反転と別に数えたもの。両者が近いことが
    # 「指令の振動が車体の動きとして実際に出ていた」の根拠になる。
    wig = [by[("l1", r)]["wiggles"] for r in reps]
    ax1.text(0.015, 0.975,
             f"経路のうねり {min(wig)}〜{max(wig)} 回（n=3）\n"
             f"指令 ω の符号反転は 18 回",
             transform=ax1.transAxes, ha="left", va="top", fontsize=12,
             color=FS.COLOR["l1"],
             bbox=dict(boxstyle="round,pad=0.4", fc="white", ec=FS.COLOR["l1"],
                       lw=1.2, alpha=0.95))
    ax1.text(0.985, 0.975, "＋＝コーナー外側", transform=ax1.transAxes,
             ha="right", va="top", fontsize=10.5, color=FS.INK_MUTED)

    fig.text(0.008, 0.012,
             "車輪オドメトリ基準（滑りは写らない）。指令の振動が経路のうねりとして"
             "現れていることの定性的な可視化であり、追従精度の比較には用いない。",
             fontsize=10.5, color=FS.INK_MUTED, ha="left", va="bottom")
    fig.suptitle("実機 L字走行の軌跡：上から見ると何が起きていたか", fontsize=16)
    fig.tight_layout(rect=(0, 0.045, 1, 1))

    paths = []
    for ext in ("png", "pdf"):
        p = os.path.join(outdir, f"fig_traj_pres.{ext}")
        fig.savefig(p)
        paths.append(p)
        print(f"  {os.path.basename(p)}")
    plt.close(fig)
    return paths


def _save(fig, outdir, stem):
    for ext in ("png", "pdf"):
        p = os.path.join(outdir, f"{stem}.{ext}")
        fig.savefig(p)
        print(f"  {os.path.basename(p)}")
    plt.close(fig)


def fig_p4(traj, st, outdir, reps=(1, 2, 3)):
    """p4「実機での問題」用。ω(t) 図の下に縦積みする横長1面。

    **移動抑制つき（l1_ms2）は描かない。** p4 の時点では対策をまだ提示して
    いないため、緑の線が出ていると「もう解けている」と読めてしまう。
    比較対象は p4 の表と同じく L2 と素の L1 に限る。

    縦横比は fig_omega_pres の1段ぶん（11.5×3.3 in）に合わせてあるので、
    ω 図の直下に置いたときに軸の幅と文字の大きさが揃う。
    """
    conds = ("l2", "l1")
    by = {(s["cond"], s["rep"]): s for s in st}

    fig, ax = plt.subplots(figsize=(11.5, 3.3))
    ax.axhline(0, color=FS.INK, ls="--", lw=1.6, zorder=1, label="理想経路")
    for cond in conds:
        for j, rep in enumerate(reps):
            seg = second_leg(*traj[(cond, rep)])
            ax.plot([y for y, _ in seg], [d for _, d in seg],
                    color=FS.COLOR[cond], ls=FS.LINESTYLE[cond], lw=2.0,
                    alpha=0.85, zorder=3,
                    label=FS.LABEL[cond] if j == 0 else None)

    lo = min(by[(c, r)]["dev_min_cm"] for c in conds for r in reps)
    hi = max(by[(c, r)]["dev_max_cm"] for c in conds for r in reps)
    ax.set_xlim(Y_TURN, LEG)
    ax.set_ylim(lo - 0.8, hi + 3.6)
    ax.set_xlabel("旋回後の進行距離 [m]")
    ax.set_ylabel("横方向偏差 [cm]")
    ax.set_title("上から見た経路（コーナー通過後を拡大）", loc="left")
    # 軸内はどこも線が通っているので、凡例はタイトル行の右へ逃がす。
    ax.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=3,
              frameon=False, fontsize=11)

    wig = [by[("l1", r)]["wiggles"] for r in reps]
    ax.text(0.012, 0.95,
            f"経路のうねり {min(wig)}〜{max(wig)} 回（n=3）／指令 ω の符号反転は 18 回",
            transform=ax.transAxes, ha="left", va="top", fontsize=12,
            color=FS.COLOR["l1"],
            bbox=dict(boxstyle="round,pad=0.35", fc="white",
                      ec=FS.COLOR["l1"], lw=1.2, alpha=0.95))
    ax.text(0.988, 0.95, "＋＝コーナー外側", transform=ax.transAxes,
            ha="right", va="top", fontsize=10.5, color=FS.INK_MUTED)
    fig.text(0.005, 0.005,
             "車輪オドメトリ基準（滑りは写らない）。振動の定性的な可視化であり、"
             "追従精度の比較には用いない。",
             fontsize=10, color=FS.INK_MUTED, ha="left", va="bottom")
    fig.tight_layout(rect=(0, 0.055, 1, 1))
    _save(fig, outdir, "fig_traj_p4")


def fig_p2(traj, outdir, reps=(1, 2, 3)):
    """p2「背景と問い」用。課題の形を伝える等尺の全体図。

    **p2 で名前を出している 2 手法（Kanayama・L2-MPC）だけを描く。**
    L1 系は p3 以降の話なので、ここに出すと筋が前後する。
    右の「ω ゼロ入力率」ボックスの 0.650 / 0.872 に絵の裏づけを与えるのが
    この図の役割。
    """
    conds = ("kanayama", "l2")
    fig, ax = plt.subplots(figsize=(5.2, 4.9))

    ax.plot([0, LEG, LEG], [0, 0, LEG], color=FS.INK, ls="--", lw=1.8,
            zorder=1, label="理想経路")
    for cond in conds:
        for j, rep in enumerate(reps):
            xs, ys = traj[(cond, rep)]
            ax.plot(xs, ys, color=FS.COLOR[cond], ls=FS.LINESTYLE[cond],
                    lw=2.0, alpha=0.85, zorder=3,
                    label=FS.LABEL[cond] if j == 0 else None)

    ax.plot(0, 0, "o", color=FS.INK, ms=8, zorder=4)
    ax.annotate("開始", (0, 0), textcoords="offset points",
                xytext=(2, -22), fontsize=12, color=FS.INK)
    ax.plot(LEG, LEG, "*", color=FS.INK, ms=17, zorder=4)
    ax.annotate("ゴール", (LEG, LEG), textcoords="offset points",
                xytext=(6, 4), fontsize=12, color=FS.INK)
    # 課題の寸法。非工学の聴衆にはこのスケール感がないと cm の議論が効かない。
    ax.annotate("1.5 m", (LEG / 2, 0), textcoords="offset points",
                xytext=(0, -30), ha="center", fontsize=12, color=FS.INK_MUTED)
    # 軌跡は理想経路のすぐ脇を通るので、寸法線は十分に離して置く
    ax.annotate("1.5 m", (LEG, LEG / 2), textcoords="offset points",
                xytext=(-100, 0), va="center", ha="center", fontsize=12,
                color=FS.INK_MUTED, rotation=90)
    ax.annotate("左 90°", (LEG, 0), textcoords="offset points",
                xytext=(16, -16), fontsize=12, color=FS.INK_MUTED)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("L 字経路の追従（実機 n=3）", loc="left")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-0.25, 1.95)
    ax.set_ylim(-0.32, 1.75)
    ax.legend(loc="upper left", framealpha=0.92, fontsize=11)
    fig.text(0.005, 0.005, "車輪オドメトリ基準", fontsize=10,
             color=FS.INK_MUTED, ha="left", va="bottom")
    fig.tight_layout(rect=(0, 0.035, 1, 1))
    _save(fig, outdir, "fig_traj_p2")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--outdir", default=OUTDIR)
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    runs = [r for r in load_runs(os.path.join(REAL, "runs.csv")) if r["ok"]]
    print(f"実機 {len(runs)} 走行を読み込み")

    traj = collect(runs)
    st = dump_csv(traj, args.outdir)
    setup_pres()
    fig_pres(traj, st, args.outdir)   # 2面版（補足・手元確認用）
    fig_p4(traj, st, args.outdir)     # p4 に貼る横長1面
    fig_p2(traj, args.outdir)         # p2 に貼る等尺の全体図

    print("\n=== 旋回後の横偏差（cm・うねり数） ===")
    for s in st:
        print(f"  {s['cond']:<9} r{s['rep']}  "
              f"偏差 {s['dev_min_cm']:+6.2f}〜{s['dev_max_cm']:+6.2f}  "
              f"振幅 {s['dev_ptp_cm']:5.2f}  うねり {s['wiggles']:>3} 回")
    print(f"\n出力: {args.outdir}")


if __name__ == "__main__":
    main()
