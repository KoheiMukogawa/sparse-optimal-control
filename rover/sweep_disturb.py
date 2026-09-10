# -*- coding: utf-8 -*-
"""外乱注入スイープ（S4）。6.7節の実機積載バッチ（R1）が取れない間の代替。

外乱を3チャネルに分けて振る（意味は exp_backends.sim_run の docstring）:

  meas : 計測外乱（pos_noise / yaw_noise）— 制御器が見る姿勢だけが汚れる
  init : 開始姿勢の横ずれ（init_lat）— 経路の外から始める
  gain : プラント側の実効ゲイン低下（gain_v = gain_w = g）—「同じ指令でも
         進まない・回らない」状況。積載増を模した**較正されていない感度解析**で
         あり、実機の積載量（0/500/1000g）とは対応づけられていない

経路・遅延・指標は第6章の実機バッチ（configs/batch_Lturn1m_load*.yaml）と揃えて
あるので、実機が復旧したら同じ表の上で対照できる。

ノイズありでは sim も確率的になるため seed を振り、平均±標準偏差で報告する。
meas 以外のチャネルにも実機相当の計測ノイズ（BASE_NOISE）を常時載せてあるので、
全チャネルで分散を議論できる。

実行: uv run python rover/sweep_disturb.py
"""

import argparse
import csv
import math
import os
from collections import namedtuple

from exp_backends import load_path, sim_run
from exp_metrics import compute_metrics

Condition = namedtuple("Condition", "name controller lam move_suppress")
# invert: 図で x 軸を反転する（ゲインは「下がるほど外乱が強い」ので、
#         他チャネルと同じく右方向を「外乱が強い」に揃える）
Channel = namedtuple("Channel", "key label unit levels opts display invert",
                     defaults=(False,))

# 第6章の4条件と同一（configs/batch_Lturn1m_load000.yaml ＋ kanayama）
CONDITIONS = [
    Condition("kanayama", "kanayama", 0.0, 0.0),
    Condition("l2", "l2", 1.0, 0.0),
    Condition("l1", "l1", 0.3, 0.0),
    Condition("l1_ms2", "l1", 0.3, 2.0),
]

PATH_FILE = "configs/path_L_turn_1m.yaml"
COMMON = dict(horizon=15, rate=10.0)
DELAY_STEPS = 2               # 実機相当（約200ms）
TIMEOUT_S = 60.0
# 実機相当の計測ノイズ。meas 以外のチャネルではこれを固定で載せる
BASE_NOISE = dict(pos_noise=0.01, yaw_noise=math.radians(1.0))
DEFAULT_SEEDS = list(range(1, 21))

CHANNELS = [
    Channel(
        key="meas", label="計測ノイズ", unit="σpos[cm]",
        levels=[0.0, 0.005, 0.01, 0.02, 0.04],
        # 位置σ[m] に対し向きσ = 1度/cm 相当で連動させる（実機の姿勢推定は
        # 位置と向きが同時に劣化するため、片方だけ振っても実機的でない）
        opts=lambda v: dict(pos_noise=v, yaw_noise=math.radians(v * 100.0)),
        display=lambda v: v * 100.0),
    Channel(
        key="init", label="初期横ずれ", unit="ずれ[cm]",
        levels=[0.0, 0.05, 0.10, 0.20],
        opts=lambda v: dict(BASE_NOISE, init_lat=v),
        display=lambda v: v * 100.0),
    Channel(
        key="gain", label="実効ゲイン低下", unit="ゲインg",
        levels=[1.0, 0.9, 0.8, 0.7],
        opts=lambda v: dict(BASE_NOISE, gain_v=v, gain_w=v),
        display=lambda v: v, invert=True),
]

AGG_KEYS = ["rmse_true_cm", "rmse_meas_cm", "end_err_cm", "end_yaw_err_deg",
            "sum_u", "flips", "w_zero_ratio", "sat_ratio", "max_w",
            "solve_p50", "solve_p95"]

RUN_FIELDS = (["channel", "name", "level", "seed", "ok"] + AGG_KEYS)


def _goal_errors(waypoints, final):
    """真の終点誤差[cm]と終端向き誤差[deg]。向きの目標は最終脚の方位。"""
    gx, gy = waypoints[-1]
    px, py = waypoints[-2]
    x, y, th = final
    goal_th = math.atan2(gy - py, gx - px)
    dth = math.atan2(math.sin(th - goal_th), math.cos(th - goal_th))
    return 100.0 * math.hypot(gx - x, gy - y), math.degrees(dth)


def run_one(cond, waypoints, v_r, sim_opts, seed):
    """1本走らせて指標dictを返す。計測RMSEと真値RMSEを両方載せる。"""
    c = dict(name=cond.name, controller=cond.controller)
    if cond.controller == "l1":
        c["lam"] = cond.lam
    if cond.move_suppress:
        c["move_suppress"] = cond.move_suppress
    opts = dict(delay_steps=DELAY_STEPS, pos_noise=0.0, yaw_noise=0.0)
    opts.update(sim_opts)
    d = sim_run(c, COMMON, waypoints, v_r, opts, TIMEOUT_S, seed=seed)
    if not d["twist"]:
        return None
    m = compute_metrics(d["twist"], d["perr"], d["solve_ms"])
    # 真値RMSE: 同じ窓・同じ式で、汚れていない真の経路距離から計算する
    m_true = compute_metrics(d["twist"], d["terr"], d["solve_ms"])
    end_err, end_yaw = _goal_errors(waypoints, d["final"])
    return dict(ok=d["ok"], rmse_meas_cm=m["rmse_cm"],
                rmse_true_cm=m_true["rmse_cm"], end_err_cm=end_err,
                end_yaw_err_deg=end_yaw, sum_u=m["sum_u"], flips=m["flips"],
                w_zero_ratio=m["w_zero_ratio"], sat_ratio=m["sat_ratio"],
                max_w=m["max_w"], solve_p50=m["solve_p50"],
                solve_p95=m["solve_p95"])


def sweep_disturb(conditions, channels, seeds=DEFAULT_SEEDS,
                  path_file=PATH_FILE, progress=None):
    """条件×チャネル×水準×seed を1行ずつ返す。"""
    waypoints, v_r = load_path(path_file)
    rows = []
    for ch in channels:
        for cond in conditions:
            for level in ch.levels:
                for seed in seeds:
                    r = run_one(cond, waypoints, v_r, ch.opts(level), seed)
                    if r is None:
                        continue
                    rows.append(dict(channel=ch.key, name=cond.name,
                                     level=level, seed=seed, **r))
                if progress:
                    progress(ch.key, cond.name, level)
    return rows


def _mean_sd(vals):
    vals = [v for v in vals if not math.isnan(v)]
    if not vals:
        return float("nan"), float("nan")
    mean = sum(vals) / len(vals)
    if len(vals) < 2:
        return mean, 0.0
    var = sum((v - mean) ** 2 for v in vals) / (len(vals) - 1)
    return mean, math.sqrt(var)


def aggregate(rows):
    """(channel, name, level) ごとに n・到達率・各指標の平均と標準偏差。"""
    order, groups = [], {}
    for r in rows:
        k = (r["channel"], r["name"], r["level"])
        if k not in groups:
            order.append(k)
            groups[k] = []
        groups[k].append(r)
    out = []
    for k in order:
        g = groups[k]
        a = dict(channel=k[0], name=k[1], level=k[2], n=len(g),
                 reach_rate=sum(1 for r in g if r["ok"]) / len(g))
        for key in AGG_KEYS:
            mean, sd = _mean_sd([float(r[key]) for r in g])
            a[f"{key}_mean"], a[f"{key}_sd"] = mean, sd
        out.append(a)
    return out


def _channel_of(key):
    for ch in CHANNELS:
        if ch.key == key:
            return ch
    return None


def write_outputs(rows, outdir, plot=True):
    """runs.csv（全走行）・agg.csv・table.md（＋図）を書く。"""
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=RUN_FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r[k] for k in RUN_FIELDS})

    agg = aggregate(rows)
    with open(os.path.join(outdir, "agg.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(agg[0].keys()) if agg else [])
        w.writeheader()
        for a in agg:
            w.writerow(a)

    lines = [
        "# 外乱注入スイープ（1m L字・遅延2step・sim）", "",
        "実機の積載外乱バッチ（R1）が取れない間の代替（S4）。",
        "各セル n=%d seed の平均±標準偏差。" % (agg[0]["n"] if agg else 0), "",
        "- **真値RMSE**: 真の車体位置から経路までの距離（実機のカメラ真値に対応）",
        "- **計測RMSE**: 制御器が見た横偏差（実機の odom に対応。計測ノイズで汚れる）",
        "- 実効ゲイン g は積載を模した**較正されていない**パラメータで、",
        "  実機の積載量（0/500/1000g）とは対応づけられていない", "",
    ]
    for ch_key in dict.fromkeys(a["channel"] for a in agg):
        ch = _channel_of(ch_key)
        label = ch.label if ch else ch_key
        unit = ch.unit if ch else "水準"
        lines += [f"## {label}", "",
                  f"| 条件 | {unit} | 到達率 | 真値RMSE | 計測RMSE | 終点誤差 |"
                  " Σ\\|u\\| | ω反転 | ω0率 |",
                  "|---|---|---|---|---|---|---|---|---|"]
        for a in [x for x in agg if x["channel"] == ch_key]:
            disp = ch.display(a["level"]) if ch else a["level"]
            lines.append(
                f"| {a['name']} | {disp:g} | {a['reach_rate']*100:.0f}% | "
                f"{a['rmse_true_cm_mean']:.2f}±{a['rmse_true_cm_sd']:.2f} | "
                f"{a['rmse_meas_cm_mean']:.2f}±{a['rmse_meas_cm_sd']:.2f} | "
                f"{a['end_err_cm_mean']:.1f}±{a['end_err_cm_sd']:.1f} | "
                f"{a['sum_u_mean']:.2f}±{a['sum_u_sd']:.2f} | "
                f"{a['flips_mean']:.1f}±{a['flips_sd']:.1f} | "
                f"{a['w_zero_ratio_mean']*100:.0f}% |")
        lines.append("")
    with open(os.path.join(outdir, "table.md"), "w") as f:
        f.write("\n".join(lines) + "\n")

    if plot:
        _plot(agg, os.path.join(outdir, "disturb.png"))


def _plot(agg, png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from fig_style import COLOR, LABEL, LINESTYLE, setup

    setup()
    chans = list(dict.fromkeys(a["channel"] for a in agg))
    metrics = [("rmse_true_cm", "真値RMSE [cm]"), ("sum_u", "入力積算 Σ|u|"),
               ("flips", "ω符号反転 [回]")]
    fig, axes = plt.subplots(len(metrics), len(chans),
                             figsize=(3.6 * len(chans), 2.7 * len(metrics)),
                             squeeze=False)
    for j, ch_key in enumerate(chans):
        ch = _channel_of(ch_key)
        for i, (key, ylabel) in enumerate(metrics):
            ax = axes[i][j]
            for name in dict.fromkeys(a["name"] for a in agg):
                pts = [a for a in agg
                       if a["channel"] == ch_key and a["name"] == name]
                if not pts:
                    continue
                xs = [ch.display(p["level"]) if ch else p["level"] for p in pts]
                ys = [p[f"{key}_mean"] for p in pts]
                es = [p[f"{key}_sd"] for p in pts]
                ax.errorbar(xs, ys, yerr=es, marker="o", capsize=3,
                            label=LABEL.get(name, name),
                            color=COLOR.get(name),
                            linestyle=LINESTYLE.get(name, "-"))
            if i == 0:
                ax.set_title(ch.label if ch else ch_key)
            if ch and ch.invert and not ax.xaxis_inverted():
                ax.invert_xaxis()
            if i == len(metrics) - 1:
                ax.set_xlabel(ch.unit if ch else "水準")
            if j == 0:
                ax.set_ylabel(ylabel)
    axes[0][0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(png, dpi=200)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="外乱注入スイープ（S4）")
    ap.add_argument("--outdir", default="results/2026-09-10_disturb_sim")
    ap.add_argument("--seeds", type=int, default=len(DEFAULT_SEEDS),
                    help="各セルの seed 本数（既定20）")
    ap.add_argument("--channels", default="meas,init,gain")
    ap.add_argument("--levels", default="",
                    help="対象チャネルの水準を上書き（例 --channels gain "
                         "--levels 0.85,0.8,0.75,0.7）。チャタ消失の境界を"
                         "細かく刻むときに使う")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    keys = [k.strip() for k in args.channels.split(",") if k.strip()]
    channels = [ch for ch in CHANNELS if ch.key in keys]
    if args.levels:
        levels = [float(v) for v in args.levels.split(",")]
        channels = [ch._replace(levels=levels) for ch in channels]
    seeds = list(range(1, args.seeds + 1))

    def progress(ch_key, name, level):
        print(f"  {ch_key:5s} {name:9s} level={level:<6g} done", flush=True)

    print(f"外乱スイープ開始: {len(channels)}チャネル × "
          f"{len(CONDITIONS)}条件 × seed{len(seeds)}本")
    rows = sweep_disturb(CONDITIONS, channels, seeds=seeds, progress=progress)
    write_outputs(rows, args.outdir, plot=not args.no_plot)
    print(f"→ {args.outdir}/ に runs.csv・agg.csv・table.md を書いた"
          f"（全{len(rows)}走行）")


if __name__ == "__main__":
    main()
