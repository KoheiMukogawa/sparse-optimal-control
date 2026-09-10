# -*- coding: utf-8 -*-
"""OSQP 反復回数と warm start の効きを閉ループで測る。

第6章 確認事項1 と第7章 確認事項2 は、どちらも同じ推測に依っている:

  「素の L1 はチャタるせいで前回解が当てにならず（warm start が効かず）、
    求解が重くなる。移動抑制つきはチャタらないので warm start が効いて軽い」

実機の求解時間（p95 31.0 vs 37.6 ms）はこの推測の傍証でしかない。ここでは
**ソルバの反復回数**を直接記録して検定する。反復回数は機種に依存しないので、
実機が無くても測れる（実機の ms 値と対照できる量ではない点には注意）。

検定は2つ:
  H1（6.4節）: warm start 時の反復が 素のL1 > l1_ms2 で、warm を切ると差が縮む
  H2（7.2節）: 符号反転の近傍ステップは、それ以外より反復が多い

`bench_qp.py` は毎回ランダムな z0 を置く単体ベンチなので、
前回解が次の問題の役に立たず、warm start の効きを測る用途には使えない。

実行: uv run python rover/bench_warmstart.py
"""

import argparse
import csv
import math
import os
from collections import namedtuple

from exp_backends import load_path, sim_run
from exp_metrics import W_ZERO, compute_metrics

Condition = namedtuple("Condition", "name controller lam move_suppress")

CONDITIONS = [
    Condition("l2", "l2", 1.0, 0.0),
    Condition("l1", "l1", 0.3, 0.0),
    Condition("l1_ms2", "l1", 0.3, 2.0),
]

PATH_FILE = "configs/path_L_turn_1m.yaml"
COMMON = dict(horizon=15, rate=10.0)
DELAY_STEPS = 2      # 実機相当（約200ms）
TIMEOUT_S = 60.0
FLIP_WINDOW = 1      # 反転ステップと、その直後 window ステップを「反転近傍」とする


def _median(vals):
    vals = sorted(v for v in vals)
    if not vals:
        return float("nan")
    n = len(vals)
    return (vals[n // 2] if n % 2 else 0.5 * (vals[n // 2 - 1] + vals[n // 2]))


def _pct(vals, p):
    s = sorted(vals)
    if not s:
        return float("nan")
    return s[min(len(s) - 1, int(p * len(s)))]


def flip_indices(ws, window=FLIP_WINDOW):
    """符号反転が起きたステップと、その直後 window ステップの添字集合。

    符号の定義は exp_metrics.compute_metrics と同一（|ω|<W_ZERO はゼロ操舵で
    あって反転ではない）。前回解が当てにならないのは反転の前後なので、
    その区間だけを切り出して反復回数を比べる。
    """
    idx, prev = set(), 0
    for i, w in enumerate(ws):
        s = 1 if w > W_ZERO else (-1 if w < -W_ZERO else 0)
        if s != 0:
            if prev != 0 and s != prev:
                idx.update(range(i, min(len(ws), i + window + 1)))
            prev = s
    return idx


def split_by_flip(ws, iters, window=FLIP_WINDOW):
    """ω系列の符号反転近傍とそれ以外に反復回数を振り分ける。"""
    idx = flip_indices(ws, window)
    near = [it for i, it in enumerate(iters) if i in idx]
    other = [it for i, it in enumerate(iters) if i not in idx]
    return near, other


def run_case(cond, warm_start, waypoints, v_r, warmup=True):
    """1条件・1 warm設定で閉ループを1本走らせ、集計と生系列を返す。

    warmup: 計測前に同じ走行を1本捨てる。cvxpy の初回 canonicalization などが
    最初の条件にだけ乗ると、求解時間[ms]の条件間比較が壊れるため
    （反復回数は決定論的で影響を受けない）。
    """
    c = dict(name=cond.name, controller=cond.controller)
    if cond.controller == "l1":
        c["lam"] = cond.lam
    if cond.move_suppress:
        c["move_suppress"] = cond.move_suppress
    opts = dict(delay_steps=DELAY_STEPS, warm_start=warm_start)
    if warmup:
        sim_run(c, COMMON, waypoints, v_r, opts, TIMEOUT_S, seed=1)
    d = sim_run(c, COMMON, waypoints, v_r, opts, TIMEOUT_S, seed=1)
    m = compute_metrics(d["twist"], d["perr"], d["solve_ms"])
    ws = [w for _, _, w in d["twist"]]
    iters = d["iters"]
    near, other = split_by_flip(ws, iters)
    row = dict(
        name=cond.name, warm_start=warm_start, ok=d["ok"], steps=len(iters),
        flips=m["flips"], rmse_cm=m["rmse_cm"], sum_u=m["sum_u"],
        iters_total=sum(iters), iters_p50=_median(iters),
        iters_p95=_pct(iters, 0.95), iters_max=max(iters) if iters else 0,
        solve_p50=m["solve_p50"], solve_p95=m["solve_p95"],
        iters_near_flip=_median(near), iters_off_flip=_median(other),
        n_near_flip=len(near))
    flip_idx = flip_indices(ws)
    series = [dict(name=cond.name, warm_start=warm_start, step=i,
                   t=d["twist"][i][0], w=ws[i], iters=iters[i],
                   solve_ms=d["solve_ms"][i], near_flip=(i in flip_idx))
              for i in range(len(iters))]
    return row, series


def bench(conditions=CONDITIONS, path_file=PATH_FILE):
    rows, series = [], []
    waypoints, v_r = load_path(path_file)
    for cond in conditions:
        for warm in (True, False):
            row, ser = run_case(cond, warm, waypoints, v_r)
            rows.append(row)
            series.extend(ser)
    return rows, series


def write_outputs(rows, series, outdir):
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "steps.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["name", "warm_start", "step", "t",
                                          "w", "iters", "solve_ms",
                                          "near_flip"])
        w.writeheader()
        for r in series:
            w.writerow(r)

    by = {(r["name"], r["warm_start"]): r for r in rows}
    lines = [
        "# OSQP 反復回数と warm start の効き（1m L字・遅延2step・sim）", "",
        "反復回数は機種に依存しない量なので、実機なしでも測れる",
        "（実機の求解時間 ms と直接対照できる量ではない）。", "",
        "## 条件別（warm start あり／なし）", "",
        "求解時間[ms]は laptop 実測で cvxpy のオーバヘッドを含む**参考値**であり、",
        "実機（RPi4）の値とも、条件間の比較としても信用しないこと。",
        "議論に使うのは反復回数のほうである。", "",
        "| 条件 | warm | 反転 | 反復p50 | 反復p95 | 反復合計 | 求解p50[ms]（参考） |",
        "|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        lines.append(
            f"| {r['name']} | {'あり' if r['warm_start'] else 'なし'} | "
            f"{r['flips']} | {r['iters_p50']:.0f} | {r['iters_p95']:.0f} | "
            f"{r['iters_total']} | {r['solve_p50']:.1f} |")

    lines += ["", "## H1: warm start はどの条件でどれだけ効いているか", "",
              "| 条件 | 反復p50(warm) | 反復p50(cold) | 削減率 |",
              "|---|---|---|---|"]
    for cond in dict.fromkeys(r["name"] for r in rows):
        w_, c_ = by[(cond, True)], by[(cond, False)]
        cut = (1 - w_["iters_p50"] / c_["iters_p50"]) * 100 if c_["iters_p50"] else float("nan")
        lines.append(f"| {cond} | {w_['iters_p50']:.0f} | "
                     f"{c_['iters_p50']:.0f} | {cut:.0f}% |")

    lines += ["", "## H2: 符号反転の近傍で反復は増えるか", "",
              "warm なしの行は対照条件。前回解を使わないなら反転近傍でも"
              "反復は増えないはずで、増えていなければ",
              "「反転が warm start を無効化している」という説明が支持される。", "",
              "| 条件 | warm | 反転近傍の反復p50 | それ以外の反復p50 | 近傍ステップ数 |",
              "|---|---|---|---|---|"]
    for cond in dict.fromkeys(r["name"] for r in rows):
        for warm in (True, False):
            r = by[(cond, warm)]
            near = ("%.0f" % r["iters_near_flip"]
                    if not math.isnan(r["iters_near_flip"]) else "—（反転なし）")
            lines.append(
                f"| {cond} | {'あり' if warm else 'なし'} | {near} | "
                f"{r['iters_off_flip']:.0f} | {r['n_near_flip']} |")

    with open(os.path.join(outdir, "table.md"), "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser(description="OSQP 反復回数ベンチ")
    ap.add_argument("--outdir", default="results/2026-09-10_osqp_iters")
    args = ap.parse_args()
    rows, series = bench()
    write_outputs(rows, series, args.outdir)
    for r in rows:
        print(f"{r['name']:8s} warm={str(r['warm_start']):5s} "
              f"flips={r['flips']:3d} iters_p50={r['iters_p50']:6.0f} "
              f"total={r['iters_total']:7d} solve_p50={r['solve_p50']:.1f}ms")
    print(f"→ {args.outdir}/ に table.md・steps.csv を書いた")


if __name__ == "__main__":
    main()
