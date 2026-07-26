# -*- coding: utf-8 -*-
"""卒論の図の元データ（S9）。

`results/<batch>/runs.csv`（odom基準の4指標＋チャタ指標）と
`external.csv`（カメラ真値以前の手計測による終点）を読み、条件ごとに集計する。
作図は `docs/make_thesis_figs.py` 側。ここには matplotlib を持ち込まない
（図の見た目と、図に載る数字の正しさを分けて守るため）。
"""

import csv
import math
import statistics
from pathlib import Path

# runs.csv の solve_p50/p95 と同じ定義を使う（最近傍順位。線形補間ではない）。
# 定義がずれると図6.4 の箱ひげと表6.4 の数字が食い違うため、再実装せず流用する。
from exp_metrics import _pct

# 卒論の表・図で使う条件の並び（比較の読み順: ベースライン → L2 → 素のL1 → 対策後）
CONDITIONS = ["kanayama", "l2", "l1", "l1_ms2"]

# 集計対象の数値列。solve_* は kanayama で nan（解析的フィードバックのため）
NUMERIC_FIELDS = ["drive_s", "rmse_cm", "sum_u", "w_zero_ratio", "flips",
                  "sat_ratio", "max_w", "solve_p50", "solve_p95", "solve_max"]

# レーダーチャートの軸（名前, 参照先, 大きいほど良いか）
RADAR_AXES = [
    ("追従精度", "rmse_cm", False),
    ("終点精度", "goal_dist_cm", False),
    ("向き精度", "abs_theta_deg", False),
    ("スパース性", "w_zero_ratio", True),
    ("計算軽さ", "solve_p95", False),
]


def percentile(values, q):
    """q パーセンタイル（0〜100）。runs.csv と同じ最近傍順位の定義。"""
    return _pct(sorted(values), q / 100.0)


def load_solve_ms(bagdir):
    """bag の /mpc_solve_ms を読み、求解時間 [ms] の列を返す（図6.4 用）。"""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS2_HUMBLE)
    out = []
    with AnyReader([Path(bagdir)], default_typestore=ts) as r:
        conns = [c for c in r.connections if c.topic == "/mpc_solve_ms"]
        for c, _, raw in r.messages(connections=conns):
            out.append(float(r.deserialize(raw, c.msgtype).data))
    return out


def _num(s):
    """空文字・nan を None に落として float 化する。"""
    if s is None or s == "":
        return None
    try:
        v = float(s)
    except ValueError:
        return None
    return None if math.isnan(v) else v


def load_runs(path):
    """runs.csv を読み、1走行1辞書のリストで返す。"""
    rows = []
    with open(path, newline="") as f:
        for raw in csv.DictReader(f):
            r = dict(raw)
            r["rep"] = int(raw["rep"])
            r["ok"] = raw["ok"] == "True"
            for k in NUMERIC_FIELDS:
                r[k] = _num(raw.get(k))
            rows.append(r)
    return rows


def _stats(values):
    """None を除いた平均・標本標準偏差・件数。"""
    vals = [v for v in values if v is not None]
    if not vals:
        return {"mean": None, "std": None, "n": 0, "values": []}
    return {
        "mean": statistics.fmean(vals),
        "std": statistics.stdev(vals) if len(vals) > 1 else 0.0,
        "n": len(vals),
        "values": vals,
    }


def aggregate_runs(runs):
    """条件ごとに各指標の平均±標本標準偏差を出す。

    標準偏差は標本標準偏差（ddof=1）。既刊の
    `results/2026-07-15_Lturn_batch_compare.md` の表がこの定義。
    """
    agg = {}
    for cond in CONDITIONS:
        sel = sorted((r for r in runs if r["cond"] == cond),
                     key=lambda r: r["rep"])
        if not sel:
            continue
        agg[cond] = {k: _stats([r[k] for r in sel]) for k in NUMERIC_FIELDS}
        agg[cond]["runs"] = sel
    return agg


def load_external(path):
    """external.csv（手計測の終点）を読む。`#` 始まりはコメント。"""
    with open(path, newline="") as f:
        lines = [ln for ln in f if not ln.lstrip().startswith("#")]
    out = []
    for raw in csv.DictReader(lines):
        out.append({
            "cond": raw["cond"],
            "rep": int(raw["rep"]),
            "x_cm": float(raw["x_cm"]),
            "y_cm": float(raw["y_cm"]),
            "theta_deg": float(raw["theta_deg"]),
            "goal_dist_cm": float(raw["goal_dist_cm"]),
        })
    return out


def external_by_condition(ext):
    """条件ごとに ゴール距離 と |θ| を集計する。"""
    out = {}
    for cond in CONDITIONS:
        sel = [e for e in ext if e["cond"] == cond]
        if not sel:
            continue
        out[cond] = {
            "goal_dist_cm": _stats([e["goal_dist_cm"] for e in sel]),
            "abs_theta_deg": _stats([abs(e["theta_deg"]) for e in sel]),
            "points": sel,
        }
    return out


COMPARE_METRICS = ["rmse_cm", "sum_u", "w_zero_ratio", "flips"]

METRIC_LABEL = {
    "rmse_cm": "追従 RMSE [cm]",
    "sum_u": "Σ|u|（操舵量）",
    "w_zero_ratio": "ω0率",
    "flips": "ω符号反転 [回]",
}


def real_vs_sim(real_agg, sim_agg, metrics=COMPARE_METRICS):
    """実機と sim の条件平均を突き合わせる（図5.7 用）。

    rel_diff は sim を基準にした相対差 (real - sim) / sim。
    sim が 0（反転0対0など）のときは比が定義できないので None を返す。
    """
    pairs = []
    for cond in CONDITIONS:
        if cond not in real_agg or cond not in sim_agg:
            continue
        for m in metrics:
            s = sim_agg[cond][m]["mean"]
            r = real_agg[cond][m]["mean"]
            if s is None or r is None:
                continue
            pairs.append({
                "cond": cond,
                "metric": m,
                "sim": s,
                "real": r,
                "abs_diff": r - s,
                "rel_diff": None if s == 0 else (r - s) / s,
            })
    return pairs


def radar_axes(agg, ext):
    """レーダー用に5軸を (0,1] （大きいほど良い）へ正規化する。

    真値の終点計測がある条件（l2 / l1 / l1_ms2）だけを対象にする。
    kanayama は本バッチで外部計測しておらず、求解時間も持たないため除外する。

    正規化は**最良条件に対する比**（小さいほど良い指標なら 最良値/自分の値、
    大きいほど良い指標なら 自分の値/最良値）。最良が 1.0 で、0.5 なら
    「最良の2倍悪い」と読める。min-max 正規化を使うと、全軸で最下位の条件が
    原点の一点に潰れて図として機能しないため採らない（素の L1 が実際にそうなる）。
    """
    extc = external_by_condition(ext)
    conds = [c for c in CONDITIONS if c in extc and c in agg]

    raw = {}
    for c in conds:
        raw[c] = {
            "rmse_cm": agg[c]["rmse_cm"]["mean"],
            "goal_dist_cm": extc[c]["goal_dist_cm"]["mean"],
            "abs_theta_deg": extc[c]["abs_theta_deg"]["mean"],
            "w_zero_ratio": agg[c]["w_zero_ratio"]["mean"],
            "solve_p95": agg[c]["solve_p95"]["mean"],
        }

    axes = {c: [] for c in conds}
    for _, key, larger_is_better in RADAR_AXES:
        vals = [raw[c][key] for c in conds]
        best = max(vals) if larger_is_better else min(vals)
        for c in conds:
            v = raw[c][key]
            if v == 0 or best == 0:
                axes[c].append(1.0 if v == best else 0.0)
            elif larger_is_better:
                axes[c].append(v / best)
            else:
                axes[c].append(best / v)
    return axes
