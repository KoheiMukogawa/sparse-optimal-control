# -*- coding: utf-8 -*-
"""卒論の図の元になるデータ読み込み・集計（S9）。

数値が既刊の比較文書（results/2026-07-15_Lturn_batch_compare.md）と一致することを
テストで固定する。図の見た目ではなく、図に載る数字が正しいことを守るのが目的。
"""

import math

import pytest

from thesis_data import (CONDITIONS, aggregate_runs, load_external, load_runs,
                         radar_axes)

REAL = "results/2026-07-15_Lturn_3way_real"


@pytest.fixture(scope="module")
def runs():
    return load_runs(f"{REAL}/runs.csv")


@pytest.fixture(scope="module")
def agg(runs):
    return aggregate_runs(runs)


@pytest.fixture(scope="module")
def ext():
    return load_external(f"{REAL}/external.csv")


def test_all_twelve_runs_loaded(runs):
    assert len(runs) == 12
    assert all(r["ok"] for r in runs)


def test_conditions_are_in_thesis_order():
    assert CONDITIONS == ["kanayama", "l2", "l1", "l1_ms2"]


@pytest.mark.parametrize("cond,rmse,sum_u", [
    ("kanayama", 11.26, 4.86),
    ("l2", 1.83, 4.65),
    ("l1", 3.49, 19.17),
    ("l1_ms2", 2.20, 5.02),
])
def test_aggregate_matches_published_compare_doc(agg, cond, rmse, sum_u):
    """batch_compare.md の表（平均, n=3）と一致すること。"""
    assert agg[cond]["rmse_cm"]["mean"] == pytest.approx(rmse, abs=0.005)
    assert agg[cond]["sum_u"]["mean"] == pytest.approx(sum_u, abs=0.005)


def test_aggregate_reports_std_over_three_reps(agg):
    assert agg["l2"]["rmse_cm"]["n"] == 3
    assert agg["l2"]["rmse_cm"]["std"] == pytest.approx(0.09, abs=0.005)
    assert agg["l1_ms2"]["flips"]["mean"] == pytest.approx(1.0)


def test_external_goal_distance_matches_hypot(ext):
    """external.md 記載のゴール距離が (x,y) と整合すること（転記検証）。"""
    assert len(ext) == 9
    for e in ext:
        assert math.hypot(e["x_cm"], e["y_cm"]) == pytest.approx(
            e["goal_dist_cm"], abs=0.05), f"{e['cond']} rep{e['rep']}"


def test_external_has_no_kanayama(ext):
    """kanayama は本バッチで外部計測していない（6/13 の n=1 参考値のみ）。"""
    assert {e["cond"] for e in ext} == {"l2", "l1", "l1_ms2"}


def test_radar_axes_are_normalized_with_larger_is_better(agg, ext):
    """レーダーは5軸とも「大きいほど良い」向きに正規化される。"""
    axes = radar_axes(agg, ext)
    assert set(axes) == {"l2", "l1", "l1_ms2"}
    for cond, vals in axes.items():
        assert len(vals) == 5
        assert all(0.0 <= v <= 1.0 for v in vals), f"{cond}: {vals}"
    names = ["追従精度", "終点精度", "向き精度", "スパース性", "計算軽さ"]
    # スパース性は l1_ms2 が最良（ω0率 0.88 > l2 0.87 > l1 0.53）
    i = names.index("スパース性")
    assert axes["l1_ms2"][i] > axes["l2"][i] > axes["l1"][i]
    # 終点精度は l2 が最良（12.7 < 21.2 < 29.9 cm）
    j = names.index("終点精度")
    assert axes["l2"][j] > axes["l1_ms2"][j] > axes["l1"][j]


def test_solve_ms_series_reproduces_recorded_p95(runs):
    """図6.4（箱ひげ）の元になる /mpc_solve_ms が runs.csv の記録と整合すること。

    bag を1本だけ読んで、その分布の p95 が runs.csv の solve_p95 に一致するかを見る。
    ここがずれていたら箱ひげは別物を描いていることになる。
    """
    from thesis_data import load_solve_ms, percentile

    r = next(x for x in runs if x["cond"] == "l1_ms2" and x["rep"] == 1)
    series = load_solve_ms(r["bagdir"])
    assert len(series) > 100
    assert percentile(series, 95) == pytest.approx(r["solve_p95"], rel=0.02)


def test_percentile_uses_the_same_definition_as_recorded_metrics():
    """runs.csv の solve_p95 は exp_metrics._pct（最近傍順位）で作られている。

    ここで線形補間を使うと、図6.4 と表6.4 の数字が定義違いでずれる。
    同じ定義であることをテストで固定する。
    """
    from exp_metrics import _pct
    from thesis_data import percentile

    xs = [4, 1, 3, 2]
    assert percentile(xs, 50) == _pct(sorted(xs), 0.50)
    assert percentile(xs, 95) == _pct(sorted(xs), 0.95)
    assert percentile(xs, 50) == 3  # 線形補間なら 2.5 になる


# --- 図5.7: real vs sim の対比（S12） -------------------------------------

SIM = "results/2026-07-15_Lturn_3way_sim"


@pytest.fixture(scope="module")
def pairs():
    from thesis_data import aggregate_runs, load_runs, real_vs_sim
    real = aggregate_runs(load_runs(f"{REAL}/runs.csv"))
    sim = aggregate_runs(load_runs(f"{SIM}/runs.csv"))
    return real_vs_sim(real, sim)


def test_pairs_cover_four_conditions_times_four_metrics(pairs):
    assert len(pairs) == 16
    assert {p["metric"] for p in pairs} == {
        "rmse_cm", "sum_u", "w_zero_ratio", "flips"}


def test_effort_and_sparsity_and_chatter_agree_within_seven_percent(pairs):
    """機序の裏付けとして効く3指標は sim と実機がよく一致する。"""
    for p in pairs:
        if p["metric"] == "rmse_cm" or p["rel_diff"] is None:
            continue
        assert abs(p["rel_diff"]) < 0.07, f"{p['cond']} {p['metric']}"


def test_rmse_relative_gap_exceeds_ten_percent_for_mpc_conditions(pairs):
    """既刊 batch_compare.md の「全指標10%以内」は RMSE では成り立たない。

    実機の方が悪い側にずれる。この事実を figure と本文で正直に書くための固定。
    """
    rmse = {p["cond"]: p for p in pairs if p["metric"] == "rmse_cm"}
    assert abs(rmse["kanayama"]["rel_diff"]) < 0.01
    for cond in ("l2", "l1", "l1_ms2"):
        assert rmse[cond]["rel_diff"] > 0.10, cond


def test_rmse_absolute_gap_is_small_for_every_condition(pairs):
    """相対差が大きいのは基準値が小さいためで、絶対差は 0.4cm 以内に収まる。"""
    for p in pairs:
        if p["metric"] != "rmse_cm":
            continue
        assert abs(p["abs_diff"]) < 0.4, f"{p['cond']}: {p['abs_diff']}"


def test_zero_flip_conditions_have_no_relative_difference(pairs):
    """反転0対0は比が定義できないので None にする（0除算で落とさない）。"""
    flips = {p["cond"]: p for p in pairs if p["metric"] == "flips"}
    assert flips["l2"]["sim"] == 0 and flips["l2"]["real"] == 0
    assert flips["l2"]["rel_diff"] is None
    assert flips["l1_ms2"]["rel_diff"] == pytest.approx(0.0)
