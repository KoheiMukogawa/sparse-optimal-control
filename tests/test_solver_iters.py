# -*- coding: utf-8 -*-
"""OSQP 反復回数の記録と warm start の効き（6.4節・7.2節の推測を潰す）。

第6章 確認事項1: 「提案手法が素の L1 より軽いのは warm start が効くため」は未検証。
第7章 確認事項2: 「チャタ → warm start が効かない → 求解が重くなる」も未検証。
どちらもソルバの反復回数を記録すれば確かめられる。反復回数は機種に依存しない
（実機の ms 値と違って laptop でも測れる）ので、実機復旧を待たずに検証できる。
"""

import math

from exp_backends import load_path, sim_run
from mpc_core import MPCFollower

WPS, VR = load_path("configs/path_L_turn_1m.yaml")
COMMON = dict(horizon=15, rate=10.0)


def test_mpc_records_solver_iterations():
    mpc = MPCFollower(N=15, ts=0.1, reg="l1", lam=0.3)
    assert mpc.command(0.05, 0.05, 0.1, 0.1) is not None
    assert mpc.last_iters > 0, "OSQP の反復回数が記録されていない"


def test_cold_start_still_solves_and_records():
    mpc = MPCFollower(N=15, ts=0.1, reg="l1", lam=0.3, warm_start=False)
    assert mpc.command(0.05, 0.05, 0.1, 0.1) is not None
    assert mpc.last_iters > 0


def test_warm_start_is_cheaper_when_the_problem_barely_moves():
    """ほぼ同じ問題を解き直す限り、warm start は反復を減らすはず。

    これが成り立たなければ「warm start が効いている/いない」という
    6.4節の議論自体が成立しないので、前提の健全性テストとして置く。
    """
    def total_iters(warm):
        mpc = MPCFollower(N=15, ts=0.1, reg="l2", warm_start=warm)
        total = 0
        for i in range(10):
            mpc.command(0.05 + 1e-4 * i, 0.05, 0.1, 0.1)
            total += mpc.last_iters
        return total

    assert total_iters(True) < total_iters(False)


def test_sim_run_records_iteration_series():
    d = sim_run(dict(name="l1", controller="l1", lam=0.3), COMMON, WPS, VR,
                dict(delay_steps=2), 60.0, seed=1)
    assert len(d["iters"]) == len(d["twist"])
    assert all(i > 0 for i in d["iters"])


def test_sim_run_honours_warm_start_flag():
    warm = sim_run(dict(name="l1", controller="l1", lam=0.3), COMMON, WPS, VR,
                   dict(delay_steps=2, warm_start=True), 60.0, seed=1)
    cold = sim_run(dict(name="l1", controller="l1", lam=0.3), COMMON, WPS, VR,
                   dict(delay_steps=2, warm_start=False), 60.0, seed=1)
    assert sum(cold["iters"]) != sum(warm["iters"])


def test_kanayama_has_no_solver_iterations():
    d = sim_run(dict(name="kanayama", controller="kanayama"), COMMON, WPS, VR,
                dict(delay_steps=2), 60.0, seed=1)
    assert d["iters"] == []


# ---- 反転ステップの切り分け（7.2節の「チャタが warm start を殺す」の検定） ----

from bench_warmstart import split_by_flip  # noqa: E402


def test_split_by_flip_marks_the_flip_step_and_its_window():
    # ω: + + - -  → index2 で符号反転。window=1 なら index2,3 が「反転近傍」
    ws = [1.0, 1.0, -1.0, -1.0, 1e-3, 1e-3]
    iters = [10, 11, 50, 40, 12, 13]
    near, other = split_by_flip(ws, iters, window=1)
    assert near == [50, 40]
    assert other == [10, 11, 12, 13]


def test_split_by_flip_ignores_deadband_crossings():
    """|ω|<W_ZERO は「ゼロ操舵」であって符号反転ではない（exp_metrics と同義）。"""
    ws = [1.0, 0.0, 1.0, 0.0]
    iters = [10, 20, 30, 40]
    near, other = split_by_flip(ws, iters, window=1)
    assert near == []
    assert other == iters


def test_split_by_flip_handles_no_flips():
    near, other = split_by_flip([0.5] * 5, [1, 2, 3, 4, 5], window=2)
    assert near == []
    assert len(other) == 5


def test_median_of_empty_is_nan():
    from bench_warmstart import _median
    assert math.isnan(_median([]))


def test_run_case_returns_row_and_series():
    """run_case まで通す（定数の取り違えのような結線ミスをここで捕まえる）。"""
    from bench_warmstart import CONDITIONS, run_case
    l1 = [c for c in CONDITIONS if c.name == "l1"][0]
    row, series = run_case(l1, True, WPS, VR)
    assert row["steps"] == len(series) > 0
    assert row["iters_total"] > 0
    assert row["flips"] >= 8, "遅延2step の素のL1 はチャタるはず"
    assert any(r["near_flip"] for r in series)
