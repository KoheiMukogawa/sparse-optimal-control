# -*- coding: utf-8 -*-
"""オープンループ最大ハンズオフ制御（卒論 2.6節・図2.5）。

2026-04-28 の予備実験を定式化から再実装したもの。当時の実装コードと図は
リポジトリに残っておらず（`docs/作業記録/sparse_rover.py` は拡張子が .py の報告書）、
記録に残る数値は「δω ゼロ入力 39/50（78%）・L1ノルム 30.65」のみ。
本テストはその数値を再現できるかの検証を兼ねる。

定式化（`docs/作業記録/2026-04-28_sparse_control_sim.md`）:
  min ||z||_1  s.t.  Φz = ζ, ||z||_inf <= 1
"""

import math

import pytest

from openloop_sparse import solve_sparse

X0 = (2.0, 1.0, math.radians(30))


@pytest.fixture(scope="module")
def sol():
    return solve_sparse(X0, n=50, h=0.1, V=1.0, u_max=1.0)


def test_terminal_state_is_driven_to_zero(sol):
    """終端制約 Φz=ζ ＝ x[n]=0 が満たされる（到達性）。"""
    x_final = sol["x_traj"][-1]
    assert max(abs(v) for v in x_final) < 1e-6


def test_input_respects_saturation(sol):
    """モータ飽和 ||z||_inf <= 1。"""
    assert max(abs(v) for v in sol["z"]) <= 1.0 + 1e-6


def test_state_trajectory_has_n_plus_one_points(sol):
    assert len(sol["x_traj"]) == 51


def test_steering_input_is_sparse(sol):
    """本理論の核心: 操舵補正 δω の大半が厳密にゼロ（bang-off-bang）。

    記録値は 39/50（78%）。L1 解は一意とは限らないため厳密一致ではなく
    「7割以上がゼロ」を境界とする。
    """
    assert len(sol["dw"]) == 50
    assert sol["dw_zero"] >= 35, f"δω ゼロ数 {sol['dw_zero']}/50 は疎とは言えない"
