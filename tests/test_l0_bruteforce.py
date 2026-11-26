# -*- coding: utf-8 -*-
"""L0 総当たり（S8）の単体テスト。小さい n だけを使い高速に保つ。"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "rover"))

from l0_bruteforce import (  # noqa: E402
    active_steps_lower_bound, brute_force_l0, feasible_on, l1_on_support,
    solve_l1, target,
)


def test_lower_bound_uses_dv_saturation():
    """δv は x にしか効かないので、活性ステップ数の下界は |x|/(h*u_max)。"""
    assert active_steps_lower_bound((2.0, 1.0, 0.0), 50, h=0.1, u_max=1.0) == 20
    assert active_steps_lower_bound((0.05, 0.0, 0.0), 10, h=0.1, u_max=1.0) == 1
    assert active_steps_lower_bound((0.0, 0.3, 0.0), 10, h=0.1, u_max=1.0) == 0


def test_empty_support_feasible_only_when_target_is_zero():
    Phi, zeta = target((0.05, 0.05, math.radians(5)), 6, 0.1, 1.0)
    assert feasible_on(Phi, zeta, [], u_max=1.0) is False
    assert feasible_on(Phi, np.zeros(3), [], u_max=1.0) is True


def test_brute_force_returns_feasible_minimal_supports():
    """返す支持はすべて実行可能で、1つ小さい濃度では実行可能解が無い。"""
    x0, n = (0.05, 0.05, math.radians(5)), 6
    Phi, zeta = target(x0, n, 0.1, 1.0)
    res = brute_force_l0(Phi, zeta, n, u_max=1.0)
    k = res["k_min"]
    assert k is not None and res["supports"]
    for S in res["supports"]:
        assert len(S) == k
        assert feasible_on(Phi, zeta, S, u_max=1.0)
    # k-1 では全滅であること（最小性）
    if k >= 1:
        import itertools
        assert not any(feasible_on(Phi, zeta, S, u_max=1.0)
                       for S in itertools.combinations(range(n), k - 1))


def test_restricted_l1_is_never_better_than_unrestricted():
    """支持を制限した L1 最小値が、制限なしの最適値を下回ることはない。"""
    x0, n = (0.05, 0.05, math.radians(5)), 6
    Phi, zeta = target(x0, n, 0.1, 1.0)
    full = solve_l1(Phi, zeta, n, 1.0)["l1_value"]
    res = brute_force_l0(Phi, zeta, n, u_max=1.0)
    for S in res["supports"]:
        v = l1_on_support(Phi, zeta, S, n, 1.0)
        if v is not None:
            assert v >= full - 1e-6


def test_l1_solution_meets_terminal_condition():
    x0, n = (0.0, 0.1, 0.0), 20
    Phi, zeta = target(x0, n, 0.1, 1.0)
    sol = solve_l1(Phi, zeta, n, 1.0)
    z = np.empty(2 * n)
    z[0::2], z[1::2] = sol["dv"], sol["dw"]
    assert np.allclose(Phi @ z, zeta, atol=1e-6)


@pytest.mark.parametrize("x0,n", [((0.05, 0.05, math.radians(5)), 6)])
def test_l1_active_steps_at_least_l0_minimum(x0, n):
    """L1 解の活性ステップ数は L0 最小値を下回れない。"""
    Phi, zeta = target(x0, n, 0.1, 1.0)
    sol = solve_l1(Phi, zeta, n, 1.0)
    res = brute_force_l0(Phi, zeta, n, u_max=1.0)
    assert sol["n_active_steps"] >= res["k_min"]
