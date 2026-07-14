import math

import pytest

from exp_metrics import compute_metrics


def make_series():
    # 10サンプル・0.1s刻み・全区間 v=0.1（走行中）。
    # ω: ゼロ7 / +2,-2,+2（符号反転2回・全て飽和|ω|>1.5）
    ws = [0.0, 0.0, 2.0, -2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    twist = [(0.1 * i, 0.1, ws[i]) for i in range(10)]
    perr = [(0.1 * i, 0.03) for i in range(10)]      # 常に3cm
    solve_ms = [10.0, 20.0, 30.0]
    return twist, perr, solve_ms


def test_basic_metrics():
    twist, perr, solve_ms = make_series()
    m = compute_metrics(twist, perr, solve_ms)
    assert m["steps"] == 10
    assert m["drive_s"] == pytest.approx(0.9)
    # dt=0.9/9=0.1, Σ(|v|+|ω|)·dt = (10*0.1 + 6.0)*0.1
    assert m["sum_u"] == pytest.approx(0.7)
    assert m["w_zero_ratio"] == pytest.approx(0.7)
    assert m["flips"] == 2
    assert m["sat_ratio"] == pytest.approx(0.3)
    assert m["max_w"] == pytest.approx(2.0)
    assert m["rmse_cm"] == pytest.approx(3.0)
    assert m["solve_p50"] == pytest.approx(20.0)
    assert m["solve_max"] == pytest.approx(30.0)


def test_no_perr_no_solve():
    twist, _, _ = make_series()
    m = compute_metrics(twist)
    assert math.isnan(m["rmse_cm"])
    assert math.isnan(m["solve_p50"])


def test_empty_twist_raises():
    with pytest.raises(ValueError):
        compute_metrics([])
