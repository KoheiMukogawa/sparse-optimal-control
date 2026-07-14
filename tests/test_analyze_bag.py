from pathlib import Path

import pytest

from analyze_bag import analyze

REPO = Path(__file__).resolve().parent.parent
LTURN_L1 = REPO / "results" / "2026-06-13_Lturn_l1"
LTURN_L2 = REPO / "results" / "2026-06-13_Lturn_l2"


@pytest.mark.skipif(not LTURN_L1.exists(), reason="実機bagなし")
def test_regression_lturn_l1():
    """Lturn_compare.md A/B節の記録値と一致（改修による数値変化がないこと）。"""
    m = analyze(str(LTURN_L1))
    assert m["rmse_cm"] == pytest.approx(4.16, abs=0.05)
    assert m["sum_u"] == pytest.approx(20.71, abs=0.05)
    assert m["w_zero_ratio"] == pytest.approx(0.50, abs=0.01)
    assert m["solve_p50"] == pytest.approx(42.9, abs=0.5)
    assert m["solve_p95"] == pytest.approx(89.7, abs=0.5)
    # チャタ指標（plot_lturn.py 記録値: 反転15回・飽和41%）
    assert 12 <= m["flips"] <= 18
    assert m["sat_ratio"] == pytest.approx(0.41, abs=0.03)


@pytest.mark.skipif(not LTURN_L2.exists(), reason="実機bagなし")
def test_regression_lturn_l2():
    m = analyze(str(LTURN_L2))
    assert m["rmse_cm"] == pytest.approx(1.42, abs=0.05)
    assert m["sum_u"] == pytest.approx(4.54, abs=0.05)
    assert m["flips"] == 0
