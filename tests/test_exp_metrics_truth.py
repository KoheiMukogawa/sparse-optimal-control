# -*- coding: utf-8 -*-
"""truth_metrics（真値rows→終点・横偏差RMSE）のテスト。"""
import math

from exp_metrics import truth_metrics

WPS = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]


def _rows_along_path(offset=0.02, n=50):
    """経路から法線方向に offset ずらした軌跡（0.5s刻み・最後の1sはゴール横）。"""
    rows = []
    for i in range(n):
        s = 2.0 * i / (n - 1)          # 経路弧長 0..2m
        if s <= 1.0:
            x, y, th = s, -offset, 0.0
        else:
            x, y, th = 1.0 + offset, s - 1.0, math.pi / 2
        rows.append((0.5 * i, x, y, th, 5, 300.0))
    return rows


def test_truth_metrics_endpoint_and_rmse():
    rows = _rows_along_path(offset=0.02)
    m = truth_metrics(rows, WPS)
    # 終点=末尾0.5sの平均 ≈ (1.02, 2.0-1.0=1.0近傍)
    assert abs(m['truth_end_x'] - 1.02) < 0.005
    assert abs(m['truth_end_y'] - 1.0) < 0.03
    assert abs(m['truth_end_theta'] - math.pi / 2) < 0.01
    # ゴール(1,1)からの距離 ≈ 2cm、横偏差RMSE ≈ 2cm
    assert abs(m['truth_end_dist_cm'] - 2.0) < 1.0
    assert abs(m['truth_rmse_cm'] - 2.0) < 0.5


def test_truth_metrics_ignores_missing_and_empty():
    rows = _rows_along_path()
    rows[10] = (5.0, None, None, None, 4, 0.0)   # 欠測行は無視
    assert 'truth_rmse_cm' in truth_metrics(rows, WPS)
    assert truth_metrics([(0.0, None, None, None, 0, 0.0)], WPS) == {}
