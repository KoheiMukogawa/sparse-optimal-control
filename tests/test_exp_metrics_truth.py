# -*- coding: utf-8 -*-
"""truth_metrics（真値rows→終点・横偏差RMSE）のテスト。

追従指標は「実測開始poseに固定したコース」基準（2026-07-17変更）。
開始向きの残差が追従誤差に混入しないことが主契約。
"""
import math

from exp_metrics import truth_metrics

WPS = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]


def _rows_along_path(offset=0.02, n=50):
    """経路から法線方向に offset ずらした軌跡。

    実運用同様、先頭にノード起動待ちの静止区間（1.5s）を置く
    （開始pose推定が移動で偏らない）。
    """
    rows = [(0.5 * i, 0.0, -offset, 0.0, 5, 300.0) for i in range(4)]
    t0 = 2.0
    for i in range(n):
        s = 2.0 * i / (n - 1)          # 経路弧長 0..2m
        if s <= 1.0:
            x, y, th = s, -offset, 0.0
        else:
            x, y, th = 1.0 + offset, s - 1.0, math.pi / 2
        rows.append((t0 + 0.5 * i, x, y, th, 5, 300.0))
    return rows


def _transform(rows, dx, dy, ang):
    """軌跡全体を回転＋平行移動（開始poseがズレた走行を模擬）。"""
    c, s = math.cos(ang), math.sin(ang)
    return [(t, dx + c * x - s * y, dy + s * x + c * y, th + ang, n, q)
            for (t, x, y, th, n, q) in rows]


def test_truth_metrics_endpoint_and_rmse():
    rows = _rows_along_path(offset=0.02)
    m = truth_metrics(rows, WPS)
    # 終点（タグ座標系の生値）
    assert abs(m['truth_end_x'] - 1.02) < 0.005
    assert abs(m['truth_end_y'] - 1.0) < 0.03
    assert abs(m['truth_end_theta'] - math.pi / 2) < 0.01
    # 開始pose基準: コースは開始点(0,-0.02)に平行移動される。leg1は誤差0、
    # leg2の法線オフセット2cmは剛体変換で消えない=真の追従誤差 →
    # rmse≈2/√2cm。終点は窓平均で(1.02,0.98)となり課コースゴール(1.0,0.98)
    # からx方向2cm
    assert abs(m['truth_rmse_cm'] - 2.0 / math.sqrt(2)) < 0.3
    assert abs(m['truth_end_dist_cm'] - 2.0) < 0.5
    # タグ座標系の絶対ゴール(1,1)からは (0.02,-0.02)ずれ≈2.8cm
    assert abs(m['truth_end_dist_abs_cm'] - 2.0 * math.sqrt(2)) < 1.0


def test_truth_metrics_invariant_to_start_pose():
    """開始向き±数°の残差（homing公差）が追従指標に混入しないこと。"""
    base = truth_metrics(_rows_along_path(offset=0.02), WPS)
    rot = truth_metrics(
        _transform(_rows_along_path(offset=0.02), 0.05, -0.03,
                   math.radians(4)), WPS)
    assert abs(rot['truth_rmse_cm'] - base['truth_rmse_cm']) < 0.2
    assert abs(rot['truth_end_dist_cm'] - base['truth_end_dist_cm']) < 0.5
    # 絶対距離は開始poseの情報を保持する＝不変で「ない」こと
    assert abs(rot['truth_end_dist_abs_cm'] -
               base['truth_end_dist_abs_cm']) > 0.5


def test_truth_metrics_ignores_missing_and_empty():
    rows = _rows_along_path()
    rows[10] = (5.0, None, None, None, 4, 0.0)   # 欠測行は無視
    assert 'truth_rmse_cm' in truth_metrics(rows, WPS)
    assert truth_metrics([(0.0, None, None, None, 0, 0.0)], WPS) == {}
