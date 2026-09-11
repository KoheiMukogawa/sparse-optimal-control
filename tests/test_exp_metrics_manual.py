# -*- coding: utf-8 -*-
"""方眼直読の手入力値 → 真値指標（カメラが使えない大学環境用）。

設計: specs/2026-09-12-大学実験環境への移行-design.md
コースが方眼の格子線と一致するため、横偏差は「格子線から何マスずれたか」を
読むだけで得られる。ここはその読み値を指標に変換する純関数のテスト。
"""

import math

import pytest

from exp_metrics import manual_metrics, truth_metrics

WPS = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]   # configs/path_L_turn_1m.yaml


def test_perfect_run_has_zero_error():
    m = manual_metrics([0.0] * 20, (100.0, 100.0, 90.0), (0.0, 0.0, 0.0), WPS)
    assert m["truth_rmse_cm"] == pytest.approx(0.0)
    assert m["truth_end_dist_cm"] == pytest.approx(0.0)
    assert m["truth_end_dist_abs_cm"] == pytest.approx(0.0)


def test_rmse_is_rms_of_the_readings():
    m = manual_metrics([3.0, -4.0], (100.0, 100.0, 90.0), (0.0, 0.0, 0.0), WPS)
    assert m["truth_rmse_cm"] == pytest.approx(math.sqrt(12.5))


def test_sign_does_not_change_rmse():
    a = manual_metrics([3.0, -4.0], (100.0, 100.0, 90.0), (0, 0, 0), WPS)
    b = manual_metrics([-3.0, 4.0], (100.0, 100.0, 90.0), (0, 0, 0), WPS)
    assert a["truth_rmse_cm"] == pytest.approx(b["truth_rmse_cm"])


def test_end_pose_is_returned_in_camera_units():
    """truth_end_x/y は [m]、theta は [rad]（カメラ版と同単位）。"""
    m = manual_metrics([0.0], (93.0, 97.0, 4.0), (0, 0, 0), WPS)
    assert m["truth_end_x"] == pytest.approx(0.93)
    assert m["truth_end_y"] == pytest.approx(0.97)
    assert m["truth_end_theta"] == pytest.approx(math.radians(4.0))


def test_end_distance_uses_measured_start_pose():
    """開始姿勢がずれていれば、コースごと回してゴールを取り直す。

    カメラ版 truth_metrics と同じ契約（開始pose基準の追従誤差と、
    紙面に描いたコース基準の絶対距離を別掲する）。
    """
    # 開始で +90度ずれて置いた → コースのゴール (1,1) は (-1,1) へ回る
    m = manual_metrics([0.0], (-100.0, 100.0, 180.0), (0.0, 0.0, 90.0), WPS)
    assert m["truth_end_dist_cm"] == pytest.approx(0.0, abs=1e-6)
    # 紙面に描いたゴール (1,1) からは 200cm 離れている
    assert m["truth_end_dist_abs_cm"] == pytest.approx(200.0)


def test_start_offset_is_recorded_as_is():
    m = manual_metrics([0.0], (100.0, 100.0, 90.0), (1.5, -2.0, 0.8), WPS)
    assert m["start_dx_cm"] == pytest.approx(1.5)
    assert m["start_dy_cm"] == pytest.approx(-2.0)
    assert m["start_dtheta_deg"] == pytest.approx(0.8)


def test_empty_readings_is_an_error():
    with pytest.raises(ValueError):
        manual_metrics([], (100.0, 100.0, 90.0), (0, 0, 0), WPS)


def test_keys_match_the_camera_version():
    """runs.csv のスキーマを分岐させないため、カメラ版のキーを全て含むこと。"""
    rows = [(0.0, 0.0, 0.0, 0.0, 4, 0.2), (1.0, 1.0, 1.0, 1.57, 4, 0.2)]
    cam = set(truth_metrics(rows, WPS))
    man = set(manual_metrics([0.0], (100.0, 100.0, 90.0), (0, 0, 0), WPS))
    assert cam <= man
