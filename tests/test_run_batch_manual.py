# -*- coding: utf-8 -*-
"""--manual（方眼直読の手入力）モード。

大学ではカメラ俯瞰を置けないため --auto が使えない。走行後に方眼の読み値を
入力して truth_* 列を埋める経路のテスト。
設計: specs/2026-09-12-大学実験環境への移行-design.md
"""

import csv
import math

import pytest

from run_batch import (CSV_COLUMNS, needs_manual_reading,
                       format_truth_metrics, read_manual_metrics,
                       write_manual_readings)

WPS = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]


def feeder(lines):
    """input_fn の差し替え: 与えた行を順に返す。"""
    it = iter(lines)
    return lambda prompt: next(it)


def test_reads_three_prompts_in_order():
    metrics, raw = read_manual_metrics(
        feeder(["0 0 0", "100 100 90", "0 0"]), WPS)
    assert raw["start_pose_cm"] == [0.0, 0.0, 0.0]
    assert raw["end_pose_cm"] == [100.0, 100.0, 90.0]
    assert raw["devs_cm"] == [0.0, 0.0]
    assert metrics["truth_rmse_cm"] == pytest.approx(0.0)


def test_blank_start_offset_means_zero():
    _, raw = read_manual_metrics(feeder(["", "100 100 90", "1 -1"]), WPS)
    assert raw["start_pose_cm"] == [0.0, 0.0, 0.0]


def test_commas_are_accepted_as_separators():
    _, raw = read_manual_metrics(
        feeder(["0,0,0", "93, 97, 4", "1.5, -2.0"]), WPS)
    assert raw["end_pose_cm"] == [93.0, 97.0, 4.0]
    assert raw["devs_cm"] == [1.5, -2.0]


def test_blank_deviations_mean_endpoint_only():
    metrics, raw = read_manual_metrics(
        feeder(["0 0 0", "93 97 4", ""]), WPS)
    assert raw["devs_cm"] == []
    assert math.isnan(metrics["truth_rmse_cm"])
    assert metrics["truth_end_x"] == pytest.approx(0.93)


def test_missing_manual_rmse_is_blank_in_runs_csv():
    formatted = format_truth_metrics({
        "truth_end_dist_cm": 5.25,
        "truth_rmse_cm": float("nan"),
    })
    assert formatted["truth_end_dist_cm"] == "5.2500"
    assert formatted["truth_rmse_cm"] == ""


def test_q_aborts():
    with pytest.raises(KeyboardInterrupt):
        read_manual_metrics(feeder(["q"]), WPS)


def test_bad_input_is_reprompted():
    """数値でない行は読み直す（実験中の打ち間違いでバッチを落とさない）。"""
    _, raw = read_manual_metrics(
        feeder(["0 0 0", "abc", "100 100 90", "0 0"]), WPS)
    assert raw["end_pose_cm"] == [100.0, 100.0, 90.0]


def test_end_pose_needs_three_numbers():
    _, raw = read_manual_metrics(
        feeder(["0 0 0", "100 100", "100 100 90", "0 0"]), WPS)
    assert raw["end_pose_cm"] == [100.0, 100.0, 90.0]


def test_readings_are_saved_for_reanalysis(tmp_path):
    raw = {"devs_cm": [1.0, -2.0], "end_pose_cm": [93.0, 97.0, 4.0],
           "start_pose_cm": [0.5, 0.0, 1.0]}
    p = write_manual_readings(str(tmp_path), "l2", 1, raw)
    assert p.name == "manual_l2_r1.csv"
    vals = {r["key"]: float(r["value"]) for r in csv.DictReader(p.open())}
    assert vals["dev_1_cm"] == 1.0
    assert vals["dev_2_cm"] == -2.0
    assert vals["end_x_cm"] == 93.0
    assert vals["start_dtheta_deg"] == 1.0


def test_endpoint_only_readings_are_saved_without_fake_deviations(tmp_path):
    raw = {"devs_cm": [], "end_pose_cm": [93.0, 97.0, 4.0],
           "start_pose_cm": [0.5, 0.0, 1.0]}
    p = write_manual_readings(str(tmp_path), "l2", 2, raw)
    rows = list(csv.DictReader(p.open()))
    assert not any(r["key"].startswith("dev_") for r in rows)
    assert {r["key"] for r in rows} == {
        "start_dx_cm", "start_dy_cm", "start_dtheta_deg",
        "end_x_cm", "end_y_cm", "end_theta_deg",
    }


def test_start_offset_columns_exist():
    for c in ("start_dx_cm", "start_dy_cm", "start_dtheta_deg"):
        assert c in CSV_COLUMNS


def test_manual_reading_skipped_when_run_failed():
    """走行失敗（ok=False）では手計測を求めない（再実行で読み値は破棄されるため）。"""
    assert needs_manual_reading({"ok": False, "metrics": {}}) is False


def test_manual_reading_requested_when_run_succeeded():
    assert needs_manual_reading({"ok": True, "metrics": {}}) is True
