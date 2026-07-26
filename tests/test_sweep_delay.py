# -*- coding: utf-8 -*-
"""遅延ステップ数に対する各制御器の耐性スイープ（S2）。

既存 sim_delay_probe は 0/1/2 step × {L2, L1} を print するだけだった。
「対策が何ステップの遅延まで耐えるか」を条件×遅延の表として残す。
"""

import csv

import pytest

from sweep_delay import Condition, sweep_delay, write_outputs

L2 = Condition(name="l2", reg="l2", lam=1.0, move_suppress=0.0)
L1 = Condition(name="l1", reg="l1", lam=0.3, move_suppress=0.0)


@pytest.fixture(scope="module")
def small_sweep():
    return sweep_delay([L2, L1], delay_steps_list=[0, 2])


def test_one_row_per_condition_and_delay(small_sweep):
    assert [(r["name"], r["delay_steps"]) for r in small_sweep] == [
        ("l2", 0), ("l2", 2), ("l1", 0), ("l1", 2),
    ]


def test_rows_carry_metrics(small_sweep):
    assert small_sweep, "行が1つも無い"
    for r in small_sweep:
        for key in ("ok", "rmse", "sum_u", "flips", "sat", "zero"):
            assert key in r, f"{key} が行に無い"


def test_l2_is_delay_robust_but_naive_l1_is_not(small_sweep):
    """5.6節の主張: 遅延で壊れるのは L1 だけで、L2 は反転0のまま。"""
    by = {(r["name"], r["delay_steps"]): r for r in small_sweep}
    assert by[("l1", 0)]["flips"] <= 2, "理想simでは L1 もチャタらない"
    assert by[("l1", 2)]["flips"] >= 10, "遅延2step で L1 はチャタるはず"
    assert by[("l2", 0)]["flips"] == 0
    assert by[("l2", 2)]["flips"] == 0, "L2 は遅延下でも反転0のはず"


def test_write_outputs_records_condition_and_delay(tmp_path):
    rows = [
        dict(name="l2", reg="l2", lam=1.0, move_suppress=0.0, delay_steps=2,
             ok=True, t=19.0, rmse=1.5, sum_u=4.54, flips=0, maxw=1.2,
             sat=0.0, zero=0.85),
        dict(name="l1", reg="l1", lam=0.3, move_suppress=0.0, delay_steps=2,
             ok=True, t=19.0, rmse=3.12, sum_u=18.49, flips=19, maxw=2.0,
             sat=0.36, zero=0.53),
    ]
    write_outputs(rows, str(tmp_path))
    with open(tmp_path / "delay.csv") as f:
        got = list(csv.DictReader(f))
    assert [(r["name"], r["delay_steps"], r["flips"]) for r in got] == [
        ("l2", "2", "0"), ("l1", "2", "19"),
    ]
    assert "遅延" in (tmp_path / "table.md").read_text()
