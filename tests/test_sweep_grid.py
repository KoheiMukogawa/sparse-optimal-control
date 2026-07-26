# -*- coding: utf-8 -*-
"""λ × move_suppress の2次元スイープ（遅延下）。

論文の主軸は「遅延下での成立条件」なので、設計パラメータの選定も遅延を入れた
条件で行う必要がある（既存の sweep_lambda.py は遅延なし条件）。
遅延モデルは実機と定量一致した実績のある sim_delay_probe.simulate を使う。
"""

import csv

import pytest

from sweep_grid import sweep_grid, write_outputs


def fake_rows():
    """実測に近い2行（表5.9 の ms=0 と ms=2.0）。書き出しのみを試すため sim は回さない。"""
    return [
        dict(lam=0.3, move_suppress=0.0, ok=True, t=19.0, rmse=3.12,
             sum_u=18.49, flips=19, maxw=2.0, sat=0.36, zero=0.53),
        dict(lam=0.3, move_suppress=2.0, ok=True, t=19.5, rmse=1.90,
             sum_u=5.01, flips=1, maxw=2.0, sat=0.04, zero=0.89),
    ]


@pytest.fixture(scope="module")
def small_grid():
    # 遅延2step = 実機の約200ms 相当（results/2026-06-13_Lturn_compare.md C節）
    return sweep_grid(lams=[0.3], move_suppresses=[0.0, 2.0], delay_steps=2)


def test_grid_has_one_row_per_combination(small_grid):
    assert [(r["lam"], r["move_suppress"]) for r in small_grid] == [
        (0.3, 0.0),
        (0.3, 2.0),
    ]


def test_rows_carry_delay_sim_metrics(small_grid):
    assert small_grid, "行が1つも無い"
    for r in small_grid:
        for key in ("ok", "rmse", "sum_u", "flips", "sat", "zero"):
            assert key in r, f"{key} が行に無い"


def test_move_suppress_removes_chatter_under_delay(small_grid):
    """表5.9 の主張: 遅延2step下で ms=2.0 が反転を消しスパース性は保つ。"""
    naive = next(r for r in small_grid if r["move_suppress"] == 0.0)
    damped = next(r for r in small_grid if r["move_suppress"] == 2.0)
    assert naive["flips"] >= 10, "ナイーブL1は遅延下でチャタるはず"
    assert damped["flips"] <= 2, "move_suppress でチャタが消えるはず"
    assert damped["zero"] >= 0.85, "対策後もスパース性は保たれるはず"


def test_write_outputs_csv_has_one_data_row_per_combination(tmp_path):
    write_outputs(fake_rows(), str(tmp_path), delay_steps=2)
    with open(tmp_path / "grid.csv") as f:
        got = list(csv.DictReader(f))
    assert [(r["lam"], r["move_suppress"], r["flips"]) for r in got] == [
        ("0.3", "0", "19"),
        ("0.3", "2", "1"),
    ]


def test_write_outputs_table_records_delay_condition(tmp_path):
    """遅延条件が表に明記されないと、遅延なしスイープと取り違えられる。"""
    write_outputs(fake_rows(), str(tmp_path), delay_steps=2)
    md = (tmp_path / "table.md").read_text()
    assert "遅延2step" in md
    assert "0.89" in md or "89%" in md


def test_write_outputs_saves_figure_for_thesis(tmp_path):
    """図5.5（w_ms に対する反転・Σ|u|・ω0率・RMSE の変化）の元になる図。"""
    pytest.importorskip("matplotlib")
    write_outputs(fake_rows(), str(tmp_path), delay_steps=2)
    assert (tmp_path / "sweep_ms.png").exists()
