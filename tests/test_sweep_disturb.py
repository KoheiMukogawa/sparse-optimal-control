# -*- coding: utf-8 -*-
"""外乱注入スイープ（S4）。

6.7節の積載外乱バッチ（R1）が実機故障で取れないため、外乱を3チャネルに分けて
sim で振る。チャネルの意味は exp_backends.sim_run の docstring を参照。

ノイズありなら sim も確率的になるので、seed を振って平均±標準偏差で見る
（S1・S2 の教訓「条件を1点に固定して最適化すると見かけの最適を掴む」の対策）。
"""

import csv
import math

import pytest

from sweep_disturb import (CONDITIONS, Channel, aggregate, sweep_disturb,
                           write_outputs)

L2 = [c for c in CONDITIONS if c.name == "l2"]
L1 = [c for c in CONDITIONS if c.name == "l1"]

SMALL = Channel(key="meas", label="計測ノイズ",
                unit="σ_pos[cm]", levels=[0.0, 0.02],
                opts=lambda v: dict(pos_noise=v, yaw_noise=v * 1.75),
                display=lambda v: v * 100)


@pytest.fixture(scope="module")
def small_rows():
    return sweep_disturb(L2 + L1, [SMALL], seeds=[1, 2])


def test_one_row_per_condition_level_and_seed(small_rows):
    assert len(small_rows) == 2 * 2 * 2
    assert {(r["name"], r["level"], r["seed"]) for r in small_rows} == {
        (n, lv, s) for n in ("l2", "l1") for lv in (0.0, 0.02) for s in (1, 2)
    }


def test_row_carries_both_measured_and_true_rmse(small_rows):
    for r in small_rows:
        for key in ("rmse_meas_cm", "rmse_true_cm", "end_err_cm",
                    "end_yaw_err_deg", "sum_u", "flips", "ok"):
            assert key in r, f"{key} が行に無い"


def test_true_rmse_is_unaffected_by_measurement_noise_at_zero_level(small_rows):
    """外乱0のセルは決定論的（seed によらず同一）でなければならない。"""
    zero = [r for r in small_rows if r["level"] == 0.0 and r["name"] == "l2"]
    assert zero[0]["rmse_true_cm"] == pytest.approx(zero[1]["rmse_true_cm"])


def test_aggregate_reports_mean_sd_and_reach_rate():
    rows = [
        dict(channel="meas", name="l2", level=0.02, seed=1, ok=True,
             rmse_true_cm=2.0, rmse_meas_cm=3.0, end_err_cm=5.0,
             end_yaw_err_deg=4.0, sum_u=4.0, flips=0, w_zero_ratio=0.8,
             sat_ratio=0.0, max_w=1.0, solve_p50=20.0, solve_p95=30.0),
        dict(channel="meas", name="l2", level=0.02, seed=2, ok=False,
             rmse_true_cm=4.0, rmse_meas_cm=5.0, end_err_cm=7.0,
             end_yaw_err_deg=6.0, sum_u=6.0, flips=2, w_zero_ratio=0.6,
             sat_ratio=0.1, max_w=2.0, solve_p50=22.0, solve_p95=34.0),
    ]
    agg = aggregate(rows)
    assert len(agg) == 1
    a = agg[0]
    assert a["n"] == 2
    assert a["reach_rate"] == pytest.approx(0.5)
    assert a["rmse_true_cm_mean"] == pytest.approx(3.0)
    # 2.0 と 4.0 の標本標準偏差（ddof=1）は √2
    assert a["rmse_true_cm_sd"] == pytest.approx(math.sqrt(2.0))


def test_write_outputs_writes_runs_csv_and_table(tmp_path, small_rows):
    write_outputs(small_rows, str(tmp_path), plot=False)
    with open(tmp_path / "runs.csv") as f:
        got = list(csv.DictReader(f))
    assert len(got) == len(small_rows)
    assert {"name", "channel", "level", "seed", "rmse_true_cm"} <= set(got[0])
    table = (tmp_path / "table.md").read_text()
    assert "計測ノイズ" in table
    assert "真値RMSE" in table
