"""runs.csv のOSQP反復数を卒論集計へ渡す回帰テスト。"""

import csv

from thesis_data import NUMERIC_FIELDS, load_runs


def test_solver_iteration_fields_are_numeric_and_loadable(tmp_path):
    for key in ("iters_p50", "iters_p95", "iters_max"):
        assert key in NUMERIC_FIELDS

    path = tmp_path / "runs.csv"
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=["cond", "rep", "ok", "iters_p50",
                           "iters_p95", "iters_max"])
        writer.writeheader()
        writer.writerow(dict(cond="l2", rep=1, ok=True,
                             iters_p50="25", iters_p95="50",
                             iters_max="75"))

    row = load_runs(path)[0]
    assert row["iters_p50"] == 25.0
    assert row["iters_p95"] == 50.0
    assert row["iters_max"] == 75.0
