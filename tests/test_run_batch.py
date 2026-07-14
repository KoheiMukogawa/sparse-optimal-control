import pytest

from run_batch import (CSV_COLUMNS, append_row, done_keys, load_batch,
                       make_row)

BATCH_YAML = """\
name: mini
path_file: configs/path_L_turn.yaml
conditions:
  - {name: l2, controller: l2}
  - {name: l1, controller: l1, lam: 0.3}
"""


def test_load_batch_defaults(tmp_path):
    f = tmp_path / "b.yaml"
    f.write_text(BATCH_YAML)
    b = load_batch(str(f))
    assert b["name"] == "mini"
    assert b["repeats"] == 1          # 既定値
    assert b["timeout_s"] == 60       # 既定値
    assert b["backend"] == "sim"      # 既定値
    assert len(b["conditions"]) == 2


def test_load_batch_rejects_bad_controller(tmp_path):
    f = tmp_path / "b.yaml"
    f.write_text(BATCH_YAML.replace("controller: l2", "controller: lqr"))
    with pytest.raises(ValueError):
        load_batch(str(f))


def test_append_and_resume(tmp_path):
    csv_path = tmp_path / "runs.csv"
    row = {c: "" for c in CSV_COLUMNS}
    row.update(cond="l2", rep=1, ok=True, rmse_cm=1.5)
    append_row(str(csv_path), row)
    row2 = dict(row, rep=2, ok=False)
    append_row(str(csv_path), row2)
    keys = done_keys(str(csv_path))
    assert ("l2", 1) in keys          # ok=true → スキップ対象
    assert ("l2", 2) not in keys      # 失敗 → 再走対象


def test_done_keys_missing_file(tmp_path):
    assert done_keys(str(tmp_path / "nai.csv")) == set()


def test_make_row():
    batch = dict(name="mini", common=dict(horizon=15))
    cond = dict(name="l1", controller="l1", lam=0.3)
    result = dict(ok=True, metrics=dict(rmse_cm=2.0, flips=1),
                  bagdir="", note="")
    row = make_row(batch, cond, 1, "sim", result, "abc123", v_r=0.1)
    assert row["cond"] == "l1" and row["lam"] == 0.3
    assert row["ok"] is True and row["rmse_cm"] == 2.0
    assert row["git_hash"] == "abc123" and row["v_r"] == 0.1
    assert set(row) == set(CSV_COLUMNS)


def test_write_summary(tmp_path):
    from run_batch import write_summary
    csv_path = tmp_path / "runs.csv"
    for rep, rmse, flips in [(1, 2.0, 0), (2, 3.0, 2)]:
        row = {c: "" for c in CSV_COLUMNS}
        row.update(batch="mini", cond="l1", rep=rep, ok=True,
                   rmse_cm=rmse, sum_u=5.0, flips=flips,
                   w_zero_ratio=0.9, solve_p95=50.0)
        append_row(str(csv_path), row)
    write_summary(str(tmp_path))
    text = (tmp_path / "summary.md").read_text()
    assert "l1" in text
    assert "2.50" in text       # rmse平均 (2.0+3.0)/2
    assert "±" in text          # 標準偏差表記
    assert "2/2" in text        # 到達率
