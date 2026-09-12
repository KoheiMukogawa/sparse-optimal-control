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


@pytest.mark.parametrize("value", [-1, 1.5, True])
def test_load_batch_rejects_invalid_command_delay(tmp_path, value):
    f = tmp_path / "b.yaml"
    f.write_text(BATCH_YAML.replace(
        "controller: l2", f"controller: l2, cmd_delay_steps: {value}"))
    with pytest.raises(ValueError, match="cmd_delay_steps"):
        load_batch(str(f))


def test_load_batch_rejects_command_delay_for_kanayama(tmp_path):
    f = tmp_path / "b.yaml"
    f.write_text(BATCH_YAML.replace(
        "controller: l2", "controller: kanayama, cmd_delay_steps: 1"))
    with pytest.raises(ValueError, match="Kanayama"):
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


def test_make_row_records_command_delay():
    batch = dict(name="mini", common=dict(horizon=15))
    cond = dict(name="delayed", controller="l1", lam=0.3,
                cmd_delay_steps=3)
    result = dict(ok=True, metrics={}, bagdir="", note="")
    row = make_row(batch, cond, 1, "sim", result, "abc123", v_r=0.1)
    assert "cmd_delay_steps" in CSV_COLUMNS
    assert row["cmd_delay_steps"] == 3


def test_make_row_defaults_command_delay_to_zero():
    batch = dict(name="mini", common={})
    cond = dict(name="plain", controller="l2")
    result = dict(ok=True, metrics={}, bagdir="", note="")
    row = make_row(batch, cond, 1, "sim", result, "abc123", v_r=0.1)
    assert row["cmd_delay_steps"] == 0


@pytest.mark.parametrize("name", ["load500", "load1000"])
def test_university_load_configs_show_manual_command(name):
    from pathlib import Path
    text = Path(f"configs/batch_Lturn1m_{name}.yaml").read_text()
    assert "--backend real --manual" in text
    assert "--backend real --auto" not in text


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


def test_write_summary_nan_solve(tmp_path):
    """kanayama行のsolve_*はnan → クラッシュせず n/a になる（回帰テスト）。"""
    from run_batch import write_summary
    csv_path = tmp_path / "runs.csv"
    for rep, p95 in [(1, float("nan")), (2, float("nan")), (3, "")]:
        row = {c: "" for c in CSV_COLUMNS}
        row.update(batch="mini", cond="kanayama", rep=rep, ok=True,
                   rmse_cm=11.2, sum_u=4.8, flips=0,
                   w_zero_ratio=0.65, solve_p95=p95)
        append_row(str(csv_path), row)
    write_summary(str(tmp_path))  # nan混在でも例外を出さない
    text = (tmp_path / "summary.md").read_text()
    line = next(ln for ln in text.splitlines() if ln.startswith("| kanayama"))
    assert "n/a" in line          # solve_p95 は全て nan/空 → n/a
    assert "11.20" in line        # 他の列は通常どおり平均される
    assert "3/3" in line
