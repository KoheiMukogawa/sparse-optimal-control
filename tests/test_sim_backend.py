from exp_backends import SimBackend, load_path

BATCH = dict(
    name="t", path_file="configs/path_L_turn.yaml",
    repeats=1, timeout_s=120,
    common=dict(horizon=15, rate=10.0),
    sim=dict(delay_steps=2, pos_noise=0.0, yaw_noise=0.0),
)


def run(controller, **kw):
    cond = dict(name=controller, controller=controller, **kw)
    return SimBackend(BATCH).run_one(cond, rep=1, outdir=None)


def test_load_path():
    wps, v_r = load_path("configs/path_L_turn.yaml")
    assert len(wps) >= 2 and v_r > 0


def test_kanayama_reaches():
    r = run("kanayama")
    assert r["ok"]
    assert r["metrics"]["flips"] == 0


def test_l2_robust_to_delay():
    r = run("l2")
    assert r["ok"]
    assert r["metrics"]["flips"] == 0
    assert r["metrics"]["solve_p50"] > 0  # 求解時間が記録される


def test_l1_chatters_under_delay():
    r = run("l1", lam=0.3)
    assert r["metrics"]["flips"] >= 8  # sim_delay_probe実績: 19回


def test_l1_move_suppress_fixes_chatter():
    r = run("l1", lam=0.3, move_suppress=2.0)
    assert r["ok"]
    assert r["metrics"]["flips"] <= 3  # sim_delay_probe実績: 1回


def test_empty_run_returns_record(tmp_path):
    # スタート地点がほぼゴール → 1ステップも走らず終了。例外でなくrecordを返すこと
    p = tmp_path / "path_tiny.yaml"
    p.write_text("waypoints: [[0.0, 0.0], [0.001, 0.0]]\nv_r: 0.1\n")
    batch = dict(BATCH, path_file=str(p))
    r = SimBackend(batch).run_one(dict(name="l2", controller="l2"), 1, None)
    assert r["ok"] is True
    assert r["metrics"] == {}
    assert "走行データなし" in r["note"]
