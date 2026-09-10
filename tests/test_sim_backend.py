import math

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


# ---- 外乱チャネル（S4: 計測ノイズ・初期横ずれ・プラント側の実効ゲイン） --------
# pos_noise/yaw_noise は「制御器に渡す観測姿勢」に載る計測外乱で、車体そのものは
# 無擾乱である。積載のようなプラント側の外乱は gain_v/gain_w（指令に対する実効
# ゲイン）で模す。真の追従誤差 terr は観測ノイズに汚れないので、計測RMSE（perr）
# との対比が実機の odom vs カメラ真値（6.6節）と同型になる。

from exp_backends import sim_run  # noqa: E402

WPS_1M, VR_1M = load_path("configs/path_L_turn_1m.yaml")
COMMON = dict(horizon=15, rate=10.0)


def drun(controller, seed=1, timeout_s=60, **sim_opts):
    cond = dict(name=controller, controller=controller)
    for k in ("lam", "move_suppress"):
        if k in sim_opts:
            cond[k] = sim_opts.pop(k)
    opts = dict(delay_steps=2, pos_noise=0.0, yaw_noise=0.0)
    opts.update(sim_opts)
    return sim_run(cond, COMMON, WPS_1M, VR_1M, opts, timeout_s, seed=seed)


def test_true_error_series_is_recorded():
    d = drun("l2")
    assert d["terr"], "真の追従誤差の時系列が無い"
    assert d["terr"][0][1] < 1e-9, "外乱なしなら開始時点は経路上"
    assert len(d["terr"]) == len(d["twist"])


def test_final_pose_is_recorded_and_near_goal():
    d = drun("l2")
    assert d["ok"]
    gx, gy = WPS_1M[-1]
    x, y, _ = d["final"]
    assert math.hypot(gx - x, gy - y) < 0.15


def test_init_lat_starts_off_path_and_controller_recovers():
    d = drun("l2", init_lat=0.20)
    assert abs(d["terr"][0][1] - 0.20) < 0.01, "初期横ずれが真値に載っていない"
    assert d["terr"][-1][1] < 0.05, "L2 が 20cm の初期横ずれに復帰できていない"


def test_plant_gain_acts_on_plant_only():
    """gain_w は車体の応答だけを弱める。制御器は指令を出し続ける。"""
    d = drun("l2", gain_w=0.0)
    assert abs(d["final"][2]) < 1e-9, "gain_w=0 なのに車体が回っている"
    assert max(abs(w) for _, _, w in d["twist"]) > 0.1, "指令自体は出ているはず"
    assert not d["ok"], "回頭できなければ L 字は曲がれない"


def test_gain_v_slows_travel():
    fast = drun("l2", gain_v=1.0)
    slow = drun("l2", gain_v=0.5)
    assert len(slow["twist"]) > len(fast["twist"])


def test_same_seed_reproduces_and_different_seed_differs():
    a = drun("l2", pos_noise=0.02, yaw_noise=0.02, seed=3)
    b = drun("l2", pos_noise=0.02, yaw_noise=0.02, seed=3)
    c = drun("l2", pos_noise=0.02, yaw_noise=0.02, seed=4)
    assert a["final"] == b["final"]
    assert a["final"] != c["final"]


def test_measurement_noise_does_not_touch_true_state_directly():
    """ノイズなしと同一seedでも、ノイズ有りは制御を介してのみ軌跡が変わる。"""
    clean = drun("l2")
    noisy = drun("l2", pos_noise=0.02)
    assert clean["terr"][0][1] == noisy["terr"][0][1] == 0.0
    assert clean["final"] != noisy["final"]
