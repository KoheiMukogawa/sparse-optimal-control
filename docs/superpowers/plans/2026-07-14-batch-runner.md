# バッチ実験ランナー Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 条件YAML 1ファイルから実験（sim/実機）を反復実行し、`runs.csv` と `summary.md` を自動生成するバッチランナーを作る。

**Architecture:** ラップトップで動く条件ループ（`run_batch.py`）が、バックエンド（`SimBackend`=ROS不要の閉ループsim / `RealBackend`=SSHでRPiのノード起動・bag記録・回収）を差し替えて1本ずつ実行し、共通の指標計算（`exp_metrics.py`）でCSVに追記する。詳細スペック: `docs/superpowers/specs/2026-07-14-batch-runner-design.md`。

**Tech Stack:** Python 3.12+ / uv / pytest（devのみ新規）/ 既存: cvxpy, rosbags, PyYAML(ROS由来でなくpip yaml。既にrover各所でimport実績あり)

## Global Constraints

- 実行系の新規依存は追加しない。dev依存に `pytest` のみ追加（`uv add --dev pytest`）。
- 実機ノード（`rover/path_follower.py`, `rover/mpc_follower.py`）と `rover/mpc_core.py`, `rover/follower_core.py` は**一切変更しない**（実機検証済みのため）。
- 新規コードは既存の rover/ フラット構成・日本語docstring・snake_caseに合わせる。
- テストはROS環境なしで `uv run pytest` で全部通ること。
- コミットメッセージは日本語で簡潔に（本文末に `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`）。
- 安全系: 実機の停止はノード既存機構に委ねる。ランナーが送るのは SIGINT のみ。
- 実機を実際に動かすステップは本計画に**含まれない**（RealBackendは dry-run 検証まで。実機統合テストは別セッションでユーザー立ち会い）。

---

### Task 1: pytest基盤 + `rover/exp_metrics.py`（bag/sim共通の指標計算）

**Files:**
- Modify: `pyproject.toml`（uvコマンド経由）
- Create: `tests/conftest.py`
- Create: `tests/test_exp_metrics.py`
- Create: `rover/exp_metrics.py`

**Interfaces:**
- Produces: `exp_metrics.compute_metrics(twist, perr=(), solve_ms=()) -> dict`
  - `twist=[(t_s, v, w)]`, `perr=[(t_s, y_e)]`, `solve_ms=[float(ms)]`
  - 返り値キー: `drive_s, steps, rmse_cm, sum_u, w_zero_ratio, flips, sat_ratio, max_w, solve_p50, solve_p95, solve_max`
  - `twist` が空なら `ValueError`。走行区間＝|v|>V_ACTIVE のサンプル列（analyze_bag.py と同一ロジック）。
  - 定数: `W_ZERO=0.05, V_ACTIVE=0.005, W_SAT=1.5`（analyze_bag.py / sim_delay_probe.py と同値）

- [ ] **Step 1: pytest を dev 依存に追加**

Run: `uv add --dev pytest`
Expected: pyproject に `[dependency-groups] dev = ["pytest>=..."]` が入り uv.lock 更新。

- [ ] **Step 2: conftest（rover/ をimportパスに追加）**

`tests/conftest.py`:
```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "rover"))
```

- [ ] **Step 3: 失敗するテストを書く**

`tests/test_exp_metrics.py`:
```python
import math

import pytest

from exp_metrics import compute_metrics


def make_series():
    # 10サンプル・0.1s刻み・全区間 v=0.1（走行中）。
    # ω: ゼロ7 / +2,-2,+2（符号反転2回・全て飽和|ω|>1.5）
    ws = [0.0, 0.0, 2.0, -2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    twist = [(0.1 * i, 0.1, ws[i]) for i in range(10)]
    perr = [(0.1 * i, 0.03) for i in range(10)]      # 常に3cm
    solve_ms = [10.0, 20.0, 30.0]
    return twist, perr, solve_ms


def test_basic_metrics():
    twist, perr, solve_ms = make_series()
    m = compute_metrics(twist, perr, solve_ms)
    assert m["steps"] == 10
    assert m["drive_s"] == pytest.approx(0.9)
    # dt=0.9/9=0.1, Σ(|v|+|ω|)·dt = (10*0.1 + 6.0)*0.1
    assert m["sum_u"] == pytest.approx(0.7)
    assert m["w_zero_ratio"] == pytest.approx(0.7)
    assert m["flips"] == 2
    assert m["sat_ratio"] == pytest.approx(0.3)
    assert m["max_w"] == pytest.approx(2.0)
    assert m["rmse_cm"] == pytest.approx(3.0)
    assert m["solve_p50"] == pytest.approx(20.0)
    assert m["solve_max"] == pytest.approx(30.0)


def test_no_perr_no_solve():
    twist, _, _ = make_series()
    m = compute_metrics(twist)
    assert math.isnan(m["rmse_cm"])
    assert math.isnan(m["solve_p50"])


def test_empty_twist_raises():
    with pytest.raises(ValueError):
        compute_metrics([])
```

- [ ] **Step 4: 失敗を確認**

Run: `uv run pytest tests/test_exp_metrics.py -v`
Expected: FAIL（`ModuleNotFoundError: exp_metrics`）

- [ ] **Step 5: 実装**

`rover/exp_metrics.py`:
```python
# -*- coding: utf-8 -*-
"""時系列 → 評価指標（bag/sim共通）。

analyze_bag.py（bag由来）と exp_backends.SimBackend（sim由来）の両方から使う。
指標は卒論4指標（RMSE・Σ|u|・スパース性・計算負荷）＋チャタ診断
（ω符号反転・飽和率。Lturn_compare.md B節と同定義）。
"""

import math

W_ZERO = 0.05    # |ω|<これ をゼロ操舵とみなす [rad/s]
V_ACTIVE = 0.005  # |v|>これ を「走行中」とみなす [m/s]
W_SAT = 1.5      # |ω|>これ を飽和側とみなす [rad/s]（上限2.0の75%）


def _pct(sorted_vals, p):
    if not sorted_vals:
        return float('nan')
    return sorted_vals[min(len(sorted_vals) - 1, int(p * len(sorted_vals)))]


def compute_metrics(twist, perr=(), solve_ms=()):
    """時系列から指標dictを返す。

    twist   : [(t_s, v, w)] 適用された速度指令
    perr    : [(t_s, y_e)]  横偏差（odom基準）
    solve_ms: [ms]          求解時間（Kanayamaは空でよい）
    """
    if not twist:
        raise ValueError("twist が空（走行データなし）")

    # 走行区間 = |v|>V_ACTIVE（最初の発進〜最後の駆動）: analyze_bag.py と同一
    active = [(t, v, w) for t, v, w in twist if abs(v) > V_ACTIVE]
    if not active:
        active = list(twist)
    t0, t1 = active[0][0], active[-1][0]
    dt = (t1 - t0) / max(1, len(active) - 1)

    sum_u = sum((abs(v) + abs(w)) for _, v, w in active) * dt
    ws = [w for _, _, w in active]
    w_zero = sum(1 for w in ws if abs(w) < W_ZERO) / len(ws)
    sat = sum(1 for w in ws if abs(w) > W_SAT) / len(ws)

    # ω符号反転（デッドバンドW_ZERO）: sim_delay_probe.py と同一
    flips, prev = 0, 0
    for w in ws:
        s = 1 if w > W_ZERO else (-1 if w < -W_ZERO else 0)
        if s != 0:
            if prev != 0 and s != prev:
                flips += 1
            prev = s

    ye = [y for t, y in perr if t0 - 0.2 <= t <= t1 + 0.2]
    rmse_cm = (100.0 * math.sqrt(sum(y * y for y in ye) / len(ye))
               if ye else float('nan'))

    sv = sorted(solve_ms)
    return dict(
        drive_s=t1 - t0,
        steps=len(active),
        rmse_cm=rmse_cm,
        sum_u=sum_u,
        w_zero_ratio=w_zero,
        flips=flips,
        sat_ratio=sat,
        max_w=max(abs(w) for w in ws),
        solve_p50=_pct(sv, 0.50),
        solve_p95=_pct(sv, 0.95),
        solve_max=sv[-1] if sv else float('nan'),
    )
```

- [ ] **Step 6: テストが通ることを確認**

Run: `uv run pytest tests/test_exp_metrics.py -v`
Expected: 3 passed

- [ ] **Step 7: コミット**

```bash
git add pyproject.toml uv.lock tests/ rover/exp_metrics.py
git commit -m "指標計算をexp_metricsに共通化（bag/sim両対応・チャタ指標統合、pytest導入）"
```

---

### Task 2: `analyze_bag.py` を exp_metrics 利用に改修＋既存bagで回帰確認

**Files:**
- Modify: `rover/analyze_bag.py`
- Create: `tests/test_analyze_bag.py`

**Interfaces:**
- Consumes: `exp_metrics.compute_metrics`
- Produces: `analyze_bag.read_bag(bagdir) -> (twist, perr, solve_ms)`（RealBackendが使う）、
  `analyze_bag.analyze(bagdir) -> dict`（`name` キー＋compute_metricsの全キー）

- [ ] **Step 1: 回帰テストを書く（既存bag＝リポジトリ内の実機データを固定値と照合）**

期待値は `results/2026-06-13_Lturn_compare.md` A/B節の実測値（コミット済み）。

`tests/test_analyze_bag.py`:
```python
from pathlib import Path

import pytest

from analyze_bag import analyze

REPO = Path(__file__).resolve().parent.parent
LTURN_L1 = REPO / "results" / "2026-06-13_Lturn_l1"
LTURN_L2 = REPO / "results" / "2026-06-13_Lturn_l2"


@pytest.mark.skipif(not LTURN_L1.exists(), reason="実機bagなし")
def test_regression_lturn_l1():
    """Lturn_compare.md A/B節の記録値と一致（改修による数値変化がないこと）。"""
    m = analyze(str(LTURN_L1))
    assert m["rmse_cm"] == pytest.approx(4.16, abs=0.05)
    assert m["sum_u"] == pytest.approx(20.71, abs=0.05)
    assert m["w_zero_ratio"] == pytest.approx(0.50, abs=0.01)
    assert m["solve_p50"] == pytest.approx(42.9, abs=0.5)
    assert m["solve_p95"] == pytest.approx(89.7, abs=0.5)
    # チャタ指標（plot_lturn.py 記録値: 反転15回・飽和41%）
    assert 12 <= m["flips"] <= 18
    assert m["sat_ratio"] == pytest.approx(0.41, abs=0.03)


@pytest.mark.skipif(not LTURN_L2.exists(), reason="実機bagなし")
def test_regression_lturn_l2():
    m = analyze(str(LTURN_L2))
    assert m["rmse_cm"] == pytest.approx(1.42, abs=0.05)
    assert m["sum_u"] == pytest.approx(4.54, abs=0.05)
    assert m["flips"] == 0
```

- [ ] **Step 2: 失敗を確認**

Run: `uv run pytest tests/test_analyze_bag.py -v`
Expected: FAIL（`ImportError: cannot import name 'read_bag'`… analyze はあるが flips キーが無く assert で落ちる。いずれの失敗でも可）

- [ ] **Step 3: analyze_bag.py を改修**

`rover/analyze_bag.py` の `analyze()` を read_bag＋compute_metrics に分割し、
出力に 反転/飽和 列を追加（docstring・注記・main は既存のまま維持）:

```python
# analyze() を以下に置き換え、exp_metrics を import
from exp_metrics import compute_metrics


def read_bag(bagdir):
    """bag → 時系列 (twist, perr, solve_ms)。compute_metrics に渡す形式。"""
    twist, perr, solve = [], [], []
    with AnyReader([Path(bagdir)], default_typestore=TYPESTORE) as reader:
        for conn, ts, raw in reader.messages():
            t = ts * 1e-9
            if conn.topic == '/rover_twist':
                m = reader.deserialize(raw, conn.msgtype)
                twist.append((t, m.linear.x, m.angular.z))
            elif conn.topic == '/path_error':
                m = reader.deserialize(raw, conn.msgtype)
                perr.append((t, m.y))
            elif conn.topic == '/mpc_solve_ms':
                m = reader.deserialize(raw, conn.msgtype)
                solve.append(float(m.data))
    return twist, perr, solve


def analyze(bagdir):
    twist, perr, solve = read_bag(bagdir)
    if not twist:
        raise SystemExit(f"{bagdir}: /rover_twist が空")
    return dict(name=Path(bagdir).name, **compute_metrics(twist, perr, solve))
```

旧 `analyze()` 内の指標計算コード（W_ZERO/V_ACTIVE 定数含む）は削除。
`main()` の表出力に2列追加:

```python
    hdr = (f"{'bag':<26} {'走行s':>6} {'RMSE_cm':>8} {'Σ|u|':>7} "
           f"{'ω0率':>6} {'反転':>4} {'飽和':>5} {'解p50':>6} {'解p95':>6} {'解max':>6}")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(f"{r['name']:<26} {r['drive_s']:>6.1f} {r['rmse_cm']:>8.2f} "
              f"{r['sum_u']:>7.2f} {r['w_zero_ratio']*100:>5.0f}% "
              f"{r['flips']:>4d} {r['sat_ratio']*100:>4.0f}% "
              f"{r['solve_p50']:>5.1f} {r['solve_p95']:>5.1f} {r['solve_max']:>5.1f}")
```

- [ ] **Step 4: テスト＋全6bagのCLI出力を目視確認**

Run: `uv run pytest tests/test_analyze_bag.py -v`
Expected: 2 passed

Run: `uv run python rover/analyze_bag.py results/2026-06-13_floor_kanayama results/2026-06-13_floor_l2 results/2026-06-13_floor_l1 results/2026-06-13_Lturn_kanayama results/2026-06-13_Lturn_l2 results/2026-06-13_Lturn_l1`
Expected: floor_compare.md / Lturn_compare.md の表と同じ RMSE/Σ|u|/ω0率/解p50/p95（±丸め誤差）。

- [ ] **Step 5: コミット**

```bash
git add rover/analyze_bag.py tests/test_analyze_bag.py
git commit -m "analyze_bagをexp_metrics利用に改修（反転・飽和列を追加、既存bagで回帰テスト）"
```

---

### Task 3: `rover/exp_backends.py` — SimBackend（ROS不要の閉ループsim）

**Files:**
- Create: `rover/exp_backends.py`（このタスクではsim部分のみ。RealBackendはTask 6で同ファイルに追記）
- Create: `tests/test_sim_backend.py`

**Interfaces:**
- Consumes: `exp_metrics.compute_metrics`、`follower_core`（kanayama_cmd, clamp, goal_crossed, goal_scaled_vr, reference_pose, tracking_error）、`mpc_core.MPCFollower(N, ts, reg, lam, v_max, w_max, move_suppress)` / `.command(x_e,y_e,th_e,v_r,w_r) -> (v,w)|None` / `.last_solve_s`
- Produces:
  - `exp_backends.load_path(path_file) -> (waypoints, v_r)`（configs/path_*.yaml を読む）
  - `exp_backends.SimBackend(batch)` — `batch` は Task 4 の load_batch が返す dict
  - `SimBackend.run_one(cond, rep, outdir) -> dict(ok, metrics, bagdir, note)`
  - `cond` スキーマ: `{name, controller: 'kanayama'|'l2'|'l1', lam?, move_suppress?}`

- [ ] **Step 1: 失敗するテストを書く**

期待値は sim_delay_probe.py の確立済み結果（遅延2step: L2反転0 / L1チャタ / L1+ms=2.0でチャタ消失）。

`tests/test_sim_backend.py`:
```python
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
```

- [ ] **Step 2: 失敗を確認**

Run: `uv run pytest tests/test_sim_backend.py -v`
Expected: FAIL（`ModuleNotFoundError: exp_backends`）

- [ ] **Step 3: 実装**

`rover/exp_backends.py`:
```python
# -*- coding: utf-8 -*-
"""バッチ実験ランナーの実行バックエンド。

SimBackend : ROS不要。差動二輪プラント＋入力遅延＋測定ノイズの閉ループsim
             （sim_delay_probe.py と同機序）。開発・λ/msスイープ用。
RealBackend: SSHでRPiのノード起動・bag記録・回収を行う実機用（Task 6で追加）。

両者は run_one(cond, rep, outdir) -> dict(ok, metrics, bagdir, note) を実装する。
設計: docs/superpowers/specs/2026-07-14-batch-runner-design.md
"""

import math
import random
from collections import deque
from pathlib import Path

import yaml

from exp_metrics import compute_metrics
from follower_core import (clamp, goal_crossed, goal_scaled_vr, kanayama_cmd,
                           reference_pose, tracking_error)
from mpc_core import MPCFollower

V_MAX, W_MAX = 0.15, 2.0          # mpc_follower.py と同値
LOOKAHEAD = 0.15
GOAL_TOL = 0.05
KAN_GAINS = dict(k_x=0.5, k_y=5.0, k_th=3.0)  # path_follower.py 既定値

REPO_ROOT = Path(__file__).resolve().parent.parent


def load_path(path_file):
    """configs/path_*.yaml → (waypoints, v_r)。リポジトリルート相対も可。"""
    p = Path(path_file)
    if not p.exists():
        p = REPO_ROOT / path_file
    with open(p) as f:
        cfg = yaml.safe_load(f)
    waypoints = [(float(a), float(b)) for a, b in cfg['waypoints']]
    return waypoints, float(cfg.get('v_r', 0.1))


def sim_run(cond, common, waypoints, v_r, sim_opts, timeout_s, seed):
    """1本の閉ループsim。時系列と到達可否を返す。"""
    ts = 1.0 / float(common.get('rate', 10.0))
    delay = int(sim_opts.get('delay_steps', 2))
    pn = float(sim_opts.get('pos_noise', 0.0))
    yn = float(sim_opts.get('yaw_noise', 0.0))
    rng = random.Random(seed)

    mpc = None
    if cond['controller'] in ('l2', 'l1'):
        mpc = MPCFollower(N=int(common.get('horizon', 15)), ts=ts,
                          reg=cond['controller'],
                          lam=float(cond.get('lam', 0.3)),
                          v_max=V_MAX, w_max=W_MAX,
                          move_suppress=float(cond.get('move_suppress', 0.0)))

    x = y = th = t = 0.0
    buf = deque()
    twist, perr, solve_ms = [], [], []
    ok = False
    while t < timeout_s:
        gx, gy = waypoints[-1]
        gd = math.hypot(gx - x, gy - y)
        if gd < GOAL_TOL or goal_crossed(waypoints, x, y):
            ok = True
            break
        # 測定（ノイズ付き）→ 制御器
        xm = x + rng.gauss(0, pn)
        ym = y + rng.gauss(0, pn)
        thm = th + rng.gauss(0, yn)
        x_r, y_r, th_r = reference_pose(waypoints, xm, ym, LOOKAHEAD)
        x_e, y_e, th_e = tracking_error(xm, ym, thm, x_r, y_r, th_r)
        vr = goal_scaled_vr(v_r, gd)
        if mpc is not None:
            cmd = mpc.command(x_e, y_e, th_e, vr, w_r=0.0)
            solve_ms.append(mpc.last_solve_s * 1e3)
            if cmd is None:
                break  # 求解失敗 → 未達で終了（実機の安全停止に対応）
        else:
            v, w = kanayama_cmd(x_e, y_e, th_e, vr, **KAN_GAINS)
            cmd = (clamp(v, V_MAX), clamp(w, W_MAX))
        # 入力遅延: delay 前の指令を適用（sim_delay_probe.py と同一）
        buf.append(cmd)
        v, w = buf.popleft() if len(buf) > delay else (vr, 0.0)
        twist.append((t, v, w))
        perr.append((t, y_e))
        # 真の非線形プラントを1ステップ
        x += v * math.cos(th) * ts
        y += v * math.sin(th) * ts
        th += w * ts
        t += ts
    return dict(ok=ok, twist=twist, perr=perr, solve_ms=solve_ms)


class SimBackend:
    """ROS不要のsimバックエンド。outdir は使わない（bagを作らない）。"""

    name = 'sim'

    def __init__(self, batch):
        self.batch = batch
        self.waypoints, self.v_r = load_path(batch['path_file'])

    def run_one(self, cond, rep, outdir):
        data = sim_run(cond, self.batch.get('common', {}),
                       self.waypoints, self.v_r,
                       self.batch.get('sim', {}),
                       float(self.batch.get('timeout_s', 60)),
                       seed=rep)
        metrics = compute_metrics(data['twist'], data['perr'], data['solve_ms'])
        note = '' if data['ok'] else 'タイムアウト/求解失敗'
        return dict(ok=data['ok'], metrics=metrics, bagdir='', note=note)
```

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/test_sim_backend.py -v`
Expected: 5 passed（MPCのsimは1本数十秒かかる場合あり。合計〜2分は正常）

- [ ] **Step 5: コミット**

```bash
git add rover/exp_backends.py tests/test_sim_backend.py
git commit -m "SimBackendを追加（遅延・ノイズ付き閉ループsim、L1チャタ/ms対策の既知結果を再現）"
```

---

### Task 4: `rover/run_batch.py` のヘルパー（設定読込・CSV追記・resume）

**Files:**
- Create: `rover/run_batch.py`（このタスクはヘルパー関数まで。CLIはTask 5）
- Create: `tests/test_run_batch.py`

**Interfaces:**
- Produces:
  - `run_batch.CSV_COLUMNS`（下記の列順リスト）
  - `run_batch.load_batch(yaml_path) -> dict`（既定値補完・検証済みバッチ設定）
  - `run_batch.append_row(csv_path, row_dict)`（無ければヘッダ付きで作成）
  - `run_batch.done_keys(csv_path) -> set[(cond, rep)]`（ok=true の走行のみ。resume用）
  - `run_batch.make_row(batch, cond, rep, backend_name, result, git_hash, v_r) -> dict`

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_run_batch.py`:
```python
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
```

- [ ] **Step 2: 失敗を確認**

Run: `uv run pytest tests/test_run_batch.py -v`
Expected: FAIL（`ModuleNotFoundError: run_batch`）

- [ ] **Step 3: 実装**

`rover/run_batch.py`:
```python
# -*- coding: utf-8 -*-
"""バッチ実験ランナー CLI（設計: specs/2026-07-14-batch-runner-design.md）。

configs/batch_*.yaml の条件×反復を実行し、results/<日付>_<名前>/ に
runs.csv（1走行1行・逐次追記）と summary.md（条件別 平均±標準偏差）を出力。

使い方:
  uv run python rover/run_batch.py configs/batch_Lturn_3way.yaml
      [--backend sim|real] [--resume] [--only 条件名] [--dry-run]
      [--summarize] [--outdir DIR]
"""

import argparse
import csv
import datetime
import statistics
import subprocess
from pathlib import Path

import yaml

CSV_COLUMNS = [
    'batch', 'cond', 'rep', 'backend', 'timestamp', 'git_hash',
    'controller', 'lam', 'move_suppress', 'horizon', 'v_r', 'ok',
    'drive_s', 'rmse_cm', 'sum_u', 'w_zero_ratio', 'flips', 'sat_ratio',
    'max_w', 'solve_p50', 'solve_p95', 'solve_max', 'bagdir', 'note',
]

CONTROLLERS = ('kanayama', 'l2', 'l1')


def load_batch(yaml_path):
    """バッチ設定を読み、既定値を補完して検証する。"""
    with open(yaml_path) as f:
        b = yaml.safe_load(f)
    for key in ('name', 'path_file', 'conditions'):
        if key not in b:
            raise ValueError(f'batch yaml に {key} がありません')
    b.setdefault('repeats', 1)
    b.setdefault('timeout_s', 60)
    b.setdefault('backend', 'sim')
    b.setdefault('common', {})
    b.setdefault('sim', {})
    names = set()
    for c in b['conditions']:
        if c.get('controller') not in CONTROLLERS:
            raise ValueError(f"不正な controller: {c.get('controller')}")
        if 'name' not in c or c['name'] in names:
            raise ValueError(f'条件 name が無いか重複: {c}')
        names.add(c['name'])
    return b


def append_row(csv_path, row):
    """runs.csv に1行追記（無ければヘッダ付きで作成）。"""
    p = Path(csv_path)
    new = not p.exists()
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        if new:
            w.writeheader()
        w.writerow(row)


def done_keys(csv_path):
    """ok=true 済みの (cond, rep) 集合（--resume 用）。"""
    p = Path(csv_path)
    if not p.exists():
        return set()
    with open(p, newline='') as f:
        return {(r['cond'], int(r['rep']))
                for r in csv.DictReader(f)
                if str(r['ok']).lower() == 'true'}


def make_row(batch, cond, rep, backend_name, result, git_hash, v_r):
    m = result.get('metrics', {})
    row = {c: '' for c in CSV_COLUMNS}
    row.update(
        batch=batch['name'], cond=cond['name'], rep=rep,
        backend=backend_name,
        timestamp=datetime.datetime.now().isoformat(timespec='seconds'),
        git_hash=git_hash,
        controller=cond['controller'],
        lam=cond.get('lam', ''),
        move_suppress=cond.get('move_suppress', ''),
        horizon=batch.get('common', {}).get('horizon', ''),
        v_r=v_r, ok=result['ok'],
        bagdir=result.get('bagdir', ''), note=result.get('note', ''),
    )
    for k in ('drive_s', 'rmse_cm', 'sum_u', 'w_zero_ratio', 'flips',
              'sat_ratio', 'max_w', 'solve_p50', 'solve_p95', 'solve_max'):
        if k in m:
            row[k] = m[k]
    return row


def git_hash_short():
    try:
        return subprocess.run(['git', 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True,
                              check=True).stdout.strip()
    except Exception:
        return 'unknown'
```

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/test_run_batch.py -v`
Expected: 5 passed

- [ ] **Step 5: コミット**

```bash
git add rover/run_batch.py tests/test_run_batch.py
git commit -m "バッチ設定読込・runs.csv追記・resume判定を実装"
```

---

### Task 5: CLI＋summary.md 生成＋バッチ設定ファイル＋sim E2E

**Files:**
- Modify: `rover/run_batch.py`（write_summary と main を追記）
- Create: `configs/batch_Lturn_3way.yaml`
- Modify: `tests/test_run_batch.py`（write_summary のテスト追加）

**Interfaces:**
- Consumes: `exp_backends.SimBackend`, `exp_backends.load_path`（Task 6 で `RealBackend` も同じ分岐に追加）
- Produces: `run_batch.write_summary(outdir)`、CLI `main()`

- [ ] **Step 1: write_summary の失敗するテストを追加**

`tests/test_run_batch.py` に追記:
```python
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
```

Run: `uv run pytest tests/test_run_batch.py::test_write_summary -v`
Expected: FAIL（`ImportError: write_summary`）

- [ ] **Step 2: write_summary と main を実装**

`rover/run_batch.py` に追記:
```python
def _mean_std(vals):
    if not vals:
        return 'n/a'
    m = statistics.mean(vals)
    s = statistics.stdev(vals) if len(vals) > 1 else 0.0
    return f'{m:.2f}±{s:.2f}'


def write_summary(outdir):
    """runs.csv → summary.md（条件ごとの 到達率・平均±標準偏差）。"""
    outdir = Path(outdir)
    with open(outdir / 'runs.csv', newline='') as f:
        rows = list(csv.DictReader(f))
    conds = []
    for r in rows:
        if r['cond'] not in conds:
            conds.append(r['cond'])
    lines = [
        f"# バッチ結果まとめ: {rows[0]['batch']}（{rows[0]['backend']}）",
        '',
        f"生成: {datetime.datetime.now().isoformat(timespec='seconds')} / "
        f"git {rows[0]['git_hash']} / 全{len(rows)}走行",
        '',
        '| 条件 | 到達 | RMSE_cm | Σ\\|u\\| | 反転 | ω0率 | 解p95ms |',
        '|------|------|---------|--------|------|------|---------|',
    ]
    for c in conds:
        rs = [r for r in rows if r['cond'] == c]
        oks = [r for r in rs if str(r['ok']).lower() == 'true']

        def col(key, rs=oks):
            return _mean_std([float(r[key]) for r in rs if r[key] != ''])

        lines.append(
            f"| {c} | {len(oks)}/{len(rs)} | {col('rmse_cm')} | "
            f"{col('sum_u')} | {col('flips')} | {col('w_zero_ratio')} | "
            f"{col('solve_p95')} |")
    lines += ['', '注: RMSEはodom基準（真値は外部計測）。ok=false の行は'
              '到達数のみ反映し平均から除外。']
    (outdir / 'summary.md').write_text('\n'.join(lines) + '\n')


def main():
    ap = argparse.ArgumentParser(description='バッチ実験ランナー')
    ap.add_argument('batch_yaml')
    ap.add_argument('--backend', choices=['sim', 'real'])
    ap.add_argument('--resume', action='store_true')
    ap.add_argument('--only', help='この条件名だけ実行')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--summarize', action='store_true',
                    help='走行せず summary.md のみ再生成')
    ap.add_argument('--outdir', help='出力先（既定 results/<日付>_<名前>）')
    args = ap.parse_args()

    from exp_backends import SimBackend, load_path
    batch = load_batch(args.batch_yaml)
    backend_kind = args.backend or batch['backend']
    outdir = Path(args.outdir or
                  f"results/{datetime.date.today()}_{batch['name']}")
    csv_path = outdir / 'runs.csv'

    if args.summarize:
        write_summary(outdir)
        print(f'summary.md を再生成: {outdir}')
        return

    if backend_kind == 'sim':
        backend = SimBackend(batch)
    else:
        from exp_backends import RealBackend
        backend = RealBackend(batch, dry_run=args.dry_run)
        if not args.dry_run:
            ans = input(f"実機バッチ {batch['name']} を開始します。"
                        f"nav_base 起動済み・走行エリア確保を確認 [y/N]: ")
            if ans.strip().lower() != 'y':
                print('中止しました')
                return
        backend.preflight()

    _, v_r = load_path(batch['path_file'])
    ghash = git_hash_short()
    done = done_keys(csv_path) if args.resume else set()

    for cond in batch['conditions']:
        if args.only and cond['name'] != args.only:
            continue
        for rep in range(1, int(batch['repeats']) + 1):
            if (cond['name'], rep) in done:
                print(f"skip(済): {cond['name']} rep{rep}")
                continue
            try:
                result = backend.run_one(cond, rep, outdir)
            except KeyboardInterrupt:
                print('\nバッチ中断')
                return
            except Exception as e:  # 1本の失敗はバッチを止めない（設計方針）
                result = dict(ok=False, metrics={}, bagdir='',
                              note=f'error: {e}')
            row = make_row(batch, cond, rep, backend_kind, result, ghash, v_r)
            append_row(csv_path, row)
            m = result.get('metrics', {})
            print(f"{cond['name']} rep{rep}: ok={result['ok']} "
                  f"rmse={m.get('rmse_cm', float('nan')):.2f}cm "
                  f"Σ|u|={m.get('sum_u', float('nan')):.2f} "
                  f"反転={m.get('flips', '-')}")
    if csv_path.exists():
        write_summary(outdir)
        print(f'完了: {outdir}/runs.csv, summary.md')


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: バッチ設定ファイルを作成**

`configs/batch_Lturn_3way.yaml`（スペックの例と同一・本番でそのまま使う）:
```yaml
# L字 3手法＋修正版L1 比較バッチ（sim開発用・実機は --backend real）
name: Lturn_3way
path_file: configs/path_L_turn.yaml
backend: sim
repeats: 3
timeout_s: 60
common: {horizon: 15, rate: 10.0}
sim: {delay_steps: 2, pos_noise: 0.0, yaw_noise: 0.0}   # 遅延2step=実機相当
conditions:
  - {name: kanayama, controller: kanayama}
  - {name: l2,       controller: l2}
  - {name: l1,       controller: l1, lam: 0.3}
  - {name: l1_ms2,   controller: l1, lam: 0.3, move_suppress: 2.0}
```

- [ ] **Step 4: テスト＋sim E2E**

Run: `uv run pytest tests/test_run_batch.py -v`
Expected: 6 passed

Run（E2E・出力はscratchpadへ。数分かかる）:
```bash
uv run python rover/run_batch.py configs/batch_Lturn_3way.yaml \
    --outdir /tmp/claude-1000/-home-mukougawakouhei-projects-sparse-optimal-control/738458b0-d6e2-43ee-bf63-77cdce76eb1e/scratchpad/e2e_batch
cat .../e2e_batch/summary.md
```
Expected: 4条件×3本=12行の runs.csv。summary.md で l1 の反転が大（≥8）、
l1_ms2 と l2 の反転が ≈0〜1、全条件到達 3/3。
（sim_delay_probe.py の既知結果と整合すること）

Run（resume確認）: 同コマンドに `--resume` を付けて再実行
Expected: 全行 `skip(済)` 表示で即終了。

- [ ] **Step 5: コミット**

```bash
git add rover/run_batch.py tests/test_run_batch.py configs/batch_Lturn_3way.yaml
git commit -m "バッチランナーCLIとsummary生成を実装（sim E2Eで既知のチャタ結果を再現）"
```

---

### Task 6: RealBackend（SSHオーケストレーション・dry-run検証まで）

**Files:**
- Modify: `rover/exp_backends.py`（RealBackend を追記）
- Create: `tests/test_real_backend.py`

**Interfaces:**
- Consumes: `analyze_bag.read_bag(bagdir)`, `exp_metrics.compute_metrics`
- Produces: `exp_backends.RealBackend(batch, dry_run=False)` — `.preflight()`, `.run_one(cond, rep, outdir)`。
  補助（テスト対象の純関数）: `node_command(cond, common, path_file) -> str`,
  `bag_record_command(remote_dir) -> str`

- [ ] **Step 1: コマンド生成の失敗するテストを書く**

`tests/test_real_backend.py`:
```python
from exp_backends import bag_record_command, node_command

COMMON = dict(horizon=15, rate=10.0)
PATH = "configs/path_L_turn.yaml"


def test_node_command_kanayama():
    cmd = node_command(dict(name="k", controller="kanayama"), COMMON, PATH)
    assert "path_follower.py" in cmd
    assert "path_L_turn.yaml" in cmd
    assert "reg:=" not in cmd


def test_node_command_l1_ms():
    cmd = node_command(
        dict(name="l1_ms2", controller="l1", lam=0.3, move_suppress=2.0),
        COMMON, PATH)
    assert "mpc_follower.py" in cmd
    assert "-p reg:=l1" in cmd
    assert "-p lam:=0.3" in cmd
    assert "-p move_suppress:=2.0" in cmd
    assert "-p horizon:=15" in cmd


def test_bag_record_command():
    cmd = bag_record_command("/tmp/batch_l2_r1")
    assert cmd.startswith("ros2 bag record -o /tmp/batch_l2_r1")
    for topic in ("/odom", "/rover_twist", "/path_error", "/mpc_solve_ms"):
        assert topic in cmd
```

Run: `uv run pytest tests/test_real_backend.py -v`
Expected: FAIL（ImportError）

- [ ] **Step 2: RealBackend を実装**

`rover/exp_backends.py` に追記（import に `shlex, subprocess, sys, time, select` を追加）:
```python
# ---- RealBackend（SSHオーケストレーション） ------------------------------
# RPi側は従来の手動手順（handoff.md 環境メモ）を自動化しただけ:
#   nav_base は事前にユーザーが起動 / ノード起動→bag record→SIGINT停止→scp回収。
# 安全系はノード既存機構（到達停止・SIGINT停止・odom途絶停止）に委ねる。

SSH_USER = 'mukougawakouhei'
SSH_HOSTS = ['192.168.0.31', '192.168.4.1']   # 家Wi-Fi → ホットスポットの順
RPI_ROOT = '/home/mukougawakouhei/sparse_control'
BAG_TOPICS = '/odom /rover_twist /path_error /mpc_solve_ms'
SYNC_FILES = ['rover/mpc_follower.py', 'rover/path_follower.py',
              'rover/follower_core.py', 'rover/mpc_core.py']
STARTUP_MARGIN_S = {'kanayama': 15, 'l2': 90, 'l1': 90}  # cvxpy import 30-60s


def node_command(cond, common, path_file):
    """RPi上で実行する follower ノード起動コマンド文字列。"""
    params = [f'-p path_file:={RPI_ROOT}/{path_file}']
    if cond['controller'] == 'kanayama':
        script = 'path_follower.py'
    else:
        script = 'mpc_follower.py'
        params += [f"-p reg:={cond['controller']}",
                   f"-p lam:={cond.get('lam', 0.3)}",
                   f"-p move_suppress:={cond.get('move_suppress', 0.0)}",
                   f"-p horizon:={common.get('horizon', 15)}",
                   f"-p rate:={common.get('rate', 10.0)}"]
    return (f'cd {RPI_ROOT}/rover && python3 {script} --ros-args '
            + ' '.join(params))


def bag_record_command(remote_dir):
    return f'ros2 bag record -o {remote_dir} {BAG_TOPICS}'


KILL_CMD = ("pkill -INT -f '[m]pc_follower.py'; "
            "pkill -INT -f '[p]ath_follower.py'; true")
KILL_BAG_CMD = "pkill -INT -f '[r]os2 bag record'; true"


class RealBackend:
    """SSHでRPiを操作する実機バックエンド。1本ごとに Enter 待ち＝走行許可。"""

    name = 'real'

    def __init__(self, batch, dry_run=False):
        self.batch = batch
        self.dry_run = dry_run
        self.host = None

    def _ssh_args(self, cmd):
        return ['ssh', f'{SSH_USER}@{self.host}',
                f'bash -lc {shlex.quote(cmd)}']

    def _ssh(self, cmd, timeout=30):
        return subprocess.run(self._ssh_args(cmd), capture_output=True,
                              text=True, timeout=timeout)

    def preflight(self):
        """SSH疎通・nav_base稼働・コード同期を確認（バッチ開始時に1回）。"""
        if self.dry_run:
            self.host = SSH_HOSTS[0]
            print(f'[dry-run] preflight: ssh {SSH_USER}@{self.host} で '
                  f'ros2 node list / md5sum 確認を行う')
            return
        for h in SSH_HOSTS:
            try:
                if subprocess.run(['ssh', '-o', 'ConnectTimeout=5',
                                   f'{SSH_USER}@{h}', 'true'],
                                  timeout=15).returncode == 0:
                    self.host = h
                    break
            except subprocess.TimeoutExpired:
                pass
        if self.host is None:
            raise RuntimeError(f'RPiにSSH接続できません: {SSH_HOSTS}')
        print(f'RPi接続: {self.host}')

        nodes = self._ssh('source /opt/ros/humble/setup.bash 2>/dev/null; '
                          'ros2 node list').stdout
        if 'pos_controller' not in nodes or 'odom_manager' not in nodes:
            raise RuntimeError(
                'nav_base が稼働していません。RPiで先に起動してください:\n'
                '  ros2 launch lightrover_ros nav_base.launch.py')

        # コード同期チェック（差異は警告のみ・配置更新は手動）
        for f in SYNC_FILES:
            local = subprocess.run(['md5sum', str(REPO_ROOT / f)],
                                   capture_output=True,
                                   text=True).stdout.split()[0]
            remote_out = self._ssh(f'md5sum {RPI_ROOT}/{f}').stdout.split()
            if not remote_out or remote_out[0] != local:
                ans = input(f'警告: {f} がRPi側と異なります（要scp配置）。'
                            f'続行しますか [y/N]: ')
                if ans.strip().lower() != 'y':
                    raise RuntimeError('中止（コードを同期してください）')

        self._ssh(KILL_CMD)  # 残存followerを掃除

    def run_one(self, cond, rep, outdir):
        common = self.batch.get('common', {})
        n_cmd = node_command(cond, common, self.batch['path_file'])
        remote_bag = f"/tmp/batch_{cond['name']}_r{rep}"
        r_cmd = bag_record_command(remote_bag)

        if self.dry_run:
            print(f"[dry-run] {cond['name']} rep{rep}:")
            print(f'  bag : ssh ... {r_cmd}')
            print(f'  node: ssh ... {n_cmd}')
            print(f'  stop: ssh ... {KILL_CMD} / {KILL_BAG_CMD}')
            print(f"  scp : {remote_bag} -> {outdir}/{cond['name']}_r{rep}")
            return dict(ok=False, metrics={}, bagdir='', note='dry-run')

        ans = input(f"\n次: {cond['name']} rep{rep}/{self.batch['repeats']}。"
                    f'ロボットを原点に戻して Enter（q で中断）: ')
        if ans.strip().lower() == 'q':
            raise KeyboardInterrupt

        self._ssh(f'rm -rf {remote_bag}')
        rec = subprocess.Popen(self._ssh_args(r_cmd),
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
        node = subprocess.Popen(self._ssh_args(n_cmd),
                                stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)
        margin = STARTUP_MARGIN_S[cond['controller']]
        deadline = time.time() + float(self.batch['timeout_s']) + margin
        reached = False
        try:
            while time.time() < deadline:
                ready, _, _ = select.select([node.stdout], [], [], 1.0)
                if not ready:
                    if node.poll() is not None:
                        break  # ノードが落ちた（求解失敗等）
                    continue
                line = node.stdout.readline()
                if not line:
                    break
                sys.stdout.write('  | ' + line)
                if '目標到達' in line:
                    reached = True
                    break
        finally:
            self._ssh(KILL_CMD)
            time.sleep(1.0)          # 停止指令publish・bag flush待ち
            self._ssh(KILL_BAG_CMD)
            node.wait(timeout=15)
            rec.wait(timeout=15)

        bagdir = Path(outdir) / f"{cond['name']}_r{rep}"
        bagdir.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(['scp', '-q', '-r',
                        f'{SSH_USER}@{self.host}:{remote_bag}',
                        str(bagdir)], check=True)
        self._ssh(f'rm -rf {remote_bag}')

        from analyze_bag import read_bag
        twist, perr, solve = read_bag(str(bagdir))
        metrics = compute_metrics(twist, perr, solve)
        return dict(ok=reached, metrics=metrics, bagdir=str(bagdir),
                    note='' if reached else 'タイムアウト/ノード異常終了')
```

- [ ] **Step 3: テスト＋dry-run確認**

Run: `uv run pytest tests/test_real_backend.py -v`
Expected: 3 passed

Run: `uv run python rover/run_batch.py configs/batch_Lturn_3way.yaml --backend real --dry-run --outdir /tmp/claude-1000/-home-mukougawakouhei-projects-sparse-optimal-control/738458b0-d6e2-43ee-bf63-77cdce76eb1e/scratchpad/dryrun_batch`
Expected: 12本分の ssh/scp コマンド列が表示され、実行はされない
（runs.csv には ok=false, note=dry-run で12行記録される）。
コマンド内容が handoff.md の手動手順（nav_base前提・python3 起動・-p パラメータ）
と一致することを目視確認。

- [ ] **Step 4: 全テスト実行**

Run: `uv run pytest -v`
Expected: 全テストPASS（exp_metrics 3 / analyze_bag 2 / sim_backend 5 / run_batch 6 / real_backend 3 = 19）

- [ ] **Step 5: コミット**

```bash
git add rover/exp_backends.py tests/test_real_backend.py
git commit -m "RealBackendを追加（SSHでノード起動・bag記録・回収、Enter待ち許可・dry-run対応）"
```

---

### Task 7: ドキュメント更新（handoff・CLAUDE.md 現在地）

**Files:**
- Modify: `docs/handoff.md`（先頭に新セッション節を追加、タイトル日付を 2026-07-14 に）
- Modify: `CLAUDE.md`（「現在地」に1項目追加・「次:」を更新）

**Interfaces:** なし（ドキュメントのみ）

- [ ] **Step 1: handoff.md 先頭に追記**

タイトル行を `# Handoff - 2026-07-14` に変え、リポジトリ整理注記の直後に:

```markdown
## 2026-07-14 セッション11（バッチ実験ランナー実装）

- 設計: `docs/superpowers/specs/2026-07-14-batch-runner-design.md`（承認済み）。
  実験サイクル（起動→bag→解析→CSV）を条件YAMLで自動化。sim/実機バックエンド差し替え式。
- 実装（テスト19本・`uv run pytest`）:
  - `rover/exp_metrics.py`: 指標計算を共通化（4指標＋ω反転・飽和）。analyze_bag は
    これを使う形に改修し、6/13実機bagで回帰テスト（記録値と一致）。
  - `rover/exp_backends.py`: SimBackend（遅延・ノイズ付き閉ループsim。
    L1チャタ/ms=2.0対策の既知結果を再現）＋ RealBackend（SSHでノード起動・
    bag記録・SIGINT停止・scp回収。1本ごと Enter待ち＝走行許可。dry-run可）。
  - `rover/run_batch.py`: CLI（--resume/--only/--dry-run/--summarize/--outdir）。
    `results/<日付>_<名前>/runs.csv`＋`summary.md`（条件別 平均±標準偏差）。
  - `configs/batch_Lturn_3way.yaml`: Kanayama/L2/L1/L1+ms2.0 × 3本のL字バッチ。
- 実機ではまだ走らせていない（dry-run検証まで）。次回実機時の手順:
  1. RPi へ rover/ を scp 配置（preflight が md5 差異を警告する）
  2. RPi で nav_base 起動 → `uv run python rover/run_batch.py
     configs/batch_Lturn_3way.yaml --backend real`
  3. 1本ごとに原点復帰して Enter（初回バッチが RealBackend の統合テスト）
```

- [ ] **Step 2: CLAUDE.md「現在地」を更新**

「現在地（2026-06-13時点）」の見出しを「現在地（2026-07-14時点）」に変え、
teleop 行の後・「次:」行の前に追記:

```markdown
- バッチ実験ランナー: **実装完了（実機は未走行）** → `rover/run_batch.py`,
  `rover/exp_backends.py`, `rover/exp_metrics.py`, `configs/batch_Lturn_3way.yaml`
  - sim E2E で L1チャタ→ms=2.0で消失の既知結果を再現。実機は `--backend real`
    （1本ごと Enter待ち＝許可、`--dry-run` で手順確認可）。テストは `uv run pytest`
```

「次:」行を以下に置き換え:

```markdown
- 次: バッチランナーで実機 L字バッチ（Kanayama/L2/L1/L1+ms2.0 ×3本、要許可・
  初回が統合テスト）→ チャタ消失の実測確認と統計化 → 外乱条件 / カメラ真値
```

- [ ] **Step 3: コミット**

```bash
git add docs/handoff.md CLAUDE.md
git commit -m "handoff/CLAUDE.mdを更新（バッチ実験ランナー実装完了・次は実機L字バッチ）"
```

---

## Self-Review 結果（作成時に実施済み）

- スペック対応: 指標共通化=T1-2 / SimBackend=T3 / 条件YAML・CSV・resume=T4 /
  CLI・summary=T5 / RealBackend・preflight・dry-run=T6 / テスト計画1〜3=T2,T5,T6。
  スペックのテスト4（実機統合）は Global Constraints どおり本計画の範囲外（次回実機セッション）。
- スペックからの軽微な変更: runs.csv の「ヘッダコメント」は csv パーサ互換のため
  git_hash **列**のみで代替（スペックの意図＝再現性は満たす）。
- 型整合: `run_one(cond, rep, outdir) -> dict(ok, metrics, bagdir, note)` を
  Sim/Real で統一、`compute_metrics(twist, perr, solve_ms)` の3引数順を
  read_bag / sim_run の返却順と一致させた。
