# カメラ真値 Phase 2+3（ライブ化＋自動原点復帰＋全自動バッチ）実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 走行→カメラ真値の自動保存→自動原点復帰→次走行、を一括許可で回す全自動バッチを作る。

**Architecture:** 検出・記録・復帰誘導は全てラップトップ（truth_live がC270ライブ検出、homing が Kanayama で原点誘導）。RPi 追加は UDP→rover_twist ブリッジ1ファイルのみ（DDS回避・案A）。run_batch に --auto ループを足し、既存 RealBackend の走行フローはそのまま使う。

**Tech Stack:** Python (uv), OpenCV (cv2.aruco), 既存 rover/follower_core.py・truth_core.py・truth_offline.py を流用。RPi側は stdlib + rclpy のみ。

**設計spec:** docs/superpowers/specs/2026-07-16-camera-truth-pipeline-design.md（承認済み）
**前提:** Phase 1 完了（truth_core/truth_offline、実走行で手実測と2.5cm一致を確認済み 2026-07-16）

## Global Constraints

- 単位は SI（m, rad）。CSV・表示のみ cm/deg 換算可
- RPi 側新規コードは `rover/udp_twist_bridge.py` 1ファイルのみ、依存は stdlib + rclpy + geometry_msgs に限定
- laptop 側の新規依存なし（opencv-python は導入済み）
- 速度クランプは既存と同値: V_MAX=0.15, W_MAX=2.0（ブリッジで最終防衛）
- 復帰パラメータ（spec準拠）: v=0.08 / ω上限1.0 / 到達判定±3cm・±5° / タイムアウト30s / リトライ1回 / pose途絶1s停止 / コース外接矩形+30cm逸脱停止 / UDP watchdog 0.5s
- 安全操作は「q＋Enter」で即停止（停止指令送信＋既存KILL_CMD。rawキー入力は使わない＝端末状態を壊さない）
- 各タスク完了時 `uv run pytest tests/` 全green・日本語コミット
- テストは実カメラ・実ロボット・実ソケット禁止（fake/合成画像で書く）

---

### Task 1: exp_metrics に真値指標（純関数）

**Files:**
- Modify: `rover/exp_metrics.py`（末尾に追記）
- Test: `tests/test_exp_metrics_truth.py`（新規）

**Interfaces:**
- Consumes: なし（純関数）
- Produces: `truth_metrics(rows, waypoints) -> dict`。
  `rows`: `[(t_s, x, y, theta, n_tags, quality_px), ...]`（欠測は x=None、truth_live.stop() の戻り値形式）。
  返り値キー: `truth_end_x, truth_end_y, truth_end_theta`（float, m/rad）、
  `truth_end_dist_cm, truth_rmse_cm`（float, cm）。有効行ゼロなら `{}`。

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_exp_metrics_truth.py`:

```python
# -*- coding: utf-8 -*-
"""truth_metrics（真値rows→終点・横偏差RMSE）のテスト。"""
import math

from exp_metrics import truth_metrics

WPS = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]


def _rows_along_path(offset=0.02, n=50):
    """経路から法線方向に offset ずらした軌跡（0.5s刻み・最後の1sはゴール横）。"""
    rows = []
    for i in range(n):
        s = 2.0 * i / (n - 1)          # 経路弧長 0..2m
        if s <= 1.0:
            x, y, th = s, -offset, 0.0
        else:
            x, y, th = 1.0 + offset, s - 1.0, math.pi / 2
        rows.append((0.5 * i, x, y, th, 5, 300.0))
    return rows


def test_truth_metrics_endpoint_and_rmse():
    rows = _rows_along_path(offset=0.02)
    m = truth_metrics(rows, WPS)
    # 終点=末尾0.5sの平均 ≈ (1.02, 2.0-1.0=1.0近傍)
    assert abs(m['truth_end_x'] - 1.02) < 0.005
    assert abs(m['truth_end_y'] - 1.0) < 0.03
    assert abs(m['truth_end_theta'] - math.pi / 2) < 0.01
    # ゴール(1,1)からの距離 ≈ 2cm、横偏差RMSE ≈ 2cm
    assert abs(m['truth_end_dist_cm'] - 2.0) < 1.0
    assert abs(m['truth_rmse_cm'] - 2.0) < 0.5


def test_truth_metrics_ignores_missing_and_empty():
    rows = _rows_along_path()
    rows[10] = (5.0, None, None, None, 4, 0.0)   # 欠測行は無視
    assert 'truth_rmse_cm' in truth_metrics(rows, WPS)
    assert truth_metrics([(0.0, None, None, None, 0, 0.0)], WPS) == {}
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `uv run pytest tests/test_exp_metrics_truth.py -q`
Expected: FAIL（ImportError: cannot import name 'truth_metrics'）

- [ ] **Step 3: 実装（rover/exp_metrics.py の末尾に追記）**

```python
def _dist_to_polyline(waypoints, x, y):
    """点 (x,y) から折れ線経路への最短距離 [m]。"""
    best = float('inf')
    for i in range(len(waypoints) - 1):
        ax, ay = waypoints[i]
        bx, by = waypoints[i + 1]
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy)
                         / (dx * dx + dy * dy)))
        d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
        best = min(best, d)
    return best


def truth_metrics(rows, waypoints, window_s=0.5):
    """カメラ真値 rows → 終点（末尾window_sの平均・θは円形平均）と横偏差RMSE。

    rows: [(t_s, x, y, theta, n_tags, quality_px)]（欠測は x=None）。
    有効行ゼロなら {}（runs.csv の truth列は空欄のまま）。
    """
    valid = [r for r in rows if r[1] is not None]
    if not valid:
        return {}
    t_end = valid[-1][0]
    win = [r for r in valid if r[0] >= t_end - window_s]
    ex = sum(r[1] for r in win) / len(win)
    ey = sum(r[2] for r in win) / len(win)
    eth = math.atan2(sum(math.sin(r[3]) for r in win),
                     sum(math.cos(r[3]) for r in win))
    gx, gy = waypoints[-1]
    rmse = math.sqrt(sum(_dist_to_polyline(waypoints, r[1], r[2]) ** 2
                         for r in valid) / len(valid))
    return dict(truth_end_x=ex, truth_end_y=ey, truth_end_theta=eth,
                truth_end_dist_cm=math.hypot(ex - gx, ey - gy) * 100.0,
                truth_rmse_cm=rmse * 100.0)
```

注: `rover/exp_metrics.py` は先頭で `import math` 済みか確認し、無ければ追加する。

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/test_exp_metrics_truth.py -q` → 2 passed
Run: `uv run pytest tests/ -q` → 全green

- [ ] **Step 5: コミット**

```bash
git add rover/exp_metrics.py tests/test_exp_metrics_truth.py
git commit -m "真値rowsから終点・横偏差RMSEを算出する truth_metrics を追加"
```

---

### Task 2: udp_twist_bridge（RPi側・純ロジック分離）

**Files:**
- Create: `rover/udp_twist_bridge.py`
- Modify: `configs/camera_truth.yaml`（末尾に `udp:` セクション追記）
- Test: `tests/test_udp_bridge.py`

**Interfaces:**
- Consumes: UDPペイロード `b'{"v": 0.08, "w": -0.3, "seq": 12}'`（homing の UdpTwistSender が送る形式）
- Produces: `BridgeCore(watchdog_s=0.5)` — `accept(data: bytes, now: float) -> (v, w) | None`（破棄時None・クランプ済み）、`watchdog_zero(now: float) -> bool`（途絶検知で零速度を1回流すべきときTrue）。main() は rclpy ノード（テスト対象外・rclpy import は main 内）

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_udp_bridge.py`:

```python
# -*- coding: utf-8 -*-
"""udp_twist_bridge の純ロジック（seq破棄・watchdog・クランプ）。"""
import json

from udp_twist_bridge import BridgeCore


def _msg(v, w, seq):
    return json.dumps(dict(v=v, w=w, seq=seq)).encode()


def test_accept_clamp_and_seq_order():
    core = BridgeCore()
    assert core.accept(_msg(0.08, -0.3, 1), now=0.0) == (0.08, -0.3)
    # クランプ（V_MAX=0.15, W_MAX=2.0）
    assert core.accept(_msg(9.0, -9.0, 2), now=0.1) == (0.15, -2.0)
    # 古い/同一 seq は破棄
    assert core.accept(_msg(0.05, 0.0, 2), now=0.2) is None
    assert core.accept(_msg(0.05, 0.0, 1), now=0.2) is None
    assert core.accept(_msg(0.05, 0.0, 3), now=0.2) == (0.05, 0.0)


def test_malformed_payload_discarded():
    core = BridgeCore()
    assert core.accept(b'not json', 0.0) is None
    assert core.accept(json.dumps(dict(v=0.1)).encode(), 0.0) is None
    assert core.accept(b'\xff\xfe', 0.0) is None


def test_watchdog_fires_once_after_silence():
    core = BridgeCore(watchdog_s=0.5)
    assert not core.watchdog_zero(0.0)          # 指令前は発火しない
    core.accept(_msg(0.1, 0.0, 1), now=1.0)
    assert not core.watchdog_zero(1.4)          # 0.4s: まだ
    assert core.watchdog_zero(1.6)              # 0.6s: 発火
    assert not core.watchdog_zero(1.7)          # 1回だけ（連打しない）
    core.accept(_msg(0.1, 0.0, 2), now=2.0)     # 再開すれば再武装
    assert core.watchdog_zero(2.6)
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `uv run pytest tests/test_udp_bridge.py -q`
Expected: FAIL（ModuleNotFoundError: udp_twist_bridge）

- [ ] **Step 3: 実装**

`rover/udp_twist_bridge.py`:

```python
# -*- coding: utf-8 -*-
"""UDP {v, w, seq} JSON → /rover_twist ブリッジ（RPi・Phase 3）。

laptop の homing（自動原点復帰）からの速度指令を rover_twist に流す唯一の
入口。走行フェーズでは使わない（follower は RPi 内で完結）。
安全機構: 受信 0.5s 途絶で零速度を1回配信（watchdog）、seq 逆行・重複は
破棄、v/w は V_MAX/W_MAX でクランプ。

使い方（RPi）:
  source /opt/ros/humble/setup.bash && python3 udp_twist_bridge.py [--port 8890]
停止: Ctrl-C（終了時に零速度を配信）。
BridgeCore は ROS 非依存（tests/test_udp_bridge.py で単体テスト）。
"""
import json

V_MAX, W_MAX = 0.15, 2.0     # mpc_follower.py と同値（最終防衛クランプ）
WATCHDOG_S = 0.5


class BridgeCore:
    """受信判定・watchdog の純ロジック。now は time.monotonic() 相当の秒。"""

    def __init__(self, watchdog_s=WATCHDOG_S):
        self.watchdog_s = watchdog_s
        self.last_seq = -1
        self.last_rx = None
        self.active = False   # 有効指令を流している間 True

    def accept(self, data, now):
        """UDPペイロード → (v, w)（クランプ済み）か None（破棄）。"""
        try:
            m = json.loads(data.decode())
            seq, v, w = int(m['seq']), float(m['v']), float(m['w'])
        except (ValueError, KeyError, TypeError, UnicodeDecodeError):
            return None
        if seq <= self.last_seq:
            return None                    # 順序乱れ・重複は破棄
        self.last_seq = seq
        self.last_rx = now
        self.active = True
        return (max(-V_MAX, min(V_MAX, v)), max(-W_MAX, min(W_MAX, w)))

    def watchdog_zero(self, now):
        """途絶検知。零速度を1回だけ流すべきとき True（発火後は再受信まで沈黙）。"""
        if self.active and now - self.last_rx > self.watchdog_s:
            self.active = False
            return True
        return False


def main():
    import argparse
    import socket
    import time

    import rclpy
    from geometry_msgs.msg import Twist

    ap = argparse.ArgumentParser(description='UDP→rover_twist ブリッジ')
    ap.add_argument('--port', type=int, default=8890)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('udp_twist_bridge')
    pub = node.create_publisher(Twist, 'rover_twist', 10)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.port))
    sock.setblocking(False)
    core = BridgeCore()

    def publish(v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        pub.publish(msg)

    def tick():
        now = time.monotonic()
        while True:
            try:
                data, _ = sock.recvfrom(256)
            except BlockingIOError:
                break
            cmd = core.accept(data, now)
            if cmd is not None:
                publish(*cmd)
        if core.watchdog_zero(now):
            node.get_logger().warn('UDP途絶: 零速度を配信')
            publish(0.0, 0.0)

    node.create_timer(0.05, tick)   # 20Hz で受信・watchdog
    node.get_logger().info(f'udp_twist_bridge 起動: port {args.port}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    publish(0.0, 0.0)
    node.destroy_node()


if __name__ == '__main__':
    main()
```

`configs/camera_truth.yaml` の末尾に追記:

```yaml
udp:             # Phase 3: laptop→RPi の復帰指令ブリッジ
  port: 8890     # 宛先ホストは run_batch が RealBackend の接続先を使う
```

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/test_udp_bridge.py -q` → 3 passed
Run: `uv run pytest tests/ -q` → 全green

- [ ] **Step 5: コミット**

```bash
git add rover/udp_twist_bridge.py tests/test_udp_bridge.py configs/camera_truth.yaml
git commit -m "UDP→rover_twistブリッジ（watchdog0.5s・seq破棄・クランプ、RPi用）"
```

---

### Task 3: homing（自動原点復帰・純ロジック＋UDP送信）

**Files:**
- Create: `rover/homing.py`
- Test: `tests/test_homing.py`

**Interfaces:**
- Consumes: `follower_core` の `reference_pose / tracking_error / kanayama_cmd / goal_scaled_vr / clamp / normalize_angle`。Task 2 の UDP ペイロード形式
- Produces:
  - `course_bounds(floor_tags: dict[int, (x,y)], margin=0.30) -> (x0, y0, x1, y1)`
  - `HomingController(target=(0,0,0), bounds=None, timeout_s=30.0)` — `step(pose: (x,y,th)|None, t: float) -> (v, w, status)`、status ∈ {'run','done','fail_pose','fail_bounds','fail_timeout'}
  - `UdpTwistSender(host, port=8890)` — `send(v, w)`（seq自動増分）、`stop()`（零速度3連送）、`close()`
  - `home(pose_fn, sender, floor_tags, target=(0,0,0), rate_hz=10.0, stop_event=None, timeout_s=30.0, sleep_fn=time.sleep, clock=time.monotonic) -> dict(ok, reason, pose, duration_s)`
    （`pose_fn() -> (x,y,th)|None`。None は「1s以上新鮮なposeが無い」の意味＝truth_live.pose(1.0) をそのまま渡す）

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_homing.py`:

```python
# -*- coding: utf-8 -*-
"""homing: 差動二輪プラントのsimで収束・安全停止を検証（実ソケット不使用）。"""
import math

from homing import HomingController, course_bounds, home

TAGS = {0: (0.0, -0.2), 1: (0.434, 0.392), 2: (1.13, 1.15),
        3: (1.194, -0.297)}


class FakeSender:
    def __init__(self):
        self.last = (0.0, 0.0)
        self.stopped = False

    def send(self, v, w):
        self.last = (v, w)
        self.stopped = False

    def stop(self):
        self.last = (0.0, 0.0)
        self.stopped = True


class Sim:
    """一輪車プラント＋擬似カメラ。sleep/clock を差し替えて実時間ゼロで回す。"""

    def __init__(self, start, noise=0.005, dropout_after=None):
        self.x, self.y, self.th = start
        self.t = 0.0
        self.sender = FakeSender()
        self.noise = noise
        self.dropout_after = dropout_after
        self._k = 0

    def clock(self):
        return self.t

    def sleep(self, dt):
        v, w = self.sender.last
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.th += w * dt
        self.t += dt

    def pose(self):
        if self.dropout_after is not None and self.t > self.dropout_after:
            return None
        self._k += 1
        n = self.noise * (1 if self._k % 2 else -1)   # 決定的な±ノイズ
        return self.x + n, self.y + n, self.th + n


def _run(start, **kw):
    sim = Sim(start, **kw)
    res = home(sim.pose, sim.sender, TAGS,
               sleep_fn=sim.sleep, clock=sim.clock)
    return sim, res


def test_homing_converges_from_goal_area():
    for start in [(0.98, 1.0, math.pi / 2), (0.5, 0.5, -2.5),
                  (1.1, -0.1, math.pi)]:
        sim, res = _run(start)
        assert res['ok'], (start, res)
        assert math.hypot(sim.x, sim.y) < 0.05, start
        assert abs(math.atan2(math.sin(sim.th), math.cos(sim.th))) \
            < math.radians(8), start
        assert sim.sender.stopped                     # 終了時は必ず停止送信


def test_homing_pose_loss_stops():
    sim, res = _run((0.9, 0.9, math.pi / 2), dropout_after=2.0)
    assert not res['ok'] and res['reason'] == 'fail_pose'
    assert sim.sender.stopped


def test_homing_out_of_bounds_stops():
    sim, res = _run((3.0, 3.0, 0.0))   # コース外接矩形+30cmの外
    assert not res['ok'] and res['reason'] == 'fail_bounds'
    assert sim.sender.stopped


def test_homing_timeout():
    sim = Sim((0.9, 0.9, math.pi / 2))
    sim.sender.send = lambda v, w: None   # 指令が届かない＝動かない
    res = home(sim.pose, sim.sender, TAGS,
               sleep_fn=lambda dt: sim.__setattr__('t', sim.t + dt),
               clock=sim.clock)
    assert not res['ok'] and res['reason'] == 'fail_timeout'


def test_course_bounds():
    x0, y0, x1, y1 = course_bounds(TAGS)
    assert x0 == -0.3 and abs(y0 - (-0.597)) < 1e-9
    assert abs(x1 - 1.494) < 1e-9 and abs(y1 - 1.45) < 1e-9
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `uv run pytest tests/test_homing.py -q`
Expected: FAIL（ModuleNotFoundError: homing）

- [ ] **Step 3: 実装**

`rover/homing.py`:

```python
# -*- coding: utf-8 -*-
"""カメラ真値による自動原点復帰（Phase 3・laptop側）。

truth_live の pose を状態量に、follower_core の Kanayama で「現在地→原点」の
直線経路を追従し、最後にその場旋回で θ を合わせる（TURN→GO→ALIGN）。
速度指令は UDP で udp_twist_bridge（RPi）へ送る。
安全: pose途絶（pose_fnがNone）・コース外接矩形+30cm逸脱・タイムアウトで
即停止（零速度送信）。設計: specs/2026-07-16-camera-truth-pipeline-design.md
"""
import json
import math
import socket
import time

from follower_core import (clamp, goal_scaled_vr, kanayama_cmd,
                           normalize_angle, reference_pose, tracking_error)

V_HOME = 0.08
W_MAX = 1.0
TOL_POS = 0.03
TOL_YAW = math.radians(5)
TIMEOUT_S = 30.0
MARGIN_M = 0.30
KAN_GAINS = dict(k_x=0.5, k_y=5.0, k_th=3.0)   # exp_backends と同値
K_TURN = 3.0                    # その場旋回の比例ゲイン
TURN_ENTER = math.radians(45)   # これ以下の向き誤差で GO へ
LOOKAHEAD = 0.15
NEAR_R = 0.10                   # この距離からは点収束則（後退許可）に切替
K_NEAR_V, K_NEAR_W = 0.5, 2.0


def course_bounds(floor_tags, margin=MARGIN_M):
    """床タグ群の外接矩形＋余白 (x0, y0, x1, y1)。逸脱＝即停止の安全柵。"""
    xs = [x for x, _ in floor_tags.values()]
    ys = [y for _, y in floor_tags.values()]
    return (min(xs) - margin, min(ys) - margin,
            max(xs) + margin, max(ys) + margin)


class HomingController:
    """1tickごとに (v, w, status) を返す純ロジック。

    status: 'run'（継続） / 'done' / 'fail_pose' / 'fail_bounds' /
    'fail_timeout'。TURN=目標方向へ旋回 → GO=Kanayamaで直線追従 →
    ALIGN=最終θ合わせ。
    """

    def __init__(self, target=(0.0, 0.0, 0.0), bounds=None,
                 timeout_s=TIMEOUT_S):
        self.target = target
        self.bounds = bounds
        self.timeout_s = timeout_s
        self.t0 = None
        self.phase = 'TURN'
        self.start_xy = None

    def step(self, pose, t):
        if self.t0 is None:
            self.t0 = t
        if t - self.t0 > self.timeout_s:
            return 0.0, 0.0, 'fail_timeout'
        if pose is None:
            return 0.0, 0.0, 'fail_pose'
        x, y, th = pose
        if self.bounds is not None:
            x0, y0, x1, y1 = self.bounds
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                return 0.0, 0.0, 'fail_bounds'
        tx, ty, tth = self.target
        dist = math.hypot(tx - x, ty - y)
        if self.phase in ('TURN', 'GO') and dist < TOL_POS:
            self.phase = 'ALIGN'
        if self.phase == 'TURN':
            if self.start_xy is None:
                self.start_xy = (x, y)
            err = normalize_angle(math.atan2(ty - y, tx - x) - th)
            if abs(err) >= TURN_ENTER:
                return 0.0, clamp(K_TURN * err, W_MAX), 'run'
            self.phase = 'GO'
        if self.phase == 'GO':
            if dist < NEAR_R:
                # Kanayama は「行き過ぎ」で前進FFと引き戻しが釣り合い
                # 平衡点ができる（simで4.8cm停滞を確認）→ 後退を許す
                # go-to-point 則で点収束させる
                err = normalize_angle(math.atan2(ty - y, tx - x) - th)
                sign = 1.0
                if abs(err) > math.pi / 2:
                    err = normalize_angle(err + math.pi)
                    sign = -1.0
                return (clamp(sign * K_NEAR_V * dist, V_HOME),
                        clamp(K_NEAR_W * err, W_MAX), 'run')
            x_r, y_r, th_r = reference_pose([self.start_xy, (tx, ty)],
                                            x, y, LOOKAHEAD)
            x_e, y_e, th_e = tracking_error(x, y, th, x_r, y_r, th_r)
            v, w = kanayama_cmd(x_e, y_e, th_e,
                                goal_scaled_vr(V_HOME, dist), **KAN_GAINS)
            return clamp(v, V_HOME), clamp(w, W_MAX), 'run'
        err = normalize_angle(tth - th)          # ALIGN
        if abs(err) < TOL_YAW:
            return 0.0, 0.0, 'done'
        return 0.0, clamp(K_TURN * err, W_MAX), 'run'


class UdpTwistSender:
    """{v, w, seq} JSON を udp_twist_bridge へ送る。stop() は零速度を3連送。"""

    def __init__(self, host, port=8890):
        self.addr = (str(host), int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

    def send(self, v, w):
        self.seq += 1
        self.sock.sendto(json.dumps(
            dict(v=round(float(v), 4), w=round(float(w), 4),
                 seq=self.seq)).encode(), self.addr)

    def stop(self):
        for _ in range(3):
            self.send(0.0, 0.0)
            time.sleep(0.02)

    def close(self):
        self.stop()
        self.sock.close()


def home(pose_fn, sender, floor_tags, target=(0.0, 0.0, 0.0), rate_hz=10.0,
         stop_event=None, timeout_s=TIMEOUT_S,
         sleep_fn=time.sleep, clock=time.monotonic):
    """原点復帰を1回実行。pose_fn() -> (x,y,th) | None（None=新鮮なpose無し）。

    sleep_fn / clock はテストで差し替える（実時間ゼロのsim実行）。
    戻り値 dict(ok, reason, pose, duration_s)。reason は最終status又は 'user_stop'。
    """
    ctrl = HomingController(target, course_bounds(floor_tags), timeout_s)
    t_start = clock()
    while True:
        if stop_event is not None and stop_event.is_set():
            sender.stop()
            return dict(ok=False, reason='user_stop', pose=None,
                        duration_s=clock() - t_start)
        pose = pose_fn()
        v, w, status = ctrl.step(pose, clock())
        if status == 'run':
            sender.send(v, w)
            sleep_fn(1.0 / rate_hz)
            continue
        sender.stop()
        return dict(ok=(status == 'done'), reason=status, pose=pose,
                    duration_s=clock() - t_start)
```

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/test_homing.py -q` → 5 passed
Run: `uv run pytest tests/ -q` → 全green
（収束テストが落ちる場合: ゲインいじりは禁止。原因（発振・平衡点）を数値で報告して停止）

- [ ] **Step 5: コミット**

```bash
git add rover/homing.py tests/test_homing.py
git commit -m "homing: カメラposeで原点復帰（TURN→GO→ALIGN・安全停止つき）"
```

---

### Task 4: truth_live（C270ライブ検出・start/stop記録）

**Files:**
- Create: `rover/truth_live.py`
- Modify: `rover/truth_offline.py`（load_config に live/udp セクションの受け渡しを追加）
- Modify: `configs/camera_truth.yaml`（`live:` セクション追記）
- Test: `tests/test_truth_live.py`

**Interfaces:**
- Consumes: `truth_core`（detect_tags 等）、`truth_offline.load_config / _solve_from_avg`、Task 1 の rows 形式
- Produces:
  - `CameraSource(device=0, width=1280, height=720)` — `read() -> (t_mono, gray) | None`、`release()`
  - `TruthLive(cfg, source)` — `calibrate(calib_frames=30, timeout_s=15.0)`（床タグでカメラ姿勢推定→キャプチャスレッド開始。失敗で CalibError）、`pose(max_age_s=1.0) -> (x,y,th) | None`、`start(csv_path)`、`stop() -> rows`（`[(t_rel_s, x, y, th, n_tags, quality_px)]`・CSVも書く）、`close()`
  - `load_config` の返り値に `live: dict`, `udp: dict` を追加（無ければ `{}`）

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_truth_live.py`:

```python
# -*- coding: utf-8 -*-
"""truth_live: 合成フレーム供給スレッドでライブ検出・記録を検証。"""
import queue
import time

import numpy as np

from synth_scene import DIST0, K_TEST, look_down_pose, render_scene, tag_corners3d
from test_truth_core import FLOOR_TAGS
from truth_live import TruthLive

CFG = dict(K=K_TEST, dist=DIST0, image_size=(1280, 720),
           floor_tags=FLOOR_TAGS,
           robot=dict(id=10, z_m=0.13, yaw_offset_rad=0.0))


class QueueSource:
    """テスト用: キューに積んだフレームを返す。空なら None。"""

    def __init__(self):
        self.q = queue.Queue()
        self.released = False

    def push(self, robot_pose=None):
        rvec, tvec = look_down_pose()
        tags = [(tid, tag_corners3d(xy, 0.15, yaw=0.1 * tid))
                for tid, xy in FLOOR_TAGS.items()]
        if robot_pose is not None:
            (rx, ry), ryaw = robot_pose
            tags.append((10, tag_corners3d((rx, ry), 0.12, yaw=ryaw, z=0.13)))
        self.q.put(render_scene((1280, 720), K_TEST, DIST0,
                                rvec, tvec, tags))

    def read(self):
        try:
            return time.monotonic(), self.q.get(timeout=0.05)
        except queue.Empty:
            return None

    def release(self):
        self.released = True


def _wait_until(fn, timeout=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if fn():
            return True
        time.sleep(0.02)
    return False


def test_live_calibrate_pose_and_recording():
    src = QueueSource()
    for _ in range(3):
        src.push(robot_pose=((0.2, 0.5), 0.1))
    live = TruthLive(CFG, src)
    live.calibrate(calib_frames=3)
    try:
        # 最新pose（キャリブ後のフレームで更新される）
        src.push(robot_pose=((0.3, 0.5), 0.1))
        assert _wait_until(lambda: live.pose(10.0) is not None)
        x, y, th = live.pose(10.0)
        assert np.hypot(x - 0.3, y - 0.5) < 0.01 and abs(th - 0.1) < 0.02
        # 記録: 2枚（うち1枚はロボット無し=欠測行）
        import tempfile
        from pathlib import Path
        out = Path(tempfile.mkdtemp()) / 't.csv'
        live.start(out)
        src.push(robot_pose=((0.4, 0.5), 0.1))
        src.push(robot_pose=None)
        assert _wait_until(lambda: src.q.empty())
        time.sleep(0.2)                       # スレッドが最後の行を書くまで
        rows = live.stop()
        assert len(rows) == 2
        assert abs(rows[0][1] - 0.4) < 0.01 and rows[1][1] is None
        assert out.exists() and 't_s' in out.read_text().splitlines()[0]
    finally:
        live.close()
    assert src.released


def test_live_pose_staleness():
    src = QueueSource()
    for _ in range(2):
        src.push(robot_pose=((0.2, 0.5), 0.0))
    live = TruthLive(CFG, src)
    live.calibrate(calib_frames=2)
    try:
        src.push(robot_pose=((0.2, 0.5), 0.0))
        assert _wait_until(lambda: live.pose(10.0) is not None)
        time.sleep(0.3)
        assert live.pose(max_age_s=0.1) is None   # 古いposeは返さない
    finally:
        live.close()
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `uv run pytest tests/test_truth_live.py -q`
Expected: FAIL（ModuleNotFoundError: truth_live）

- [ ] **Step 3: 実装**

`rover/truth_live.py`:

```python
# -*- coding: utf-8 -*-
"""ウェブカメラのライブ検出 → 最新ロボットpose／走行ごとCSV（Phase 2）。

truth_offline と同じ数式（床タグsolvePnP→視差補正）のライブ版。
セッション開始時に calibrate() でカメラ姿勢を1回推定し、以後は
キャプチャスレッドが最新poseを保持する。start/stop で走行区間を記録。
カメラは usbipd attach 済みの /dev/videoN（WSL2）を想定。
"""
import csv
import threading
import time

import cv2
import numpy as np

from truth_core import (CalibError, detect_tags, make_detector, robot_pose,
                        tag_center)
from truth_offline import _solve_from_avg


class CameraSource:
    """cv2.VideoCapture ラッパ。read() -> (t_mono, gray) | None。"""

    def __init__(self, device=0, width=1280, height=720):
        self.cap = cv2.VideoCapture(int(device))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        if not self.cap.isOpened():
            raise CalibError(f'カメラ /dev/video{device} を開けない'
                             '（usbipd attach 済みか確認）')

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        return time.monotonic(), cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def release(self):
        self.cap.release()


class TruthLive:
    """ライブ真値。calibrate() 成功後にキャプチャスレッドが走る。"""

    def __init__(self, cfg, source):
        self.cfg = cfg
        self.source = source
        self.detector = make_detector()
        self.rvec = self.tvec = None
        self.lock = threading.Lock()
        self.latest = None            # (t_mono, x, y, th, n_tags, q)
        self.rows = None              # start()〜stop() の間だけ list
        self.t0 = None
        self.csv_path = None
        self.stop_flag = threading.Event()
        self.thread = None
        self._size_checked = False

    def _check_size(self, gray):
        if self._size_checked:
            return
        expect = self.cfg.get('image_size')
        if expect is not None and \
                (gray.shape[1], gray.shape[0]) != tuple(expect):
            raise CalibError(
                f'フレーム {gray.shape[1]}x{gray.shape[0]} が設定 image_size '
                f'{tuple(expect)} と不一致（キャリブ時と同じ設定にする）')
        self._size_checked = True

    def calibrate(self, calib_frames=30, timeout_s=15.0):
        """床タグ4枚同時検出フレームを平均して solvePnP → スレッド開始。"""
        per_tag, used = {}, 0
        t_end = time.monotonic() + timeout_s
        while used < calib_frames and time.monotonic() < t_end:
            got = self.source.read()
            if got is None:
                continue
            _, gray = got
            self._check_size(gray)
            det = detect_tags(gray, self.detector)
            if all(t in det for t in self.cfg['floor_tags']):
                for tid in self.cfg['floor_tags']:
                    per_tag.setdefault(tid, []).append(tag_center(det[tid]))
                used += 1
        if used == 0:
            raise CalibError('床基準タグ4枚が同時に写らない（配置・照明・'
                             'ロボットのタグ隠しを確認）')
        self.rvec, self.tvec = _solve_from_avg(per_tag, self.cfg)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        rid = int(self.cfg['robot']['id'])
        z = float(self.cfg['robot']['z_m'])
        yofs = float(self.cfg['robot'].get('yaw_offset_rad', 0.0))
        while not self.stop_flag.is_set():
            got = self.source.read()
            if got is None:
                time.sleep(0.01)
                continue
            t, gray = got
            det = detect_tags(gray, self.detector)
            if rid in det:
                x, y, th = robot_pose(det[rid], self.cfg['K'],
                                      self.cfg['dist'], self.rvec, self.tvec,
                                      z, yofs)
                q = float(cv2.arcLength(det[rid].astype(np.float32),
                                        closed=True))
                rec = (t, x, y, th, len(det), q)
            else:
                rec = (t, None, None, None, len(det), 0.0)
            with self.lock:
                if rec[1] is not None:
                    self.latest = rec
                if self.rows is not None:
                    self.rows.append(rec)

    def pose(self, max_age_s=1.0):
        """新鮮な (x, y, th) か None（homing の pose_fn にそのまま渡せる）。"""
        with self.lock:
            latest = self.latest
        if latest is None or time.monotonic() - latest[0] > max_age_s:
            return None
        return latest[1], latest[2], latest[3]

    def start(self, csv_path):
        with self.lock:
            self.rows = []
            self.t0 = time.monotonic()
            self.csv_path = str(csv_path)

    def stop(self):
        """記録終了。CSVを書き、相対時刻の rows を返す（truth_metrics 入力）。"""
        with self.lock:
            rows, t0, path = self.rows, self.t0, self.csv_path
            self.rows = None
        rel = [(t - t0, x, y, th, n, q) for t, x, y, th, n, q in rows]
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'x_m', 'y_m', 'theta_rad', 'n_tags',
                        'quality_px'])
            for t, x, y, th, n, q in rel:
                w.writerow([f'{t:.3f}',
                            '' if x is None else f'{x:.4f}',
                            '' if y is None else f'{y:.4f}',
                            '' if th is None else f'{th:.4f}',
                            n, f'{q:.0f}'])
        return rel

    def close(self):
        self.stop_flag.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        self.source.release()
```

`rover/truth_offline.py` の `load_config` の return を次に差し替え（live/udp を透過）:

```python
    return dict(K=np.asarray(cam['K'], dtype=np.float64),
                dist=np.asarray(cam['dist'], dtype=np.float64),
                image_size=tuple(cam['image_size']),
                floor_tags={int(k): tuple(v) for k, v in floor.items()},
                robot=cfg['robot_tag'],
                live=cfg.get('live', {}),
                udp=cfg.get('udp', {}))
```

`configs/camera_truth.yaml` の末尾に追記:

```yaml
live:            # Phase 2: ライブ検出（WSL2は usbipd attach が必要）
  device: 0      # /dev/videoN の N
  width: 1280
  height: 720
```

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/test_truth_live.py -q` → 2 passed
Run: `uv run pytest tests/ -q` → 全green（truth_offline のテストが live/udp 追加で壊れないこと）

- [ ] **Step 5: コミット**

```bash
git add rover/truth_live.py tests/test_truth_live.py rover/truth_offline.py configs/camera_truth.yaml
git commit -m "truth_live: ウェブカメラのライブ検出とstart/stop記録（Phase 2）"
```

---

### Task 5: run_batch --auto（全自動ループ統合）

**Files:**
- Modify: `rover/run_batch.py`（CSV列追加・--auto・auto_batch関数・stdinリスナー）
- Modify: `rover/exp_backends.py`（RealBackend に auto/stop_event、watch_node に stop_event）
- Test: `tests/test_run_batch_auto.py`

**Interfaces:**
- Consumes: Task 1 `truth_metrics`、Task 3 `home / UdpTwistSender`、Task 4 `TruthLive / CameraSource`、`truth_offline.load_config`
- Produces:
  - `CSV_COLUMNS` に `truth_end_x, truth_end_y, truth_end_theta, truth_end_dist_cm, truth_rmse_cm` を `bagdir` の直前に挿入
  - `auto_batch(batch, backend, truth, sender, cfg, outdir, csv_path, ghash, v_r, stop_event, homing_fn, input_fn) -> None`（テスト可能なループ本体）
  - `RealBackend(batch, dry_run=False, auto=False, stop_event=None)`（auto時はEnter待ちを省略）
  - `watch_node(proc, deadline_s, stop_event=None)`（stop_eventで早期break）
  - CLI: `--auto`（real専用）、`--camera-config`（既定 configs/camera_truth.yaml）

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_run_batch_auto.py`:

```python
# -*- coding: utf-8 -*-
"""auto_batch のループ制御をfakeで検証（カメラ・SSH・ソケット不使用）。"""
import csv
import threading
from pathlib import Path

from run_batch import CSV_COLUMNS, auto_batch

BATCH = dict(name='t', path_file='configs/path_L_turn_1m.yaml', repeats=2,
             timeout_s=60, backend='real', common={}, sim={},
             conditions=[dict(name='l2', controller='l2')])
CFG = dict(floor_tags={0: (0.0, -0.2), 1: (0.4, 0.4), 2: (1.1, 1.1),
                       3: (1.2, -0.3)})


class FakeBackend:
    name = 'real'

    def __init__(self, oks):
        self.oks = list(oks)
        self.calls = 0

    def run_one(self, cond, rep, outdir):
        self.calls += 1
        return dict(ok=self.oks.pop(0), metrics=dict(rmse_cm=2.0),
                    bagdir='', note='')


class FakeTruth:
    def __init__(self):
        self.started = []

    def start(self, path):
        self.started.append(str(path))

    def stop(self):
        # ゴール到達済みの軌跡もどき（最後の0.5sは(1.0,1.0)付近）
        return [(0.1 * i, 1.0, 1.0, 1.57, 5, 300.0) for i in range(20)]

    def pose(self, max_age_s=1.0):
        return (1.0, 1.0, 1.57)


def _run(oks, homing_results, inputs=('',), stop=None):
    tmp = Path('/tmp/claude-autobatch-test')
    import shutil
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True)
    backend = FakeBackend(oks)
    truth = FakeTruth()
    calls = []
    hr = list(homing_results)

    def homing_fn(pose_fn, sender, floor_tags, stop_event=None):
        calls.append(1)
        return hr.pop(0) if hr else dict(ok=True, reason='done')

    it = iter(inputs)
    auto_batch(BATCH, backend, truth, sender=None, cfg=CFG, outdir=tmp,
               csv_path=tmp / 'runs.csv', ghash='x', v_r=0.1,
               stop_event=stop or threading.Event(),
               homing_fn=homing_fn, input_fn=lambda p: next(it))
    rows = list(csv.DictReader(open(tmp / 'runs.csv')))
    return backend, truth, calls, rows


def test_auto_batch_records_truth_and_homes_between_runs():
    backend, truth, calls, rows = _run(oks=[True, True],
                                       homing_results=[dict(ok=True,
                                                            reason='done')])
    assert backend.calls == 2 and len(rows) == 2
    assert len(calls) == 1                 # 復帰は「次がある」時だけ=1回
    assert truth.started == [str(Path('/tmp/claude-autobatch-test')
                                 / 'truth_l2_r1.csv'),
                             str(Path('/tmp/claude-autobatch-test')
                                 / 'truth_l2_r2.csv')]
    assert abs(float(rows[0]['truth_end_dist_cm'])) < 1e-6
    assert 'truth_end_x' in CSV_COLUMNS


def test_auto_batch_aborts_after_two_consecutive_failures():
    backend, _, _, rows = _run(oks=[False, False], homing_results=[])
    assert backend.calls == 2 and len(rows) == 2   # 3本目には進まない


def test_auto_batch_homing_retry_then_ask_human():
    backend, _, calls, _ = _run(
        oks=[True, True],
        homing_results=[dict(ok=False, reason='fail_timeout'),
                        dict(ok=False, reason='fail_timeout')],
        inputs=[''])
    assert len(calls) == 2                 # リトライ1回まで（その後は人へ）
    assert backend.calls == 2              # Enterで続行して2本目も走る


def test_auto_batch_q_stops_loop():
    stop = threading.Event()
    stop.set()
    backend, _, _, rows = _run(oks=[True, True], homing_results=[],
                               stop=stop)
    assert backend.calls == 0 and rows == []
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `uv run pytest tests/test_run_batch_auto.py -q`
Expected: FAIL（ImportError: cannot import name 'auto_batch'）

- [ ] **Step 3: 実装**

`rover/run_batch.py` の変更点（4か所）:

(a) CSV_COLUMNS を差し替え:

```python
CSV_COLUMNS = [
    'batch', 'cond', 'rep', 'backend', 'timestamp', 'git_hash',
    'controller', 'lam', 'move_suppress', 'horizon', 'v_r', 'ok',
    'drive_s', 'rmse_cm', 'sum_u', 'w_zero_ratio', 'flips', 'sat_ratio',
    'max_w', 'solve_p50', 'solve_p95', 'solve_max',
    'truth_end_x', 'truth_end_y', 'truth_end_theta',
    'truth_end_dist_cm', 'truth_rmse_cm',
    'bagdir', 'note',
]
```

(b) import 部（ファイル先頭）に追加: `import queue`, `import sys`, `import threading`

(c) main() の argparse に追加（既存の `ap.add_argument('--outdir', ...)` の直後・
`ap.parse_args()` より前に置く）:

```python
    ap.add_argument('--auto', action='store_true',
                    help='実機のみ: 真値記録＋自動原点復帰の全自動ループ')
    ap.add_argument('--camera-config', default='configs/camera_truth.yaml')
```

`if backend_kind == 'sim':` の分岐より**前**（backend選択の直前）に:

```python
    if args.auto and backend_kind != 'real':
        raise SystemExit('--auto は --backend real 専用です')
```

（else側にネストするとyaml既定のsimで発火しない＝レビューで検出済みの罠）

RealBackend 生成を次に変更（stop_event を先に作る）:

```python
        stop_event = threading.Event()
        backend = RealBackend(batch, dry_run=args.dry_run,
                              auto=args.auto, stop_event=stop_event)
        if not args.dry_run:
            n_runs = len(batch['conditions']) * int(batch['repeats'])
            extra = (f'\n  全{n_runs}本を自動実行（走行→真値→原点復帰）。'
                     '\n  q+Enter でいつでも停止・有人監視を続けること。'
                     if args.auto else '')
            ans = input(f"実機バッチ {batch['name']} を開始します。"
                        f"nav_base 起動済み・走行エリア確保を確認{extra} [y/N]: ")
            if ans.strip().lower() != 'y':
                print('中止しました')
                return
        backend.preflight()
```

`done = ...` の後・既存forループの前に auto 分岐:

```python
    if args.auto:
        from exp_backends import RPI_BRIDGE
        from homing import UdpTwistSender
        from truth_live import CameraSource, TruthLive
        from truth_offline import load_config
        cfg = load_config(args.camera_config)
        live = cfg.get('live', {})
        truth = TruthLive(cfg, CameraSource(live.get('device', 0),
                                            live.get('width', 1280),
                                            live.get('height', 720)))
        print('カメラ姿勢をキャリブレーション中（床タグ4枚が写ること）...')
        truth.calibrate()
        bridge_ok = backend._ssh(
            "pgrep -f '[u]dp_twist_bridge'").stdout.strip()
        if not bridge_ok:
            raise SystemExit(
                'RPiで udp_twist_bridge が起動していません:\n'
                '  source /opt/ros/humble/setup.bash && '
                f'python3 {RPI_BRIDGE} --port '
                f"{cfg.get('udp', {}).get('port', 8890)}")
        sender = UdpTwistSender(backend.host,
                                cfg.get('udp', {}).get('port', 8890))
        line_q = queue.Queue()
        _start_stdin_listener(stop_event, line_q)
        try:
            auto_batch(batch, backend, truth, sender, cfg, outdir,
                       csv_path, ghash, v_r, stop_event,
                       input_fn=lambda p: _prompt(p, line_q, stop_event))
        finally:
            sender.close()
            truth.close()
        if csv_path.exists():
            write_summary(outdir)
            print(f'完了: {outdir}/runs.csv, summary.md')
        return
```

（`RPI_BRIDGE` は `from exp_backends import ...` 行に追記して
`RPI_BRIDGE = f'{RPI_ROOT}/rover/udp_twist_bridge.py'` を exp_backends に定数追加）

(d) モジュールレベルに追加（main() の上）:

```python
def _start_stdin_listener(stop_event, line_q):
    """qでstop_event、その他の行はline_qへ（auto中のEnter待ちに使う）。"""
    def _listen():
        for line in sys.stdin:
            s = line.strip().lower()
            if s == 'q':
                stop_event.set()
                print('q受信: 停止します（走行中なら現走行の停止後）')
                break
            line_q.put(s)
    threading.Thread(target=_listen, daemon=True).start()


def _prompt(prompt, line_q, stop_event):
    print(prompt, end='', flush=True)
    while not stop_event.is_set():
        try:
            return line_q.get(timeout=0.2)
        except queue.Empty:
            continue
    return 'q'


def auto_batch(batch, backend, truth, sender, cfg, outdir, csv_path,
               ghash, v_r, stop_event, homing_fn=None, input_fn=input):
    """--auto のループ本体: 走行→真値stop→CSV→原点復帰→次へ。

    homing_fn / input_fn はテストで差し替える。復帰は「次の走行がある」
    ときだけ行う。復帰失敗はリトライ1回→それでも駄目なら人にEnterを求める。
    連続2本の走行失敗でループ停止（spec）。
    """
    from exp_metrics import truth_metrics
    if homing_fn is None:
        from homing import home as homing_fn
    waypoints, _ = load_path(batch['path_file'])
    runs = [(c, r) for c in batch['conditions']
            for r in range(1, int(batch['repeats']) + 1)]
    fails = 0
    for i, (cond, rep) in enumerate(runs):
        if stop_event.is_set():
            print('停止要求によりループ終了')
            break
        truth.start(Path(outdir) / f"truth_{cond['name']}_r{rep}.csv")
        try:
            result = backend.run_one(cond, rep, outdir)
        except Exception as e:
            result = dict(ok=False, metrics={}, bagdir='', note=f'error: {e}')
        rows = truth.stop()
        tm = truth_metrics(rows, waypoints)
        row = make_row(batch, cond, rep, backend.name, result, ghash, v_r)
        row.update({k: f'{v:.4f}' for k, v in tm.items()})
        append_row(csv_path, row)
        print(f"{cond['name']} rep{rep}: ok={result['ok']} "
              f"truth_end={tm.get('truth_end_dist_cm', float('nan')):.1f}cm")
        fails = 0 if result['ok'] else fails + 1
        if fails >= 2:
            print('連続2本失敗: ループを停止します（状態を確認して再開）')
            break
        if i == len(runs) - 1 or stop_event.is_set():
            break
        hres = homing_fn(lambda: truth.pose(1.0), sender,
                         cfg['floor_tags'], stop_event=stop_event)
        if not hres['ok'] and not stop_event.is_set():
            print(f"復帰失敗({hres['reason']}) → リトライ")
            hres = homing_fn(lambda: truth.pose(1.0), sender,
                             cfg['floor_tags'], stop_event=stop_event)
        if not hres['ok'] and not stop_event.is_set():
            ans = input_fn(f"復帰失敗({hres['reason']})。手で原点に戻して "
                           'Enter（qで中断）: ')
            if ans.strip().lower() == 'q':
                break
```

`rover/exp_backends.py` の変更点（3か所）:

(a) 定数追加（RPI_ROOT の直後）:

```python
RPI_BRIDGE = f'{RPI_ROOT}/rover/udp_twist_bridge.py'
```

(b) `watch_node` のシグネチャと監視ループに stop_event を追加:

```python
def watch_node(proc, deadline_s, stop_event=None):
```

while ループ内の `if not lines and proc.poll() is not None:` の直前に:

```python
        if stop_event is not None and stop_event.is_set():
            break        # q停止: finally 節の KILL_CMD が follower を止める
```

(c) `RealBackend.__init__` と run_one:

```python
    def __init__(self, batch, dry_run=False, auto=False, stop_event=None):
        self.batch = batch
        self.dry_run = dry_run
        self.auto = auto
        self.stop_event = stop_event
        self.host = None
```

run_one の Enter 待ち部分を差し替え:

```python
        if self.auto:
            print(f"\n[auto] {cond['name']} rep{rep}/{self.batch['repeats']} "
                  '走行開始')
        else:
            ans = input(f"\n次: {cond['name']} rep{rep}/"
                        f"{self.batch['repeats']}。"
                        f'ロボットを原点に戻して Enter（q で中断）: ')
            if ans.strip().lower() == 'q':
                raise KeyboardInterrupt
```

`watch_node(node, deadline)` の呼び出しを
`watch_node(node, deadline, self.stop_event)` に変更。

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/test_run_batch_auto.py -q` → 4 passed
Run: `uv run pytest tests/ -q` → 全green（既存 run_batch/backends テストの回帰確認）

- [ ] **Step 5: sim動作の回帰確認（非autoが壊れていないこと）**

Run: `uv run python rover/run_batch.py configs/batch_Lturn_1m_smoke.yaml --outdir /tmp/claude-smoke-sim-p23`
Expected: `l2 rep1: ok=True ...` と完了メッセージ

- [ ] **Step 6: コミット**

```bash
git add rover/run_batch.py rover/exp_backends.py tests/test_run_batch_auto.py
git commit -m "run_batch --auto: 走行→真値→自動原点復帰の全自動ループ"
```

---

### Task 6: 運用手順ドキュメント＋handoff/CLAUDE.md 更新＋push

**Files:**
- Create: `docs/作業記録/全自動バッチ運用手順.md`
- Modify: `docs/handoff.md`（セッション先頭に追記）、`CLAUDE.md`（現在地）

**Interfaces:**
- Consumes: Task 1〜5 の成果物一式

- [ ] **Step 1: 運用手順を書く**

`docs/作業記録/全自動バッチ運用手順.md`:

```markdown
# 全自動バッチ（走行→カメラ真値→自動原点復帰）運用手順

設計: specs/2026-07-16-camera-truth-pipeline-design.md（Phase 2+3）

## 前提（1回だけ）
1. カメラキャリブ・床タグ座標が configs/camera_truth.yaml に記入済み
   （手順: docs/作業記録/カメラ真値_精度検証手順.md）
2. usbipd で C270 を WSL に接続（管理者PowerShell）:
   `usbipd list` → `usbipd bind --busid <ID>` → `usbipd attach --wsl --busid <ID>`
   → WSLで `/dev/video0` を確認（PC再起動・抜き差し後は attach をやり直す）
3. RPi へ配置: `scp rover/udp_twist_bridge.py <rpi>:~/sparse_control/rover/`

## 毎セッション
1. RPi: nav_base 起動（1つだけ！）
   `ros2 launch lightrover_ros nav_base.launch.py`
2. RPi: ブリッジ起動
   `source /opt/ros/humble/setup.bash && python3 ~/sparse_control/rover/udp_twist_bridge.py`
3. ロボットを原点に置く（タグ中心を原点に・床タグ4枚がカメラに写ること）
4. laptop:
   `uv run python rover/run_batch.py configs/batch_<name>.yaml --backend real --auto --outdir results/<日付>_<name>`
   - 開始時に一括許可 [y/N]（全走行数が表示される）
   - 以後は全自動: 走行 → truth_*.csv 保存 → 原点復帰 → 次走行

## 安全（必ず有人監視）
- **q + Enter で即停止**（走行中は現走行のkill、復帰中は零速度送信）
- ブリッジ watchdog: UDP 0.5s 途絶で自動停止（WiFi断・laptopクラッシュ対策）
- 復帰中の安全停止: pose途絶1s / コース外接矩形+30cm逸脱 / 30sタイムアウト
- 復帰失敗はリトライ1回 → それでも失敗なら停止して人に Enter を求める
- 連続2本の走行失敗でループ自動停止

## 出力
- runs.csv に truth_end_x/y/theta・truth_end_dist_cm・truth_rmse_cm 列が追加
- 走行ごとの真値時系列: results/<dir>/truth_<cond>_r<rep>.csv
```

- [ ] **Step 2: handoff.md 先頭・CLAUDE.md 現在地を更新**

`docs/handoff.md` の当日セクションに追記:

```markdown
- カメラ真値 Phase 2+3 実装完了: truth_live（C270ライブ）・udp_twist_bridge
  （RPi・watchdog0.5s）・homing（TURN→GO→ALIGN、±3cm/±5°）・run_batch --auto
  （一括許可・q+Enter停止・連続2失敗で停止・truth列追加）。
  テストは fake/合成画像のみで全green。実機E2E（ブリッジ配置→復帰単体→
  短autoループ）は未実施＝次回冒頭にユーザー同席で行う。
  運用手順: docs/作業記録/全自動バッチ運用手順.md
```

`CLAUDE.md` の「次:」を更新:

```markdown
- 次: Phase 2+3 実機E2E（udp_twist_bridge配置 → homing単体の有人確認 →
  短いautoループ → フルバッチ）→ 外乱条件バッチを全自動で反復 →
  広角カメラ到着後に1.5mコースへ拡大 → 中間発表ストーリー整理
```

- [ ] **Step 3: 全テスト確認・コミット・push**

Run: `uv run pytest tests/ -q` → 全green

```bash
git add docs/作業記録/全自動バッチ運用手順.md docs/handoff.md CLAUDE.md
git commit -m "カメラ真値Phase2+3完了: 全自動バッチ運用手順とhandoff更新"
git push
```

---

## スコープ外（明示）

- spec の `--save-video`（検証用生動画の保存・既定OFF）は Phase 2+3 では実装しない
  （検証はライブCSVと手実測の突き合わせで足りる。必要になったら追加）
- summary.md への truth 列の集計追加は今回見送り（runs.csv に列はある）

## 実機E2E（計画外・ユーザー同席で実施）

コード完了後、以下の順で有人確認する（各ステップで許可を取る）:
1. `scp rover/udp_twist_bridge.py` → RPi、ブリッジ起動、laptopから
   `UdpTwistSender` で零速度送信 → ブリッジログに受信が出ること
2. homing 単体: ロボットをゴール付近に手で置き、pythonワンライナーで
   `home(truth.pose, sender, cfg['floor_tags'])` → 原点±3cm/±5°に戻ること
   （初回は手を添えられる距離で監視）
3. `--auto` で l2×2本の短ループ → 12本フルバッチ
