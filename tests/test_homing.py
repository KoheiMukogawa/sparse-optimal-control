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
