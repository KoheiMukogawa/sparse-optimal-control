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
