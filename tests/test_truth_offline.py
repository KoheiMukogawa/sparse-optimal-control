# -*- coding: utf-8 -*-
"""truth_offline: 合成フレーム列 → 真値rows の検証（動画コーデック非依存）。"""
import numpy as np

from synth_scene import (DIST0, K_TEST, look_down_pose, render_scene,
                         tag_corners3d)
from test_truth_core import FLOOR_TAGS
from truth_offline import run_video

CFG = dict(K=K_TEST, dist=DIST0,
           floor_tags=FLOOR_TAGS,
           robot=dict(id=10, z_m=0.13, yaw_offset_rad=0.0))


def _traj_frames(n=40, drop=(15, 16)):
    """直線移動するロボットの合成フレーム列と真値。dropはロボットタグ無し。"""
    rvec, tvec = look_down_pose()
    floor = [(tid, tag_corners3d(xy, 0.15, yaw=0.1 * tid))
             for tid, xy in FLOOR_TAGS.items()]
    frames, truth = [], []
    for i in range(n):
        x = 0.2 + 0.02 * i
        pose = ((x, 0.5), 0.1)   # (中心xy, yaw)
        tags = list(floor)
        if i not in drop:
            tags.append((10, tag_corners3d(pose[0], 0.12,
                                           yaw=pose[1], z=0.13)))
        frames.append((i / 30.0,
                       render_scene((1280, 720), K_TEST, DIST0,
                                    rvec, tvec, tags)))
        truth.append(pose)
    return frames, truth


def test_run_video_tracks_trajectory():
    frames, truth = _traj_frames()
    rows, info = run_video(lambda: iter(frames), CFG, calib_frames=5)
    assert info['n_frames'] == 40 and info['n_valid'] == 38
    assert info['cam_drift_m'] < 0.01
    for row, ((tx, ty), tyaw) in zip(rows, truth):
        t, x, y, th, n_tags, q = row
        if x is None:
            continue
        assert np.hypot(x - tx, y - ty) < 0.01
        assert abs((th - tyaw + np.pi) % (2 * np.pi) - np.pi) < 0.02


def test_run_video_marks_missing():
    frames, _ = _traj_frames()
    rows, _ = run_video(lambda: iter(frames), CFG, calib_frames=5)
    assert rows[15][1] is None and rows[16][1] is None


def test_run_video_checks_image_size():
    """K はキャリブ解像度にしか合わない: 一致で通過、不一致は CalibError。"""
    import pytest

    from truth_core import CalibError
    frames, _ = _traj_frames(n=5, drop=())
    ok_cfg = dict(CFG, image_size=(1280, 720))
    rows, _ = run_video(lambda: iter(frames), ok_cfg, calib_frames=2)
    assert rows[0][1] is not None
    bad_cfg = dict(CFG, image_size=(1920, 1080))
    with pytest.raises(CalibError):
        run_video(lambda: iter(frames), bad_cfg, calib_frames=2)
