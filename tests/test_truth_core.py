# -*- coding: utf-8 -*-
"""truth_core の合成画像テスト（設計spec: 合成レンダリングで精度担保）。"""
import cv2
import numpy as np

from synth_scene import (DIST0, K_TEST, look_down_pose, render_scene,
                         tag_corners3d)
from truth_core import detect_tags, make_detector, tag_center


def test_render_and_detect_roundtrip():
    """合成画像のタグが正しいIDで検出され、四隅が投影位置と1px以内で一致。"""
    rvec, tvec = look_down_pose()
    c3d = tag_corners3d((0.4, 0.6), 0.15, yaw=0.3)
    img = render_scene((1280, 720), K_TEST, DIST0, rvec, tvec, [(3, c3d)])
    det = detect_tags(img, make_detector())
    assert set(det) == {3}
    proj, _ = cv2.projectPoints(c3d, rvec, tvec, K_TEST, DIST0)
    assert np.allclose(det[3], proj.reshape(4, 2), atol=1.0)


def test_detect_tags_empty_image():
    """タグなし画像 → 空dict（Noneでない）。"""
    img = np.full((720, 1280), 255, np.uint8)
    assert detect_tags(img, make_detector()) == {}


def test_tag_center_is_corner_mean():
    corners = np.array([[0., 0.], [10., 0.], [10., 10.], [0., 10.]])
    assert np.allclose(tag_center(corners), [5.0, 5.0])


FLOOR_TAGS = {0: (-0.20, -0.20), 1: (1.70, -0.20),
              2: (1.70, 1.70), 3: (-0.20, 1.70)}


def _floor_scene(rvec, tvec):
    tags = [(tid, tag_corners3d(xy, 0.15, yaw=0.1 * tid))
            for tid, xy in FLOOR_TAGS.items()]
    return render_scene((1280, 720), K_TEST, DIST0, rvec, tvec, tags)


def test_solve_camera_pose_recovers_camera_center():
    """床タグ4枚からカメラ中心 (0.75,0.75,2.4) を2cm以内で復元。"""
    from truth_core import camera_center, solve_camera_pose
    rvec, tvec = look_down_pose()
    det = detect_tags(_floor_scene(rvec, tvec), make_detector())
    rv, tv = solve_camera_pose(FLOOR_TAGS, det, K_TEST, DIST0)
    assert np.allclose(camera_center(rv, tv), [0.75, 0.75, 2.4], atol=0.065)


def test_solve_camera_pose_needs_4_tags():
    """3枚しか写らないと CalibError。"""
    import pytest
    from truth_core import CalibError, solve_camera_pose
    rvec, tvec = look_down_pose()
    det = detect_tags(_floor_scene(rvec, tvec), make_detector())
    det.pop(0)
    with pytest.raises(CalibError):
        solve_camera_pose(FLOOR_TAGS, det, K_TEST, DIST0)
