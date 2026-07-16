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
    assert np.allclose(camera_center(rv, tv), [0.75, 0.75, 2.4], atol=0.02)


def test_solve_camera_pose_rejects_collinear_tags():
    """床タグ4枚が一直線だと退化 → 分かるメッセージの CalibError。"""
    import pytest

    from truth_core import CalibError, solve_camera_pose
    line_tags = {0: (-0.2, -0.2), 1: (0.4, -0.2), 2: (1.0, -0.2),
                 3: (1.7, -0.2)}
    rvec, tvec = look_down_pose()
    tags = [(tid, tag_corners3d(xy, 0.15, yaw=0.1 * tid))
            for tid, xy in line_tags.items()]
    img = render_scene((1280, 720), K_TEST, DIST0, rvec, tvec, tags)
    det = detect_tags(img, make_detector())
    with pytest.raises(CalibError, match='一直線'):
        solve_camera_pose(line_tags, det, K_TEST, DIST0)


def test_solve_camera_pose_needs_4_tags():
    """3枚しか写らないと CalibError。"""
    import pytest
    from truth_core import CalibError, solve_camera_pose
    rvec, tvec = look_down_pose()
    det = detect_tags(_floor_scene(rvec, tvec), make_detector())
    det.pop(0)
    with pytest.raises(CalibError):
        solve_camera_pose(FLOOR_TAGS, det, K_TEST, DIST0)


def _full_scene(robot_xy, robot_yaw, z=0.13):
    """床タグ4枚＋ロボットタグ(id10, 12cm, 高さz)のシーンと真のカメラ姿勢。"""
    rvec, tvec = look_down_pose()
    tags = [(tid, tag_corners3d(xy, 0.15, yaw=0.1 * tid))
            for tid, xy in FLOOR_TAGS.items()]
    tags.append((10, tag_corners3d(robot_xy, 0.12, yaw=robot_yaw, z=z)))
    img = render_scene((1280, 720), K_TEST, DIST0, rvec, tvec, tags)
    return img, rvec, tvec


def test_robot_pose_accuracy():
    """コース中央と端の複数姿勢で 位置≤1cm・角度≤0.02rad。"""
    from truth_core import robot_pose, solve_camera_pose
    cases = [((0.75, 0.75), 0.0), ((1.50, 0.20), 2.0), ((0.10, 1.40), -2.5)]
    for xy, yaw in cases:
        img, _, _ = _full_scene(xy, yaw)
        det = detect_tags(img, make_detector())
        rv, tv = solve_camera_pose(FLOOR_TAGS, det, K_TEST, DIST0)
        x, y, th = robot_pose(det[10], K_TEST, DIST0, rv, tv, 0.13)
        assert np.hypot(x - xy[0], y - xy[1]) < 0.01, (xy, yaw)
        dth = (th - yaw + np.pi) % (2 * np.pi) - np.pi
        assert abs(dth) < 0.02, (xy, yaw)


def test_parallax_correction_is_necessary():
    """z=0 のホモグラフィ扱いだとコース端で3cm超ズレる（補正の必要性の担保）。"""
    from truth_core import pixel_to_plane, solve_camera_pose, tag_center
    img, _, _ = _full_scene((1.50, 0.20), 0.0)
    det = detect_tags(img, make_detector())
    rv, tv = solve_camera_pose(FLOOR_TAGS, det, K_TEST, DIST0)
    c_px = tag_center(det[10])
    p_ok = pixel_to_plane(c_px, K_TEST, DIST0, rv, tv, 0.13)
    p_naive = pixel_to_plane(c_px, K_TEST, DIST0, rv, tv, 0.0)
    assert np.linalg.norm(p_ok - p_naive) > 0.03
