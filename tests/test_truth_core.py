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
