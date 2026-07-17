# -*- coding: utf-8 -*-
"""survey_tags のテスト（全て合成画像。実カメラ不使用）。"""
import numpy as np
import pytest

from synth_scene import (K_TEST, DIST0, look_down_pose, tag_corners3d,
                         render_scene)
from truth_core import CalibError, detect_tags, make_detector
from survey_tags import FLOOR_IDS, initial_guess, square_corners3d

SIZE = 0.15
# 7/16実配置ふう。tag0→tag3 が+xに揃わない一般配置
TRUE = {0: (0.0, -0.2, 0.1), 1: (0.43, 0.4, -0.3), 2: (1.13, 1.15, 0.05),
        3: (1.19, -0.3, 0.2)}


def to_intermediate(true_tags):
    """真値配置を中間フレーム（tag0原点・tag0→tag3=+x）へ変換した期待値。"""
    p0 = np.array(true_tags[0][:2])
    d = np.array(true_tags[3][:2]) - p0
    ang = np.arctan2(d[1], d[0])
    R = np.array([[np.cos(-ang), -np.sin(-ang)],
                  [np.sin(-ang), np.cos(-ang)]])
    return {tid: R @ (np.array(v[:2]) - p0) for tid, v in true_tags.items()}


def make_det(true_tags, cam_kw=None):
    rvec, tvec = look_down_pose(**(cam_kw or {}))
    img = render_scene((1280, 720), K_TEST, DIST0, rvec, tvec,
                       [(tid, tag_corners3d(v[:2], SIZE, yaw=v[2]))
                        for tid, v in true_tags.items()])
    det = detect_tags(img, make_detector())
    assert set(FLOOR_IDS) <= set(det), '合成シーンで床タグ4枚が検出できていない'
    return det


def test_square_corners3d_matches_synth_scene():
    got = square_corners3d((0.3, -0.1), SIZE, yaw=0.4)
    want = tag_corners3d((0.3, -0.1), SIZE, yaw=0.4)
    assert np.allclose(got, want)


def test_initial_guess_recovers_layout_roughly():
    det = make_det(TRUE)
    _, _, tags0 = initial_guess(det, SIZE, K_TEST, DIST0)
    want = to_intermediate(TRUE)
    assert tags0[0][:2] == (0.0, 0.0)
    assert abs(tags0[3][1]) < 1e-9          # ゲージ: tag3 は y=0
    for tid in FLOOR_IDS:                    # 初期解は5cm以内でよい
        assert np.linalg.norm(np.array(tags0[tid][:2]) - want[tid]) < 0.05
