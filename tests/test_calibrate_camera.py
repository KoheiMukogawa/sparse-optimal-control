# -*- coding: utf-8 -*-
"""calibrate_camera の合成チェスボードテスト。"""
import cv2
import numpy as np

from calibrate_camera import calibrate
from synth_scene import DIST0, K_TEST


def _board_img(cols=9, rows=6, sq=60, margin=80):
    """内側コーナー cols×rows の市松ビットマップ（白余白つき）。"""
    pattern = (np.indices((rows + 1, cols + 1)).sum(axis=0) % 2)
    img = np.kron(pattern, np.ones((sq, sq))) * 255
    return cv2.copyMakeBorder(img.astype(np.uint8), margin, margin,
                              margin, margin, cv2.BORDER_CONSTANT, value=255)


def _render_views(n=12, cols=9, rows=6, sq=60, margin=80):
    """ボードを複数姿勢で射影した合成ビュー群（歪みゼロ・K_TEST）。

    回転±0.5rad・並進±0.06m のばらつきが必要（正対に近いビューだけだと
    焦点距離と距離が縮退して fx が2%超ずれる。実測: ±0.3では719、±0.5で701.4）。
    """
    board = _board_img(cols, rows, sq, margin)
    h, w = board.shape
    # ビットマップ四隅の3D座標: 1マス=0.024m とし、余白も同スケールで換算
    s = 0.024 / sq
    obj4 = np.array([[-margin * s, -margin * s, 0],
                     [(w - margin) * s, -margin * s, 0],
                     [(w - margin) * s, (h - margin) * s, 0],
                     [-margin * s, (h - margin) * s, 0]])
    src = np.array([[0, 0], [w, 0], [w, h], [0, h]], np.float32)
    rng = np.random.default_rng(0)
    views = []
    for _ in range(n):
        rvec = rng.uniform(-0.5, 0.5, 3) + np.array([np.pi, 0, 0])
        tvec = np.array([[0.05], [0.1], [0.9]]) + rng.uniform(-0.06, 0.06, (3, 1))
        # ボード原点をカメラ正面に置く: コース系は使わず board系→cam 直指定
        proj, _ = cv2.projectPoints(obj4, rvec, tvec, K_TEST, DIST0)
        H = cv2.getPerspectiveTransform(src, proj.reshape(4, 2).astype(np.float32))
        views.append(cv2.warpPerspective(board, H, (1280, 720),
                                         borderValue=255))
    return views


def test_calibrate_recovers_intrinsics():
    K, dist, err = calibrate(_render_views(), cols=9, rows=6, square_m=0.024)
    assert abs(K[0, 0] - 700) / 700 < 0.02
    assert abs(K[1, 1] - 700) / 700 < 0.02
    assert err < 1.0
    assert np.all(np.abs(dist) < 0.05)


def test_calibrate_rejects_too_few():
    import pytest
    with pytest.raises(ValueError):
        calibrate([np.full((720, 1280), 255, np.uint8)] * 3)
