# -*- coding: utf-8 -*-
"""合成シーン: 既知のカメラ姿勢・タグ配置からテスト画像をレンダリングする。

truth_core の精度テスト用。マーカー画像を射影ワープで白背景キャンバスへ
描き込む。歪みゼロ前提（projectPoints と warpPerspective の整合が厳密）。
"""
import cv2
import numpy as np

from truth_core import TAG_DICT_ID

DICT = cv2.aruco.getPredefinedDictionary(TAG_DICT_ID)
K_TEST = np.array([[700.0, 0, 640], [0, 700.0, 360], [0, 0, 1]])
DIST0 = np.zeros(5)
MARKER_PX = 160  # 36h11は8モジュール角 → 20px/モジュール


def look_down_pose(cam_xy=(0.75, 0.75), height=2.4, roll_deg=3.0):
    """ほぼ真下向きカメラの (rvec, tvec)（コース座標→カメラ座標）。

    真下向きはx軸まわりπ回転（カメラz軸=-z_w, y軸=-y_w）、
    roll_deg でわずかな設置傾きを模擬する。
    """
    R, _ = cv2.Rodrigues(np.array([np.pi + np.radians(roll_deg), 0.0, 0.0]))
    C = np.array([cam_xy[0], cam_xy[1], height])
    rvec, _ = cv2.Rodrigues(R)
    tvec = (-R @ C).reshape(3, 1)
    return rvec, tvec


def tag_corners3d(center_xy, size_m, yaw=0.0, z=0.0):
    """床置き(z=0)/ロボット上面(z=h)タグの四隅3D座標（正準順[左上,右上,右下,左下]）。

    タグ正準+x（左辺→右辺）がコースx軸から yaw だけ回転して置かれているとする。
    上から見た正準配置: 左上=(-h,+h), 右上=(+h,+h), 右下=(+h,-h), 左下=(-h,-h)。
    """
    h = size_m / 2
    u = np.array([np.cos(yaw), np.sin(yaw)])   # 正準+x（コース系）
    v = np.array([-np.sin(yaw), np.cos(yaw)])  # 正準+y（コース系, 上から見て）
    c = np.asarray(center_xy, dtype=np.float64)
    pts2 = [c - h * u + h * v, c + h * u + h * v,
            c + h * u - h * v, c - h * u - h * v]
    return np.array([[p[0], p[1], z] for p in pts2])


def render_scene(size_px, K, dist, rvec, tvec, tags):
    """tags: [(tag_id, corners3d(4,3)), ...] を白背景に描画して返す。"""
    img = np.full((size_px[1], size_px[0]), 255, np.uint8)
    pad = MARKER_PX // 8  # 静穏域1モジュール分
    side = MARKER_PX + 2 * pad
    src = np.array([[pad, pad], [pad + MARKER_PX, pad],
                    [pad + MARKER_PX, pad + MARKER_PX],
                    [pad, pad + MARKER_PX]], np.float32)
    for tid, c3d in tags:
        proj, _ = cv2.projectPoints(c3d, rvec, tvec, K, dist)
        proj = proj.reshape(4, 2).astype(np.float32)
        canvas = np.full((side, side), 255, np.uint8)
        canvas[pad:pad + MARKER_PX, pad:pad + MARKER_PX] = \
            cv2.aruco.generateImageMarker(DICT, tid, MARKER_PX)
        # 縮小ワープのエイリアシング対策に軽くぼかす
        canvas = cv2.GaussianBlur(canvas, (5, 5), 1.5)
        H = cv2.getPerspectiveTransform(src, proj)
        warped = cv2.warpPerspective(canvas, H, size_px,
                                     flags=cv2.INTER_LINEAR, borderValue=0)
        mask = cv2.warpPerspective(np.full((side, side), 255, np.uint8), H,
                                   size_px, flags=cv2.INTER_NEAREST)
        img[mask > 127] = warped[mask > 127]
    return img
