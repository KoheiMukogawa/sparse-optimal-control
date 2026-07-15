# -*- coding: utf-8 -*-
"""カメラ真値計測の純ロジック（I/Oなし）。

AprilTag(36h11) を cv2.aruco で検出し、床基準タグ4枚の中心実測座標から
solvePnP でカメラ外部姿勢を推定、ロボット上面タグ（高さ z_m）を
光線と平面 z=z_m の交点で厳密にコース座標へ変換する。
座標・角度は SI（m・rad）。
設計: docs/superpowers/specs/2026-07-16-camera-truth-pipeline-design.md
"""
import cv2
import numpy as np

TAG_DICT_ID = cv2.aruco.DICT_APRILTAG_36h11


class CalibError(RuntimeError):
    """セッション開始不能（床基準タグ不足・設定不備など）。"""


def make_detector():
    d = cv2.aruco.getPredefinedDictionary(TAG_DICT_ID)
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return cv2.aruco.ArucoDetector(d, p)


def detect_tags(gray, detector):
    """グレースケール画像 → {tag_id: corners(4,2)float64}。

    corners はマーカー正準向きで [左上, 右上, 右下, 左下] の画素座標
    （ビットパターンの復号で決まるため、カメラの回転には依存しない）。
    """
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2).astype(np.float64)
            for i, c in zip(ids.flatten(), corners)}


def tag_center(corners):
    return corners.mean(axis=0)
