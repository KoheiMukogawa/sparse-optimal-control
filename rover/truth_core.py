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


def solve_camera_pose(floor_tags, detections, K, dist, max_residual_px=2.0):
    """床基準タグ中心（コース座標z=0）と検出中心から solvePnP。

    floor_tags: {id: (x_m, y_m)}。検出できた共通タグが4枚未満なら CalibError。
    返り値 (rvec, tvec): コース座標→カメラ座標の剛体変換。
    中心のみ使う（タグの向き・印刷サイズの実測が不要になるため）。
    残差 max_residual_px 超で CalibError（タグ移動・yaml陳腐化の検出）。
    """
    obj, img = [], []
    for tid, xy in floor_tags.items():
        if tid in detections:
            obj.append([xy[0], xy[1], 0.0])
            img.append(tag_center(detections[tid]))
    if len(obj) < 4:
        raise CalibError(f'床基準タグ検出 {len(obj)}/4 枚（4枚必要）')
    # 正対面付近の平面PnPは IPPE だと画素ノイズがcm級に増幅されるため、
    # ホモグラフィ初期化＋LM精緻化の ITERATIVE を使う（4点共面でOK）
    try:
        ok, rvec, tvec = cv2.solvePnP(
            np.asarray(obj, dtype=np.float64),
            np.asarray(img, dtype=np.float64),
            K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    except cv2.error as e:
        # 4枚がほぼ一直線だとホモグラフィ初期化が退化してcv2.errorになる
        raise CalibError('solvePnP 失敗: 床タグ4枚がほぼ一直線の疑い。'
                         '広がる（三角形以上の面積を持つ）配置に貼り直す') from e
    if not ok:
        raise CalibError('solvePnP 失敗')
    proj, _ = cv2.projectPoints(
        np.asarray(obj, dtype=np.float64), rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(
        np.sum((proj.reshape(-1, 2) - np.asarray(img)) ** 2, axis=1))))
    if rms > max_residual_px:
        raise CalibError(
            f'床タグ再投影残差 {rms:.1f}px（>{max_residual_px}px）: '
            'タグが動いた/floor_tags が古い疑い。survey_tags.py を再実行')
    return rvec, tvec


def camera_center(rvec, tvec):
    """カメラ中心のコース座標（設置ズレ検知にも使う）。"""
    R, _ = cv2.Rodrigues(rvec)
    return (-R.T @ np.asarray(tvec).reshape(3, 1)).flatten()


def pixel_to_plane(pt, K, dist, rvec, tvec, z):
    """画素座標 pt を高さ z[m] の水平面へ逆投影（光線と平面の交点）。

    ロボット上面タグは床から z_m 浮いているため、床ホモグラフィだと
    コース端で数cmの視差誤差が出る。カメラ姿勢既知なので厳密に解く。
    """
    xn = cv2.undistortPoints(
        np.asarray(pt, dtype=np.float64).reshape(1, 1, 2), K, dist)[0, 0]
    R, _ = cv2.Rodrigues(rvec)
    cam = (-R.T @ np.asarray(tvec).reshape(3, 1)).flatten()
    ray = R.T @ np.array([xn[0], xn[1], 1.0])
    s = (z - cam[2]) / ray[2]
    return (cam + s * ray)[:2]


def robot_pose(corners, K, dist, rvec, tvec, z, yaw_offset=0.0):
    """ロボット上面タグ corners → (x, y, theta) コース座標。

    θ はタグ正準+x方向（左辺中点→右辺中点）を平面 z へ投影して算出し、
    yaw_offset（タグ+xと機体前方のズレ）を差し引く。θは[-π,π]。
    """
    c = pixel_to_plane(tag_center(corners), K, dist, rvec, tvec, z)
    left = pixel_to_plane((corners[0] + corners[3]) / 2,
                          K, dist, rvec, tvec, z)
    right = pixel_to_plane((corners[1] + corners[2]) / 2,
                           K, dist, rvec, tvec, z)
    d = right - left
    theta = np.arctan2(d[1], d[0]) - yaw_offset
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    return float(c[0]), float(c[1]), float(theta)
