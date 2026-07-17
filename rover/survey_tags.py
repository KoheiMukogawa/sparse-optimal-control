# -*- coding: utf-8 -*-
"""床タグ自動サーベイ: 1コマから floor_tags を推定し yaml を更新する。

毎セッション、タグを適当に置いて本CLIを1回実行すればメジャー実測が不要になる。
アルゴリズム: タグごと IPPE_SQUARE で初期解 → 全16隅の再投影誤差を
一括最小化（拘束: 床平面z=0・実寸正方形、ゲージ: tag0原点・tag0→tag3=+x）。
設計: docs/superpowers/specs/2026-07-17-tag-auto-survey-design.md
"""
import numpy as np
import cv2
from scipy.optimize import least_squares

from truth_core import CalibError

FLOOR_IDS = (0, 1, 2, 3)
ORIGIN_OFFSET_M = 0.20   # 原点 = tag0中心 + 0.20m·(+y)


def square_corners3d(center_xy, size_m, yaw=0.0):
    """床置きタグの四隅3D座標（正準順[左上,右上,右下,左下]、z=0）。"""
    h = size_m / 2
    u = np.array([np.cos(yaw), np.sin(yaw)])
    v = np.array([-np.sin(yaw), np.cos(yaw)])
    c = np.asarray(center_xy, dtype=np.float64)
    pts2 = [c - h * u + h * v, c + h * u + h * v,
            c + h * u - h * v, c - h * u - h * v]
    return np.array([[p[0], p[1], 0.0] for p in pts2])


def _tag_pose_ippe(corners, size_m, K, dist):
    """タグ1枚の姿勢（タグ正準系→カメラ系）。初期値用途。"""
    h = size_m / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
                   dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, np.asarray(corners, dtype=np.float64),
                                  K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise CalibError('タグ単体PnP失敗（検出品質を疑う）')
    return rvec, tvec


def initial_guess(det, size_m, K, dist):
    """タグごとPnP → 床平面フィット → 中間フレームの初期値。

    中間フレーム: tag0中心=原点、tag0→tag3の平面内方向=+x、法線=+z。
    返り値: (rvec0, tvec0, tags0)。tags0={id: (x, y, yaw)}、tag3のyは厳密0。
    """
    missing = [t for t in FLOOR_IDS if t not in det]
    if missing:
        raise CalibError(f'床タグ未検出: id={missing}（4枚必要）')
    centers, xaxes = {}, {}
    for tid in FLOOR_IDS:
        rvec, tvec = _tag_pose_ippe(det[tid], size_m, K, dist)
        R, _ = cv2.Rodrigues(rvec)
        centers[tid] = np.asarray(tvec).flatten()
        xaxes[tid] = R[:, 0]
    # 4中心に平面フィット（SVD）。法線はカメラ原点側へ向ける＝世界+z（上）
    P = np.array([centers[t] for t in FLOOR_IDS])
    mid = P.mean(axis=0)
    _, _, Vt = np.linalg.svd(P - mid)
    n = Vt[2]
    if n @ (-mid) < 0:
        n = -n
    x = centers[3] - centers[0]
    x = x - (x @ n) * n
    nx = np.linalg.norm(x)
    if nx < 0.3:
        raise CalibError('tag0とtag3が近すぎる（0.3m以上離して置く）')
    x = x / nx
    y = np.cross(n, x)
    R_cw = np.stack([x, y, n], axis=1)   # 世界軸をカメラ系で表した列
    rvec0, _ = cv2.Rodrigues(R_cw)
    tvec0 = centers[0].reshape(3, 1)     # p_c = R_cw @ p_w + tvec0
    tags0 = {}
    for tid in FLOOR_IDS:
        pw = R_cw.T @ (centers[tid] - centers[0])
        ax = R_cw.T @ xaxes[tid]
        tags0[tid] = (float(pw[0]), float(pw[1]),
                      float(np.arctan2(ax[1], ax[0])))
    tags0[0] = (0.0, 0.0, tags0[0][2])
    tags0[3] = (tags0[3][0], 0.0, tags0[3][2])
    return rvec0, tvec0, tags0


MAX_RMS_PX = 1.0


def _unpack(p):
    """最適化パラメータ→(rvec, tvec, tags)。ゲージはtag0=(0,0)・tag3のy=0。"""
    tags = {0: (0.0, 0.0, p[6]),
            1: (p[7], p[8], p[9]),
            2: (p[10], p[11], p[12]),
            3: (p[13], 0.0, p[14])}
    return p[:3], p[3:6].reshape(3, 1), tags


def _residuals(p, det, size_m, K, dist):
    rvec, tvec, tags = _unpack(p)
    res = []
    for tid in FLOOR_IDS:
        x, y, yaw = tags[tid]
        proj, _ = cv2.projectPoints(square_corners3d((x, y), size_m, yaw),
                                    rvec, tvec, K, dist)
        res.append((proj.reshape(4, 2) - det[tid]).ravel())
    return np.concatenate(res)


def refine(det, size_m, K, dist):
    """16隅の一括最適化。返り値 ({id: (x, y)} 中間フレーム, 再投影RMS[px])。

    単体PnPの奥行きノイズを床平面拘束＋16点同時フィットで抑える。
    RMS > MAX_RMS_PX なら CalibError（配置・照明・キャリブずれの疑い）。
    """
    rvec0, tvec0, tags0 = initial_guess(det, size_m, K, dist)
    p0 = np.concatenate([
        np.asarray(rvec0).ravel(), np.asarray(tvec0).ravel(),
        [tags0[0][2]],
        [tags0[1][0], tags0[1][1], tags0[1][2]],
        [tags0[2][0], tags0[2][1], tags0[2][2]],
        [tags0[3][0], tags0[3][2]]])
    sol = least_squares(_residuals, p0, args=(det, size_m, K, dist),
                        method='lm')
    err = sol.fun.reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))
    if rms > MAX_RMS_PX:
        raise CalibError(
            f'サーベイ再投影残差 {rms:.2f}px（>{MAX_RMS_PX}px）: '
            'タグの平坦性・照明・キャリブを疑う。置き直して再実行')
    _, _, tags = _unpack(sol.x)
    return {tid: (float(t[0]), float(t[1]))
            for tid, t in tags.items()}, rms
