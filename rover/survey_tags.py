# -*- coding: utf-8 -*-
"""床タグ自動サーベイ: 1コマから floor_tags を推定し yaml を更新する。

毎セッション、タグを適当に置いて本CLIを1回実行すればメジャー実測が不要になる。
アルゴリズム: タグごと IPPE_SQUARE で初期解 → 全16隅の再投影誤差を
一括最小化（拘束: 床平面z=0・実寸正方形、ゲージ: tag0原点・tag0→tag3=+x）。
設計: docs/superpowers/specs/2026-07-17-tag-auto-survey-design.md
"""
import numpy as np
import cv2
import re
import datetime
from pathlib import Path
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


MIN_HULL_AREA_M2 = 0.3


def to_course_frame(tags_xy):
    """中間フレーム→コース座標（原点=tag0中心+0.20m·(+y)）。"""
    return {tid: (float(x), float(y - ORIGIN_OFFSET_M))
            for tid, (x, y) in tags_xy.items()}


def check_layout(tags_xy, waypoints):
    """共線・面積・コース内包チェック。違反は CalibError。

    waypoints: コース座標の経路頂点 [(x, y), ...]（tags_xy と同じ系で渡す）。
    コースが凸包の外＝外挿地帯は実績10-15cm誤差（7/16）のため拒否する。
    """
    hull = cv2.convexHull(
        np.array([tags_xy[t] for t in FLOOR_IDS], dtype=np.float32))
    area = float(cv2.contourArea(hull))
    if area < MIN_HULL_AREA_M2:
        raise CalibError(
            f'床タグ凸包が小さすぎる（{area:.2f}m²<{MIN_HULL_AREA_M2}）: '
            'ほぼ一直線か密集。コースを囲む四隅に広げて置き直す')
    # 経路を5cm刻みでサンプルして全点が凸包内にあるか
    bad = []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(2, int(np.linalg.norm(b - a) / 0.05) + 1)
        for s in np.linspace(0.0, 1.0, n):
            p = a + s * (b - a)
            if cv2.pointPolygonTest(hull, (float(p[0]), float(p[1])),
                                    False) < 0:
                bad.append(p)
    if bad:
        c = np.mean(bad, axis=0)
        raise CalibError(
            f'コースがタグ凸包の外（外挿地帯・{len(bad)}点、'
            f'中心 x={c[0]:.2f} y={c[1]:.2f} 付近）: '
            'その方向のタグをコースの外側へ動かして囲む配置にする')


def update_yaml(path, tags_xy, rms_px, when=None):
    """configs/camera_truth.yaml の floor_tags ブロックだけ書き換える。"""
    path = Path(path)
    text = path.read_text()
    when = when or datetime.date.today().isoformat()
    block = ['floor_tags:      # id: [x_m, y_m]。survey_tags.py が自動生成',
             f'  # {when} 自動サーベイ（再投影RMS {rms_px:.2f}px）']
    for tid in FLOOR_IDS:
        x, y = tags_xy[tid]
        block.append(f'  {tid}: [{x:.3f}, {y:.3f}]')
    new, n = re.subn(r'floor_tags:.*?(?=\nrobot_tag:)',
                     '\n'.join(block) + '\n', text, flags=re.S)
    if n != 1:
        raise CalibError(f'{path} の floor_tags ブロックを特定できない'
                         '（floor_tags: と robot_tag: の並びが前提）')
    path.write_text(new)
