# -*- coding: utf-8 -*-
"""チェスボード動画からカメラ内部パラメータ (K, dist) を求めるCLI。

使い方:
  uv run python rover/calibrate_camera.py <チェスボード動画>
      [--cols 9] [--rows 6] [--square-m 0.024] [--max-frames 30]
出力: 再投影誤差と、configs/camera_truth.yaml に貼る K / dist のyaml片。
チェスボードは9x6内側コーナー（10x7マス）をA4印刷し、動画は
カメラを固定・ボードを傾けながら全域で撮る（本番と同じ解像度設定で）。
"""
import argparse

import cv2
import numpy as np


def calibrate(grays, cols=9, rows=6, square_m=0.024):
    """グレースケール画像群 → (K, dist(5,), 再投影誤差px)。有効<3枚でValueError。"""
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    obj_pts, img_pts = [], []
    size = None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    for g in grays:
        size = (g.shape[1], g.shape[0])
        ok, corners = cv2.findChessboardCorners(g, (cols, rows))
        if not ok:
            continue
        corners = cv2.cornerSubPix(g, corners, (11, 11), (-1, -1), crit)
        obj_pts.append(objp)
        img_pts.append(corners)
    if len(obj_pts) < 3:
        raise ValueError(f'チェスボード検出 {len(obj_pts)} 枚（3枚以上必要）')
    # k3は固定（通常画角では不要で、ビュー多様性が足りないと暴走する。
    # 広角カメラ導入時にk3が要るなら再検討）
    err, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size,
                                             None, None,
                                             flags=cv2.CALIB_FIX_K3)
    return K, dist.flatten()[:5], err


def main():
    ap = argparse.ArgumentParser(description='カメラ内部パラメータのキャリブ')
    ap.add_argument('video')
    ap.add_argument('--cols', type=int, default=9)
    ap.add_argument('--rows', type=int, default=6)
    ap.add_argument('--square-m', type=float, default=0.024)
    ap.add_argument('--max-frames', type=int, default=30)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    step = max(1, total // args.max_frames)
    grays = []
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i % step == 0:
            grays.append(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        i += 1
    cap.release()

    K, dist, err = calibrate(grays, args.cols, args.rows, args.square_m)
    print(f'フレーム {len(grays)} 枚使用 / 再投影誤差 {err:.3f} px'
          f'（0.5px以下が目安）')
    print('--- configs/camera_truth.yaml に貼り付け ---')
    h, w = grays[0].shape
    print(f'  image_size: [{w}, {h}]')
    print('  K:')
    for row in K:
        print(f'    - [{row[0]:.2f}, {row[1]:.2f}, {row[2]:.2f}]')
    print(f'  dist: [{", ".join(f"{d:.5f}" for d in dist)}]')


if __name__ == '__main__':
    main()
