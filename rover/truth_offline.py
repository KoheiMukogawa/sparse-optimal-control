# -*- coding: utf-8 -*-
"""俯瞰動画 → コース座標の真値CSV (t,x,y,theta) を出すオフラインCLI。

使い方:
  uv run python rover/truth_offline.py <動画> --config configs/camera_truth.yaml
      [--out 出力CSV] [--calib-frames 30] [--path-file configs/path_L_turn.yaml]

処理: 1パス目で床基準タグ4枚の中心を calib_frames 枚ぶん平均し
solvePnP（カメラは固定前提）。2パス目で全フレームのロボットタグを
視差補正つきでコース座標化してCSVへ。末尾30フレームで床タグから
カメラ姿勢を再推定し、1cm超ズレていたら「カメラが動いた」警告を出す。
iPhoneは「カメラ設定→フォーマット→互換性優先(H.264)」での撮影を推奨。
"""
import argparse
import csv
import math
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import yaml

from truth_core import (CalibError, camera_center, detect_tags,
                        make_detector, robot_pose, solve_camera_pose,
                        tag_center)


def load_config(path):
    """camera_truth.yaml → dict(K, dist, floor_tags, robot)。未記入を検知。"""
    with open(path) as f:
        cfg = yaml.safe_load(f)
    cam = cfg['camera']
    if cam.get('K') is None or cam.get('dist') is None:
        raise CalibError('camera.K/dist が未記入。rover/calibrate_camera.py '
                         'の出力を貼り付けること')
    floor = cfg['floor_tags']
    if any(v is None for v in floor.values()):
        raise CalibError('floor_tags に未記入あり。タグ中心のコース座標[m]を実測して記入')
    return dict(K=np.asarray(cam['K'], dtype=np.float64),
                dist=np.asarray(cam['dist'], dtype=np.float64),
                floor_tags={int(k): tuple(v) for k, v in floor.items()},
                robot=cfg['robot_tag'])


def _solve_from_avg(per_tag, cfg):
    """{tid: [中心px,...]} を平均して solvePnP。擬似検出dictを作って渡す。"""
    det = {tid: np.tile(np.mean(pts, axis=0), (4, 1))
           for tid, pts in per_tag.items()}
    return solve_camera_pose(cfg['floor_tags'], det, cfg['K'], cfg['dist'])


def run_video(frames_fn, cfg, calib_frames=30):
    """フレーム列を2パス処理して (rows, info) を返す。

    rows: [(t, x, y, theta, n_tags, quality_px)]。欠測は x,y,theta=None。
    quality_px はロボットタグの周長（解像度・ブレの代理指標）。
    """
    detector = make_detector()
    # ---- 1パス目: 床タグ中心を平均してカメラ姿勢 ----
    per_tag, used = {}, 0
    for _, gray in frames_fn():
        det = detect_tags(gray, detector)
        if all(t in det for t in cfg['floor_tags']):
            for tid in cfg['floor_tags']:
                per_tag.setdefault(tid, []).append(tag_center(det[tid]))
            used += 1
            if used >= calib_frames:
                break
    if used == 0:
        raise CalibError('床基準タグ4枚が同時に写るフレームがない')
    rvec, tvec = _solve_from_avg(per_tag, cfg)

    # ---- 2パス目: ロボットpose ----
    rid = int(cfg['robot']['id'])
    z = float(cfg['robot']['z_m'])
    yofs = float(cfg['robot'].get('yaw_offset_rad', 0.0))
    rows = []
    tail = deque(maxlen=30)   # 末尾のカメラずれ検知用
    for t, gray in frames_fn():
        det = detect_tags(gray, detector)
        if all(k in det for k in cfg['floor_tags']):
            tail.append({tid: tag_center(det[tid])
                         for tid in cfg['floor_tags']})
        if rid in det:
            x, y, th = robot_pose(det[rid], cfg['K'], cfg['dist'],
                                  rvec, tvec, z, yofs)
            q = float(cv2.arcLength(
                det[rid].astype(np.float32), closed=True))
            rows.append((t, x, y, th, len(det), q))
        else:
            rows.append((t, None, None, None, len(det), 0.0))

    drift = float('nan')
    if tail:
        per_tag2 = {}
        for d in tail:
            for tid, c in d.items():
                per_tag2.setdefault(tid, []).append(c)
        rv2, tv2 = _solve_from_avg(per_tag2, cfg)
        drift = float(np.linalg.norm(
            camera_center(rv2, tv2) - camera_center(rvec, tvec)))
    n_valid = sum(1 for r in rows if r[1] is not None)
    info = dict(cam_center=camera_center(rvec, tvec), cam_drift_m=drift,
                n_frames=len(rows), n_valid=n_valid)
    return rows, info


def _video_frames(path):
    """動画ファイル → (t_s, gray) イテレータを返す callable。"""
    def gen():
        cap = cv2.VideoCapture(str(path))
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            yield i / fps, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            i += 1
        cap.release()
    return gen


def _endpoint(rows, window_s=0.5):
    """末尾 window_s の有効行の平均 pose（θは円形平均）。無ければ None。"""
    valid = [r for r in rows if r[1] is not None]
    if not valid:
        return None
    t_end = valid[-1][0]
    win = [r for r in valid if r[0] >= t_end - window_s]
    x = float(np.mean([r[1] for r in win]))
    y = float(np.mean([r[2] for r in win]))
    th = float(np.arctan2(np.mean([np.sin(r[3]) for r in win]),
                          np.mean([np.cos(r[3]) for r in win])))
    return x, y, th


def main():
    ap = argparse.ArgumentParser(description='俯瞰動画→真値CSV')
    ap.add_argument('video')
    ap.add_argument('--config', default='configs/camera_truth.yaml')
    ap.add_argument('--out')
    ap.add_argument('--calib-frames', type=int, default=30)
    ap.add_argument('--path-file', help='終点をゴールと比較する経路yaml')
    args = ap.parse_args()

    cfg = load_config(args.config)
    rows, info = run_video(_video_frames(args.video), cfg,
                           args.calib_frames)
    out = Path(args.out or Path(args.video).with_suffix('.truth.csv'))
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'x_m', 'y_m', 'theta_rad', 'n_tags', 'quality_px'])
        for t, x, y, th, n, q in rows:
            w.writerow([f'{t:.3f}',
                        '' if x is None else f'{x:.4f}',
                        '' if y is None else f'{y:.4f}',
                        '' if th is None else f'{th:.4f}', n, f'{q:.0f}'])

    print(f'{out}: {info["n_valid"]}/{info["n_frames"]} フレーム有効')
    cc = info['cam_center']
    print(f'カメラ位置 ({cc[0]:.2f}, {cc[1]:.2f}, 高さ {cc[2]:.2f}) m / '
          f'セッション中のずれ {info["cam_drift_m"] * 100:.1f} cm')
    if info['cam_drift_m'] > 0.01:
        print('警告: カメラが動いた可能性。この動画の値は信頼しないこと')
    ep = _endpoint(rows)
    if ep:
        x, y, th = ep
        print(f'終点: x={x * 100:.1f}cm y={y * 100:.1f}cm '
              f'θ={math.degrees(th):.1f}°')
        if args.path_file:
            with open(args.path_file) as f:
                gx, gy = yaml.safe_load(f)['waypoints'][-1]
            d = math.hypot(x - gx, y - gy)
            print(f'ゴール({gx}, {gy})からの距離: {d * 100:.1f} cm')


if __name__ == '__main__':
    main()
