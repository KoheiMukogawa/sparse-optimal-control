# -*- coding: utf-8 -*-
"""ウェブカメラのライブ検出 → 最新ロボットpose／走行ごとCSV（Phase 2）。

truth_offline と同じ数式（床タグsolvePnP→視差補正）のライブ版。
セッション開始時に calibrate() でカメラ姿勢を1回推定し、以後は
キャプチャスレッドが最新poseを保持する。start/stop で走行区間を記録。
カメラは usbipd attach 済みの /dev/videoN（WSL2）を想定。
"""
import csv
import threading
import time

import cv2
import numpy as np

from truth_core import (CalibError, detect_tags, make_detector, robot_pose,
                        tag_center)
from truth_offline import _solve_from_avg


class CameraSource:
    """cv2.VideoCapture ラッパ。read() -> (t_mono, gray) | None。"""

    def __init__(self, device=0, width=1280, height=720):
        self.cap = cv2.VideoCapture(int(device))
        # C270はYUYVだと720pで~10fpsに落ち、V4L2バッファ滞留で0.2-0.4sの
        # 遅延が乗る（homingのALIGN精度に効く）→ MJPG＋バッファ1に固定
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        if not self.cap.isOpened():
            raise CalibError(f'カメラ /dev/video{device} を開けない'
                             '（usbipd attach 済みか確認）')

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        return time.monotonic(), cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def release(self):
        self.cap.release()


class TruthLive:
    """ライブ真値。calibrate() 成功後にキャプチャスレッドが走る。"""

    def __init__(self, cfg, source):
        self.cfg = cfg
        self.source = source
        self.detector = make_detector()
        self.rvec = self.tvec = None
        self.lock = threading.Lock()
        self.latest = None            # (t_mono, x, y, th, n_tags, q)
        self.rows = None              # start()〜stop() の間だけ list
        self.t0 = None
        self.csv_path = None
        self.stop_flag = threading.Event()
        self.thread = None
        self._size_checked = False

    def _check_size(self, gray):
        if self._size_checked:
            return
        expect = self.cfg.get('image_size')
        if expect is not None and \
                (gray.shape[1], gray.shape[0]) != tuple(expect):
            raise CalibError(
                f'フレーム {gray.shape[1]}x{gray.shape[0]} が設定 image_size '
                f'{tuple(expect)} と不一致（キャリブ時と同じ設定にする）')
        self._size_checked = True

    def calibrate(self, calib_frames=30, timeout_s=15.0):
        """床タグ4枚同時検出フレームを平均して solvePnP → スレッド開始。"""
        per_tag, used = {}, 0
        t_end = time.monotonic() + timeout_s
        while used < calib_frames and time.monotonic() < t_end:
            got = self.source.read()
            if got is None:
                continue
            _, gray = got
            self._check_size(gray)
            det = detect_tags(gray, self.detector)
            if all(t in det for t in self.cfg['floor_tags']):
                for tid in self.cfg['floor_tags']:
                    per_tag.setdefault(tid, []).append(tag_center(det[tid]))
                used += 1
        if used == 0:
            raise CalibError('床基準タグ4枚が同時に写らない（配置・照明・'
                             'ロボットのタグ隠しを確認）')
        self.rvec, self.tvec = _solve_from_avg(per_tag, self.cfg)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        rid = int(self.cfg['robot']['id'])
        z = float(self.cfg['robot']['z_m'])
        yofs = float(self.cfg['robot'].get('yaw_offset_rad', 0.0))
        while not self.stop_flag.is_set():
            got = self.source.read()
            if got is None:
                time.sleep(0.01)
                continue
            t, gray = got
            det = detect_tags(gray, self.detector)
            if rid in det:
                x, y, th = robot_pose(det[rid], self.cfg['K'],
                                      self.cfg['dist'], self.rvec, self.tvec,
                                      z, yofs)
                q = float(cv2.arcLength(det[rid].astype(np.float32),
                                        closed=True))
                rec = (t, x, y, th, len(det), q)
            else:
                rec = (t, None, None, None, len(det), 0.0)
            with self.lock:
                if rec[1] is not None:
                    self.latest = rec
                if self.rows is not None:
                    self.rows.append(rec)

    def pose(self, max_age_s=1.0):
        """新鮮な (x, y, th) か None（homing の pose_fn にそのまま渡せる）。"""
        with self.lock:
            latest = self.latest
        if latest is None or time.monotonic() - latest[0] > max_age_s:
            return None
        return latest[1], latest[2], latest[3]

    def start(self, csv_path):
        with self.lock:
            self.rows = []
            self.t0 = time.monotonic()
            self.csv_path = str(csv_path)

    def stop(self):
        """記録終了。CSVを書き、相対時刻の rows を返す（truth_metrics 入力）。"""
        with self.lock:
            rows, t0, path = self.rows, self.t0, self.csv_path
            self.rows = None
        rel = [(t - t0, x, y, th, n, q) for t, x, y, th, n, q in rows]
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'x_m', 'y_m', 'theta_rad', 'n_tags',
                        'quality_px'])
            for t, x, y, th, n, q in rel:
                w.writerow([f'{t:.3f}',
                            '' if x is None else f'{x:.4f}',
                            '' if y is None else f'{y:.4f}',
                            '' if th is None else f'{th:.4f}',
                            n, f'{q:.0f}'])
        return rel

    def close(self):
        self.stop_flag.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        self.source.release()
