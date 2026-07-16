# -*- coding: utf-8 -*-
"""カメラ真値による自動原点復帰（Phase 3・laptop側）。

truth_live の pose を状態量に、follower_core の Kanayama で「現在地→原点」の
直線経路を追従し、最後にその場旋回で θ を合わせる（TURN→GO→ALIGN）。
速度指令は UDP で udp_twist_bridge（RPi）へ送る。
安全: pose途絶（pose_fnがNone）・コース外接矩形+30cm逸脱・タイムアウトで
即停止（零速度送信）。設計: specs/2026-07-16-camera-truth-pipeline-design.md
"""
import json
import math
import socket
import time

from follower_core import (clamp, goal_scaled_vr, kanayama_cmd,
                           normalize_angle, reference_pose, tracking_error)

V_HOME = 0.08
W_MAX = 1.0
TOL_POS = 0.03
TOL_YAW = math.radians(5)
TIMEOUT_S = 30.0
MARGIN_M = 0.30
KAN_GAINS = dict(k_x=0.5, k_y=5.0, k_th=3.0)   # exp_backends と同値
K_TURN = 3.0                    # その場旋回の比例ゲイン
TURN_ENTER = math.radians(45)   # これ以下の向き誤差で GO へ
LOOKAHEAD = 0.15
NEAR_R = 0.10                   # この距離からは点収束則（後退許可）に切替
K_NEAR_V, K_NEAR_W = 0.5, 2.0


def course_bounds(floor_tags, margin=MARGIN_M):
    """床タグ群の外接矩形＋余白 (x0, y0, x1, y1)。逸脱＝即停止の安全柵。"""
    xs = [x for x, _ in floor_tags.values()]
    ys = [y for _, y in floor_tags.values()]
    return (min(xs) - margin, min(ys) - margin,
            max(xs) + margin, max(ys) + margin)


class HomingController:
    """1tickごとに (v, w, status) を返す純ロジック。

    status: 'run'（継続） / 'done' / 'fail_pose' / 'fail_bounds' /
    'fail_timeout'。TURN=目標方向へ旋回 → GO=Kanayamaで直線追従 →
    ALIGN=最終θ合わせ。
    """

    def __init__(self, target=(0.0, 0.0, 0.0), bounds=None,
                 timeout_s=TIMEOUT_S):
        self.target = target
        self.bounds = bounds
        self.timeout_s = timeout_s
        self.t0 = None
        self.phase = 'TURN'
        self.start_xy = None

    def step(self, pose, t):
        if self.t0 is None:
            self.t0 = t
        if t - self.t0 > self.timeout_s:
            return 0.0, 0.0, 'fail_timeout'
        if pose is None:
            return 0.0, 0.0, 'fail_pose'
        x, y, th = pose
        if self.bounds is not None:
            x0, y0, x1, y1 = self.bounds
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                return 0.0, 0.0, 'fail_bounds'
        tx, ty, tth = self.target
        dist = math.hypot(tx - x, ty - y)
        if self.phase in ('TURN', 'GO') and dist < TOL_POS:
            self.phase = 'ALIGN'
        if self.phase == 'TURN':
            if self.start_xy is None:
                self.start_xy = (x, y)
            err = normalize_angle(math.atan2(ty - y, tx - x) - th)
            if abs(err) >= TURN_ENTER:
                return 0.0, clamp(K_TURN * err, W_MAX), 'run'
            self.phase = 'GO'
        if self.phase == 'GO':
            if dist < NEAR_R:
                # Kanayama は「行き過ぎ」で前進FFと引き戻しが釣り合い
                # 平衡点ができる（simで4.8cm停滞を確認）→ 後退を許す
                # go-to-point 則で点収束させる
                err = normalize_angle(math.atan2(ty - y, tx - x) - th)
                sign = 1.0
                if abs(err) > math.pi / 2:
                    err = normalize_angle(err + math.pi)
                    sign = -1.0
                return (clamp(sign * K_NEAR_V * dist, V_HOME),
                        clamp(K_NEAR_W * err, W_MAX), 'run')
            x_r, y_r, th_r = reference_pose([self.start_xy, (tx, ty)],
                                            x, y, LOOKAHEAD)
            x_e, y_e, th_e = tracking_error(x, y, th, x_r, y_r, th_r)
            v, w = kanayama_cmd(x_e, y_e, th_e,
                                goal_scaled_vr(V_HOME, dist), **KAN_GAINS)
            return clamp(v, V_HOME), clamp(w, W_MAX), 'run'
        err = normalize_angle(tth - th)          # ALIGN
        if abs(err) < TOL_YAW:
            return 0.0, 0.0, 'done'
        return 0.0, clamp(K_TURN * err, W_MAX), 'run'


class UdpTwistSender:
    """{v, w, seq} JSON を udp_twist_bridge へ送る。stop() は零速度を3連送。"""

    def __init__(self, host, port=8890):
        self.addr = (str(host), int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

    def send(self, v, w):
        self.seq += 1
        self.sock.sendto(json.dumps(
            dict(v=round(float(v), 4), w=round(float(w), 4),
                 seq=self.seq)).encode(), self.addr)

    def stop(self):
        for _ in range(3):
            self.send(0.0, 0.0)
            time.sleep(0.02)

    def close(self):
        self.stop()
        self.sock.close()


def home(pose_fn, sender, floor_tags, target=(0.0, 0.0, 0.0), rate_hz=10.0,
         stop_event=None, timeout_s=TIMEOUT_S,
         sleep_fn=time.sleep, clock=time.monotonic):
    """原点復帰を1回実行。pose_fn() -> (x,y,th) | None（None=新鮮なpose無し）。

    sleep_fn / clock はテストで差し替える（実時間ゼロのsim実行）。
    戻り値 dict(ok, reason, pose, duration_s)。reason は最終status又は 'user_stop'。
    """
    ctrl = HomingController(target, course_bounds(floor_tags), timeout_s)
    t_start = clock()
    while True:
        if stop_event is not None and stop_event.is_set():
            sender.stop()
            return dict(ok=False, reason='user_stop', pose=None,
                        duration_s=clock() - t_start)
        pose = pose_fn()
        v, w, status = ctrl.step(pose, clock())
        if status == 'run':
            sender.send(v, w)
            sleep_fn(1.0 / rate_hz)
            continue
        sender.stop()
        return dict(ok=(status == 'done'), reason=status, pose=pose,
                    duration_s=clock() - t_start)
