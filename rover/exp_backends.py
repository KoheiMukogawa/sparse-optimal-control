# -*- coding: utf-8 -*-
"""バッチ実験ランナーの実行バックエンド。

SimBackend : ROS不要。差動二輪プラント＋入力遅延＋測定ノイズの閉ループsim
             （sim_delay_probe.py と同機序）。開発・λ/msスイープ用。
RealBackend: SSHでRPiのノード起動・bag記録・回収を行う実機用（Task 6で追加）。

両者は run_one(cond, rep, outdir) -> dict(ok, metrics, bagdir, note) を実装する。
設計: docs/superpowers/specs/2026-07-14-batch-runner-design.md
"""

import math
import random
from collections import deque
from pathlib import Path

import yaml

from exp_metrics import compute_metrics
from follower_core import (clamp, goal_crossed, goal_scaled_vr, kanayama_cmd,
                           reference_pose, tracking_error)
from mpc_core import MPCFollower

V_MAX, W_MAX = 0.15, 2.0          # mpc_follower.py と同値
LOOKAHEAD = 0.15
GOAL_TOL = 0.05
KAN_GAINS = dict(k_x=0.5, k_y=5.0, k_th=3.0)  # path_follower.py 既定値

REPO_ROOT = Path(__file__).resolve().parent.parent


def load_path(path_file):
    """configs/path_*.yaml → (waypoints, v_r)。リポジトリルート相対も可。"""
    p = Path(path_file)
    if not p.exists():
        p = REPO_ROOT / path_file
    with open(p) as f:
        cfg = yaml.safe_load(f)
    waypoints = [(float(a), float(b)) for a, b in cfg['waypoints']]
    return waypoints, float(cfg.get('v_r', 0.1))


def sim_run(cond, common, waypoints, v_r, sim_opts, timeout_s, seed):
    """1本の閉ループsim。時系列と到達可否を返す。"""
    ts = 1.0 / float(common.get('rate', 10.0))
    delay = int(sim_opts.get('delay_steps', 2))
    pn = float(sim_opts.get('pos_noise', 0.0))
    yn = float(sim_opts.get('yaw_noise', 0.0))
    rng = random.Random(seed)

    mpc = None
    if cond['controller'] in ('l2', 'l1'):
        mpc = MPCFollower(N=int(common.get('horizon', 15)), ts=ts,
                          reg=cond['controller'],
                          lam=float(cond.get('lam', 0.3)),
                          v_max=V_MAX, w_max=W_MAX,
                          move_suppress=float(cond.get('move_suppress', 0.0)))

    x = y = th = t = 0.0
    buf = deque()
    twist, perr, solve_ms = [], [], []
    ok = False
    while t < timeout_s:
        gx, gy = waypoints[-1]
        gd = math.hypot(gx - x, gy - y)
        if gd < GOAL_TOL or goal_crossed(waypoints, x, y):
            ok = True
            break
        # 測定（ノイズ付き）→ 制御器
        xm = x + rng.gauss(0, pn)
        ym = y + rng.gauss(0, pn)
        thm = th + rng.gauss(0, yn)
        x_r, y_r, th_r = reference_pose(waypoints, xm, ym, LOOKAHEAD)
        x_e, y_e, th_e = tracking_error(xm, ym, thm, x_r, y_r, th_r)
        vr = goal_scaled_vr(v_r, gd)
        if mpc is not None:
            cmd = mpc.command(x_e, y_e, th_e, vr, w_r=0.0)
            solve_ms.append(mpc.last_solve_s * 1e3)
            if cmd is None:
                break  # 求解失敗 → 未達で終了（実機の安全停止に対応）
        else:
            v, w = kanayama_cmd(x_e, y_e, th_e, vr, **KAN_GAINS)
            cmd = (clamp(v, V_MAX), clamp(w, W_MAX))
        # 入力遅延: delay 前の指令を適用（sim_delay_probe.py と同一）
        buf.append(cmd)
        v, w = buf.popleft() if len(buf) > delay else (vr, 0.0)
        twist.append((t, v, w))
        perr.append((t, y_e))
        # 真の非線形プラントを1ステップ
        x += v * math.cos(th) * ts
        y += v * math.sin(th) * ts
        th += w * ts
        t += ts
    return dict(ok=ok, twist=twist, perr=perr, solve_ms=solve_ms)


class SimBackend:
    """ROS不要のsimバックエンド。outdir は使わない（bagを作らない）。"""

    name = 'sim'

    def __init__(self, batch):
        self.batch = batch
        self.waypoints, self.v_r = load_path(batch['path_file'])

    def run_one(self, cond, rep, outdir):
        data = sim_run(cond, self.batch.get('common', {}),
                       self.waypoints, self.v_r,
                       self.batch.get('sim', {}),
                       float(self.batch.get('timeout_s', 60)),
                       seed=rep)
        if not data['twist']:
            return dict(ok=data['ok'], metrics={}, bagdir='',
                        note='走行データなし（即到達または初手求解失敗）')
        metrics = compute_metrics(data['twist'], data['perr'], data['solve_ms'])
        note = '' if data['ok'] else 'タイムアウト/求解失敗'
        return dict(ok=data['ok'], metrics=metrics, bagdir='', note=note)
