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
import select
import shlex
import subprocess
import sys
import time
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


# ---- RealBackend（SSHオーケストレーション） -----------------------------------
# RPi側は従来の手動手順（handoff.md 環境メモ）を自動化しただけ:
#   nav_base は事前にユーザーが起動 / ノード起動→bag record→SIGINT停止→scp回収。
# 安全系はノード既存機構（到達停止・SIGINT停止・odom途絶停止）に委ねる。

SSH_USER = 'mukougawakouhei'
SSH_HOSTS = ['192.168.0.31', '192.168.4.1']   # 家Wi-Fi → ホットスポットの順
RPI_ROOT = '/home/mukougawakouhei/sparse_control'
BAG_TOPICS = '/odom /rover_twist /path_error /mpc_solve_ms'
SYNC_FILES = ['rover/mpc_follower.py', 'rover/path_follower.py',
              'rover/follower_core.py', 'rover/mpc_core.py']
STARTUP_MARGIN_S = {'kanayama': 15, 'l2': 90, 'l1': 90}  # cvxpy import 30-60s


def node_command(cond, common, path_file):
    """RPi上で実行する follower ノード起動コマンド文字列。"""
    params = [f'-p path_file:={RPI_ROOT}/{path_file}']
    if cond['controller'] == 'kanayama':
        script = 'path_follower.py'
    else:
        script = 'mpc_follower.py'
        params += [f"-p reg:={cond['controller']}",
                   f"-p lam:={cond.get('lam', 0.3)}",
                   f"-p move_suppress:={cond.get('move_suppress', 0.0)}",
                   f"-p horizon:={common.get('horizon', 15)}",
                   f"-p rate:={common.get('rate', 10.0)}"]
    return (f'cd {RPI_ROOT}/rover && python3 {script} --ros-args '
            + ' '.join(params))


def bag_record_command(remote_dir):
    return f'ros2 bag record -o {remote_dir} {BAG_TOPICS}'


KILL_CMD = ("pkill -INT -f '[m]pc_follower.py'; "
            "pkill -INT -f '[p]ath_follower.py'; true")
KILL_BAG_CMD = "pkill -INT -f '[r]os2 bag record'; true"


class RealBackend:
    """SSHでRPiを操作する実機バックエンド。1本ごとに Enter 待ち＝走行許可。"""

    name = 'real'

    def __init__(self, batch, dry_run=False):
        self.batch = batch
        self.dry_run = dry_run
        self.host = None

    def _ssh_args(self, cmd):
        return ['ssh', f'{SSH_USER}@{self.host}',
                f'bash -lc {shlex.quote(cmd)}']

    def _ssh(self, cmd, timeout=30):
        return subprocess.run(self._ssh_args(cmd), capture_output=True,
                              text=True, timeout=timeout)

    def preflight(self):
        """SSH疎通・nav_base稼働・コード同期を確認（バッチ開始時に1回）。"""
        if self.dry_run:
            self.host = SSH_HOSTS[0]
            print(f'[dry-run] preflight: ssh {SSH_USER}@{self.host} で '
                  f'ros2 node list / md5sum 確認を行う')
            return
        for h in SSH_HOSTS:
            try:
                if subprocess.run(['ssh', '-o', 'ConnectTimeout=5',
                                   f'{SSH_USER}@{h}', 'true'],
                                  timeout=15).returncode == 0:
                    self.host = h
                    break
            except subprocess.TimeoutExpired:
                pass
        if self.host is None:
            raise RuntimeError(f'RPiにSSH接続できません: {SSH_HOSTS}')
        print(f'RPi接続: {self.host}')

        nodes = self._ssh('source /opt/ros/humble/setup.bash 2>/dev/null; '
                          'ros2 node list').stdout
        if 'pos_controller' not in nodes or 'odom_manager' not in nodes:
            raise RuntimeError(
                'nav_base が稼働していません。RPiで先に起動してください:\n'
                '  ros2 launch lightrover_ros nav_base.launch.py')

        # コード同期チェック（差異は警告のみ・配置更新は手動）
        for f in SYNC_FILES:
            local = subprocess.run(['md5sum', str(REPO_ROOT / f)],
                                   capture_output=True,
                                   text=True).stdout.split()[0]
            remote_out = self._ssh(f'md5sum {RPI_ROOT}/{f}').stdout.split()
            if not remote_out or remote_out[0] != local:
                ans = input(f'警告: {f} がRPi側と異なります（要scp配置）。'
                            f'続行しますか [y/N]: ')
                if ans.strip().lower() != 'y':
                    raise RuntimeError('中止（コードを同期してください）')

        self._ssh(KILL_CMD)  # 残存followerを掃除

    def run_one(self, cond, rep, outdir):
        common = self.batch.get('common', {})
        n_cmd = node_command(cond, common, self.batch['path_file'])
        remote_bag = f"/tmp/batch_{cond['name']}_r{rep}"
        r_cmd = bag_record_command(remote_bag)

        if self.dry_run:
            print(f"[dry-run] {cond['name']} rep{rep}:")
            print(f'  bag : ssh ... {r_cmd}')
            print(f'  node: ssh ... {n_cmd}')
            print(f'  stop: ssh ... {KILL_CMD} / {KILL_BAG_CMD}')
            print(f"  scp : {remote_bag} -> {outdir}/{cond['name']}_r{rep}")
            return dict(ok=False, metrics={}, bagdir='', note='dry-run')

        ans = input(f"\n次: {cond['name']} rep{rep}/{self.batch['repeats']}。"
                    f'ロボットを原点に戻して Enter（q で中断）: ')
        if ans.strip().lower() == 'q':
            raise KeyboardInterrupt

        self._ssh(f'rm -rf {remote_bag}')
        rec = subprocess.Popen(self._ssh_args(r_cmd),
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
        node = subprocess.Popen(self._ssh_args(n_cmd),
                                stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)
        margin = STARTUP_MARGIN_S[cond['controller']]
        deadline = time.time() + float(self.batch['timeout_s']) + margin
        reached = False
        try:
            while time.time() < deadline:
                ready, _, _ = select.select([node.stdout], [], [], 1.0)
                if not ready:
                    if node.poll() is not None:
                        break  # ノードが落ちた（求解失敗等）
                    continue
                line = node.stdout.readline()
                if not line:
                    break
                sys.stdout.write('  | ' + line)
                if '目標到達' in line:
                    reached = True
                    break
        finally:
            self._ssh(KILL_CMD)
            time.sleep(1.0)          # 停止指令publish・bag flush待ち
            self._ssh(KILL_BAG_CMD)
            node.wait(timeout=15)
            rec.wait(timeout=15)

        bagdir = Path(outdir) / f"{cond['name']}_r{rep}"
        bagdir.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(['scp', '-q', '-r',
                        f'{SSH_USER}@{self.host}:{remote_bag}',
                        str(bagdir)], check=True)
        self._ssh(f'rm -rf {remote_bag}')

        from analyze_bag import read_bag
        twist, perr, solve = read_bag(str(bagdir))
        metrics = compute_metrics(twist, perr, solve)
        return dict(ok=reached, metrics=metrics, bagdir=str(bagdir),
                    note='' if reached else 'タイムアウト/ノード異常終了')
