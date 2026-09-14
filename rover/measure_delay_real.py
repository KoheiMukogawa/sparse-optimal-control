# -*- coding: utf-8 -*-
"""実機の入力遅延をodom基準で実測する（R5 の暫定版・RPi上で実行）。

ωのステップ指令を打ち、/odom の**積算角度θ**の立ち上がりから、むだ時間Dと
時定数τを推定する。

**卒論5.6節の「約200ms」とは別物**であることに注意。
  - 設計版(R5, outline.md:781): 指令 → **カメラ真値**の角速度応答の時間差
  - 本スクリプト        : 指令 → **odom**が応答を報告するまでの時間

odom は約31Hzなのでサンプリング分解能は約32msあり、odom自身の報告遅れも乗る。
得られる値は「MPCループが見ている実効遅延」であって物理的な入力遅延そのものでは
ない。カメラが使えるようになったら設計版で測り直すこと。値を引用するときは必ず
「暫定値・odom基準」と明記する。

**なぜθを使うか**: ωは31ms毎のエンコーダ差分＝微分量でノイズが大きく、閾値交差で
時刻を決めると誤検出する（ω版では t63 に 812ms のような異常値が出た）。θは積分量で
滑らかなため、立ち上がり時刻をはるかに安定して決められる。

推定方法:
  1. 指令前の静止区間からθのノイズ幅σを実測し、閾値 max(5σ, 0.5°) を決める
  2. 閾値超えの時刻を「むだ時間（閾値法）」とする
  3. 一次遅れ＋むだ時間モデル
       θ(t) = ω_ss · [ (t-D) - τ(1 - exp(-(t-D)/τ)) ]   (t ≥ D), 0 (t < D)
     を格子探索で当てはめ、D と τ を推定する（ω_ss は保持区間後半の傾きから）

安全: その場旋回のみ（並進は0）。試行ごとに符号を反転させて原点付近に留める。
終了時・異常時は必ず零速度を配信する。

前提: nav_base 起動済み（rover/start_nav_base.sh）。
実行: source /opt/ros/humble/setup.bash && python3 measure_delay_real.py [--trials 20] [--w 0.5]
"""
import argparse
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

W_CMD_DEFAULT = 0.5      # rad/s。W_MAX=2.0 の25%。その場旋回
BASELINE_S = 1.0         # 指令前にθのノイズを測る区間
SETTLE_S = 1.5           # 静止を確認する時間
HOLD_S = 1.5             # ステップを保持する時間
STOP_S = 2.0             # 零速度後に停止を待つ時間
PUB_HZ = 10.0            # mpc_follower と同じ配信レート
MIN_THRESH_RAD = math.radians(0.5)


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return 2.0 * math.atan2(q.z, q.w)


class DelayProbe(Node):
    def __init__(self):
        super().__init__('delay_probe')
        self.pub = self.create_publisher(Twist, 'rover_twist', 1)
        self.create_subscription(Odometry, 'odom', self._cb, 50)
        self.samples = []          # [(t_stamp, yaw)]
        self.collecting = False

    def _cb(self, msg):
        if self.collecting:
            self.samples.append((stamp_s(msg), yaw_of(msg)))

    def publish_w(self, w):
        m = Twist()
        m.angular.z = float(w)
        self.pub.publish(m)

    def hold(self, w, seconds):
        """seconds の間 PUB_HZ で w を配信し続ける。戻り値は配信開始時刻。"""
        t0 = self.get_clock().now().nanoseconds * 1e-9
        for _ in range(max(1, int(seconds * PUB_HZ))):
            self.publish_w(w)
            end = time.monotonic() + 1.0 / PUB_HZ
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.01)
        return t0

    def stop(self):
        self.hold(0.0, STOP_S)


def unwrap(vals):
    out, off = [], 0.0
    for i, v in enumerate(vals):
        if i:
            d = v + off - out[-1]
            if d > math.pi:
                off -= 2 * math.pi
            elif d < -math.pi:
                off += 2 * math.pi
        out.append(v + off)
    return out


def slope(pts):
    """最小二乗の傾き。pts=[(t, y)]"""
    n = len(pts)
    if n < 2:
        return 0.0
    mt = statistics.fmean(t for t, _ in pts)
    my = statistics.fmean(y for _, y in pts)
    num = sum((t - mt) * (y - my) for t, y in pts)
    den = sum((t - mt) ** 2 for t, _ in pts)
    return num / den if den else 0.0


def model_theta(t, D, tau, w_ss):
    if t <= D:
        return 0.0
    dt = t - D
    return w_ss * (dt - tau * (1.0 - math.exp(-dt / tau)))


def analyze(samples, t_cmd, sign):
    ts = [t for t, _ in samples]
    ys = unwrap([y for _, y in samples])
    pre = [(t, y) for t, y in zip(ts, ys) if t < t_cmd]
    post = [(t - t_cmd, (y) * sign) for t, y in zip(ts, ys) if t >= t_cmd]
    if len(pre) < 5 or len(post) < 10:
        return None

    # 静止区間の基準値とノイズ幅
    base_vals = [y * sign for _, y in pre]
    y0 = statistics.fmean(base_vals)
    sigma = statistics.pstdev(base_vals) if len(base_vals) > 1 else 0.0
    rel = [(t, y - y0) for t, y in post]

    # 定常角速度は保持区間後半の傾き
    tail = [(t, y) for t, y in rel if t >= HOLD_S * 0.5]
    w_ss = slope(tail)
    if w_ss <= 0.05:
        return None

    # 閾値法
    thr = max(5.0 * sigma, MIN_THRESH_RAD)
    d_thr = None
    for i in range(len(rel) - 1):
        if rel[i][1] >= thr and rel[i + 1][1] >= thr:
            d_thr = rel[i][0]
            break

    # 一次遅れ＋むだ時間の格子探索
    best = None
    fit_pts = [(t, y) for t, y in rel if t <= HOLD_S]
    for di in range(0, 81):                 # D: 0〜400ms, 5ms刻み
        D = di * 0.005
        for ti in range(1, 101):            # tau: 5〜500ms, 5ms刻み
            tau = ti * 0.005
            sse = sum((y - model_theta(t, D, tau, w_ss)) ** 2
                      for t, y in fit_pts)
            if best is None or sse < best[0]:
                best = (sse, D, tau)
    return {'d_thr': d_thr, 'd_fit': best[1], 'tau': best[2],
            'w_ss': w_ss, 'sigma_deg': math.degrees(sigma),
            'thr_deg': math.degrees(thr), 'n': len(rel)}


def report(label, vals, unit='ms'):
    if not vals:
        print(f'{label}: 有効値なし')
        return
    v = sorted(vals)
    print(f'{label}: 中央値 {statistics.median(v):6.1f} {unit}  '
          f'四分位 {v[len(v) // 4]:.1f}〜{v[(3 * len(v)) // 4]:.1f}  '
          f'範囲 {v[0]:.1f}〜{v[-1]:.1f}  n={len(v)}')


def main():
    ap = argparse.ArgumentParser(description='実機入力遅延のodom基準実測（θベース）')
    ap.add_argument('--trials', type=int, default=20)
    ap.add_argument('--w', type=float, default=W_CMD_DEFAULT)
    args = ap.parse_args()

    rclpy.init()
    node = DelayProbe()
    res = []
    try:
        node.collecting = True
        node.samples.clear()
        node.hold(0.0, 2.0)
        node.collecting = False
        if len(node.samples) >= 2:
            span = node.samples[-1][0] - node.samples[0][0]
            rate = (len(node.samples) - 1) / span if span > 0 else float('nan')
            print(f'odom配信レート: {rate:.1f} Hz (分解能 {1000.0 / rate:.1f} ms)')

        for i in range(args.trials):
            sign = 1.0 if i % 2 == 0 else -1.0
            node.stop()
            node.hold(0.0, SETTLE_S)

            node.samples.clear()
            node.collecting = True
            node.hold(0.0, BASELINE_S)          # 静止区間（ノイズ測定）
            t_cmd = node.hold(args.w * sign, HOLD_S)
            node.collecting = False
            node.stop()

            r = analyze(list(node.samples), t_cmd, sign)
            if r is None:
                print(f'  trial{i + 1}: 応答を検出できず（スキップ）')
                continue
            dt = 'n/a' if r['d_thr'] is None else f'{r["d_thr"] * 1000:6.1f}'
            print(f'  trial{i + 1:2d}: D(閾値)={dt} ms  '
                  f'D(fit)={r["d_fit"] * 1000:6.1f} ms  '
                  f'τ={r["tau"] * 1000:6.1f} ms  '
                  f'ω_ss={r["w_ss"]:.3f}  θノイズσ={r["sigma_deg"]:.3f}°')
            res.append(r)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

    if not res:
        print('有効な試行がありません')
        return
    print()
    report('むだ時間 D（閾値法）',
           [r['d_thr'] * 1000 for r in res if r['d_thr'] is not None])
    report('むだ時間 D（モデル当てはめ）', [r['d_fit'] * 1000 for r in res])
    report('時定数 τ（モデル当てはめ）', [r['tau'] * 1000 for r in res])
    report('定常角速度 ω_ss', [r['w_ss'] for r in res], unit='rad/s')
    print('注: odom基準の暫定値。カメラ真値による設計版(R5)とは別物として扱うこと。')


if __name__ == '__main__':
    main()
