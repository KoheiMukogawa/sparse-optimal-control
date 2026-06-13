#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MPC（L2/L1切替）による経路追従ノード（LiteRover用）。

path_follower.py（Kanayama）と同じ I/O・同じ経路原点ロジックで、制御則だけを
mpc_core.MPCFollower に差し替えたもの。3者比較（Kanayama/L2/L1）の MPC 側。

  購読: odom (nav_msgs/Odometry)         ← odom_manager が配信
  配信: rover_twist (geometry_msgs/Twist) ← pos_controller が購読
  配信: path_error (geometry_msgs/Vector3) 誤差ログ (x_e, y_e, θ_e)
  配信: mpc_solve_ms (std_msgs/Float32)   求解時間ログ（計算負荷の指標）

実行（RPi上、ベースノード起動後）:
  ros2 launch lightrover_ros nav_base.launch.py   # 別端末で
  python3 mpc_follower.py --ros-args \
    -p path_file:=../configs/path_straight_2m.yaml -p reg:=l1 -p lam:=0.3

注意: cvxpy の初回 import は RPi で 30〜60s かかる。起動ログ「MPC準備完了」を待つ。
"""

import math

import rclpy
import yaml
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

from follower_core import (goal_crossed, goal_scaled_vr, reference_pose,
                           tracking_error, yaw_from_quat_xyzw)
from mpc_core import MPCFollower

# ゲームパッド操作時のレンジに合わせた安全上限（pos_controller と整合）
V_MAX = 0.15   # m/s
W_MAX = 2.0    # rad/s
ODOM_TIMEOUT = 0.5  # s: odomが途絶えたら停止指令


class MpcFollowerNode(Node):
    def __init__(self):
        super().__init__('mpc_follower')
        self.declare_parameter('path_file', '')
        self.declare_parameter('reg', 'l2')        # 'l2' or 'l1'
        self.declare_parameter('lam', 0.3)         # L1 正則化係数（sim採用域 0.25〜2）
        self.declare_parameter('horizon', 15)      # N（実機ベンチで N≤30 が0.1s内）
        self.declare_parameter('v_r', 0.1)         # 基準速度 [m/s]
        self.declare_parameter('lookahead', 0.15)  # 参照点を前方に置く距離 [m]
        self.declare_parameter('goal_tol', 0.05)   # 終点到達判定 [m]
        self.declare_parameter('rate', 10.0)       # 制御周期 [Hz]（MPC設計=0.1s）

        path_file = self.get_parameter('path_file').value
        if not path_file:
            raise RuntimeError('path_file パラメータを指定してください')
        with open(path_file) as f:
            cfg = yaml.safe_load(f)
        self.waypoints = [(float(p[0]), float(p[1])) for p in cfg['waypoints']]
        if 'v_r' in cfg:
            self.set_parameters([rclpy.parameter.Parameter(
                'v_r', rclpy.Parameter.Type.DOUBLE, float(cfg['v_r']))])
        if len(self.waypoints) < 2:
            raise RuntimeError('waypointsは2点以上必要です')

        reg = self.get_parameter('reg').value
        lam = self.get_parameter('lam').value
        N = self.get_parameter('horizon').value
        rate = self.get_parameter('rate').value
        ts = 1.0 / rate
        # cvxpy 問題構築（RPiでは初回 import が遅い）
        self.mpc = MPCFollower(N=N, ts=ts, reg=reg, lam=lam,
                               v_max=V_MAX, w_max=W_MAX)

        self.pose = None            # (x, y, th)
        self.last_odom_time = None
        self.goal_reached = False
        self.origin = None          # 起動時自己位置を経路原点に
        self.local_waypoints = list(self.waypoints)
        self.solve_warn_count = 0

        self.cmd_pub = self.create_publisher(Twist, 'rover_twist', 1)
        self.err_pub = self.create_publisher(Vector3, 'path_error', 1)
        self.solve_pub = self.create_publisher(Float32, 'mpc_solve_ms', 1)
        self.create_subscription(Odometry, 'odom', self.cb_odom, 1)
        self.create_timer(ts, self.control_step)
        self.get_logger().info(
            f'MPC準備完了: reg={reg} '
            f'{("lam=%.3f " % lam) if reg == "l1" else ""}N={N} '
            f'rate={rate}Hz, {len(self.waypoints)}点, '
            f'v_r={self.get_parameter("v_r").value} m/s')

    def cb_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        self.pose = (p.position.x, p.position.y,
                     yaw_from_quat_xyzw(q.x, q.y, q.z, q.w))
        self.last_odom_time = self.get_clock().now()
        if self.origin is None:
            x0, y0, th0 = self.pose
            self.origin = (x0, y0, th0)
            c, s = math.cos(th0), math.sin(th0)
            self.local_waypoints = [
                (x0 + wx * c - wy * s, y0 + wx * s + wy * c)
                for wx, wy in self.waypoints]
            self.get_logger().info(
                f'経路原点を現在位置に設定: ({x0:.3f}, {y0:.3f}, {math.degrees(th0):.1f}deg)')

    def control_step(self):
        if self.pose is None or self.goal_reached:
            return
        elapsed = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
        if elapsed > ODOM_TIMEOUT:
            self.cmd_pub.publish(Twist())
            self.get_logger().warn('odom途絶のため停止', throttle_duration_sec=2.0)
            return

        x, y, th = self.pose
        gx, gy = self.local_waypoints[-1]
        goal_dist = math.hypot(gx - x, gy - y)
        if (goal_dist < self.get_parameter('goal_tol').value
                or goal_crossed(self.local_waypoints, x, y)):
            self.cmd_pub.publish(Twist())
            self.goal_reached = True
            self.get_logger().info('目標到達。停止します')
            return

        x_r, y_r, th_r = reference_pose(
            self.local_waypoints, x, y, self.get_parameter('lookahead').value)
        x_e, y_e, th_e = tracking_error(x, y, th, x_r, y_r, th_r)
        self.err_pub.publish(Vector3(x=x_e, y=y_e, z=th_e))

        v_r = goal_scaled_vr(self.get_parameter('v_r').value, goal_dist)
        cmd_uv = self.mpc.command(x_e, y_e, th_e, v_r, w_r=0.0)
        self.solve_pub.publish(Float32(data=float(self.mpc.last_solve_s * 1e3)))

        if cmd_uv is None:
            # 求解失敗時は安全に停止（実行不可能・ソルバーエラー）
            self.cmd_pub.publish(Twist())
            self.solve_warn_count += 1
            self.get_logger().warn(
                f'MPC求解失敗のため停止（{self.solve_warn_count}回目）',
                throttle_duration_sec=1.0)
            return

        v, w = cmd_uv
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MpcFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # 終了時は必ず停止指令
        node.destroy_node()


if __name__ == '__main__':
    main()
