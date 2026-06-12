#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kanayama制御則による経路追従ノード（LiteRover用）。

公式サンプル lightrover_ros の上に載せる自作制御層:
  購読: odom (nav_msgs/Odometry)  ← odom_manager が配信
  配信: rover_twist (geometry_msgs/Twist)  ← pos_controller が購読
  配信: path_error (geometry_msgs/Vector3) 誤差ログ用 (x_e, y_e, θ_e)

実行（RPi上、ベースノード起動後）:
  ros2 launch lightrover_ros nav_base.launch.py   # 別端末で
  python3 path_follower.py --ros-args -p path_file:=../configs/path_straight_2m.yaml
"""

import math

import rclpy
import yaml
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node

from follower_core import (clamp, goal_crossed, goal_scaled_vr, kanayama_cmd,
                           reference_pose, tracking_error, yaw_from_quat_xyzw)

# ゲームパッド操作時のレンジ（rover_gamepad.py）に合わせた安全上限
V_MAX = 0.15   # m/s
W_MAX = 2.0    # rad/s
ODOM_TIMEOUT = 0.5  # s: odomが途絶えたら停止指令


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.declare_parameter('path_file', '')
        self.declare_parameter('v_r', 0.1)      # 基準速度 [m/s]
        self.declare_parameter('k_x', 0.5)
        self.declare_parameter('k_y', 5.0)
        self.declare_parameter('k_th', 3.0)
        self.declare_parameter('lookahead', 0.15)  # 参照点を経路上で前方に置く距離 [m]
        self.declare_parameter('goal_tol', 0.05)   # 終点到達判定 [m]
        self.declare_parameter('rate', 20.0)       # 制御周期 [Hz]

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

        self.pose = None  # (x, y, th)
        self.last_odom_time = None
        self.goal_reached = False
        # 起動時の自己位置を経路原点にする（odomはノード再起動でも
        # 基板の累積カウントを引き継ぐためゼロとは限らない）
        self.origin = None  # (x0, y0, th0)
        self.local_waypoints = list(self.waypoints)

        self.cmd_pub = self.create_publisher(Twist, 'rover_twist', 1)
        self.err_pub = self.create_publisher(Vector3, 'path_error', 1)
        self.create_subscription(Odometry, 'odom', self.cb_odom, 1)
        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self.control_step)
        self.get_logger().info(
            f'経路追従開始: {len(self.waypoints)}点, v_r={self.get_parameter("v_r").value} m/s')

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

        v, w = kanayama_cmd(
            x_e, y_e, th_e,
            goal_scaled_vr(self.get_parameter('v_r').value, goal_dist),
            self.get_parameter('k_x').value,
            self.get_parameter('k_y').value,
            self.get_parameter('k_th').value)

        cmd = Twist()
        cmd.linear.x = clamp(v, V_MAX)
        cmd.angular.z = clamp(w, W_MAX)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # 終了時は必ず停止指令
        node.destroy_node()


if __name__ == '__main__':
    main()
