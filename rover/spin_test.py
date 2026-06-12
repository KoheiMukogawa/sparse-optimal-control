#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""その場旋回テスト: odom上で指定角度回転したら停止する。

回転方向の規約確認と、旋回スリップ（実回転角/odom回転角）の
キャリブレーションに使う。

実行例（odom上で左に90度）:
  python3 spin_test.py --ros-args -p target_deg:=90.0 -p omega:=0.4
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from follower_core import normalize_angle, yaw_from_quat_xyzw


class SpinTest(Node):
    def __init__(self):
        super().__init__('spin_test')
        self.declare_parameter('target_deg', 90.0)   # 正=左回り
        self.declare_parameter('omega', 0.4)         # 旋回速度 [rad/s]
        self.target = math.radians(self.get_parameter('target_deg').value)
        self.omega = self.get_parameter('omega').value
        self.accum = 0.0
        self.prev_yaw = None
        self.done = False
        self.cmd_pub = self.create_publisher(Twist, 'rover_twist', 1)
        self.create_subscription(Odometry, 'odom', self.cb_odom, 1)
        self.create_timer(0.05, self.step)
        self.get_logger().info(
            f'旋回テスト開始: 目標{math.degrees(self.target):.0f}度 '
            f'(正=左回り), ω={self.omega} rad/s')

    def cb_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat_xyzw(q.x, q.y, q.z, q.w)
        if self.prev_yaw is not None:
            self.accum += normalize_angle(yaw - self.prev_yaw)
        self.prev_yaw = yaw

    def step(self):
        if self.done or self.prev_yaw is None:
            return
        sign = 1.0 if self.target >= 0 else -1.0
        if sign * self.accum >= sign * self.target:
            self.cmd_pub.publish(Twist())
            self.done = True
            self.get_logger().info(
                f'停止: odom累積回転 {math.degrees(self.accum):.1f}度。'
                '実際の回転角を計測してください')
            return
        cmd = Twist()
        cmd.angular.z = sign * self.omega
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SpinTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()


if __name__ == '__main__':
    main()
