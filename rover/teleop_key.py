#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LiteRover 用キーボード teleop（連続・前進と旋回が独立）。

teleop_twist_keyboard の不満点を解消:
  - 旋回キーが前進を0にする → 本ノードは v と ω を独立に保持（前進しながら曲がれる）
  - 既定速度が速すぎる(0.5m/s) → 常用域に制限し刻みで増減

設計: 現在の (v, ω) を保持し、RATE Hz で rover_twist に出し続ける（RC的な
「セットした速度が持続」モデル）。キーで増減・スペースで即停止。

実行（RPi上、nav_base 起動中／follower停止中で）:
  python3 teleop_key.py
停止: スペースで停止 → q または Ctrl-C で終了（終了時に停止指令送出）。
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

V_MAX = 0.15    # 前進速度の上限 [m/s]
W_MAX = 1.5     # 旋回速度の上限 [rad/s]
V_STEP = 0.02   # 前進の1キーあたり増分 [m/s]
W_STEP = 0.2    # 旋回の1キーあたり増分 [rad/s]
RATE = 15.0     # 指令配信周波数 [Hz]

HELP = """\
=== LiteRover keyboard teleop ===
  w / s : 前進速度 +/-      （前進と旋回は独立。曲がっても前進は保持）
  a / d : 左旋回 / 右旋回 +/-
  space : 即停止（v=0, w=0）
  k     : 旋回だけ0（前進は保持）
  q     : 終了
上限: v=±%.2f m/s, w=±%.1f rad/s / 刻み v=%.2f, w=%.1f
""" % (V_MAX, W_MAX, V_STEP, W_STEP)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.pub = self.create_publisher(Twist, 'rover_twist', 1)
        self.v = 0.0
        self.w = 0.0
        self.create_timer(1.0 / RATE, self.tick)

    def tick(self):
        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.w
        self.pub.publish(msg)

    def handle_key(self, c):
        if c == 'w':
            self.v = clamp(self.v + V_STEP, -V_MAX, V_MAX)
        elif c == 's':
            self.v = clamp(self.v - V_STEP, -V_MAX, V_MAX)
        elif c == 'a':
            self.w = clamp(self.w + W_STEP, -W_MAX, W_MAX)
        elif c == 'd':
            self.w = clamp(self.w - W_STEP, -W_MAX, W_MAX)
        elif c == 'k':
            self.w = 0.0
        elif c == ' ':
            self.v = 0.0
            self.w = 0.0
        return c == 'q'


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            # 非ブロッキングでキー1つ読む
            if select.select([sys.stdin], [], [], 1.0 / RATE)[0]:
                c = sys.stdin.read(1)
                if node.handle_key(c):
                    break
            sys.stdout.write("\r v=%+.2f m/s  w=%+.2f rad/s    " % (node.v, node.w))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        try:
            node.pub.publish(Twist())  # 停止指令
        except Exception:
            pass
        node.destroy_node()
        print("\n停止して終了しました")


if __name__ == '__main__':
    main()
