# -*- coding: utf-8 -*-
"""UDP {v, w, seq} JSON → /rover_twist ブリッジ（RPi・Phase 3）。

laptop の homing（自動原点復帰）からの速度指令を rover_twist に流す唯一の
入口。走行フェーズでは使わない（follower は RPi 内で完結）。
安全機構: 受信 0.5s 途絶で零速度を1回配信（watchdog）、seq 逆行・重複は
破棄、v/w は V_MAX/W_MAX でクランプ。

使い方（RPi）:
  source /opt/ros/humble/setup.bash && python3 udp_twist_bridge.py [--port 8890]
停止: Ctrl-C（終了時に零速度を配信）。
BridgeCore は ROS 非依存（tests/test_udp_bridge.py で単体テスト）。
"""
import json

V_MAX, W_MAX = 0.15, 2.0     # mpc_follower.py と同値（最終防衛クランプ）
WATCHDOG_S = 0.5


class BridgeCore:
    """受信判定・watchdog の純ロジック。now は time.monotonic() 相当の秒。"""

    def __init__(self, watchdog_s=WATCHDOG_S):
        self.watchdog_s = watchdog_s
        self.last_seq = -1
        self.last_rx = None
        self.active = False   # 有効指令を流している間 True

    def accept(self, data, now):
        """UDPペイロード → (v, w)（クランプ済み）か None（破棄）。"""
        try:
            m = json.loads(data.decode())
            seq, v, w = int(m['seq']), float(m['v']), float(m['w'])
        except (ValueError, KeyError, TypeError, UnicodeDecodeError):
            return None
        if seq <= self.last_seq:
            return None                    # 順序乱れ・重複は破棄
        self.last_seq = seq
        self.last_rx = now
        self.active = True
        return (max(-V_MAX, min(V_MAX, v)), max(-W_MAX, min(W_MAX, w)))

    def watchdog_zero(self, now):
        """途絶検知。零速度を1回だけ流すべきとき True（発火後は再受信まで沈黙）。"""
        if self.active and now - self.last_rx > self.watchdog_s:
            self.active = False
            return True
        return False


def main():
    import argparse
    import socket
    import time

    import rclpy
    from geometry_msgs.msg import Twist

    ap = argparse.ArgumentParser(description='UDP→rover_twist ブリッジ')
    ap.add_argument('--port', type=int, default=8890)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('udp_twist_bridge')
    pub = node.create_publisher(Twist, 'rover_twist', 10)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.port))
    sock.setblocking(False)
    core = BridgeCore()

    def publish(v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        pub.publish(msg)

    def tick():
        now = time.monotonic()
        while True:
            try:
                data, _ = sock.recvfrom(256)
            except BlockingIOError:
                break
            cmd = core.accept(data, now)
            if cmd is not None:
                publish(*cmd)
        if core.watchdog_zero(now):
            node.get_logger().warn('UDP途絶: 零速度を配信')
            publish(0.0, 0.0)

    node.create_timer(0.05, tick)   # 20Hz で受信・watchdog
    node.get_logger().info(f'udp_twist_bridge 起動: port {args.port}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    publish(0.0, 0.0)
    node.destroy_node()


if __name__ == '__main__':
    main()
