# -*- coding: utf-8 -*-
"""経路追従の純粋ロジック（ROS非依存）。

path_follower.py（ROS2ノード）とシミュレーション検証の両方から使う。
"""

import math


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat_xyzw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def reference_pose(waypoints, x, y, lookahead):
    """折れ線経路への射影点からlookahead先の参照姿勢 (x_r, y_r, θ_r) を返す。"""
    best = (float('inf'), 0, 0.0)  # (距離^2, セグメント番号, セグメント内パラメータ)
    for i in range(len(waypoints) - 1):
        ax, ay = waypoints[i]
        bx, by = waypoints[i + 1]
        dx, dy = bx - ax, by - ay
        seg_len2 = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg_len2))
        px, py = ax + t * dx, ay + t * dy
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 < best[0]:
            best = (d2, i, t)

    _, i, t = best
    remain = lookahead
    while i < len(waypoints) - 1:
        ax, ay = waypoints[i]
        bx, by = waypoints[i + 1]
        seg_len = math.hypot(bx - ax, by - ay)
        ahead = (1.0 - t) * seg_len
        if remain <= ahead:
            t += remain / seg_len
            th_r = math.atan2(by - ay, bx - ax)
            return ax + t * (bx - ax), ay + t * (by - ay), th_r
        remain -= ahead
        i += 1
        t = 0.0
    gx, gy = waypoints[-1]
    ax, ay = waypoints[-2]
    return gx, gy, math.atan2(gy - ay, gx - ax)


def tracking_error(x, y, th, x_r, y_r, th_r):
    """Kanayama誤差（ロボット座標系） (x_e, y_e, θ_e)。"""
    x_e = math.cos(th) * (x_r - x) + math.sin(th) * (y_r - y)
    y_e = -math.sin(th) * (x_r - x) + math.cos(th) * (y_r - y)
    th_e = normalize_angle(th_r - th)
    return x_e, y_e, th_e


def kanayama_cmd(x_e, y_e, th_e, v_r, k_x, k_y, k_th, w_r=0.0):
    """Kanayama制御則。(v, ω) を返す。"""
    v = v_r * math.cos(th_e) + k_x * x_e
    w = w_r + v_r * (k_y * y_e + k_th * math.sin(th_e))
    return v, w


def clamp(val, limit):
    return max(-limit, min(limit, val))


def goal_crossed(waypoints, x, y):
    """最終セグメントに垂直なゴール線を越えたらTrue。

    円判定だけだと、横偏差が残ったままゴール脇を通過したときに
    到達と判定されず走り続ける（または脇で釣り合って停止する）。
    """
    gx, gy = waypoints[-1]
    ax, ay = waypoints[-2]
    seg_len = math.hypot(gx - ax, gy - ay)
    ux, uy = (gx - ax) / seg_len, (gy - ay) / seg_len
    return (x - gx) * ux + (y - gy) * uy >= 0.0


def goal_scaled_vr(v_r, goal_dist, slow_radius=0.3):
    """ゴール接近時に基準速度を距離比例で減速させる。

    減速しないと v_r/k_x だけゴールを通り過ぎた点が平衡点になり、
    到達判定圏に入らず停止しないままになる。
    """
    if goal_dist >= slow_radius:
        return v_r
    return v_r * goal_dist / slow_radius
