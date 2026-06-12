# -*- coding: utf-8 -*-
"""follower_coreのクローズドループ検証（ROS不要）。

差動二輪の運動学でプラントを回し、Kanayama制御則で経路追従できるか確認する。
実行: uv run python rover/test_follower_sim.py
"""

import math

from follower_core import (clamp, goal_crossed, goal_scaled_vr, kanayama_cmd,
                           reference_pose, tracking_error)

V_MAX, W_MAX = 0.15, 2.0
DT = 0.05  # 20Hz
PARAMS = dict(v_r=0.1, k_x=0.5, k_y=5.0, k_th=3.0)
LOOKAHEAD = 0.15
GOAL_TOL = 0.05


def simulate(waypoints, x0, y0, th0, t_max=120.0):
    x, y, th = x0, y0, th0
    t = 0.0
    cross_track_sq = []
    while t < t_max:
        gx, gy = waypoints[-1]
        goal_dist = math.hypot(gx - x, gy - y)
        if goal_dist < GOAL_TOL or goal_crossed(waypoints, x, y):
            return True, t, math.sqrt(sum(cross_track_sq) / len(cross_track_sq))
        x_r, y_r, th_r = reference_pose(waypoints, x, y, LOOKAHEAD)
        x_e, y_e, th_e = tracking_error(x, y, th, x_r, y_r, th_r)
        v, w = kanayama_cmd(x_e, y_e, th_e,
                            goal_scaled_vr(PARAMS['v_r'], goal_dist),
                            PARAMS['k_x'], PARAMS['k_y'], PARAMS['k_th'])
        v, w = clamp(v, V_MAX), clamp(w, W_MAX)
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        th += w * DT
        cross_track_sq.append(y_e ** 2)
        t += DT
    return False, t, math.sqrt(sum(cross_track_sq) / len(cross_track_sq))


def run_case(name, waypoints, x0, y0, th0):
    ok, t, rmse = simulate(waypoints, x0, y0, th0)
    status = "到達" if ok else "未到達(タイムアウト)"
    print(f"{name}: {status} 所要{t:.1f}s 横偏差RMSE={rmse*100:.1f}cm "
          f"(初期位置 x={x0}, y={y0}, θ={math.degrees(th0):.0f}deg)")
    return ok


straight = [(0.0, 0.0), (2.0, 0.0)]
l_turn = [(0.0, 0.0), (1.5, 0.0), (1.5, 1.5)]

results = [
    run_case("直線2m（経路上スタート）", straight, 0.0, 0.0, 0.0),
    run_case("直線2m（横20cm・30度ずれ）", straight, 0.0, 0.2, math.radians(30)),
    run_case("直線2m（横-30cm・-45度ずれ）", straight, 0.1, -0.3, math.radians(-45)),
    run_case("L字（経路上スタート）", l_turn, 0.0, 0.0, 0.0),
    run_case("L字（横15cmずれ）", l_turn, 0.0, -0.15, 0.0),
]
print("\n全ケース到達:", all(results))
