# -*- coding: utf-8 -*-
"""L2-MPC のクローズドループ検証（ROS不要）。

差動二輪の非線形運動学をプラントに、mpc_core.MPCFollower で経路追従する。
test_follower_sim.py（Kanayama）と同じケース・同じ指標で比較できるようにし、
「MPC が Kanayama と同条件で追従できる」ことを P1.2 の完了条件とする。

実行: uv run python rover/test_mpc_sim.py
      uv run python rover/test_mpc_sim.py --reg l1 --lam 2.0
"""

import argparse
import math

from follower_core import goal_crossed, goal_scaled_vr, reference_pose, tracking_error
from mpc_core import MPCFollower

V_MAX, W_MAX = 0.15, 2.0
DT = 0.1            # MPC 制御周期（ベンチと一致, 10Hz）
BASE_VR = 0.1
LOOKAHEAD = 0.15
GOAL_TOL = 0.05


def simulate(mpc, waypoints, x0, y0, th0, t_max=120.0):
    x, y, th = x0, y0, th0
    t = 0.0
    cross_track_sq = []
    abs_u = 0.0          # Σ|v|+|ω| 相当（消費エネルギー代理・スパース性の素）
    zero_v_steps = 0
    steps = 0
    solve_times = []
    while t < t_max:
        gx, gy = waypoints[-1]
        goal_dist = math.hypot(gx - x, gy - y)
        if goal_dist < GOAL_TOL or goal_crossed(waypoints, x, y):
            break
        x_r, y_r, th_r = reference_pose(waypoints, x, y, LOOKAHEAD)
        x_e, y_e, th_e = tracking_error(x, y, th, x_r, y_r, th_r)
        v_r = goal_scaled_vr(BASE_VR, goal_dist)
        cmd = mpc.command(x_e, y_e, th_e, v_r, w_r=0.0)
        if cmd is None:
            return dict(ok=False, t=t, rmse=_rmse(cross_track_sq),
                        reason="solve_fail", abs_u=abs_u,
                        zero_ratio=0.0, p95_ms=0.0)
        v, w = cmd
        solve_times.append(mpc.last_solve_s)
        # 真の非線形プラントを1ステップ進める
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        th += w * DT
        cross_track_sq.append(y_e ** 2)
        abs_u += abs(v) + abs(w)
        if abs(v) < 1e-3:
            zero_v_steps += 1
        steps += 1
        t += DT
    ok = (goal_dist < GOAL_TOL) or goal_crossed(waypoints, x, y)
    solve_times.sort()
    p95 = solve_times[min(len(solve_times) - 1, int(0.95 * len(solve_times)))] if solve_times else 0.0
    return dict(ok=ok, t=t, rmse=_rmse(cross_track_sq), reason="",
                abs_u=abs_u, zero_ratio=zero_v_steps / max(1, steps),
                p95_ms=p95 * 1e3)


def _rmse(sq):
    return math.sqrt(sum(sq) / len(sq)) if sq else 0.0


def run_case(mpc, name, waypoints, x0, y0, th0):
    r = simulate(mpc, waypoints, x0, y0, th0)
    status = "到達" if r["ok"] else f"未到達({r['reason'] or 'timeout'})"
    print(f"{name}: {status} 所要{r['t']:.1f}s 横偏差RMSE={r['rmse']*100:.1f}cm "
          f"Σ|u|={r['abs_u']:.1f} v0率={r['zero_ratio']*100:.0f}% "
          f"求解p95={r['p95_ms']:.1f}ms")
    return r["ok"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reg", choices=["l2", "l1"], default="l2")
    ap.add_argument("--lam", type=float, default=1.0)
    ap.add_argument("--horizon", type=int, default=15)
    args = ap.parse_args()

    mpc = MPCFollower(N=args.horizon, ts=DT, reg=args.reg, lam=args.lam,
                      v_max=V_MAX, w_max=W_MAX)
    print(f"== {args.reg.upper()}-MPC (N={args.horizon}, "
          f"{'λ=%.1f' % args.lam if args.reg == 'l1' else 'R二次'}) ==")

    straight = [(0.0, 0.0), (2.0, 0.0)]
    l_turn = [(0.0, 0.0), (1.5, 0.0), (1.5, 1.5)]
    results = [
        run_case(mpc, "直線2m（経路上スタート）", straight, 0.0, 0.0, 0.0),
        run_case(mpc, "直線2m（横20cm・30度ずれ）", straight, 0.0, 0.2, math.radians(30)),
        run_case(mpc, "直線2m（横-30cm・-45度ずれ）", straight, 0.1, -0.3, math.radians(-45)),
        run_case(mpc, "L字（経路上スタート）", l_turn, 0.0, 0.0, 0.0),
        run_case(mpc, "L字（横15cmずれ）", l_turn, 0.0, -0.15, 0.0),
    ]
    print("\n全ケース到達:", all(results))


if __name__ == "__main__":
    main()
