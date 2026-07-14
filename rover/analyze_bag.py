# -*- coding: utf-8 -*-
"""rosbag から経路追従の評価指標を算出（研究計画 P1.4 解析パイプライン）。

評価指標（卒論4指標）:
  - 横偏差RMSE    : /path_error.y のRMS（追従精度）
  - 消費エネルギー: Σ(|v|+|ω|)·dt の代理（/rover_twist）
  - 入力スパース性: |ω|<閾 のステップ率（操舵の疎度）
  - 計算負荷      : /mpc_solve_ms の p50/p95/max（実機求解時間）

チャタ診断（反転・飽和指標）:
  - ω符号反転    : 走行中の角速度の符号変化回数（ジッター検知）
  - 飽和率        : |ω|>1.5 の時間割合（ロボ上限=2.0の75%）

注意: ジャッキアップ空転 bag では path_error は「仮想odomへの追従」で外乱が
ないため RMSE はほぼ0になる（参考値）。実機RMSEの比較は床走行 bag で行う。
求解時間と入力プロファイルは空転でも実機の実測として有効。

使い方:
  uv run python rover/analyze_bag.py results/2026-06-13_spin_l2 results/2026-06-13_spin_l1
"""

import argparse
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from exp_metrics import compute_metrics

# Humble の bag は型定義を埋め込まないため既定 typestore を渡す
TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)


def read_bag(bagdir):
    """bag → 時系列 (twist, perr, solve_ms)。compute_metrics に渡す形式。"""
    twist = []   # (t_s, v, w)
    perr = []    # (t_s, y_e)
    solve = []   # ms
    with AnyReader([Path(bagdir)], default_typestore=TYPESTORE) as reader:
        for conn, ts, raw in reader.messages():
            t = ts * 1e-9
            if conn.topic == '/rover_twist':
                m = reader.deserialize(raw, conn.msgtype)
                twist.append((t, m.linear.x, m.angular.z))
            elif conn.topic == '/path_error':
                m = reader.deserialize(raw, conn.msgtype)
                perr.append((t, m.y))
            elif conn.topic == '/mpc_solve_ms':
                m = reader.deserialize(raw, conn.msgtype)
                solve.append(float(m.data))
    return twist, perr, solve


def analyze(bagdir):
    twist, perr, solve = read_bag(bagdir)
    if not twist:
        raise SystemExit(f"{bagdir}: /rover_twist が空")
    return dict(name=Path(bagdir).name, **compute_metrics(twist, perr, solve))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+", help="rosbag ディレクトリ（複数可）")
    args = ap.parse_args()

    rows = [analyze(b) for b in args.bags]

    hdr = (f"{'bag':<26} {'走行s':>6} {'RMSE_cm':>8} {'Σ|u|':>7} "
           f"{'ω0率':>6} {'反転':>4} {'飽和':>5} {'解p50':>6} {'解p95':>6} {'解max':>6}")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(f"{r['name']:<26} {r['drive_s']:>6.1f} {r['rmse_cm']:>8.2f} "
              f"{r['sum_u']:>7.2f} {r['w_zero_ratio']*100:>5.0f}% "
              f"{r['flips']:>4d} {r['sat_ratio']*100:>4.0f}% "
              f"{r['solve_p50']:>5.1f} {r['solve_p95']:>5.1f} {r['solve_max']:>5.1f}")
    print("\n注: 空転bagのRMSEは仮想odom追従の参考値（外乱なし）。"
          "求解時間[ms]と入力プロファイルは実機実測。", file=sys.stderr)


if __name__ == "__main__":
    main()
