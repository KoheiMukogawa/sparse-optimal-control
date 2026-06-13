# sparse-optimal-control

農業用ロボットの経路追従における **スパース最適制御（Maximum Hands-off Control）** の
研究リポジトリ（卒業研究）。MPC + L1 正則化を、シミュレーションと実機（LiteRover）の
両輪で評価する。評価指標: 追従精度(RMSE)・消費エネルギー・入力スパース性・計算負荷。

- 言語/管理: Python（[uv](https://docs.astral.sh/uv/) 管理）、QP は cvxpy + OSQP
- 実機: LiteRover（RPi4, ROS2 Humble） / 開発機: WSL2 Ubuntu 24.04, ROS2 Jazzy

> **まず読む**: 現在地・次にやること・懸念は [`docs/handoff.md`](docs/handoff.md)。
> 実験結果の一覧は [`results/README.md`](results/README.md)。

## クイックスタート

```bash
uv sync                                                 # 依存をインストール
uv run python rover/test_mpc_sim.py                     # L2-MPC クローズドループ sim
uv run python rover/test_mpc_sim.py --reg l1 --lam 0.3  # L1（スパース）版
uv run python rover/sweep_lambda.py                     # λスイープ（スパース性↔追従）
uv run python rover/analyze_bag.py results/2026-06-13_Lturn_l2   # 実機 bag の4指標
```

## ディレクトリ構成

| パス | 内容 |
|------|------|
| `rover/` | 制御・解析コード（下表） |
| `configs/` | 経路（waypoints）定義 YAML |
| `results/` | 実験結果（日付プレフィックス）。索引 → [`results/README.md`](results/README.md) |
| `docs/` | 研究計画・引き継ぎ・作業記録 |
| `docs/handoff.md` | **現在地・次・懸念**（セッション毎更新／最初に読む） |
| `docs/research_plan.md` | 研究計画（P1〜P7） |
| `docs/作業記録/` | 日次の作業ログ、初期のオープンループ sim `sparse_rover.py` |
| `data/paper/` | 参考論文 PDF |
| `CLAUDE.md` | リポジトリ規約＋現在地サマリ |

### rover/ の主なコード

| ファイル | 役割 |
|------|------|
| `follower_core.py` | 経路射影・Kanayama 誤差・ゴール判定（ROS 非依存・全制御で共用） |
| `mpc_core.py` | MPC 本体（L2/L1 切替・終端ソフトコスト・移動抑制 `move_suppress`） |
| `path_follower.py` | Kanayama 制御の実機ノード（ベースライン） |
| `mpc_follower.py` | MPC(L2/L1) の実機ノード |
| `bench_qp.py` | RPi 求解時間ベンチ（実現可能性ゲート） |
| `test_mpc_sim.py` / `test_follower_sim.py` | クローズドループ検証（MPC / Kanayama） |
| `sweep_lambda.py` | λスイープ（トレードオフ曲線） |
| `sim_delay_probe.py` | 入力遅延下のチャタ再現・対策（移動抑制）検証 |
| `analyze_bag.py` / `plot_lturn.py` | rosbag 解析・可視化 |
| `teleop_key.py` / `spin_test.py` | 手動操作 / その場旋回テスト |

## 主な成果（2026-06 時点）

- **実現可能性ゲート通過**: RPi で N≤30 が 0.1s 周期内（`results/2026-06-13_rpi_bench/`）
- **L2-MPC が Kanayama を追従精度で上回る**（sim・実機とも）
- **床直線3者比較**: 外部計測で L1 0 < L2 4 < Kanayama 14cm（`results/2026-06-13_floor_compare.md`）
- **床L字3者比較**: L1 がコーナーで bang-bang チャタリング。原因（実機遅延）と対策（移動抑制）を
  sim で立証（[`results/2026-06-13_Lturn_compare.md`](results/2026-06-13_Lturn_compare.md)）

詳細・最新は `docs/handoff.md` を参照。
