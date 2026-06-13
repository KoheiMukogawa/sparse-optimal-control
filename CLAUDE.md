# sparse-optimal-control

農業用ロボットの経路追従制御におけるスパース最適制御（Maximum Hands-off Control）の
開発・シミュレーション評価を行う卒業研究リポジトリ。
MPC + L1正則化、評価指標はRMSE・消費エネルギー・入力スパース性・計算負荷。
実装スタック: cvxpy / CasADi + IPOPT, Python (uv管理)。

## 環境

- 開発環境: Surface Laptop Go, WSL2 Ubuntu 24.04, ROS2 Jazzy（メモリ少なめ）
- 実機: LiteRover（ヴィストン, RPi4, Ubuntu 22.04, ROS2 Humble）
- DDS設定: ~/cyclonedds_laptop.xml（ユニキャスト、ラップトップ192.168.0.9↔RPi192.168.0.31）
- RPi接続:
  - 家Wi-Fi: `ssh mukougawakouhei@192.168.0.31`
  - RPiホットスポット: `ssh mukougawakouhei@192.168.4.1`
  - どちらか繋がらない場合はもう一方を試す

## ブランチ運用

- `master`: 常に動く状態を維持
- `exp/<実験名>`: 実験・実装は必ずこちらで実施

## 実験管理

- 実験条件はYAML（`configs/`以下）で管理し、コミットに含める
- 結果は `notebooks/` または `results/` 以下に保存

## セッション終了時

- `docs/handoff.md` を更新（現在の状況・次にやること・懸念事項）

## コミットメッセージ

- 日本語で簡潔に（例: `MPCの雛形実装`, `L1正則化を追加`）

## 現在地（2026-06-13時点）

- 実現可能性ゲート（RPi求解時間ベンチ）: **通過** → `results/2026-06-13_rpi_bench/`
  - L2/L1 とも N=30 まで 0.1s 周期内（L2 p95≈18ms, L1 p95≈29ms）、低電圧なし
  - 設計上限 N≤30。ベンチは `rover/bench_qp.py`
- L2-MPC シミュレーション: **完了** → `rover/mpc_core.py`, `rover/test_mpc_sim.py`
  - 全5ケース到達、横偏差RMSE は Kanayama を全ケースで下回る（L字 11.6→2.2cm 等）
  - L1 切替も動作（λ=1.0 で hands-off により直線到達・L字停滞を確認）
- 次: P1.3 L1化とλスイープ（トレードオフ曲線）→ P1.4 `mpc_follower.py` 実機ノード化

### 旧現在地（2026-06-12時点）

- スパース制御シミュレーション（オープンループ）: 完了 → `docs/作業記録/sparse_rover.py`
- 自立走行（Kanayama経路追従・odomのみ・RPi完結）: 完了 → `rover/path_follower.py`
  - 床実走行で直線2m・L字を完走、rosbag7本を `results/` に保存
  - 回転オドメトリをトレッド定数補正済み（RPi側ROVER_D=0.1514、アーク旋回ほぼ90°）
- SLAM: 初回地図作成・保存完了 → `results/2026-06-12_slam_map/`（品質改善は壁沿い周回で）
- DDS通信（ラップトップ↔RPi）: 未解決のまま運用回避（全ノードRPi実行＋成果物回収方式）
- 次の目標: L1スパース制御ノード（cvxpy求解時間計測→実装）→ Kanayama vs L1 実機比較
- 注意: 実機を動かすコマンドは毎回ユーザーの許可を得てから実行する
