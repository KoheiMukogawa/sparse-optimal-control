# sparse-optimal-control

農業用ロボットの経路追従制御におけるスパース最適制御（Maximum Hands-off Control）の
開発・シミュレーション評価を行う卒業研究リポジトリ。
MPC + L1正則化、評価指標はRMSE・消費エネルギー・入力スパース性・計算負荷。
実装スタック: cvxpy / CasADi + IPOPT, Python (uv管理)。

## 環境

- 開発環境: Surface Laptop Go, WSL2 Ubuntu 24.04, ROS2 Jazzy（メモリ少なめ）
- 実機: LiteRover（ヴィストン, RPi4, Ubuntu 22.04, ROS2 Humble）
- DDS設定: ~/scripts/cyclonedds_laptop.xml（ユニキャスト、ラップトップ192.168.0.9↔RPi192.168.0.31）
- RPi接続:
  - 家Wi-Fi: `ssh mukougawakouhei@192.168.0.31`
  - RPiホットスポット: `ssh mukougawakouhei@192.168.4.1`
  - どちらか繋がらない場合はもう一方を試す

## ブランチ運用

- `main` 一本で運用（個人の卒研リポジトリ）。実験・実装も `main` に直接コミットし、
  区切りで `docs/handoff.md` を更新して push。大きく壊しうる変更時のみ一時ブランチを切る。

## 実験管理

- 実験条件はYAML（`configs/`以下）で管理し、コミットに含める
- 結果は `notebooks/` または `results/` 以下に保存

## セッション終了時

- `docs/handoff.md` を更新（現在の状況・次にやること・懸念事項）

## コミットメッセージ

- 日本語で簡潔に（例: `MPCの雛形実装`, `L1正則化を追加`）

## 現在地（2026-07-14時点）

- 実現可能性ゲート（RPi求解時間ベンチ）: **通過** → `results/2026-06-13_rpi_bench/`
  - L2/L1 とも N=30 まで 0.1s 周期内（L2 p95≈18ms, L1 p95≈29ms）、低電圧なし
  - 設計上限 N≤30。ベンチは `rover/bench_qp.py`
- L2-MPC シミュレーション: **完了** → `rover/mpc_core.py`, `rover/test_mpc_sim.py`
  - 全5ケース到達、横偏差RMSE は Kanayama を全ケースで下回る（L字 11.6→2.2cm 等）
  - L1 切替も動作（λ=1.0 で hands-off により直線到達・L字停滞を確認）
- L1化・λスイープ: **完了** → `rover/sweep_lambda.py`, `results/2026-06-13_lambda_sweep/`
  - L1 は補正 δu を penalize（オープンループ版δω78%ゼロと整合）
  - λ≈0.25 が L2 を3軸とも上回る、λ≥3 で破綻、採用レンジ 0.25〜2
  - 無外乱simでは L2 も操舵が疎（85%）→ L1 の真価は外乱下（P4）で出る見込み
- 実機ノード化: **実装完了** → `rover/mpc_follower.py`（reg=l2/l1, lam, N, 求解時間配信）
  - ローカル検証済み。L1 は bang-off 特性（補正時ωが上限へ）
  - 未実施: RPi配置→空転テスト→床で3者比較（実機を動かすので毎回許可を取る）
- 実機空転テスト（L2/L1）: **完了** → `results/2026-06-13_spin_*`, `spin_compare.md`
  - 求解時間は負荷込みで単体ベンチ約2倍（N=15: L2 p95 44ms/L1 51ms）も0.1s周期内
  - 空転・直線・無外乱では L2≈L1（差なし）→ L1優位は曲線・外乱・床滑りで出る
  - 解析: `rover/analyze_bag.py`（rosbags依存をuv add）
- 床走行 3者比較（直線2m）: **実施完了** → `results/2026-06-13_floor_compare.md`
  - 外部計測の横ずれ: **L1 0cm < L2 4cm < Kanayama 14-15cm**（提案手法L1が物理最良）
  - bag の odom基準RMSEとは逆転（odomは滑りを見られない→真値は外部計測必須）
  - 計算負荷: Kanayama≈0 < L2 p95 44ms < L1 50ms、全器0.1s周期内
  - 限界: n=1・手動原点復帰。要反復確認（各3-5本）
- 床走行 3者比較（L字）: **実施完了** → `results/2026-06-13_Lturn_compare.md`
  - 予想と逆: **L1 がコーナーで bang-bang チャタ**（Σ|u| L1 20.7 ≫ L2 4.5、ω反転 L1 15回 vs L2 0回）
  - 原因を sim で立証: 入力遅延2step(~200ms)で L1 のみチャタ（実機 Σ|u|20.7/反転15 と定量一致）、L2 は頑健
  - 対策: `mpc_core` に `move_suppress`（Δu率ペナルティ・既定0で後方互換）追加。L1+ms=2.0 で sim チャタ消失かつスパース性維持
  - 外部計測(真値)も一致: L2最良(終点2cm/傾き0°)、L1はジグザグを目視確認(最終向き右40°)、Kanayama大周り(左5cm/9°)
  - 位置づけ: 「L1優位」でなく「ナイーブL1は実機遅延下でチャタ＝実機検証必須」。総合最良は現状 L2-MPC
- teleop: `rover/teleop_key.py`（連続・前進と旋回独立・V_MAX0.15）RPi配置済み
- バッチ実験ランナー: **実装完了（実機は未走行）** → `rover/run_batch.py`,
  `rover/exp_backends.py`, `rover/exp_metrics.py`, `configs/batch_Lturn_3way.yaml`
  - sim E2E で L1チャタ→ms=2.0で消失の既知結果を再現。実機は `--backend real`
    （1本ごと Enter待ち＝許可、`--dry-run` で手順確認可）。テストは `uv run pytest tests/`
- 次: バッチランナーで実機 L字バッチ（Kanayama/L2/L1/L1+ms2.0 ×3本、要許可・
  初回が統合テスト）→ チャタ消失の実測確認と統計化 → 外乱条件 / カメラ真値

## 環境変更（2026-07-05）

- WSLホーム整理により、リポジトリが `~/sparse-optimal-control` → `~/projects/sparse-optimal-control` に移動。
- DDS設定ファイルも `~/cyclonedds_laptop.xml` → `~/scripts/cyclonedds_laptop.xml` に移動済み（`.bashrc` の `CYCLONEDDS_URI` は更新済み。RPi側は変更なし）。
- ホーム直下のnano復旧ファイルから、6/13 L字床走行の外部計測の生メモ（Kanayama/L2/L1、CLAUDE.mdの外部計測値の一次記録）を `docs/作業記録/2026-06-13_L字外部計測メモ.md` として回収。

### 旧現在地（2026-06-12時点）

- スパース制御シミュレーション（オープンループ）: 完了 → `docs/作業記録/sparse_rover.py`
- 自立走行（Kanayama経路追従・odomのみ・RPi完結）: 完了 → `rover/path_follower.py`
  - 床実走行で直線2m・L字を完走、rosbag7本を `results/` に保存
  - 回転オドメトリをトレッド定数補正済み（RPi側ROVER_D=0.1514、アーク旋回ほぼ90°）
- SLAM: 初回地図作成・保存完了 → `results/2026-06-12_slam_map/`（品質改善は壁沿い周回で）
- DDS通信（ラップトップ↔RPi）: 未解決のまま運用回避（全ノードRPi実行＋成果物回収方式）
- 次の目標: L1スパース制御ノード（cvxpy求解時間計測→実装）→ Kanayama vs L1 実機比較
- 注意: 実機を動かすコマンドは毎回ユーザーの許可を得てから実行する
