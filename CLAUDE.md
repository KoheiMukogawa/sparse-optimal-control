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
  - 家Wi-Fi: `ssh mukougawakouhei@192.168.0.32`（2026-07-15にDHCPで.31→.32へ変動。
    恒久対策はルーターでMAC固定割当。繋がらない時はポート22走査で現IPを探す）
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

## 運用ルール（恒久）

- **実機を動かすコマンドは毎回ユーザーの許可を得る**（`run_batch --auto` は
  開始時の一括許可＋q+Enter停止で代替、必ず有人監視）
- nav_base は**1つだけ**起動（多重起動でモータ指令競合の事故歴あり）。
  起動時は `~/ros2_ws/install/setup.bash` の追いsource必須
- UDPブリッジは laptop 側を再起動したら RPi 側も再起動（seq巻き戻りで全破棄）
- **RPiをベンチで長時間つけっぱなしにする時はバッテリーでなくAC給電にする**
  （バッテリー切れの電源断でSDカードのファイルシステムが破損した事故歴あり、
  2026-07-20）。バッチ実行前に充電残量を確認する
- テストは `uv run pytest tests/`

## 確定した設計事実（変わらないもの）

- MPCホライズン設計上限 **N≤30**（RPi実測で0.1s周期内。常用はN=15）→
  `results/2026-06-13_rpi_bench/`
- L1は補正δuをpenalize、採用レンジ λ=0.25〜2（λ≥3で破綻）
- **ナイーブL1は実機遅延(~200ms)下でbang-bangチャタ** → `move_suppress=2.0` で
  消失（sim・実機の両方で立証済み）。現状の総合最良は L2-MPC、l1_ms2 が僅差
- **odomは滑りを見られない**（真値絶対値はodom RMSEの5〜10倍）→ 追従精度評価は
  カメラ真値（AprilTag俯瞰・C270）必須。床タグは**コースを囲む配置**
  （片側偏在はゴール外挿で10cm超誤差）
- 広角カメラは買わない。C270＋1m L字で完結（検証済み2.5cm/1.6°精度）
- odom補正は積算源（RPi側odometry, ROVER_D=0.1514）に入れる。follower側では効かない
- 主要コード: `rover/mpc_core.py`(MPC本体) `rover/mpc_follower.py`(実機ノード)
  `rover/truth_*.py`(カメラ真値) `rover/homing.py`(自動原点復帰)
  `rover/run_batch.py`(バッチランナー) `rover/analyze_bag.py`(bag解析)

## 現在地

**最新の状況・次にやることは `docs/handoff.md` を参照**（過去セッション全文は
`docs/作業記録/handoff_archive.md`）。2026-07-17時点の要点: カメラ真値
Phase 2+3 実装完了・実機E2E未実施。床タグ移動につき**タグ自動サーベイを設計中**
（configs/camera_truth.yaml の floor_tags は再測定まで無効）。
