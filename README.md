# sparse-optimal-control

[![test](https://github.com/KoheiMukogawa/sparse-optimal-control/actions/workflows/test.yml/badge.svg)](https://github.com/KoheiMukogawa/sparse-optimal-control/actions/workflows/test.yml)

農業用ロボットの経路追従制御における**スパース最適制御（Maximum Hands-off Control）**の研究リポジトリ（卒業研究）。

## 解いた問題

農業用ロボットに経路追従をさせる際、素朴なフィードバック制御（Kanayama法など）は目標経路からわずかでもずれると常時アクチュエータを動かし続ける。連続的な操舵はバッテリーを消費し、機構を摩耗させる。追従精度を落とさずに「必要なときだけ動く」制御ができれば、この摩耗とエネルギー消費を減らせるはずだ、というのが出発点。

## 手法

MPC（モデル予測制御）の評価関数に、入力偏差 δu の **L1 正則化**（`λ‖δu‖₁`）を加える（Maximum Hands-off Control）。L2（二次コスト）が誤差を全区間で滑らかにゼロへ寄せようとするのに対し、L1 は解を「δu=0 に張り付く区間」と「必要な瞬間だけ動く区間」に分ける性質を持つため、操舵入力そのものが疎（スパース）になる。QP は [cvxpy](https://www.cvxpy.org/) + OSQP で解く（`rover/mpc_core.py`）。

## 設計判断

### 1. なぜ L2 ではなく L1 か

`rover/mpc_core.py` の `MPCFollower` は `reg="l2"` なら `cp.quad_form(du[:, k], R)`、`reg="l1"` なら `lam * cp.norm1(du[:, k])` をステージコストに使う（同ファイル 78〜82行目）。L2 は二乗誤差なので小さな偏差でも常に非ゼロの補正を返すが、L1 は劣微分がゼロ点で不連続なため最適解が `δu=0` に張り付きやすい。コード中のコメントの通り「補正をゼロに張り付かせ、巡航は維持したまま『補正を打つ瞬間だけ動かす』スパース操舵を得る」ことが目的で、これは二乗コストでは原理的に得られない。

`rover/sweep_lambda.py` を遅延なし条件で流すと、L2 は横偏差RMSE 9.1cm・Σ|δu| 98.8・δωゼロ率85%に対し、L1(λ=0.25) は RMSE 7.7cm・Σ|δu| 92.7・δωゼロ率97%と、**追従精度を犠牲にせずスパース性が上がる**ことを確認している（自分で実行して確認、下記コマンド）。

### 2. なぜ実機に Raspberry Pi 4 を選び、計算負荷を評価指標に入れたか

実機 LiteRover の搭載計算機は RPi4 で、これは研究のために選んだのではなく既存の農業ロボットプラットフォームの構成そのもの。つまり「非力な組込み計算機の上で成立するか」が最初から制約条件としてある。`docs/research_plan.md` の P1 は最初のタスクを「実現可能性ゲート: RPi求解時間ベンチ」と定義し、`rover/bench_qp.py` で状態3×ホライズンNのQPが制御周期0.1s以内に収まるかを実測、収まる N の上限を以降の設計パラメータにする、としている。実測の結果 N≤30 が0.1s周期内に収まることを確認済み（常用は N=15）。

さらに研究計画には「L1は入力回数を減らすが計算は増える。そのトレードオフ定量化が貢献の柱」と明記されている。L1 は非ゼロ入力を減らして機構的な摩耗・エネルギーを減らす一方、L1ノルムの追加でQPの求解自体は重くなる（コーナリングなど操舵が必要な区間で顕著）。だから「精度が保たれるか」だけでなく「非力な実機のCPUで周期内に解けるか」を評価指標の1つに独立して立てないと、L1の実用性を正しく評価できないと判断した。

### 3. λ をどう決めたか

まず `rover/sweep_lambda.py` で遅延なし・無外乱のシミュレーションによりλを1次元スイープし、RMSE・Σ|δu|・ゼロ率のトレードオフ曲線を取った。ここでは λ=0.25 が3指標すべてでL2を上回る「甘い」結果が出たが、これは実機の遅延（無線通信＋QP求解で約200ms＝2ステップ相当）を考慮していない条件だった。

実機で試すと、素のL1は遅延下でbang-bangなチャタリング（コーナーで入力が限界まで往復し続ける現象）を起こした。これを踏まえ `rover/sweep_grid.py`（λ×move_suppressの2次元・固定遅延2step）と `rover/sweep_delay.py`（遅延0〜4stepに対する各条件の耐性）で条件を振って再評価したところ、単一条件（遅延2step）だけで最良に見えた λ=2 は実際には遅延2stepでのみ現れる偶然の谷で、遅延0/1/3/4stepではRMSEが11〜14cmまで悪化し不採用と判断した（`docs/handoff.md` 2026-07-26セッションに詳細）。最終的に採用したのは **λ=0.3 + 移動抑制項 move_suppress=2.0**（`mpc_core.py` の `move_suppress`、Δu=δu_k−δu_{k-1} の二乗ペナルティでチャタを抑制）で、これは単一条件での最良値ではなく、遅延0〜4stepの範囲で破綻しない「マージン」を優先して選んだ値である。単一条件での最適化は見かけの最適を掴む、という教訓をここで得ている。

## 評価指標

- 追従精度（横偏差 RMSE）
- 消費エネルギーの代理指標（入力積算 Σ|δu|）
- 入力スパース性（操舵補正 δω のゼロ率）
- 計算負荷（QP求解時間、RPi4上でp95が制御周期0.1s内に収まるか）

## 動かし方（実機不要）

以下は clean clone から実際に実行して動作を確認済み（`uv sync` → `uv run pytest -q` で **82 passed / 20 skipped**、skip は全て `results/` に依存するテスト）。

```bash
uv sync                                                 # 依存をインストール
uv run pytest -q                                        # 82 passed / 20 skipped

uv run python rover/test_mpc_sim.py                     # L2-MPC クローズドループ sim
uv run python rover/test_mpc_sim.py --reg l1 --lam 0.3  # L1（スパース、採用値）版

uv run python rover/sweep_lambda.py                     # λスイープ（結果は results/ に出力・gitignore対象）
```

## 実機評価について

LiteRover（Raspberry Pi 4, ROS2 Humble）を使った実機評価は実施済み（3者比較・遅延下でのチャタ検証・λスイープの実機版など）。ただし卒業研究として**未発表**のため、実機の走行結果・rosbag・図表は本リポジトリには含めていない（`results/` と `data/paper/` は履歴からも除去済み）。上記のシミュレーションコマンドのみで、L1/L2の挙動差とトレードオフの構造は再現できる。

## ディレクトリ構成

| パス | 内容 |
|------|------|
| `rover/` | 制御・解析コード |
| `configs/` | 経路（waypoints）定義 YAML |
| `docs/` | 研究計画・引き継ぎ・作業記録 |
| `docs/handoff.md` | 現在地・次・懸念（セッション毎更新） |
| `docs/research_plan.md` | 研究計画（P1〜P7） |
| `tests/` | pytest（`uv run pytest -q` で実行） |
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
| `sweep_lambda.py` | λスイープ（遅延なし条件のトレードオフ曲線） |
| `sweep_grid.py` / `sweep_delay.py` | λ×move_suppress／遅延ステップに対する耐性スイープ |
| `sim_delay_probe.py` | 入力遅延下のチャタ再現・対策（移動抑制）検証 |
| `analyze_bag.py` / `plot_lturn.py` | rosbag 解析・可視化 |
| `teleop_key.py` / `spin_test.py` | 手動操作 / その場旋回テスト |
