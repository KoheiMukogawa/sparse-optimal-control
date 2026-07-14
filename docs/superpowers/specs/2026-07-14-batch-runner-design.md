# バッチ実験ランナー 設計 (2026-07-14)

## 目的

実機実験サイクル（ノード起動→走行→bag保存→解析→集計）を条件リスト1ファイルで
自動化し、反復実験（各条件 n=3〜5）の統計化を現実的な手間にする。
開発・検証はロボット不要の sim バックエンドで行い、実機バックエンドと差し替え可能にする。

- 本命ユース: L字で Kanayama / L2 / L1(λ=0.3) / 修正版L1(ms=2.0) を各3本 →
  `runs.csv` と条件別 平均±標準偏差 の `summary.md` が自動生成される。
- 実機走行は毎回ユーザー立ち会い。**各本の前に Enter 待ちで停止し、Enter=走行許可**
  （CLAUDE.md の「実機を動かすのは毎回許可」をこの方式で満たす。バッチ開始時にも全体確認）。

## アーキテクチャ（承認済み: 案B）

ランナーはラップトップ（本リポジトリ）で動く。実機バックエンドは SSH で RPi に
指示を出し、bag を即回収してその場で解析・CSV追記する（1本ごとに指標が見える）。
DDS ラップトップ↔RPi 直結は使わない（未解決のため従来どおり回避）。

```
configs/batch_*.yaml ──▶ run_batch.py（条件ループ・resume・集計）
                              │  run_one(condition, rep) → RunRecord
                ┌─────────────┴─────────────┐
        RealBackend (SSH+scp)        SimBackend (ROS不要)
        nav_base確認→Enter待ち→       sim_delay_probe と同じ
        node起動+bag record→          差動二輪プラント+入力遅延
        ゴール/タイムアウト検知→        +ノイズで閉ループ実行
        停止→bag回収→解析
                └─────────────┬─────────────┘
                     exp_metrics.py（時系列→指標、bag/sim共通）
                              │
              results/<日付>_<バッチ名>/{bags, runs.csv, summary.md}
```

## コンポーネント

既存の rover/ フラット構成に合わせ、新規3ファイル＋既存1ファイル改修。

### 1. `rover/exp_metrics.py`（新規・共通指標計算）

入力: 時系列 `TimeSeries(twist=[(t,v,w)], perr=[(t,y_e)], solve_ms=[...])`。
出力: 指標 dict。`analyze_bag.py` の4指標に、L字比較で使ったチャタ指標を統合する:

- `drive_s, steps`（走行区間: |v|>0.005 の窓）
- `rmse_cm`（path_error.y、odom基準である注記は従来どおり）
- `sum_u`（Σ(|v|+|ω|)·dt）
- `w_zero_ratio`（|ω|<0.05）
- `flips`（ω符号反転回数、デッドバンド0.05）、`sat_ratio`（|ω|>1.5）、`max_w`
- `solve_p50/p95/max`（Kanayama は NaN）

`analyze_bag.py` は「bag→TimeSeries」変換＋本モジュール呼び出しに改修
（**回帰チェック: 6/13 の既存 bag 6本で改修前後の数値一致を確認**）。

### 2. `rover/exp_backends.py`（新規・実行バックエンド2種）

共通インターフェース: `run_one(cond, rep, outdir) → RunRecord`
（RunRecord = 指標 dict + `ok`(到達/タイムアウト/失敗) + `bagdir` + メモ）。

**SimBackend**（既定・ROS不要）
- `sim_delay_probe.simulate` の差動二輪プラント＋入力遅延＋測定ノイズを流用し、
  Kanayama（follower_core の制御則）にも対応（test_follower_sim と同ロジック）。
- sim 専用条件: `delay_steps`（既定2=実機相当）、`pos_noise`、`yaw_noise`。
  乱数 seed は rep 番号から決める（反復ごとに異なり、再実行で再現可能）。
- 時系列を直接 `exp_metrics` に渡す。bag は作らない（`bagdir` は空欄）。

**RealBackend**（SSH オーケストレーション）
- 前提: RPi 到達可能（192.168.0.31 / 192.168.4.1 の順に試行）、
  実機コードは RPi `~/sparse_control/` に配置済み（配置更新は本ランナーの範囲外、
  ただし起動前に mpc_follower.py 等の md5 を比較し、差異があれば警告して確認を求める）。
- バッチ開始時の preflight（1回）: SSH疎通 → `ros2 node list` で nav_base 系
  （odom_manager, pos_controller）の稼働確認。未稼働なら起動コマンドを提示して中断
  （自動起動はしない: 二重起動事故を避ける）。残存 follower プロセスを pkill。
- 1本の流れ:
  1. 「次: <条件名> <k>/<n>本目。ロボットを原点に戻して Enter（q で中断）」で停止。
  2. ssh で `ros2 bag record`（/odom /rover_twist /path_error /mpc_solve_ms）を
     RPi 側一時ディレクトリに開始。
  3. ssh で follower ノード起動（Kanayama=path_follower.py、L2/L1=mpc_follower.py、
     パラメータは条件から生成）。stdout をストリーム監視。
  4. 「目標到達」ログで完走検知。タイムアウト（`timeout_s`、経路ごとに設定・
     既定60s）で打ち切り。いずれも SIGINT でノード停止（ノードの finally が停止
     指令を publish。odom途絶停止も既存の保険として効く）。bag record 停止。
  5. scp で bag を `results/<バッチ>/<条件>_r<k>/` に回収 → RPi 側一時 bag を削除 →
     `exp_metrics` で解析 → CSV 追記 → 指標1行を画面表示（異常走行に即気づける）。
- SSH 切断時: ノードは RPi 側で自走し到達時に自停止（安全は既存設計で担保）。
  ランナーは再接続→残存プロセス pkill→当該 run を `失敗` 記録し、次の run の
  Enter 待ちへ（ユーザーが再走するかは resume で制御）。
- `--dry-run`: 実行する ssh コマンド列を表示するだけ（実機なし検証用）。

### 3. `rover/run_batch.py`（新規・CLIエントリ）

```
uv run python rover/run_batch.py configs/batch_Lturn_3way.yaml
    [--backend sim|real]   # 既定は YAML の backend 指定（無ければ sim）
    [--resume]             # runs.csv にある ok 済み (条件,rep) をスキップ
    [--only 条件名]        # 1条件だけ再走
    [--dry-run]
    [--summarize]          # 走行せず既存 runs.csv から summary.md のみ再生成
```

- 条件×反復の直積をループし `run_one` を呼ぶ。1本ごとに `runs.csv` へ追記
  （途中中断してもそれまでの結果は残る）。
- 全件終了時（または `--summarize` 単独実行で）`summary.md` を生成:
  条件ごとの 到達率、rmse_cm / sum_u / flips / w_zero / solve_p95 の 平均±標準偏差 表。
- 出力先: `results/<YYYY-MM-DD>_<バッチ名>/`。git hash・使用YAML の内容を
  `runs.csv` の列とヘッダコメントに記録（実験条件の再現性）。

### 4. `configs/batch_*.yaml`（条件ファイル・コミット対象）

```yaml
name: Lturn_3way          # 出力ディレクトリ名に使用
path_file: configs/path_L_turn.yaml
repeats: 3
timeout_s: 60
common: {horizon: 15, rate: 10.0}          # 全条件共通のノードパラメータ
sim: {delay_steps: 2, pos_noise: 0.0, yaw_noise: 0.0}  # simバックエンド時のみ使用
conditions:
  - {name: kanayama, controller: kanayama}
  - {name: l2,       controller: l2}
  - {name: l1,       controller: l1, lam: 0.3}
  - {name: l1_ms2,   controller: l1, lam: 0.3, move_suppress: 2.0}
```

## CSVスキーマ（runs.csv・1走行1行）

`batch, cond, rep, backend, timestamp, git_hash, controller, lam, move_suppress,
horizon, v_r, ok, drive_s, rmse_cm, sum_u, w_zero_ratio, flips, sat_ratio, max_w,
solve_p50, solve_p95, solve_max, bagdir, note`

## エラー処理方針

- 1本の失敗（タイムアウト・求解失敗・bag空・SSH切断）はその run を `ok=false` で
  記録して継続。バッチ全体は止めない（実機の再走判断はユーザー）。
- 安全系はランナーでは新設しない: 停止はノード既存機構（到達時停止・SIGINT時停止・
  odom途絶停止・求解失敗停止）に委ねる。ランナーが増やすのは「SIGINT を送る」だけ。
- Ctrl+C（バッチ中断）: 実行中ノードへ SIGINT→bag record 停止→回収を試みてから終了。

## テスト計画

1. `exp_metrics` 回帰: 6/13 既存 bag 6本（floor/Lturn × 3手法）で改修後
   `analyze_bag.py` の出力が記録済みの数値と一致。
2. Sim E2E: 上記 batch YAML（sim, 2 reps）で runs.csv / summary.md が生成され、
   L1 が flips 大・L1+ms=2.0 で flips≈1 という既知の結果を再現（sim_delay_probe と整合）。
3. Real dry-run: `--dry-run` で ssh コマンド列を目視確認。
4. 実機統合テスト = 最初の本番バッチ（修正版L1のL字再走を含む・ユーザー立ち会い）。

## 範囲外（今回作らない）

- 自動原点復帰、カメラ/AprilTag 真値計測（別プロジェクト。ただし runs.csv に
  `note` 列があるので外部計測値の手動追記は可能）。
- RPi へのコード自動デプロイ（md5 差異警告のみ）。
- ノード側の変更（実機検証済みの follower ノードには手を入れない）。
