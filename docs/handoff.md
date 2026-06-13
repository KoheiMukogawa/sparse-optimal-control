# Handoff - 2026-06-13

## 2026-06-13 セッション6（床走行 3者比較＝実施・真値取得）

- **床で Kanayama / L2 / L1(λ=0.3) を直線2m 実走**（ユーザー許可のもと）。
  1本ごとにユーザーがメジャー計測し原点復帰。bag: `results/2026-06-13_floor_*`。
  まとめ: `results/2026-06-13_floor_compare.md`。
- **外部計測（真値）の横ずれ: L1 0cm < L2 4cm < Kanayama 14〜15cm**。提案手法 L1 が
  物理的に最良（向き・前進距離も同序列）。前進は全器 188〜193cm＝目標200に対し
  3.5〜5.6%不足（odom直進スケール過大・全器共通）。
- **重要**: bag の path_error RMSE（odom基準）では L2 が最良に見えるが、外部計測の
  真値では逆転して L1 最良。odom は滑りを見られないため**追従精度評価には外部計測必須**。
- 計算負荷の実機序列: Kanayama(≈0) < L2(p95 44ms) < L1(p95 50ms)、全器0.1s周期内。
- 直線・無外乱では操舵スパース性に差なし（ω0率 全100%）→ L1の真価はL字・外乱で。
- **限界**: n=1・手動原点復帰で初期向きばらつき。「MPC>Kanayama」は頑健、
  「L1≳L2(0 vs 4cm)」は要反復。→ 次は各3〜5本反復 / L字 / 外乱。
- 付随: `rover/teleop_key.py` 追加（連続キーボードteleop・前進と旋回独立・V_MAX0.15）。
  RPi配置済み。teleop_twist_keyboard の「旋回で前進が0になる/速すぎ」を解消。
- **未処理**: nav_base がRPiで起動中の可能性（要停止確認）。

## 2026-06-13 セッション5（実機空転テスト L2/L1＝実施・解析完了）

- ユーザー許可のもと **ジャッキアップ空転で L2 と L1(λ=0.3) を実走**（直線2m）。
  nav_base 起動→mpc_follower 実行→rosbag 記録→bag 回収→nav_base 停止まで完了。
  bag: `results/2026-06-13_spin_l2`, `_l1`。比較: `results/2026-06-13_spin_compare.md`。
- `rover/analyze_bag.py` 追加（rosbags 依存を uv add）: bag から4指標を算出。
- 実機知見:
  - **求解時間は単体ベンチの約2倍**（システム負荷込み）。N=15 で L2 p95 44ms /
    L1 51ms。**0.1s 周期内は維持**だがマージンは小。設計基準は N=15 で余裕を見る。
    max 375/801ms は cvxpy 初回コールド（発進前1回のみ、走行無影響）。
  - 空転・直線・無外乱では **L2≈L1**（RMSE≈0, Σ|u|≈2.0, ω0率100%）。差が出ない。
    → L1 の優位は曲線・外乱・床の滑りで顕在化する（sim と同結論）。
- 結果: 走行時間 L2 13.2s / L1 13.4s（過去 Kanayama 空転 13.8s と整合）。
- 次: **床走行で Kanayama/L2/L1 の3者比較**（外乱あり実走で4指標を取る）。
  床は車体が動くので毎回ユーザー許可を取る。

## 2026-06-13 セッション4（P1.4 実機ノード化＝実装完了/実走行は未）

- `rover/mpc_follower.py` 追加: path_follower.py と同 I/O・同経路原点ロジックで
  制御則を MPCFollower に差し替え。reg=l2/l1, lam, horizon, rate(既定10Hz=0.1s)
  をパラメータ化。求解時間を `mpc_solve_ms`(Float32) で配信（計算負荷の指標）。
  求解失敗時は安全停止。Kanayama は既存 path_follower.py、これが MPC 側。
- ローカル検証: py_compile OK、MPCFollower 構築・求解 OK。
  注意: **L1 は bang-off 特性**で、補正時に ω が上限(2.0)へ張り付きやすい
  （sparse制御の本質。sweepでは全到達なので可制御だが実機初回は慎重に）。
- **未実施（要ユーザー許可）**: RPi へ配置 → ジャッキアップ空転テスト →
  床で3者（Kanayama/L2/L1）実機比較。実機を動かすコマンドは毎回許可を取る。
- 実機メモ: cvxpy import が RPi で 30〜60s。起動ログ「MPC準備完了」を待つ。
  起動例: `python3 mpc_follower.py --ros-args -p path_file:=.../path_straight_2m.yaml -p reg:=l1 -p lam:=0.3`

## 2026-06-13 セッション3（P1.3 L1化・λスイープ＝完了）

- 設計判断: L1 ペナルティ対象を **補正 δu**（参照速度からの偏差）に確定。
  実入力 (v,ω) の L1 だと巡航と競合し停滞 → δu の L1 ならオープンループ版
  sparse_rover.py（δω 78%ゼロ）と整合し、巡航維持＋操舵だけ疎にできる。
  mpc_core.py を修正済み。
- `rover/sweep_lambda.py` 追加: λ スイープでスパース性↔RMSE↔Σ|δu| を集計、
  table.md / sweep.csv / tradeoff.png を出力 → `results/2026-06-13_lambda_sweep/`。
- 結果（N=15, 全5ケース集計）:
  - **λ≈0.25 が L2 を3軸とも上回るスイートスポット**（RMSE 7.7<9.1cm,
    Σ|δu| 92.7<98.8, δωゼロ率 97%>85%）。
  - λ↑で Σ|δu| 単調減・RMSE 悪化。**λ≥3 で追従破綻**。採用レンジ λ=0.25〜2。
  - δωゼロ率は λ=0.25 で 97% 飽和し非弁別的。無外乱simでは L2 も既に操舵が
    疎（85%）。**→ スパース制御の真価は外乱下（P4）で出る見込み**（中間発表の
    モチベーション）。弁別軸は当面 Σ|δu| と RMSE。
- 次: P1.4 実機ノード化 `rover/mpc_follower.py`（path_follower.py の
  kanayama_cmd を MPCFollower に差し替え、L2/L1 切替パラメータ）。
  RPi へ配置→空転テスト→3者（Kanayama/L2/L1）実機比較。

## 2026-06-13 セッション2（P1.2 L2-MPC シミュレーション＝完了）

- `rover/mpc_core.py` 追加: `MPCFollower`（ローリングホライズン QP）。
  状態=Kanayama誤差3、入力=参照速度まわりの偏差δu、終端ソフトコスト、
  実速度 box 制約、L2/L1 切替（DPP パラメータ化で同一問題を再求解）。
  bench_qp.py の誤差ダイナミクスと整合。`mpc_follower.py`(ROSノード)へ流用予定。
- `rover/test_mpc_sim.py` 追加: 非線形差動二輪プラントでのクローズドループ検証。
  test_follower_sim.py と同ケース・同指標（横偏差RMSE/Σ|u|/v0率/求解p95）。
- **L2-MPC は全5ケース到達。横偏差RMSE は Kanayama を全ケースで下回る**:
  直線20cm 16.9→12.5cm、L字経路上 11.6→2.2cm、L字15cm 12.6→5.9cm。
  → P1.2 完了条件「Kanayama と同条件で追従」を達成（上回る）。
- L1 切替も動作確認。λ=1.0 では実入力L1ペナルティで v→0 が支配的（v0率84%）
  になり L字は停滞＝Maximum Hands-off の効果が出ている。直線は到達。
  → これが P1.3 で λ スイープして定量化すべきスパース性↔追従のトレードオフ。
- 注意: L1 の入力ペナルティ対象（実入力 vs 補正量）は P1.3 で要検討。
  現状は「実 v,ω の L1」=hands-off 本来の定義。巡航速度と競合する設計上の論点。
- 次: P1.3「L1化とλスイープ」→ トレードオフ曲線。その後 P1.4 実機ノード化。

## 2026-06-13 セッション1（実現可能性ゲート＝GREEN）

- `rover/bench_qp.py` 追加: 状態3×ホライズンN の MPC-QP（L2/L1切替・終端ソフト
  コスト・OSQP warm_start）の求解時間ベンチ。hwmon の `in0_lcrit_alarm` を併監視。
  `build_problem` は `mpc_follower.py` にそのまま流用する設計。
- **RPi4 実測（results/2026-06-13_rpi_bench/）**:
  - L2: N=30 で p50 17.6 / p95 17.7 ms。N=5〜30 全て 0.1s 周期内。
  - L1: N=30 で p50 26.3 / p95 29.2 ms（L1 は計画通り L2 より重いが余裕あり）。
  - 低電圧アラーム lcrit=0（計算による電源律速なし）。
- **判定: N=30 まで 0.1s 周期に収まる → ゲート通過。設計上限は N≤30 とする**
  （安全マージン込みで当面 N=15〜20 を主に使う想定）。
- 接続メモ: 家Wi-Fi `192.168.0.31` で疎通（ホットスポット4.1は不通だった）。
  RPi の cvxpy 1.7.5 / osqp 1.1.2 導入済み（import は初回30〜60s と遅い）。
- 次: P1.2「L2-MPC シミュレーション」（test_follower_sim 拡張でローリング
  ホライズン骨格をデバッグ→Kanayamaと同条件で追従確認）。その後 L1 化。

## 2026-06-12 セッション

### 研究計画

### 研究計画
- `docs/research_plan.md` 作成（P1〜P7、中間発表9〜10月・提出2月・実機必須前提）
- `docs/sprint_2026-06_自立走行.md` 作成（4日スプリント計画）

### 自立走行の実装と空転テスト（exp/autonomous-drive ブランチ）★最大の進捗
- 公式サンプル `vstoneofficial/lightrover_ros2` を調査:
  - 速度指令トピックは **`rover_twist`**（`/cmd_vel` ではない）、/odomは約30Hz
  - モータ制御・オドメトリはサンプル流用で済み、自作は経路追従ノードのみでよいと判明
- 実装（SLAM・DDS・AMCLはスキップ、odomのみ・RPi完結構成）:
  - `rover/follower_core.py` — 経路射影・Kanayama誤差・制御則（ROS非依存、L1-MPC差し替え時に流用）
  - `rover/path_follower.py` — ROS2ノード（/odom→rover_twist、odom途絶で自動停止）
  - `configs/path_straight_2m.yaml`, `configs/path_L_turn.yaml`
  - `rover/test_follower_sim.py` — 運動学シミュレーションによる検証
- シミュレーションで発見・修正したバグ:
  - ゴール手前で減速しないと v_r/k_x（20cm）先の平衡点で停止 → 距離比例減速 `goal_scaled_vr` 追加
  - 横偏差が残るとゴール判定円に入らない → ゴール線通過判定 `goal_crossed` 追加
- **実機（ジャッキアップ・空転）テスト成功**:
  - 直線2m: 13.8秒で目標到達（シミュレーション予測13.7秒とほぼ一致）
  - L字（1.5m+90度旋回+1.5m）: 横0.46m初期ズレからの復帰込みで17.4秒で完走
- RPi環境修正: transforms3d 0.3.1（apt版、np.float使用）が新numpyと非互換で
  odom_manager が起動不能 → `pip install --user -U transforms3d`（0.4.2）で解決

## 現在の進捗（同日午後更新）

| 項目 | 状態 |
|------|------|
| Kanayama誤差計算＋経路追従制御 | ✅ 実装済み |
| 床での実走行（直線2m×3本） | ✅ 完走。実測でodom直進誤差+1.5%確認 |
| 床での実走行（L字・90°旋回） | ✅ 完走。**旋回角が実測80°（10°不足）→ 回転odom約11%過大** |
| 経路原点の起動時設定 | ✅ 実装（odomオフセット問題の根本対策） |
| rosbag評価データ | ✅ 4本回収 → `results/2026-06-12_floor_runs/` |
| L1スパースMPCノード | ❌ 未着手（follower_coreに差し替える設計） |
| SLAM / DDS | 保留（自立走行に不要と判断しスキップ中） |

## 回転キャリブレーション（同日夕方に完了）

- spin_test.py（その場旋回テスト）で滑りを定量化: その場旋回は速度依存の
  滑り5〜11%、定数補正だけでは消えない（最悪条件のため許容）
- **RPiの `~/ros2_ws/src` の odometry.py / pos_controller.py の ROVER_D を
  0.143→0.1514（実効トレッド）に変更し再ビルド済み**（元は.origで保存）
- L字で検証: 旋回角が実測80°→**ほぼ90°に改善**（run7）。アーク旋回はこれでOK
- followerのyaw_scale補正は無効と判明し撤去（bag解析: 位置項支配のため）
- 教訓: odom補正は積算源（odometry本体）に入れること。follower側では効かない

## 次にやるべきこと（優先順）

研究計画2026-06-12改訂版（`docs/research_plan.md`）に準拠:

1. **RPi求解時間ベンチ（実現可能性ゲート・半日）**: cvxpy+OSQPで想定サイズの
   QP求解時間を計測。hwmon低電圧アラーム併監視。0.1s周期に収まるN上限を把握
2. **L2-MPCシミュレーション**: ローリングホライズン骨格（終端ソフト制約）を
   test_follower_sim拡張でデバッグ → そのままL1化（目的関数1行差）してλスイープ
3. **実機ノード化** `rover/mpc_follower.py`（L2/L1切替式、follower_core流用）
4. rosbag解析パイプライン（4指標自動算出）

実装順序の根拠: L2-MPCはベースラインとして必須＆MPC骨格のデバッグが先。
L1の効果はL2との差分でのみ分離できる（3者比較: Kanayama/L2/L1）。
補足: 補正後トレッド定数での直進再検証はついでに（影響軽微の見込み）

## 環境メモ（更新）

- RPi接続OK: `ssh mukougawakouhei@192.168.0.31`
- 実機コード配置: RPi `~/sparse_control/`（rover/, configs/）
- ベースノード起動: `ros2 launch lightrover_ros nav_base.launch.py`（自立走行時はpos_joycon不可: gamepadとrover_twistが競合）
- 経路追従: `cd ~/sparse_control/rover && python3 path_follower.py --ros-args -p path_file:=/home/mukougawakouhei/sparse_control/configs/path_straight_2m.yaml`
- 停止: ノードのCtrl+C（終了時に停止指令送信）/ `pkill -f "[p]ath_follower"`
- RTPS_TRANSPORT_SHMエラーは無害（UDPフォールバック）
- **odomは再起動しないとリセットされない** → 連続テスト時はodom_manager再起動 or 経路を現在位置基準に

## 電源・可視化の検証メモ（2026-06-12夜 追記）

- 新モバイルバッテリー（KIYOSO T173LP, 20000mAh/PD65W）+ **USB-A→C接続**で検証:
  実運用負荷（ベース+LiDAR+SLAM）では低電圧なし=OK。CPU全コア100%では
  低電圧検出ありで余裕は少ない。**cvxpyベンチマーク時はhwmonの
  `/sys/class/hwmon/hwmon1/in0_lcrit_alarm` を併監視すること**
- **RVizライブ表示はSSH X11転送で運用可能と確認**（DDS不要）:
  `ssh -X ... 'bash -lc "source ...; rviz2 -d ~/rviz_slam.rviz"'`
  走行中の地図構築をラップトップからリアルタイム視認できた
- SLAM地図は2回目（map_room2）が正式版。1回目は実機が宙に浮いた状態の
  無効データ（SLAM時は床上で実走行していることを確認する）

## 懸念事項

- 空転テストは負荷なし。床では摩擦・慣性でゲイン再調整が必要な可能性
- goal_tol=5cmはodomのみだと床では厳しいかも（ゴール線判定があるので完走はする）
- 実走行時の安全: 初回はv_r=0.05に下げる選択肢も
