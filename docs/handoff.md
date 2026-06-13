# Handoff - 2026-06-13

## 2026-06-13 セッション（実現可能性ゲート＝GREEN）

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
