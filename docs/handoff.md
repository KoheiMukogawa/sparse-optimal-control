# Handoff - 2026-07-17

過去セッション（2026-06-12〜2026-07-15）の全文は
`docs/作業記録/handoff_archive.md` に退避（必要時のみ参照）。

## 2026-07-17 セッション14（進行中）

- **床タグが移動済み＝旧 floor_tags 座標は無効**。solvePnP は黙って間違った
  値を出す（検出は成功するため警告なし）。configs/camera_truth.yaml の
  floor_tags は再測定まで信用しないこと。
- **方針決定: タグ自動サーベイ（案B）を実装する**。毎セッション、タグを適当に
  置いて1コマ撮影→タグ4隅＋黒枠実寸15cmで各タグをPnP→床平面拘束で中心(x,y)を
  復元→原点と軸はタグが定義（例: tag0中心=原点・tag0→tag3=+x）→ yaml 自動生成。
  メジャー実測を廃止。背景: 自宅環境で毎セッション全撤収（タグもコーステープも）
  のため、固定設営が維持できない。
  - homing は target をタグ座標系で受けるので床の原点テープ自体が不要になる
  - スケール精度は印刷黒枠の実寸1回測り（±0.5mm→1.2mで4mm）に依存。同一印刷
    ロットなら共通
  - **合成テスト（tests/synth_scene.py 流用）で位置誤差を先に検証し、1cm超なら
    案Bを捨てて手計測＋再投影残差チェック（案A）へ戻る**判断ポイントを置く
  - 一体型タグシート案は不採用（走行面を変える・シワで相対位置が狂う・製作重い）
- ついで課題（案Aに戻らなくても入れる価値あり）: solve_camera_pose に再投影
  残差チェック（例: 2px超で CalibError）。タグ移動・yaml古い・解像度違いを
  黙って通さないため。
- トークン節約: handoff.md を現役分のみに分割（本ファイル）、CLAUDE.md の
  現在地を圧縮。SDD（haiku実装/sonnetレビュー/fable最終レビュー）は継続。
- **タグ自動サーベイ実装完了（SDD・Task1-7）**: spec/plan 承認→7タスク実行。
  `rover/survey_tags.py`（初期解IPPE→16隅一括最適化→規約変換→チェック→yaml更新、
  CLI: ライブ5フレーム平均/--image/--dry-run）＋ `solve_camera_pose` に
  ランタイム残差チェック（2px超CalibError＝タグ移動・古いyamlを黙って通さない）。
  - **合成ゲート合格: floor_tags 復元 最悪0.57cm（12ケース）／robot_pose E2E
    伝播 0.033cm・0.0016rad** → 案B成立、メジャー実測は廃止
  - 副産物の知見: **7/16の実タグ配置は原点(0,0)も凸包外だった**（check_layout が
    過去実運用より厳格）。今後はサーベイがコース内包を強制する
  - テスト67本全green。詳細ledger: .superpowers/sdd/progress.md
- 次: **final review（fable・全ブランチ）→ 実機側の残作業**:
  ①floor_tag_size_m を定規実測（yaml の 0.150 は仮置き） ②サーベイ実行→
  既知点2点で静置照合（±1〜2cm） ③Phase 2+3 実機E2E（usbipd attach→
  ブリッジ疎通→homing単体→短autoループ→フルバッチ、
  手順: docs/作業記録/全自動バッチ運用手順.md）

## 2026-07-16 セッション13b（Phase 1 実走行検証＋Phase 2+3 全自動化実装）

- **C270キャリブ完了**（誤差0.261px・ブレフレーム自動除去をCLIに追加）、
  床タグ4枚の座標を距離2点測位で確定（コース実寸: 1m L字の脚99/100cm）。
- **カメラ真値の実走行検証＝合格**: 1m L字 l2 1本で手実測と **2.5cm / 1.6°一致**、
  odom が見逃す x+7.9cm の滑りを検出（results/2026-07-16_Lturn_1m_smoke2/）。
  教訓: **床タグはコースを囲む配置にする**（片側偏在だとゴールが外挿地帯で10-15cm誤差）。
- **Phase 2+3 実装完了**（計画: plans/2026-07-16-camera-truth-phase23.md、SDD実行）:
  - `rover/truth_live.py`: C270ライブ検出（calibrate→スレッド→pose/start/stop）
  - `rover/udp_twist_bridge.py`: RPi用 UDP→rover_twist（watchdog0.5s・seq破棄・クランプ）
  - `rover/homing.py`: TURN→GO→ALIGN の原点復帰（±3cm/±5°・pose途絶/コース外/
    タイムアウト停止）。**計画のバグをテストが検出**: ゴール行き過ぎでKanayamaの
    FFと引き戻しが釣り合う平衡点（4.8cm停滞）→ 10cm圏内は後退許可のgo-to-point則
  - `rover/run_batch.py --auto`: 一括許可→走行→truth_*.csv→復帰→次走行。
    q+Enter即停止・連続2失敗停止・truth_end_*/truth_rmse_cm 列追加
  - テストは全て fake/合成画像（実カメラ・実ソケット不使用）で全green
- **最終レビュー（fable・sim追試90本つき）: Important 2件を 0b34e81 で修正して Ready**:
  ①初回走行失敗時に outdir 未作成で truth CSV 書き込みがバッチ全体を落とす穴
  ②--resume/--only が --auto で黙って無視（済み走行の再走・意図しない実機走行）。
  ＋ _prompt の誤入力ドレイン・CameraSource を MJPG＋バッファ1（usbipd遅延対策）。
  テスト56本全green。homing はsim90本（遅延0.3s・loss10%込み）で全収束を追試済み。
- **実機E2Eは未実施**: usbipd で C270 を WSL に attach（ユーザー作業）→
  手順: docs/作業記録/全自動バッチ運用手順.md の「初回実機E2E」節（トラブル
  シュート節も参照）。RPi は本日シャットダウン済み。
- 運用注意: nav_base の多重起動事故あり（同名ノード3重でモータ指令競合の恐れ→
  全kill→1つだけ起動で復旧）。ブリッジはlaptop側再起動時にRPi側も再起動（seq巻き戻り対策）。
