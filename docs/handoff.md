# Handoff - 2026-07-26

過去セッション（2026-06-12〜2026-07-15）の全文は
`docs/作業記録/handoff_archive.md` に退避（必要時のみ参照）。

## 2026-07-26 セッション18（S1・S2 完了。設計パラメータの根拠が確定）

- **S1 実施**: `rover/sweep_grid.py` を新規作成（TDD）。λ 5点×$w_{ms}$ 5点=25条件を
  **遅延2step下**で掃引 → `results/2026-07-26_lambda_ms_grid/`。
  遅延モデルは `sim_delay_probe.simulate` を流用（モデルを増やさない）。
- **S2 実施**: `rover/sweep_delay.py` を新規作成（TDD）。6条件×遅延0〜4step=30条件
  → `results/2026-07-26_delay_sweep/`。
- **S1 単独の読みは2件とも誤りで、S2 が訂正した**（重要）:
  - 「λ=2/$w_{ms}$=0.5 が全指標最良」→ **遅延2stepでのみ現れる偶然の谷**。
    RMSE は遅延0/1/3/4step で 11〜14cm。**λ≥1 は採用不可**
  - 「$w_{ms}$ は 0.5 で足り 2.0 は過剰」→ **$w_{ms}$=0.5 は遅延4stepで崩壊**
    （Σ|u| 16.16・反転9）、2.0 は保つ（7.52・反転4）
  - → **現行採用値 λ=0.3 / $w_{ms}$=2.0 は妥当**。$w_{ms}$=2.0 が買っているのは
    遅延2step時点の性能ではなく**遅延マージン（3〜4stepまでの余裕）**
- **その他の知見**: L2 は遅延4stepまで反転0で頑健／ナイーブL1は遅延1stepまで無事で
  **2stepで急激に破綻**（しきい値的）／対策は遅延を無効化せず破綻を遅らせるだけ
  （$w_{ms}$=2.0 でも ω0率 93→69%）／RMSE は λ に非単調（λ=1 が谷）
- **方法論的教訓**: 単一条件での最適化は見かけの最適を掴む。
  卒論 **7.4.1節を新設**して考察に入れる。以降の sim（特に S4 外乱注入）でも
  必ず条件を振って結論の頑健性を確認すること。
- **outline.md 更新**: 3.6・5.2・5.6・5.7 節を改訂、7.4.1 新設、
  図5.4/5.5 は取得済みに、表B の S1・S2 を完了に、
  **表A に R16（実機への人工遅延注入）を追加**。
  R16 は「$w_{ms}$=2.0 は遅延マージンを買っている」を実機で直接検証できる唯一の実験で、
  `mpc_follower.py` に指令バッファを1つ足すだけで実装できる。R5 と連続実施が効率的。
- **S6 も実施（bang-off-bang 図）**: `rover/openloop_sparse.py` を新規作成（TDD）
  → `results/2026-07-26_openloop_bangoffbang/`。
  - **S6 の前提が誤っていた**: `docs/作業記録/sparse_rover.py` は Python ではなく
    **拡張子だけ .py の報告書**。実装コードも図の PNG もリポジトリにも git 履歴にも
    存在しなかった。→ 定式化から再実装した
  - **記録値を完全再現**（δω ゼロ 39/50=78%・L1ノルム 30.65・終端誤差 1e-14 オーダー）。
    独立実装で一致したので**卒論の 78% は検証済み**
  - **論文で説明が要る構造が判明**: δv のゼロ数は 0/50。$B_d$ の構造上 δv と δω は
    完全分離し、**δv 側の部分問題は退化**（終端制約が Σδv=-20 の1本だけ→同符号なら
    配り方によらず L1 同値＝最小解が無数）。δv のスパース性はソルバー依存で問題の性質でない。
    **「スパース性を δω で測る」という既存の選択がここで数学的に正当化された**
    （`rover/sweep_lambda.py` の主指標選択の根拠にもなる）
- テスト 71→**85本 all green**（`test_sweep_grid.py` 6・`test_sweep_delay.py` 4・
  `test_openloop_sparse.py` 4）
- 次: 実機は R0 待ち。sim 側は **S9（既存bag12本から論文図・1日）→ S12（real/sim散布図・
  1時間）**。第2章の執筆は実機・sim いずれにも依存せず着手可能。

## 2026-07-25 セッション17（卒論の詳細章立てを作成）

- **`docs/thesis/outline.md` を新規作成**（節レベルの詳細アウトライン・本文計約53p想定）。
  主軸は「スパース制御(L1)を実機で成立させる条件は何か」。第5章が心臓部
  （λ選定→チャタ発現→遅延で機序特定→move_suppress→sim予測性）。
  **第6章で「総合最良は L2-MPC・提案L1は勝っていない」を正直に報告**し、第7章で
  「L1が買っているもの＝ゼロ入力率0.88・最終向き精度1〜10°」として位置づける構成。
- 各節に 主張／図表番号案／**データ出所の実パス or【未取得】**／目安ページ／
  【付録候補】印を記載。数値は全て results/ 以下の実データから引いた（創作なし）。
- **末尾の2表が当面の作業計画**（RPi4 故障中のため）:
  - 表A 実機再開時の取得リスト R0〜R15。中核は R0(Pi復旧)・R15(実機写真)・
    R4(スモーク)・**R5(入力遅延の直接実測＝論文の核を推定値から実測値に格上げ)**・
    R1/R2/R3(外乱バッチ＋カメラ真値3者比較＋Kanayama真値)
  - 表B 実機なしで今すぐ取れる sim リスト S1〜S12。推奨順は
    **S6(bang-off-bang図の再生成)→S9(既存bag12本から論文図を作る)→S12(real/sim散布図)
    →S1(λ×move_suppress 2次元グリッド・遅延2step下)→S2(遅延0〜4step拡張)**
- **見つかった穴**: 現行の λスイープ（`results/2026-06-13_lambda_sweep/`）は
  **遅延なし・無外乱 sim** であり、論文の主軸（遅延下での成立条件）と条件が食い違う。
  S1 でこれを埋めるのが sim 側の最優先。
- 次: 実機は R0 待ち。実機なしで S6→S9→S12→S1→S2 を進めれば第2・3・5章と
  第6章の作図がほぼ埋まる。

## 2026-07-22 セッション16（SDカード書き込み成功・Pi起動せず＝EEPROM疑い/未完）

- **書き込み失敗の真犯人＝特定のUSBカードリーダー**（切り分け完了）:
  - そのリーダーは**読み30GBは完走するが、書き込みは毎回きっかり約1.4GB
    （1519583232 bytes）でデバイス脱落**（dmesg: `detected capacity change
    ... to 0`→再認識失敗ループ）。64GB・32GBの**2枚とも同一挙動**＝カードは無罪
  - **セルフパワーハブでも改善せず＝ハブ電力は無罪**（昨夜の仮説は否定）
  - **別のリーダーに替えたら書き込み成功**。＝元リーダーの書き込み回路の問題
    （負荷時の電力/信号。Anker 655 USB-Cハブ × Laptop Go Type-C との相性も一因かも）。
    **今後イメージ書き込みは「動いた方のリーダー」を使う**
- **イメージ書き込み成功**: Etcherで64GBカードに書いてValidate緑・32GBにも焼き直し。
  `~/sd_rescue.img`（fsck済clean）と複製 `C:\Users\mukou\sd_rescue.img` 両方健在
- **イメージ内容は健全と検証済み**（WSLから起動せず確認）:
  - FATブート: fsck.vfatクリーン・602ファイル、`start4.elf`/`vmlinuz`(10.5MB)/
    `initrd.img`(45.8MB)/`bcm2711-rpi-4-b.dtb` 全て非ゼロ、U-Boot構成
  - ルートext4: **ラベル=`writable`**（cmdlineの`root=LABEL=writable`と一致）・
    **state=clean**・`Last mounted on: /`。＝カードの中身は問題なし
- **しかしPiがどちらのカードでも起動しない**: HDMIは`hdmi_safe=1`でも完全無信号・
  **PWR赤/ACT緑とも点きっぱなし（点滅なし）**・SSH不可（LAN上に22番なし）。
  2枚の検証済みカードで同一症状 → **原因はPi本体**。7-20の電源断が
  **Pi4のEEPROMブートローダー（基板SPI ROM）も壊した**が最有力
  （EEPROM破損＝どのカードでも起動不可・HDMI無信号・LED点灯の典型）
- **EEPROM recovery カードを焼き付け済み**（Imager → Misc utility images →
  Bootloader (Pi4/400) → SD Card Boot）。**未テスト＝ライトローバーが手元に無いため**
- **次回やること（本体が戻ったら）**:
  1. **EEPROM recoveryカードで起動**（AC給電）。成功＝緑ACTが高速点滅/HDMI緑。
     10秒待って電源OFF
  2. recoveryカードを抜き、**rescueカード（32GB・我々の修復イメージ）**を挿して起動。
     32GBは「このPiで起動実績のある実績カード」なので優先（64GBはPiスロット相性の
     懸念あり・かつgrowpart要）。起動確認: HDMI or `ssh mukougawakouhei@192.168.0.32`
  3. **recoveryしても無反応（LED不変）→ Pi基板/電源のハード故障**。新しいPi4が必要
     （でもrescueカードを挿すだけでシステム完全復元できる。イメージは安全に保管済み）
  4. 起動できたら: 64GB利用時のみ `growpart /dev/mmcblk0 2`→`resize2fs
     /dev/mmcblk0p2`（32GBは不要）→ nav_base単独→サーベイ→スモーク2本→外乱バッチ
  5. `/etc/sudoers.d/sd-repair` を復旧完了後に削除（`sudo rm /etc/sudoers.d/sd-repair`）
- **教訓**: 研究システムはイメージ2本で完全保全済み。最悪Pi本体が死んでも
  新Pi4にカードを挿せば復元可能＝データ喪失リスクは無い

## 2026-07-20 セッション15（RPi SDカード破損・復旧作業/未完）

- **事故**: RPi起動放置中にバッテリー切れ→書き込み中の電源断でSDカード
  （32GB）のファイルシステム破損。ブート不能・HDMI No signal
- **実験は未実施**（サーベイ合格まで到達したが走行ゼロ）。今日やった作業:
  - カメラ画角調整用に `rover/preview_camera.py` を新規作成
    （ライブプレビュー・タグ枠描画・`--guide` で前回サーベイ合格値を
    ×印表示。画角合わせに有用、次回セッションでも使える）
  - **サーベイは合格**: floor_tags 再取得・RMS 0.24px（configs/camera_truth.yaml
    に反映済み）。カメラ位置・タグ配置はそのまま次回使える可能性が高い
    （ただしSDカード復旧後、RPi側の状態次第で再確認要）
  - SDカード救出: `ddrescue -n` で32GBカードから**完全救出**
    （エラー0・bad area 0）→ `~/sd_rescue.img`（WSL内）
  - イメージ上で `e2fsck -f -y` 修復 → ext4 superblock state=clean 確認済み
  - 無修正の複製 `~/sd_rescue_orig.img` も保持（両方 WSL 内に現存、消さないこと）
- **未解決の核心問題: USBハブが書き込み負荷で不安定**。今夜、経路の異なる
  3パターン全てで同じ壊れ方をした:
  ①WSL経由 `dd`/`ddrescue` で64GBカードへ書き込み→ USBリセット連発
  （dmesg: `reset SuperSpeed USB device` 175ms間隔・`Buffer I/O error`）
  ②Windowsネイティブ Raspberry Pi Imager で64GBカードへ書き込み→
  `I/O device error` で失敗（WSL非経由でも同じ症状＝ソフトではなくハブが原因と
  ほぼ確定） ③別の128GB USBメモリへ image コピー→ 破損（`E:¥にアクセスできません`）
  - 読み出し（ddrescue -n）は同じハブ経由で問題なく完走したので、**書き込み時の
    電流/電力不足**が疑わしい（セルフパワーでないUSBハブの限界）
  - ラップトップ本体の直挿しUSBポートの有無は**未確認**（次回最優先で確認）
- **次回やること（最優先: RPi復旧）**:
  1. **ハブを介さない直挿しポートがあるか確認**（Surface Laptop Go の内蔵USB-A等）。
     あれば同じ書き込みで切り分けられる
  2. なければ**セルフパワー(外部電源付き)USBハブ**を用意して再挑戦
  3. 書き込み先は64GBカード推奨（研究的には32GBで足りるが、今夜32GBも書き込み
     試行で状態不明瞭なため、素直な64GBが安全）。イメージは `~/sd_rescue.img`
     （WSL内, fsck修復済み・state=clean）を使う。書き込み後は
     `growpart`→`resize2fs` でパーティション拡張（64GB化）を忘れずに
  4. 書き込み成功→RPiで起動確認→nav_base単独起動確認→サーベイ再実行
     （タグ配置は動かしていなければ流用可、心配なら再実行）→
     スモーク2本→外乱バッチ（load000/500/1000, configs/batch_Lturn1m_load*.yaml）
  5. 一時的に付与した `mukougawakouhei ALL=(ALL) NOPASSWD: ...` in
     `/etc/sudoers.d/sd-repair` は**復旧完了後に削除**
     （`sudo rm /etc/sudoers.d/sd-repair`）
- **教訓（恒久ルールに追加検討）**: RPiをベンチで長時間つけっぱなしにする時は
  バッテリーでなくAC給電にする。バッチ実行前に充電残量を確認する

## 2026-07-17深夜〜18 セッション14b（初回実機E2E＝ほぼ完走・多数の実機知見）

- **サーベイ実機初回: 合格**。RMS 0.41px・dry-run→本実行で1mm再現・静置照合は
  メジャー実測と +0.5/−1.3cm 一致（results/2026-07-17_survey_e2e/notes.md）。
  メジャーレス設営が実機で成立
- **カメラは 640x360 で運用**（configs 更新済み・K は1/2スケール）。
  WSL標準カーネルの vhci_hcd が等時転送を約64KB/フレームで頭打ちにするため
  720p は MJPG/YUYV とも尻切れ（fps低減は無効）。uvcvideo は
  quirks=128 nodrop=1 timeout=5000 必須。手順書トラブルシュートに全記載
- **homing 実機: 合格**（±3cm/±2°）。ゴール→原点のフル復帰も成功。
  改良: 公差±5°→±2°・W_ALIGN_MIN=0.15（静止摩擦対策）・auto復帰timeout45s
- **--auto スモーク: rep2 が truth_end 8.5cm / rmse 1.2cm で完走**（rep1 は
  開始向きズレで100cm=原因判明済み）。results/2026-07-17_Lturn_smoke_auto/
- **重要修正3件（すべて実機で発覚）**:
  ①UdpTwistSender の seq を時刻ベース初期化（sender再生成で全指令が無言破棄
  される事故を根絶。ブリッジ再起動運用が不要に） ②originオフセット20→30cm
  （原点旋回で tag0 を踏む） ③robot_tag yaw_offset=+4.1°を直進自己校正で確定
  （タグ貼り角。機体の見た目の向きズレの正体）
- **評価系の仕様変更**: truth_end_dist_cm / truth_rmse_cm は**実測開始poseに
  固定したコース基準**（homingの向き残差±2°が追従誤差に混入しない）。
  タグ系ゴールへの絶対距離は truth_end_dist_abs_cm に分離。テスト69本全green
- 残: クリーンなスモーク2本（results/2026-07-18_Lturn_smoke_auto2 予定）→
  フルバッチ。**RPi は nav_base＋ブリッジが起動したまま**（次回そのまま使える。
  止める場合: `ssh mukougawakouhei@192.168.0.32 "pkill -INT -f nav_base;
  pkill -INT -f udp_twist"` → シャットダウン）
- 課題: 「復帰だけ」の CLI が無い（run_batch を誤実行すると走行が始まる）→
  --home-only を追加予定

## 2026-07-17 セッション14（タグ自動サーベイ実装）

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
- **final review 完了（Ready）**: fable がセッション上限で落ちたためコントローラ
  （Opus）が直接実施。Important 1件を db4a3cc で修正＝サーベイ経路の解像度
  無照合（usbipd フォールバック 640x480 で K 誤適用→もっともらしい floor_tags を
  黙って出す穴。truth_live の _check_size と同じ検証を --image・ライブ両経路に追加）。
  yaml ラウンドトリップ（update_yaml→load_config・intキー・size_m保持）検証済み。
  Minor 棚卸しは全件許容。テスト68本全green
- 次: **実機側の残作業**:
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
  全kill→1つだけ起動で復旧）。
