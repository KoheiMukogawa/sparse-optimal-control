# sparse-optimal-control

農業用ロボットの経路追従制御におけるスパース最適制御（Maximum Hands-off Control）の
開発・シミュレーション評価を行う卒業研究リポジトリ。
MPC + L1正則化、評価指標はRMSE・消費エネルギー・入力スパース性・計算負荷。
実装スタック: cvxpy / CasADi + IPOPT, Python (uv管理)。

## 環境

**2台のPCで作業する（自作機とMacBook）。マシン固有の事情は下表にだけ書き、
他の場所に散らさない。** セットアップ手順は `docs/環境構築.md`。

| | 自作機（主にこちら） | MacBook |
|---|---|---|
| OS | WSL2 Ubuntu 22.04.5 / ROS2 Humble | macOS 27.2.1（ROS2 なし） |
| CPU/メモリ | Ryzen 5 5500・6C/12T, WSL に 15 GiB・swap 4 GiB、空き 944 GB（2026-11-11 実測。**メモリは潤沢**） | Apple M5・10コア（P4/E6）・16 GiB、空き 226 GB（2026-11-25 実測） |
| ホスト名 | `kohei-desktop` | `ollo-kohei` |
| リポジトリ位置 | `~/projects/sparse-optimal-control` | `~/projects/sparse-optimal-control`（同じ） |
| 固有の罠 | `C:\Users\<Windowsユーザ名>\.wslconfig` に `memory=3GB` が残る。今は効いていないが `wsl --shutdown` で 3 GB に絞られる | 日本語ファイル名の Unicode 正規化。`git config core.precomposeunicode true` を確認する（確認済み）。LaTeX（platex）は未インストール＝概要・卒論のPDFビルドは自作機で行う |

- **手元のPCに ROS2 は要らない**（2026-07-17 確認・`docs/handoff.md` の「laptop↔RPi の DDS は
  元々不要」）。走行制御は RPi 内で完結し、手元からは **SSH と素の UDP** しか使わない。
  `rclpy` を import するのは RPi 側で動くノードだけ（`mpc_follower.py` `udp_twist_bridge.py` 等）
- 実機: LiteRover（ヴィストン, RPi4, Ubuntu 22.04, ROS2 Humble）
- **接続先は `scripts/hosts.env` に書く**（`scripts/hosts.env.example` からコピー・**追跡しない**。
  大学側がグローバルIPで本体リポジトリが public のため）。`rover/exp_backends.py` は
  環境変数 `SSH_HOSTS` を直接読む
- RPi接続: `scripts/connect_rpi.sh`（家Wi-Fi→RPiホットスポットを順に試す）／
  大学経由は `scripts/connect_rpi.sh univ`。家Wi-Fi は DHCP で変動する（2026-07-15 に .31→.32）。
  恒久対策はルーターでMAC固定割当。繋がらない時はポート22走査で現IPを探す
- DDS設定 `scripts/cyclonedds.xml` は **RPi 側で ROS2 を立てるとき用**。手元では使わない

## エージェント（Claude Code / Codex のどちらでも使う）

**規約ファイルはこの `CLAUDE.md` 1本。** `AGENTS.md` は本ファイルへの
**シンボリックリンク**（`ln -s CLAUDE.md AGENTS.md`）で、Codex は `AGENTS.md`、
Claude Code は `CLAUDE.md` を読む。**同じ規約を2箇所に書かないこと**
（片方だけ更新されて食い違う事故を防ぐ。マシン固有の事情を上表にだけ書くのと同じ理由）。

- Codex は `cd ~/projects/sparse-optimal-control && codex` で起動する
- **`~/.codex/config.toml` に本リポジトリの `trust_level = "trusted"` を書かない。**
  承認プロンプトが出る状態を保つ（実機を動かすコマンドは毎回許可を得る運用。
  → 「運用ルール（恒久）」）
- Codex の既定サンドボックスは**ネットワーク遮断**。`uv sync` / `git push` / `gh` /
  実機への SSH は素通りしないので、承認して実行するか自分でターミナルから叩く
- どのエージェントでも、環境は `docs/環境構築.md`、現在地は `docs/handoff.md` から読む

## リポジトリ構成（2つある）

| | 置き場所 | 中身 |
|---|---|---|
| 本体 | `KoheiMukogawa/sparse-optimal-control`（**public**） | コード・configs・docs |
| 実験データ | `KoheiMukogawa/sparse-optimal-control-results`（**private**） | `results/` の中身。本体では `.gitignore` 対象 |

`results/` は**本体の作業ツリー内に clone した別リポジトリ**。未発表の実機データと
他者著作物を public に置けないための構成。**両方 pull / 両方 push が必要**。

```sh
scripts/sync.sh status          # 2つの差分と ahead/behind をまとめて見る
scripts/sync.sh pull            # 2つを pull
scripts/sync.sh push "メッセージ" # results の未コミットを commit し、2つを push
```

- **GitHub 認証は HTTPS + `gh`** に統一する（`gh auth login` → `gh auth setup-git`）。
  SSH 鍵を各PCで管理すると片方だけ push できない状態に陥る
  （2026-11-12 に実際に発生。ラップトップの鍵をコピーしただけで未登録だった）
- 同期しないもの: `data/paper/`（他者の論文PDF。書誌は `docs/thesis/references.md` に
  あるので各自入手）、`.venv/`（`uv sync --frozen` で作る）、`docs/abstract/` の
  学会配布書式（private 側 `results/_abstract_style/` に退避してある）

## ブランチ運用

- `main` 一本で運用（個人の卒研リポジトリ）。実験・実装も `main` に直接コミットし、
  区切りで `docs/handoff.md` を更新して push。大きく壊しうる変更時のみ一時ブランチを切る。

## 実験管理

- 実験条件はYAML（`configs/`以下）で管理し、コミットに含める
- 結果は `notebooks/` または `results/` 以下に保存

## セッション終了時

- `docs/handoff.md` を更新（現在の状況・次にやること・懸念事項）
- **`scripts/sync.sh push "…"` で2つのリポジトリを両方 push する。**
  片方だけ push すると、もう1台のPCで図やデータが古いまま作業することになる
  （2026-11-12 に発覚: results 側が約2ヶ月・本体側が15コミット未 push で両PCが分岐していた）
- 作業開始時は `scripts/sync.sh pull`（別のPCで進めた分を先に取り込む）

## コミットメッセージ

- 日本語で簡潔に（例: `MPCの雛形実装`, `L1正則化を追加`）

## 運用ルール（恒久）

- **実機を動かすコマンドは毎回ユーザーの許可を得る**（`run_batch --auto` は
  開始時の一括許可＋q+Enter停止で代替、必ず有人監視）
- nav_base は**1つだけ**起動（多重起動でモータ指令競合の事故歴あり）。
  起動時は `~/ros2_ws/install/setup.bash` の追いsource必須
- UDPブリッジは laptop 側を再起動したら RPi 側も再起動（seq巻き戻りで全破棄）
- **電源を抜く前に必ず `sudo poweroff`。** 不正な電源断で **Pi4 本体を2台失っている**
  （2026-07-20 初代/2代目、2026-10-15 3代目。いずれも起動不能＝基板交換）。
  SDカードのファイルシステム破損も同じ原因で起きている
- **RPiをベンチで長時間つけっぱなしにする時はバッテリーでなくAC給電にする**（上と同根）。
  バッチ実行前に充電残量を確認する。走行時のバッテリーは**容量と出力に余裕のあるもの**を
  使う（Pi4 は 5V/3A 要求。2026-10-15 に 210g 級のモバイルバッテリーで出力停止→基板死亡）
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
- odom補正は積算源（RPi側odometry, ROVER_D）に入れる。follower側では効かない。
  **`ROVER_D=0.1514` は初代・2代目の車体で実測した値であり、車体固有である**
  （2026-10-13、3代目で別車体に移行して判明）。**車体を替えたら `spin_test` で
  再較正すること。** 入力遅延・モータ特性も同様に車体依存
- 主要コード: `rover/mpc_core.py`(MPC本体) `rover/mpc_follower.py`(実機ノード)
  `rover/truth_*.py`(カメラ真値) `rover/homing.py`(自動原点復帰)
  `rover/run_batch.py`(バッチランナー) `rover/analyze_bag.py`(bag解析)

## 現在地

**最新の状況・次にやることは `docs/handoff.md` を参照**（過去セッション全文は
`docs/作業記録/handoff_archive.md`）。

**締切（2026-10-15 に判明）**: 研究会の発表と概要提出が **2026-10-22（木）**、
卒論は **2026-12-24 ごろ 研究室へ一度提出**。12/24 からの逆算と工数見積りは
`docs/handoff.md`（10-15 その2）を参照。

**2026-10-13 時点の要点**: 研究会（10月下旬）に向けたスプリント中。
計画は `docs/superpowers/specs/2026-10-13-中間発表スプリント-design.md`。

- 成果物は**発表スライドとは別に LaTeX で作る卒論概要（2段組み1〜2ページ）**。
  書式とサンプルは `docs/abstract/`（**public リポジトリのため追跡していない**）
- **実験環境を確定した**: 自宅・RPi4・**別車体**・方眼紙＋ペン軌跡・1m L字。
  カメラ真値は自宅なら可能だが採らない。
  **3代目RPi4は 2026-10-15 に電源断で死亡し、4代目の調達待ち**（SDカードは健全）
- **概要は既存の 2026-07-15 バッチ（旧車体）で閉じる**ので実機を待たない。
  新車体のデータ取得は卒論本体のための作業で、Phase 3・4 は研究会後に回す
- カメラ真値 Phase 2+3 は実装完了・実機E2Eは 2026-07-17 にほぼ完走。
  `configs/camera_truth.yaml` の floor_tags は 2026-07-20 の自動サーベイ値
  （再投影RMS 0.24px）。**ただし床タグが移動していれば `survey_tags.py` で再測量が要る**
