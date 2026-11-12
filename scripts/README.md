# scripts/ — 実機接続とローカル運用のためのスクリプト

以前は `~/scripts/` に置いていて**そのPCにしか存在しなかった**ものをここに移した。
PCを増やしても `git clone` だけで揃うようにするのが目的。

| ファイル | 内容 |
|---|---|
| `connect_rpi.sh` | RPi へ SSH。接続先を上から順に試す。`univ` 引数で大学ネットワーク経由 |
| `hosts.env.example` | 接続先のテンプレート。`hosts.env` にコピーして使う（実ファイルは**追跡しない**） |
| `cyclonedds.xml` | CycloneDDS 設定（マルチキャスト無効・autodetermine）。RPi 単体で ROS2 を回すとき用 |
| `obsidian_charts.py` | 研究記録（Obsidian）用のサマリーチャート生成 |

## 接続先の設定

```sh
cp scripts/hosts.env.example scripts/hosts.env
$EDITOR scripts/hosts.env      # 自分の環境の IP とユーザ名を入れる
scripts/connect_rpi.sh
```

`hosts.env` を追跡しないのは、大学ネットワーク側がグローバルIPで、
本体リポジトリが public なためである。**IP を .example に書き戻さないこと。**

`SSH_HOSTS` は `rover/exp_backends.py` が環境変数として直接読む。
別ネットワークでバッチを回すときは:

```sh
export $(grep -v '^#' scripts/hosts.env | xargs)
uv run python rover/run_batch.py ...
```

## DDS について

**laptop↔RPi の DDS 通信は不要**だと 2026-07-17 に確認済み（`docs/handoff.md`）。
走行中の制御は RPi 内で完結し、laptop からは SSH と素の UDP
（`rover/homing.py` → `rover/udp_twist_bridge.py`）しか使わない。
したがって **2台目のPCに ROS2 を入れる必要はない**。`cyclonedds.xml` は
RPi 側で ROS2 ノードを立てるときのための設定であり、laptop 側では使わない。
