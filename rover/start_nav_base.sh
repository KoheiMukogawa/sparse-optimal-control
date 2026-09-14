#!/usr/bin/env bash
# LiDAR非搭載機向けの nav_base 起動（RPi上で実行）。
#
# 純正の nav_base.launch.py は ydlidar_x2_launch.py を無条件に include するため、
# LiDAR非搭載の機体では必ずエラーになる。追従制御は odom だけで完結しており
# （mpc_follower.py は LaserScan を一切購読しない）、実験に LiDAR は不要なので
# 必要な3ノードだけを起動する。
#
#   mpc_follower ──/rover_twist──> pos_controller ──/wrc201_i2c──> i2c_controller ──> WRC201
#   odom_manager ──/odom────────> mpc_follower, pos_controller
#
# 多重起動は過去にモータ指令の競合事故を起こしているため、既存プロセスを
# 検出したら起動しない。Ctrl-C で零速度を配信してから全ノードを停止する。
#
# 使い方:  bash ~/rover/start_nav_base.sh
set -uo pipefail

NODE_DIR="$HOME/ros2_ws/install/lightrover_ros/lib/lightrover_ros"
NODES=(i2c_controller odom_manager pos_controller)

# --- 多重起動ガード ----------------------------------------------------------
# 自分自身にマッチしないよう、インストール先の実体パスだけを見る（このスクリプトの
# コマンドラインには NODE_DIR が現れない）。念のため自PIDは明示的に除外する。
EXISTING=$(pgrep -f "$NODE_DIR/" | grep -vx "$$" || true)
if [ -n "$EXISTING" ]; then
  echo "[中止] nav_base のノードが既に動いている。多重起動はモータ指令が競合する。" >&2
  ps -o pid,args -p $EXISTING >&2
  echo "止めてからやり直すこと: kill $(echo "$EXISTING" | tr '\n' ' ')" >&2
  exit 1
fi

# --- 環境 --------------------------------------------------------------------
# ROS の setup.bash は未定義変数（AMENT_TRACE_SETUP_FILES 等）を参照するため、
# source の間だけ set -u を外す。
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# ros2_ws の追いsourceは必須（これが無いと lightrover_ros が見つからない）
# shellcheck disable=SC1091
source "$HOME/ros2_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="file://$HOME/cyclonedds_rpi.xml"

LOG_DIR="${LOG_DIR:-/tmp/nav_base}"
mkdir -p "$LOG_DIR"
PIDS=()

cleanup() {
  trap - INT TERM EXIT
  echo
  # 先に零速度を送る。ノードを先に殺すと基板が最後の指令を保持したままになりうる。
  if kill -0 "${PIDS[2]:-0}" 2>/dev/null; then
    echo "[停止] 零速度を配信..."
    timeout 5 ros2 topic pub --once /rover_twist geometry_msgs/msg/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
      >/dev/null 2>&1 || true
    sleep 1
    # 出力イネーブル(MU8_O_EN=0x10)を落として起動前と同じ状態に戻す。
    # pos_controller は起動時に 0x03 を書いて通電させるが、落とす処理は持たない。
    # i2c_controller はこの直後に kill するので、サービス呼び出しはここで行う。
    echo "[停止] 出力イネーブルを解除..."
    timeout 5 ros2 service call /wrc201_i2c lightrover_interface/srv/Wrc201Msg \
      '{addr: 16, data: 0, length: 1, cmd: w}' >/dev/null 2>&1 || true
    sleep 1
  fi
  echo "[停止] ノードを終了..."
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null || true; done
  echo "[停止] 完了。ログ: $LOG_DIR"
}
trap cleanup INT TERM EXIT

# --- 起動 --------------------------------------------------------------------
# i2c_controller を先に上げる。pos_controller はこれのサービスを待つ。
#
# `ros2 run` を使わずノード本体を直接起動する。ros2 run はラッパープロセスを挟むため、
# $! で得られるPIDはラッパーのものになり、それを kill してもノード本体が生き残る
# （実際に残骸が出た）。直接起動すればPIDが実体と一致し、確実に停止できる。
for node in "${NODES[@]}"; do
  python3 "$NODE_DIR/$node" > "$LOG_DIR/$node.log" 2>&1 &
  PIDS+=("$!")
  echo "[起動] $node (PID=$!)"
  sleep 3
done

# --- 起動確認 ----------------------------------------------------------------
sleep 2
FAILED=0
for i in "${!NODES[@]}"; do
  if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
    echo "[異常] ${NODES[$i]} が落ちた。ログ: $LOG_DIR/${NODES[$i]}.log" >&2
    tail -5 "$LOG_DIR/${NODES[$i]}.log" >&2
    FAILED=1
  fi
done
[ "$FAILED" -ne 0 ] && exit 1

# --- 起動直後の安全化 --------------------------------------------------------
# pos_controller は main() で出力イネーブル(MU8_O_EN=0x03)を書いてから待機に入るが、
# 速度レジスタ(MS32_A_POS0/1)は書かない。基板は電源が入っている間ずっと前回値を
# 保持する（odom が起動時にゼロリセットされないのと同じ）。前回値が非ゼロだと
# イネーブルONの瞬間に走り出すため、零速度を明示的に送って潰しておく。
echo "[安全] 零速度を配信して速度レジスタをクリア..."
timeout 5 ros2 topic pub --once /rover_twist geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
  >/dev/null 2>&1 || echo "[警告] 零速度の配信に失敗した。車輪を浮かせて確認すること。" >&2

echo
echo "nav_base 起動完了（LiDARなし構成）。Ctrl-C で停止する。"
echo "  odom 確認: ros2 topic hz /odom"
echo "  ログ:      $LOG_DIR/"
echo
echo "注意: odom は起動時にゼロリセットされない（基板の積算値を読む）。"
echo "      走行前に homing.py で原点復帰すること。"

# 前面で待つ。Ctrl-C が cleanup を呼ぶ。
wait
