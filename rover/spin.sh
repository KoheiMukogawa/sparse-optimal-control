#!/usr/bin/env bash
# その場旋回のラッパ。ROVER_D の較正・検証に使う。
#
#   使い方: bash ~/rover/spin.sh <deg> <omega> [timeout_s]
#   例:     bash ~/rover/spin.sh 360.0 0.4 30
#   ログ:   /tmp/spin_last.log
#
# spin_test.py は終了時に `RCLError: publisher's context is invalid` を出すが、
# このラッパが別途ゼロ速度を送るので実害はない。中断(Ctrl-C)でも必ずゼロで終わる。
#
# 2026-10-15: /tmp に置いていたため RPi の再起動で消えた。リポジトリ管理に移した。
set -u

DEG="${1:-360.0}"
OMEGA="${2:-0.4}"
TMO="${3:-30}"
LOG=/tmp/spin_last.log

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

zero() {
  timeout 5 ros2 topic pub --once /rover_twist geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
    >/dev/null 2>&1
}

trap 'echo "[中断] ゼロ速度を送る" | tee -a "$LOG"; zero; sleep 0.3; zero; exit 130' INT TERM

echo "=== $(date "+%F %T") spin deg=$DEG omega=$OMEGA timeout=$TMO ===" > "$LOG"

cd "$HOME/rover" || exit 1
timeout "$TMO" python3 spin_test.py --ros-args \
  -p target_deg:="$DEG" -p omega:="$OMEGA" 2>&1 | tee -a "$LOG"
RC=${PIPESTATUS[0]}

echo "[終了] rc=$RC → ゼロ速度を送る" | tee -a "$LOG"
zero
sleep 0.3
zero

echo "--- 停止確認: /odom の angular.z（0 付近なら停止）---" | tee -a "$LOG"
timeout 5 ros2 topic echo /odom --once --field twist.twist.angular.z 2>&1 \
  | tee -a "$LOG" || echo "(角速度の読み出しに失敗。車輪を目視で確認すること)" | tee -a "$LOG"

echo
echo "ログ: $LOG"
