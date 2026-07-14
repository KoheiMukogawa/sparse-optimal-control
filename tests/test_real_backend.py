# -*- coding: utf-8 -*-
"""RealBackend のテスト。コマンド生成の正確性を検証。"""

import subprocess
import sys
import time

from exp_backends import ROS_SETUP, bag_record_command, node_command, watch_node

COMMON = dict(horizon=15, rate=10.0)
PATH = "configs/path_L_turn.yaml"


def test_node_command_kanayama():
    cmd = node_command(dict(name="k", controller="kanayama"), COMMON, PATH)
    assert "path_follower.py" in cmd
    assert "path_L_turn.yaml" in cmd
    assert "reg:=" not in cmd
    assert cmd.startswith(f"{ROS_SETUP} && ")  # 非対話shellでもROS環境を確保


def test_node_command_l1_ms():
    cmd = node_command(
        dict(name="l1_ms2", controller="l1", lam=0.3, move_suppress=2.0),
        COMMON, PATH)
    assert "mpc_follower.py" in cmd
    assert "-p reg:=l1" in cmd
    assert "-p lam:=0.3" in cmd
    assert "-p move_suppress:=2.0" in cmd
    assert "-p horizon:=15" in cmd
    assert cmd.startswith(f"{ROS_SETUP} && ")


def test_bag_record_command():
    cmd = bag_record_command("/tmp/batch_l2_r1")
    assert cmd.startswith(f"{ROS_SETUP} && ")
    assert "ros2 bag record -o /tmp/batch_l2_r1" in cmd
    for topic in ("/odom", "/rover_twist", "/path_error", "/mpc_solve_ms"):
        assert topic in cmd


def test_watch_node_detects_goal_in_coalesced_chunk():
    """1回のflushで複数行が届いても select 版のようにブロックせず検知できること
    （select はraw fdを監視するため、readline済みバッファ内の行を見落としうる旧バグの回帰）。"""
    proc = subprocess.Popen(
        [sys.executable, "-u", "-c",
         "import sys; print('MPC準備完了\\n経路原点設定', flush=True); "
         "import time; time.sleep(0.3); print('目標到達 しました', flush=True)"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        assert watch_node(proc, time.time() + 10) is True
    finally:
        proc.wait(timeout=5)


def test_watch_node_timeout_returns_false():
    proc = subprocess.Popen(
        [sys.executable, "-u", "-c",
         "import time; time.sleep(5)"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        assert watch_node(proc, time.time() + 1.0) is False
    finally:
        proc.kill()
        proc.wait(timeout=5)
