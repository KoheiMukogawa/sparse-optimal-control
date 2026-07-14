# -*- coding: utf-8 -*-
"""RealBackend のテスト。コマンド生成の正確性を検証。"""

from exp_backends import bag_record_command, node_command

COMMON = dict(horizon=15, rate=10.0)
PATH = "configs/path_L_turn.yaml"


def test_node_command_kanayama():
    cmd = node_command(dict(name="k", controller="kanayama"), COMMON, PATH)
    assert "path_follower.py" in cmd
    assert "path_L_turn.yaml" in cmd
    assert "reg:=" not in cmd


def test_node_command_l1_ms():
    cmd = node_command(
        dict(name="l1_ms2", controller="l1", lam=0.3, move_suppress=2.0),
        COMMON, PATH)
    assert "mpc_follower.py" in cmd
    assert "-p reg:=l1" in cmd
    assert "-p lam:=0.3" in cmd
    assert "-p move_suppress:=2.0" in cmd
    assert "-p horizon:=15" in cmd


def test_bag_record_command():
    cmd = bag_record_command("/tmp/batch_l2_r1")
    assert cmd.startswith("ros2 bag record -o /tmp/batch_l2_r1")
    for topic in ("/odom", "/rover_twist", "/path_error", "/mpc_solve_ms"):
        assert topic in cmd
