# -*- coding: utf-8 -*-
"""udp_twist_bridge の純ロジック（seq破棄・watchdog・クランプ）。"""
import json

from udp_twist_bridge import BridgeCore


def _msg(v, w, seq):
    return json.dumps(dict(v=v, w=w, seq=seq)).encode()


def test_accept_clamp_and_seq_order():
    core = BridgeCore()
    assert core.accept(_msg(0.08, -0.3, 1), now=0.0) == (0.08, -0.3)
    # クランプ（V_MAX=0.15, W_MAX=2.0）
    assert core.accept(_msg(9.0, -9.0, 2), now=0.1) == (0.15, -2.0)
    # 古い/同一 seq は破棄
    assert core.accept(_msg(0.05, 0.0, 2), now=0.2) is None
    assert core.accept(_msg(0.05, 0.0, 1), now=0.2) is None
    assert core.accept(_msg(0.05, 0.0, 3), now=0.2) == (0.05, 0.0)


def test_malformed_payload_discarded():
    core = BridgeCore()
    assert core.accept(b'not json', 0.0) is None
    assert core.accept(json.dumps(dict(v=0.1)).encode(), 0.0) is None
    assert core.accept(b'\xff\xfe', 0.0) is None


def test_watchdog_fires_once_after_silence():
    core = BridgeCore(watchdog_s=0.5)
    assert not core.watchdog_zero(0.0)          # 指令前は発火しない
    core.accept(_msg(0.1, 0.0, 1), now=1.0)
    assert not core.watchdog_zero(1.4)          # 0.4s: まだ
    assert core.watchdog_zero(1.6)              # 0.6s: 発火
    assert not core.watchdog_zero(1.7)          # 1回だけ（連打しない）
    core.accept(_msg(0.1, 0.0, 2), now=2.0)     # 再開すれば再武装
    assert core.watchdog_zero(2.6)
