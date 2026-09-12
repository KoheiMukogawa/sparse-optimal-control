# -*- coding: utf-8 -*-
"""指令バッファ（R16 実機への人工遅延注入）。

S2 の中心的主張「move_suppress=2.0 は遅延マージンを買っている」を実機で
直接検証できる唯一の実験。sim の予測と突き合わせるため、遅延の意味は
exp_backends.sim_run と完全に一致していなければならない。
"""

from follower_core import CommandDelay


def test_zero_steps_is_identity():
    d = CommandDelay(0)
    assert d.push((0.1, 0.5), (0.1, 0.0)) == (0.1, 0.5)
    assert d.push((0.1, -0.5), (0.1, 0.0)) == (0.1, -0.5)


def test_two_steps_delays_by_two():
    d = CommandDelay(2)
    fill = (0.1, 0.0)
    assert d.push((0.0, 1.0), fill) == fill
    assert d.push((0.0, 2.0), fill) == fill
    assert d.push((0.0, 3.0), fill) == (0.0, 1.0)
    assert d.push((0.0, 4.0), fill) == (0.0, 2.0)


def test_sim_uses_command_delay_with_baseline_plus_injected_steps(monkeypatch):
    """simも実機と同じCommandDelayを直接使い、手写し実装を持たない。"""
    import exp_backends

    calls = {"steps": None, "pushes": []}

    class SpyDelay:
        def __init__(self, steps):
            calls["steps"] = steps

        def push(self, cmd, fill):
            calls["pushes"].append((cmd, fill))
            return fill

    monkeypatch.setattr(exp_backends, "CommandDelay", SpyDelay)
    exp_backends.sim_run(
        dict(name="k", controller="kanayama", cmd_delay_steps=3),
        dict(rate=10.0), [(0.0, 0.0), (1.0, 0.0)], 0.1,
        dict(delay_steps=2), timeout_s=0.11, seed=1)

    assert calls["steps"] == 5
    assert calls["pushes"]
    assert calls["pushes"][0][1] == (0.1, 0.0)


def test_reset_drops_pending_commands():
    """停止時に古い指令が後から出ないこと（安全要件）。"""
    d = CommandDelay(2)
    fill = (0.1, 0.0)
    d.push((0.0, 1.0), fill)
    d.push((0.0, 2.0), fill)
    d.reset()
    assert d.push((0.0, 9.0), fill) == fill


def test_negative_steps_is_treated_as_zero():
    d = CommandDelay(-3)
    assert d.push((0.1, 0.5), (0.1, 0.0)) == (0.1, 0.5)


def test_sim_adds_the_injected_delay_on_top_of_the_baseline():
    """同じ yaml を sim で回したとき、条件ごとの人工遅延が効くこと。

    効かないと R16（sim の予測 vs 実機）の突き合わせが成立しない。

    反転回数は隣接サンプルの符号跨ぎ（|w|>0.05 の閾値を1ステップで跨ぐ）では
    なく、閾値を跨いだ「側」の遷移回数で数える。move_suppress下ではw=0付近を
    複数ステップかけて通過するため、隣接1ステップ判定だと delay を足しても
    反転が検出できない（実際には delay=3 で 2→-2→2→-2→2 と明確に振動して
    いるにもかかわらず、隣接差分がどのステップも閾値を跨がずカウント0になる）。
    """
    from exp_backends import load_path, sim_run
    wps, vr = load_path("configs/path_L_turn_1m.yaml")
    common = dict(horizon=15, rate=10.0)
    base = dict(name="x", controller="l1", lam=0.3, move_suppress=0.5)

    def flips(extra):
        cond = dict(base, cmd_delay_steps=extra)
        d = sim_run(cond, common, wps, vr, dict(delay_steps=2), 60.0, seed=1)
        ws = [w for _, _, w in d["twist"]]
        side, count = None, 0
        for w in ws:
            if w > 0.05:
                s = 1
            elif w < -0.05:
                s = -1
            else:
                continue
            if side is not None and s != side:
                count += 1
            side = s
        return count

    assert flips(3) > flips(0), "人工遅延を足しても挙動が変わっていない"
