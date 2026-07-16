# -*- coding: utf-8 -*-
"""auto_batch のループ制御をfakeで検証（カメラ・SSH・ソケット不使用）。"""
import csv
import threading
from pathlib import Path

from run_batch import CSV_COLUMNS, auto_batch

BATCH = dict(name='t', path_file='configs/path_L_turn_1m.yaml', repeats=2,
             timeout_s=60, backend='real', common={}, sim={},
             conditions=[dict(name='l2', controller='l2')])
CFG = dict(floor_tags={0: (0.0, -0.2), 1: (0.4, 0.4), 2: (1.1, 1.1),
                       3: (1.2, -0.3)})


class FakeBackend:
    name = 'real'

    def __init__(self, oks):
        self.oks = list(oks)
        self.calls = 0

    def run_one(self, cond, rep, outdir):
        self.calls += 1
        return dict(ok=self.oks.pop(0), metrics=dict(rmse_cm=2.0),
                    bagdir='', note='')


class FakeTruth:
    def __init__(self):
        self.started = []

    def start(self, path):
        self.started.append(str(path))

    def stop(self):
        # 本物の TruthLive.stop() 同様にCSVを書く（outdir未作成なら落ちる）
        Path(self.started[-1]).write_text('t_s\n')
        # ゴール到達済みの軌跡もどき（最後の0.5sは(1.0,1.0)付近）
        return [(0.1 * i, 1.0, 1.0, 1.57, 5, 300.0) for i in range(20)]

    def pose(self, max_age_s=1.0):
        return (1.0, 1.0, 1.57)


def _run(oks, homing_results, inputs=('',), stop=None, done=(), only=None,
         premake=False):
    tmp = Path('/tmp/claude-autobatch-test')
    import shutil
    shutil.rmtree(tmp, ignore_errors=True)
    if premake:
        tmp.mkdir(parents=True)
    backend = FakeBackend(oks)
    truth = FakeTruth()
    calls = []
    hr = list(homing_results)

    def homing_fn(pose_fn, sender, floor_tags, stop_event=None):
        calls.append(1)
        return hr.pop(0) if hr else dict(ok=True, reason='done')

    it = iter(inputs)
    auto_batch(BATCH, backend, truth, sender=None, cfg=CFG, outdir=tmp,
               csv_path=tmp / 'runs.csv', ghash='x', v_r=0.1,
               stop_event=stop or threading.Event(),
               homing_fn=homing_fn, input_fn=lambda p: next(it),
               done=done, only=only)
    csv_file = tmp / 'runs.csv'
    rows = list(csv.DictReader(open(csv_file))) if csv_file.exists() else []
    return backend, truth, calls, rows


def test_auto_batch_records_truth_and_homes_between_runs():
    backend, truth, calls, rows = _run(oks=[True, True],
                                       homing_results=[dict(ok=True,
                                                            reason='done')])
    assert backend.calls == 2 and len(rows) == 2
    assert len(calls) == 1                 # 復帰は「次がある」時だけ=1回
    assert truth.started == [str(Path('/tmp/claude-autobatch-test')
                                 / 'truth_l2_r1.csv'),
                             str(Path('/tmp/claude-autobatch-test')
                                 / 'truth_l2_r2.csv')]
    assert abs(float(rows[0]['truth_end_dist_cm'])) < 1e-6
    assert 'truth_end_x' in CSV_COLUMNS


def test_auto_batch_aborts_after_two_consecutive_failures():
    backend, _, _, rows = _run(oks=[False, False], homing_results=[])
    assert backend.calls == 2 and len(rows) == 2   # 3本目には進まない


def test_auto_batch_homing_retry_then_ask_human():
    backend, _, calls, _ = _run(
        oks=[True, True],
        homing_results=[dict(ok=False, reason='fail_timeout'),
                        dict(ok=False, reason='fail_timeout')],
        inputs=[''])
    assert len(calls) == 2                 # リトライ1回まで（その後は人へ）
    assert backend.calls == 2              # Enterで続行して2本目も走る


def test_auto_batch_q_stops_loop():
    stop = threading.Event()
    stop.set()
    backend, _, _, rows = _run(oks=[True, True], homing_results=[],
                               stop=stop)
    assert backend.calls == 0 and rows == []


def test_auto_batch_creates_outdir_and_survives_first_run_error():
    """初回走行がssh例外等で失敗しても outdir 未作成で落ちず ok=False で続行。"""
    class BoomBackend(FakeBackend):
        def run_one(self, cond, rep, outdir):
            self.calls += 1
            if self.calls == 1:
                raise RuntimeError('ssh timeout')
            return dict(ok=True, metrics={}, bagdir='', note='')

    tmp = Path('/tmp/claude-autobatch-test')
    import shutil
    shutil.rmtree(tmp, ignore_errors=True)   # 事前mkdirしない
    backend = BoomBackend([])
    auto_batch(BATCH, backend, FakeTruth(), sender=None, cfg=CFG,
               outdir=tmp, csv_path=tmp / 'runs.csv', ghash='x', v_r=0.1,
               stop_event=threading.Event(),
               homing_fn=lambda *a, **k: dict(ok=True, reason='done'),
               input_fn=lambda p: '')
    rows = list(csv.DictReader(open(tmp / 'runs.csv')))
    assert backend.calls == 2 and len(rows) == 2
    assert rows[0]['ok'] == 'False' and 'ssh timeout' in rows[0]['note']
    assert rows[1]['ok'] == 'True'


def test_auto_batch_respects_resume_done_and_only():
    backend, _, _, rows = _run(oks=[True], homing_results=[],
                               done={('l2', 1)})
    assert backend.calls == 1 and rows[0]['rep'] == '2'   # rep1はskip
    backend, _, _, rows = _run(oks=[], homing_results=[], only='kanayama')
    assert backend.calls == 0 and rows == []              # 条件名不一致は走らない
