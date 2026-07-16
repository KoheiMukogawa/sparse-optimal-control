# -*- coding: utf-8 -*-
"""バッチ実験ランナー CLI（設計: specs/2026-07-14-batch-runner-design.md）。

configs/batch_*.yaml の条件×反復を実行し、results/<日付>_<名前>_<backend>/ に
runs.csv（1走行1行・逐次追記）と summary.md（条件別 平均±標準偏差）を出力。
（backend は sim/real。--outdir 明示時はそのまま使う。同日sim/real混在での
runs.csv汚染を防ぐため既定値にbackend種別を含める）

使い方:
  uv run python rover/run_batch.py configs/batch_Lturn_3way.yaml
      [--backend sim|real] [--resume] [--only 条件名] [--dry-run]
      [--summarize] [--outdir DIR]
"""

import argparse
import csv
import datetime
import math
import queue
import statistics
import subprocess
import sys
import threading
from pathlib import Path

import yaml

CSV_COLUMNS = [
    'batch', 'cond', 'rep', 'backend', 'timestamp', 'git_hash',
    'controller', 'lam', 'move_suppress', 'horizon', 'v_r', 'ok',
    'drive_s', 'rmse_cm', 'sum_u', 'w_zero_ratio', 'flips', 'sat_ratio',
    'max_w', 'solve_p50', 'solve_p95', 'solve_max',
    'truth_end_x', 'truth_end_y', 'truth_end_theta',
    'truth_end_dist_cm', 'truth_rmse_cm',
    'bagdir', 'note',
]

CONTROLLERS = ('kanayama', 'l2', 'l1')


def load_batch(yaml_path):
    """バッチ設定を読み、既定値を補完して検証する。"""
    with open(yaml_path) as f:
        b = yaml.safe_load(f)
    for key in ('name', 'path_file', 'conditions'):
        if key not in b:
            raise ValueError(f'batch yaml に {key} がありません')
    b.setdefault('repeats', 1)
    b.setdefault('timeout_s', 60)
    b.setdefault('backend', 'sim')
    b.setdefault('common', {})
    b.setdefault('sim', {})
    names = set()
    for c in b['conditions']:
        if c.get('controller') not in CONTROLLERS:
            raise ValueError(f"不正な controller: {c.get('controller')}")
        if 'name' not in c or c['name'] in names:
            raise ValueError(f'条件 name が無いか重複: {c}')
        names.add(c['name'])
    return b


def append_row(csv_path, row):
    """runs.csv に1行追記（無ければヘッダ付きで作成）。"""
    p = Path(csv_path)
    new = not p.exists()
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        if new:
            w.writeheader()
        w.writerow(row)


def done_keys(csv_path):
    """ok=true 済みの (cond, rep) 集合（--resume 用）。"""
    p = Path(csv_path)
    if not p.exists():
        return set()
    with open(p, newline='') as f:
        return {(r['cond'], int(r['rep']))
                for r in csv.DictReader(f)
                if str(r['ok']).lower() == 'true'}


def make_row(batch, cond, rep, backend_name, result, git_hash, v_r):
    m = result.get('metrics', {})
    row = {c: '' for c in CSV_COLUMNS}
    row.update(
        batch=batch['name'], cond=cond['name'], rep=rep,
        backend=backend_name,
        timestamp=datetime.datetime.now().isoformat(timespec='seconds'),
        git_hash=git_hash,
        controller=cond['controller'],
        lam=cond.get('lam', ''),
        move_suppress=cond.get('move_suppress', ''),
        horizon=batch.get('common', {}).get('horizon', ''),
        v_r=v_r, ok=result['ok'],
        bagdir=result.get('bagdir', ''), note=result.get('note', ''),
    )
    for k in ('drive_s', 'rmse_cm', 'sum_u', 'w_zero_ratio', 'flips',
              'sat_ratio', 'max_w', 'solve_p50', 'solve_p95', 'solve_max'):
        if k in m:
            row[k] = m[k]
    return row


def _start_stdin_listener(stop_event, line_q):
    """qでstop_event、その他の行はline_qへ（auto中のEnter待ちに使う）。"""
    def _listen():
        for line in sys.stdin:
            s = line.strip().lower()
            if s == 'q':
                stop_event.set()
                print('q受信: 停止します（走行中なら現走行の停止後）')
                break
            line_q.put(s)
    threading.Thread(target=_listen, daemon=True).start()


def _prompt(prompt, line_q, stop_event):
    print(prompt, end='', flush=True)
    while not stop_event.is_set():
        try:
            return line_q.get(timeout=0.2)
        except queue.Empty:
            continue
    return 'q'


def auto_batch(batch, backend, truth, sender, cfg, outdir, csv_path,
               ghash, v_r, stop_event, homing_fn=None, input_fn=input):
    """--auto のループ本体: 走行→真値stop→CSV→原点復帰→次へ。

    homing_fn / input_fn はテストで差し替える。復帰は「次の走行がある」
    ときだけ行う。復帰失敗はリトライ1回→それでも駄目なら人にEnterを求める。
    連続2本の走行失敗でループ停止（spec）。
    """
    from exp_backends import load_path
    from exp_metrics import truth_metrics
    if homing_fn is None:
        from homing import home as homing_fn
    waypoints, _ = load_path(batch['path_file'])
    runs = [(c, r) for c in batch['conditions']
            for r in range(1, int(batch['repeats']) + 1)]
    fails = 0
    for i, (cond, rep) in enumerate(runs):
        if stop_event.is_set():
            print('停止要求によりループ終了')
            break
        truth.start(Path(outdir) / f"truth_{cond['name']}_r{rep}.csv")
        try:
            result = backend.run_one(cond, rep, outdir)
        except Exception as e:
            result = dict(ok=False, metrics={}, bagdir='', note=f'error: {e}')
        rows = truth.stop()
        tm = truth_metrics(rows, waypoints)
        row = make_row(batch, cond, rep, backend.name, result, ghash, v_r)
        row.update({k: f'{v:.4f}' for k, v in tm.items()})
        append_row(csv_path, row)
        print(f"{cond['name']} rep{rep}: ok={result['ok']} "
              f"truth_end={tm.get('truth_end_dist_cm', float('nan')):.1f}cm")
        fails = 0 if result['ok'] else fails + 1
        if fails >= 2:
            print('連続2本失敗: ループを停止します（状態を確認して再開）')
            break
        if i == len(runs) - 1 or stop_event.is_set():
            break
        hres = homing_fn(lambda: truth.pose(1.0), sender,
                         cfg['floor_tags'], stop_event=stop_event)
        if not hres['ok'] and not stop_event.is_set():
            print(f"復帰失敗({hres['reason']}) → リトライ")
            hres = homing_fn(lambda: truth.pose(1.0), sender,
                             cfg['floor_tags'], stop_event=stop_event)
        if not hres['ok'] and not stop_event.is_set():
            ans = input_fn(f"復帰失敗({hres['reason']})。手で原点に戻して "
                           'Enter（qで中断）: ')
            if ans.strip().lower() == 'q':
                break


def git_hash_short():
    try:
        return subprocess.run(['git', 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True,
                              check=True).stdout.strip()
    except Exception:
        return 'unknown'


def _mean_std(vals):
    # NaN除外（kanayamaは求解しないため solve_* が nan になる）
    valid = [v for v in vals if not math.isnan(v)]
    if not valid:
        return 'n/a'
    m = statistics.mean(valid)
    s = statistics.stdev(valid) if len(valid) > 1 else 0.0
    return f'{m:.2f}±{s:.2f}'


def write_summary(outdir):
    """runs.csv → summary.md（条件ごとの 到達率・平均±標準偏差）。"""
    outdir = Path(outdir)
    with open(outdir / 'runs.csv', newline='') as f:
        rows = list(csv.DictReader(f))
    conds = []
    for r in rows:
        if r['cond'] not in conds:
            conds.append(r['cond'])
    lines = [
        f"# バッチ結果まとめ: {rows[0]['batch']}（{rows[0]['backend']}）",
        '',
        f"生成: {datetime.datetime.now().isoformat(timespec='seconds')} / "
        f"git {rows[0]['git_hash']} / 全{len(rows)}走行",
        '',
        '| 条件 | 到達 | RMSE_cm | Σ\\|u\\| | 反転 | ω0率 | 解p95ms |',
        '|------|------|---------|--------|------|------|---------|',
    ]
    for c in conds:
        rs = [r for r in rows if r['cond'] == c]
        oks = [r for r in rs if str(r['ok']).lower() == 'true']

        def col(key, rs=oks):
            return _mean_std([float(r[key]) for r in rs if r[key] != ''])

        lines.append(
            f"| {c} | {len(oks)}/{len(rs)} | {col('rmse_cm')} | "
            f"{col('sum_u')} | {col('flips')} | {col('w_zero_ratio')} | "
            f"{col('solve_p95')} |")
    lines += ['', '注: RMSEはodom基準（真値は外部計測）。ok=false の行は'
              '到達数のみ反映し平均から除外。']
    (outdir / 'summary.md').write_text('\n'.join(lines) + '\n')


def main():
    ap = argparse.ArgumentParser(description='バッチ実験ランナー')
    ap.add_argument('batch_yaml')
    ap.add_argument('--backend', choices=['sim', 'real'])
    ap.add_argument('--resume', action='store_true')
    ap.add_argument('--only', help='この条件名だけ実行')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--summarize', action='store_true',
                    help='走行せず summary.md のみ再生成')
    ap.add_argument('--outdir', help='出力先（既定 results/<日付>_<名前>）')
    ap.add_argument('--auto', action='store_true',
                    help='実機のみ: 真値記録＋自動原点復帰の全自動ループ')
    ap.add_argument('--camera-config', default='configs/camera_truth.yaml')
    args = ap.parse_args()

    from exp_backends import SimBackend, load_path
    batch = load_batch(args.batch_yaml)
    backend_kind = args.backend or batch['backend']
    outdir = Path(args.outdir or
                  f"results/{datetime.date.today()}_{batch['name']}"
                  f"_{backend_kind}")
    csv_path = outdir / 'runs.csv'

    if args.summarize:
        write_summary(outdir)
        print(f'summary.md を再生成: {outdir}')
        return

    if args.auto and backend_kind != 'real':
        raise SystemExit('--auto は --backend real 専用です')
    if backend_kind == 'sim':
        backend = SimBackend(batch)
    else:
        from exp_backends import RealBackend
        stop_event = threading.Event()
        backend = RealBackend(batch, dry_run=args.dry_run,
                              auto=args.auto, stop_event=stop_event)
        if not args.dry_run:
            n_runs = len(batch['conditions']) * int(batch['repeats'])
            extra = (f'\n  全{n_runs}本を自動実行（走行→真値→原点復帰）。'
                     '\n  q+Enter でいつでも停止・有人監視を続けること。'
                     if args.auto else '')
            ans = input(f"実機バッチ {batch['name']} を開始します。"
                        f"nav_base 起動済み・走行エリア確保を確認{extra} [y/N]: ")
            if ans.strip().lower() != 'y':
                print('中止しました')
                return
        backend.preflight()

    _, v_r = load_path(batch['path_file'])
    ghash = git_hash_short()
    done = done_keys(csv_path) if args.resume else set()

    if args.auto:
        from exp_backends import RPI_BRIDGE
        from homing import UdpTwistSender
        from truth_live import CameraSource, TruthLive
        from truth_offline import load_config
        cfg = load_config(args.camera_config)
        live = cfg.get('live', {})
        truth = TruthLive(cfg, CameraSource(live.get('device', 0),
                                            live.get('width', 1280),
                                            live.get('height', 720)))
        # calibrate失敗やブリッジ未起動でもカメラを確実に解放する
        # （usbipd越しのWSL2カメラは掴んだままになりやすい）
        sender = None
        try:
            print('カメラ姿勢をキャリブレーション中（床タグ4枚が写ること）...')
            truth.calibrate()
            bridge_ok = backend._ssh(
                "pgrep -f '[u]dp_twist_bridge'").stdout.strip()
            if not bridge_ok:
                raise SystemExit(
                    'RPiで udp_twist_bridge が起動していません:\n'
                    '  source /opt/ros/humble/setup.bash && '
                    f'python3 {RPI_BRIDGE} --port '
                    f"{cfg.get('udp', {}).get('port', 8890)}")
            sender = UdpTwistSender(backend.host,
                                    cfg.get('udp', {}).get('port', 8890))
            line_q = queue.Queue()
            _start_stdin_listener(stop_event, line_q)
            auto_batch(batch, backend, truth, sender, cfg, outdir,
                       csv_path, ghash, v_r, stop_event,
                       input_fn=lambda p: _prompt(p, line_q, stop_event))
        finally:
            if sender is not None:
                sender.close()
            truth.close()
        if csv_path.exists():
            write_summary(outdir)
            print(f'完了: {outdir}/runs.csv, summary.md')
        return

    for cond in batch['conditions']:
        if args.only and cond['name'] != args.only:
            continue
        for rep in range(1, int(batch['repeats']) + 1):
            if (cond['name'], rep) in done:
                print(f"skip(済): {cond['name']} rep{rep}")
                continue
            try:
                result = backend.run_one(cond, rep, outdir)
            except KeyboardInterrupt:
                print('\nバッチ中断')
                return
            except Exception as e:  # 1本の失敗はバッチを止めない（設計方針）
                result = dict(ok=False, metrics={}, bagdir='',
                              note=f'error: {e}')
            row = make_row(batch, cond, rep, backend_kind, result, ghash, v_r)
            append_row(csv_path, row)
            m = result.get('metrics', {})
            print(f"{cond['name']} rep{rep}: ok={result['ok']} "
                  f"rmse={m.get('rmse_cm', float('nan')):.2f}cm "
                  f"Σ|u|={m.get('sum_u', float('nan')):.2f} "
                  f"反転={m.get('flips', '-')}")
    if csv_path.exists():
        write_summary(outdir)
        print(f'完了: {outdir}/runs.csv, summary.md')


if __name__ == '__main__':
    main()
