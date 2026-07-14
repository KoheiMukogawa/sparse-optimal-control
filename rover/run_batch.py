# -*- coding: utf-8 -*-
"""バッチ実験ランナー CLI（設計: specs/2026-07-14-batch-runner-design.md）。

configs/batch_*.yaml の条件×反復を実行し、results/<日付>_<名前>/ に
runs.csv（1走行1行・逐次追記）と summary.md（条件別 平均±標準偏差）を出力。

使い方:
  uv run python rover/run_batch.py configs/batch_Lturn_3way.yaml
      [--backend sim|real] [--resume] [--only 条件名] [--dry-run]
      [--summarize] [--outdir DIR]
"""

import argparse
import csv
import datetime
import statistics
import subprocess
from pathlib import Path

import yaml

CSV_COLUMNS = [
    'batch', 'cond', 'rep', 'backend', 'timestamp', 'git_hash',
    'controller', 'lam', 'move_suppress', 'horizon', 'v_r', 'ok',
    'drive_s', 'rmse_cm', 'sum_u', 'w_zero_ratio', 'flips', 'sat_ratio',
    'max_w', 'solve_p50', 'solve_p95', 'solve_max', 'bagdir', 'note',
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


def git_hash_short():
    try:
        return subprocess.run(['git', 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True,
                              check=True).stdout.strip()
    except Exception:
        return 'unknown'


def _mean_std(vals):
    import math
    # Filter out NaN values
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
    args = ap.parse_args()

    from exp_backends import SimBackend, load_path
    batch = load_batch(args.batch_yaml)
    backend_kind = args.backend or batch['backend']
    outdir = Path(args.outdir or
                  f"results/{datetime.date.today()}_{batch['name']}")
    csv_path = outdir / 'runs.csv'

    if args.summarize:
        write_summary(outdir)
        print(f'summary.md を再生成: {outdir}')
        return

    if backend_kind == 'sim':
        backend = SimBackend(batch)
    else:
        from exp_backends import RealBackend
        backend = RealBackend(batch, dry_run=args.dry_run)
        if not args.dry_run:
            ans = input(f"実機バッチ {batch['name']} を開始します。"
                        f"nav_base 起動済み・走行エリア確保を確認 [y/N]: ")
            if ans.strip().lower() != 'y':
                print('中止しました')
                return
        backend.preflight()

    _, v_r = load_path(batch['path_file'])
    ghash = git_hash_short()
    done = done_keys(csv_path) if args.resume else set()

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
