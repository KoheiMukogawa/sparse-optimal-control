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
