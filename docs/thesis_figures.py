# -*- coding: utf-8 -*-
"""卒論の図の登録簿。**各章末の「本章の図表」表を唯一の出所として読む。**

以前は図番号 → パスの辞書を手で持っていたが、本文側の表と二重管理になり
実際に食い違った（2026-10-22 に判明）:

- 図2.1〜2.4 がスライド用の旧図（`docs/論文解説_figs/`）を指したまま。
  本文 2 章は「旧図はスライド用であり本文には用いない」と明記していた
- 図2.6・図5.9 が登録漏れで、**本文から参照されているのに黙って消えていた**
- 図6.7 が「実機待ちの未取得」のまま。sim 版（S4）が 9/10 に出て表は更新済み

そこで表をパースする方式に変えた。表の「内容」列はそのまま
**図のキャプション**として使う（LaTeX/HTML の両ビルダが参照する）。

表の書き方の約束:

- `| 図N.M | 内容 | 状態 |` の 3 列。状態にバックチック付きの画像パスを書く
- `.../name.png` や `fig5_6_*.png` のような略記はリポジトリ内を検索して解決する。
  **候補が 2 つ以上あるときは曖昧として採らない**（`PROBLEMS` に出る）ので、
  その場合は表にフルパスを書く
- `【未取得】` と書いた行は未取得として扱い、本文には枠だけ出る

`results/` は .gitignore 対象なので、表に載っていてもローカルに画像が無いことが
ある。その場合も未取得と同じ扱いにする（ビルドは通る）。
"""

import glob
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
CHAPTERS = [os.path.join(HERE, "thesis", f"chapter{i}.md") for i in range(1, 9)]

SECTION_RE = re.compile(r"^## 本章の図表\s*$(.*?)(?=^## |\Z)", re.M | re.S)
ROW_RE = re.compile(r"^\|\s*図(\d+\.\d+)\s*\|")
IMG_RE = re.compile(r"`([^`]*\.(?:png|jpg|jpeg))`", re.I)
NUL = "\x00"


def _cells(line):
    """表の 1 行をセルに割る。数式中の `|`（$\\Sigma|u|$ 等）を守る。"""
    store = []

    def keep(m):
        store.append(m.group(0))
        return f"{NUL}{len(store) - 1}{NUL}"

    line = re.sub(r"(?<!\$)\$[^$\n]+?\$", keep, line)
    cells = [c.strip() for c in line.strip().strip("|").split("|")]
    for i, s in enumerate(store):
        cells = [c.replace(f"{NUL}{i}{NUL}", s) for c in cells]
    return cells


def _resolve(spec):
    """状態セルの画像パスを実ファイルに解決する。

    返り値は (リポジトリ相対パス, 問題の説明)。どちらか一方が None。
    """
    if spec.startswith(".../") or "*" in spec:
        pat = os.path.join(ROOT, "**", os.path.basename(spec))
        hits = sorted(os.path.relpath(h, ROOT)
                      for h in glob.glob(pat, recursive=True)
                      if os.sep + ".git" + os.sep not in h)
        if len(hits) == 1:
            return hits[0], None
        if not hits:
            return None, f"略記 `{spec}` に当たる画像がローカルに無い"
        return None, (f"略記 `{spec}` の候補が {len(hits)} 件あり曖昧"
                      f"（{', '.join(hits)}）。表にフルパスを書くこと")
    if os.path.exists(os.path.join(ROOT, spec)):
        return spec, None
    return None, f"`{spec}` がローカルに無い（results/ は未追跡）"


def _parse():
    captions, figures, missing, problems = {}, {}, {}, []
    for path in CHAPTERS:
        if not os.path.exists(path):
            continue
        sec = SECTION_RE.search(open(path, encoding="utf-8").read())
        if not sec:
            continue
        for line in sec.group(1).split("\n"):
            s = line.strip()
            m = ROW_RE.match(s)
            if not m:
                continue
            num, cells = m.group(1), _cells(s)
            if num in captions:
                problems.append(f"図{num} の行が複数ある")
                continue
            captions[num] = cells[1] if len(cells) > 1 else ""
            state = cells[2] if len(cells) > 2 else ""
            if "未取得" in state:
                why = re.sub(r"\*\*【未取得】\*\*|【未取得】", "", state).strip()
                missing[num] = why or "未取得"
                continue
            img = IMG_RE.search(state)
            if not img:
                missing[num] = "表の状態欄に画像パスが無い"
                problems.append(f"図{num} の状態欄に画像パスが書かれていない")
                continue
            rel, why = _resolve(img.group(1))
            if rel:
                figures[num] = rel
            else:
                missing[num] = why
                if "曖昧" in why:
                    problems.append(f"図{num}: {why}")
    return captions, figures, missing, problems


CAPTIONS, FIGURES, MISSING, PROBLEMS = _parse()


def check_references(referenced):
    """本文から参照された図番号の集合を受け、登録簿との食い違いを返す。"""
    known = set(CAPTIONS)
    out = list(PROBLEMS)
    for num in sorted(referenced - known, key=lambda s: _key(s)):
        out.append(f"図{num} が本文から参照されているが「本章の図表」に行が無い")
    for num in sorted(known - referenced, key=lambda s: _key(s)):
        out.append(f"図{num} は登録されているが本文の段落から参照されていない"
                   "（本文に言及を書かないと figure が出ない）")
    return out


def _key(num):
    return [int(x) for x in num.split(".")]


if __name__ == "__main__":
    for n in sorted(CAPTIONS, key=_key):
        where = FIGURES.get(n) or f"未取得: {MISSING.get(n)}"
        print(f"図{n:5} {CAPTIONS[n][:34]:36} {where}")
    print(f"\n図 {len(CAPTIONS)} 件（画像あり {len(FIGURES)} / "
          f"未取得 {len(MISSING)}）")
    for p in PROBLEMS:
        print("  警告:", p)
