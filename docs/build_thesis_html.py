# -*- coding: utf-8 -*-
"""卒論の全章 Markdown → 通し読み用の1枚 HTML。

VS Code のプレビューでは図が出ず数式も崩れるので、
**図を本文の該当箇所に差し込み、数式を KaTeX で組んだ HTML** を作る。

- 出力を Chrome/Edge で開き、`Ctrl+P` →「PDF に保存」で製本イメージも得られる
  （印刷用 CSS を入れてあり、章ごとに改ページする）
- 数式は KaTeX を CDN から読むため、**閲覧にはネット接続が必要**
  （`--offline` を付けると数式は生の TeX のまま出力する）

使い方:
    uv run python docs/build_thesis_html.py
    uv run python docs/build_thesis_html.py --open   # 生成後にブラウザで開く

出力: docs/thesis/thesis.html（画像は相対パス参照。単体では配布できない）
"""

import argparse
import html
import os
import re
import sys

import mistune

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
OUT = os.path.join(HERE, "thesis", "thesis.html")

CHAPTERS = [f"chapter{i}.md" for i in range(1, 9)] + ["references.md"]

from thesis_figures import (  # noqa: E402
    CAPTIONS, FIGURES, MISSING, check_references)


FIG_RE = re.compile(r"図(\d+\.\d+)")
# 本文から参照された図番号（登録簿との食い違いを最後に報告する）
REFERENCED = set()


def protect_math(text):
    """数式を退避する。`$w_{ms}$` の `_` が強調記法に食われるのを防ぐ。"""
    store = []

    def keep(m):
        store.append(m.group(0))
        return f"\x00MATH{len(store) - 1}\x00"

    text = re.sub(r"\$\$.+?\$\$", keep, text, flags=re.S)
    text = re.sub(r"(?<!\$)\$[^$\n]+?\$", keep, text)
    return text, store


def restore_math(htm, store):
    for i, s in enumerate(store):
        htm = htm.replace(f"\x00MATH{i}\x00", html.escape(s))
    return htm


def figure_html(num):
    """図を1点出力する。キャプションは各章末の「本章の図表」表の内容列。"""
    cap = f'<figcaption>図{num}　{CAPTIONS.get(num, "")}</figcaption>'
    src = FIGURES.get(num)
    if src is None:
        why = MISSING.get(num, "未取得")
        return (f'<figure class="missing"><div class="ph">図{num}'
                f'<span>{html.escape(why)}</span></div>{cap}</figure>')
    rel = os.path.relpath(os.path.join(ROOT, src),
                          os.path.dirname(OUT)).replace(os.sep, "/")
    return (f'<figure><img src="{html.escape(rel)}" alt="図{num}">'
            f'{cap}</figure>')


# 草稿管理用の節。提出版（build_thesis_tex.py の DROP_SECTIONS）と同じ規則で
# 通し読み用 HTML からも落とす。Markdown 側には残す（草稿管理に使うため）。
DROP_SECTIONS = ("本章の図表", "著者が確認", "著者が原典で確認")


def drop_draft_notes(md_text):
    """草稿管理用のメモを落とす。

    落とすもの:
      - 章末の「本章の図表」「著者が確認・補完すべき箇所」節（本文ごと）
      - 章冒頭の「草稿（日付）」バナー引用

    図のパスとキャプションは `thesis_figures.py` が各章の .md を直接読むため、
    ここで表を落としても図の配置には影響しない。
    """
    lines = md_text.split("\n")
    out, i, n = [], 0, len(lines)
    banner_dropped = False
    while i < n:
        line = lines[i]
        if line.startswith("## "):
            title = line[3:].strip()
            if any(title.startswith(d) for d in DROP_SECTIONS):
                i += 1
                while i < n and not lines[i].startswith(("## ", "# ")):
                    i += 1
                continue
        # 章冒頭の「草稿（…）」バナーは最初の引用ブロックだけを対象にする
        if (not banner_dropped and line.startswith("> ") and "草稿" in line):
            banner_dropped = True
            while i < n and lines[i].startswith(">"):
                i += 1
            while i < n and not lines[i].strip():
                i += 1
            continue
        out.append(line)
        i += 1
    return "\n".join(out)


def insert_figures(md_text, placed):
    """本文で図に初めて言及した段落の直後へ画像を差し込む。

    表（| 始まり）・引用（>）・見出しは対象外。章末の図表一覧で
    重複して差し込まないため。
    """
    out, pending = [], []
    for line in md_text.split("\n"):
        stripped = line.lstrip()
        skip = (not stripped or stripped.startswith(("|", ">", "#", "```")))
        if not skip:
            for num in FIG_RE.findall(line):
                REFERENCED.add(num)
                if num in placed or num not in CAPTIONS:
                    continue
                placed.add(num)
                pending.append(num)
        out.append(line)
        if not stripped and pending:      # 段落の切れ目で吐き出す
            for num in pending:
                out.append(figure_html(num))
                out.append("")
            pending = []
    for num in pending:
        out.append(figure_html(num))
    return "\n".join(out)


CSS = """
:root{--ink:#1f2328;--muted:#57606a;--line:#d8dee4;--accent:#0072B2;
      --bg:#ffffff;--soft:#f6f8fa;}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
     font-family:"Hiragino Mincho ProN","Yu Mincho",'Noto Serif JP',serif;
     line-height:1.9;font-size:16.5px;}
.wrap{max-width:46rem;margin:0 auto;padding:3rem 1.5rem 6rem;}
h1,h2,h3{font-family:"Hiragino Kaku Gothic ProN","Yu Gothic",
         'Noto Sans JP',sans-serif;line-height:1.5;}
h1{font-size:1.75rem;margin:3.5rem 0 1.5rem;padding-bottom:.5rem;
   border-bottom:3px solid var(--accent);}
h2{font-size:1.28rem;margin:2.6rem 0 .9rem;color:#10314a;}
h3{font-size:1.08rem;margin:2rem 0 .7rem;color:var(--muted);}
p{margin:.9rem 0;text-align:justify;}
blockquote{margin:1.2rem 0;padding:.8rem 1.1rem;background:var(--soft);
  border-left:4px solid var(--accent);color:var(--muted);font-size:.94em;}
blockquote strong{color:var(--ink);}
table{border-collapse:collapse;width:100%;margin:1.3rem 0;font-size:.88em;
  font-family:"Hiragino Kaku Gothic ProN","Yu Gothic",sans-serif;}
th,td{border:1px solid var(--line);padding:.42rem .6rem;}
th{background:var(--soft);font-weight:600;text-align:left;}
tbody tr:nth-child(even){background:#fbfcfd;}
figure{margin:1.8rem 0;text-align:center;}
figure img{max-width:100%;height:auto;border:1px solid var(--line);
  border-radius:4px;}
figcaption{font-size:.85em;color:var(--muted);margin-top:.45rem;
  font-family:"Hiragino Kaku Gothic ProN",sans-serif;}
figure.missing .ph{border:2px dashed #d0a24c;border-radius:6px;padding:2.2rem 1rem;
  background:#fffaf0;color:#8a6d3b;font-size:.9em;
  font-family:"Hiragino Kaku Gothic ProN",sans-serif;}
figure.missing .ph span{display:block;font-size:.85em;margin-top:.4rem;opacity:.85;}
code{background:var(--soft);padding:.12em .35em;border-radius:3px;
  font-size:.86em;font-family:ui-monospace,Menlo,Consolas,monospace;}
hr{border:0;border-top:1px solid var(--line);margin:2.5rem 0;}
a{color:var(--accent);}
.chapter{page-break-before:always;}
.chapter:first-of-type{page-break-before:auto;}
#toc{background:var(--soft);border:1px solid var(--line);border-radius:6px;
  padding:1.2rem 1.6rem;margin:2rem 0 3rem;
  font-family:"Hiragino Kaku Gothic ProN",sans-serif;font-size:.92rem;}
#toc h2{margin:.2rem 0 .8rem;font-size:1.05rem;}
#toc ol{margin:0;padding-left:1.3rem;}
#toc li{margin:.22rem 0;}
#toc a{text-decoration:none;}
.meta{color:var(--muted);font-size:.9rem;
  font-family:"Hiragino Kaku Gothic ProN",sans-serif;}
.title{font-size:2rem;border:0;margin-bottom:.4rem;padding:0;}
@media print{
  body{font-size:10.5pt;}
  .wrap{max-width:none;padding:0;}
  #toc{page-break-after:always;}
  figure img{max-width:88%;}
  h1{page-break-after:avoid;} h2{page-break-after:avoid;}
  table,figure{page-break-inside:avoid;}
}
@media (prefers-color-scheme:dark){
  :root{--ink:#e6e6e6;--muted:#a8b0b8;--line:#39414a;--bg:#15181c;
        --soft:#1e232a;--accent:#5aa9dd;}
  figure img{background:#fff;}
  tbody tr:nth-child(even){background:#1a1f25;}
}
"""

KATEX = """
<link rel="stylesheet"
 href="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css">
<script defer src="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.js"></script>
<script defer
 src="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/contrib/auto-render.min.js"
 onload="renderMathInElement(document.body,{delimiters:[
   {left:'$$',right:'$$',display:true},{left:'$',right:'$',display:false}],
   throwOnError:false});"></script>
"""


def build(offline=False):
    # escape=False: 差し込んだ <figure> を生の HTML として通す（既定は escape=True）
    md = mistune.create_markdown(escape=False,
                                 plugins=["table", "strikethrough"])
    placed, bodies, toc = set(), [], []
    for i, name in enumerate(CHAPTERS, 1):
        path = os.path.join(HERE, "thesis", name)
        if not os.path.exists(path):
            print(f"  (skip) {name} が無い")
            continue
        text = open(path, encoding="utf-8").read()
        title = text.split("\n", 1)[0].lstrip("# ").strip()
        toc.append((i, title))
        text = drop_draft_notes(text)
        text = insert_figures(text, placed)
        text, store = protect_math(text)
        htm = restore_math(md(text), store)
        htm = htm.replace("<h1>", f'<h1 id="ch{i}">', 1)
        bodies.append(f'<section class="chapter">{htm}</section>')
        print(f"  第{i}章 {title}")

    nav = "\n".join(f'<li><a href="#ch{i}">{html.escape(t)}</a></li>'
                    for i, t in toc)
    miss = ", ".join(f"図{k}" for k in sorted(MISSING))
    head = f"""<h1 class="title">農業用ロボットの経路追従における<br>
スパース最適制御の実機適用</h1>
<p class="meta">—— 入力遅延下でのチャタリング機序と対策 ——<br>
草稿（{len(CHAPTERS)}章・図 {len(FIGURES)} 点）　未取得の図: {miss}</p>
<nav id="toc"><h2>目次</h2><ol>{nav}</ol></nav>"""

    doc = f"""<!doctype html><html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>卒業論文 草稿</title><style>{CSS}</style>
{"" if offline else KATEX}</head>
<body><div class="wrap">{head}{''.join(bodies)}</div></body></html>"""

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(doc)
    return OUT


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--offline", action="store_true",
                    help="KaTeX を読み込まない（数式は生の TeX 表示）")
    ap.add_argument("--open", action="store_true", help="生成後にブラウザで開く")
    a = ap.parse_args()
    out = build(a.offline)
    size = os.path.getsize(out) / 1024
    print(f"\n出力: {out}（{size:.0f} KB）")
    problems = check_references(REFERENCED)
    if problems:
        print("\n図の登録簿と本文の食い違い:")
        for t in problems:
            print("  -", t)
    if a.open:
        import subprocess
        try:  # WSL: Windows 側の既定ブラウザで開く
            win = subprocess.run(["wslpath", "-w", out], check=True,
                                 capture_output=True, text=True).stdout.strip()
            subprocess.run(["explorer.exe", win], check=False)
            print(f"ブラウザで開きました: {win}")
        except (FileNotFoundError, subprocess.CalledProcessError):
            print("（自動で開けませんでした。上のパスを手動で開いてください）")


if __name__ == "__main__":
    main()
