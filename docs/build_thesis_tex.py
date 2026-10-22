# -*- coding: utf-8 -*-
"""卒論の全章 Markdown → 提出形式の LaTeX（platex + dvipdfmx）。

本文は形式非依存の Markdown で書いてあるため、提出形式が決まった時点で
機械変換する方針を取っている。**書式の差し替えは `preamble.tex` だけで済む**
ように、本文（`ch1.tex` 〜 `ch8.tex`）とプリアンブルを分けて出力する。
研究室指定のクラスファイルが出てきたら `thesis.tex` の 2 行を書き換える。

変換の約束:

- `# 第N章 序論` → `\chapter{序論}`（番号は LaTeX に振らせる。本文の
  「第5.5 節」等の参照と一致するかはビルド後に `--check` で検証する）
- `$$ ... \tag{3.1} $$` → `equation` 環境。`\tag` が amsmath で効くので
  **式番号は Markdown に書いた番号がそのまま出る**
- `**表6.3 ...**` + 直後の表 → `table` 環境（キャプション付き）
- `>` 引用 → `quote`（小さめ）。ただし各章冒頭の「草稿」バナーは落とす
- 図は本文で初めて言及した段落の直後に差し込む（HTML 版と同じ規則）。
  画像が無い図は枠だけ出す

使い方:
    python3 docs/build_thesis_tex.py
    python3 docs/build_thesis_tex.py --check   # ビルド後に番号の整合を検証

出力: docs/thesis/tex/
"""

import argparse
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
OUTDIR = os.path.join(HERE, "thesis", "tex")
sys.path.insert(0, HERE)

from thesis_figures import FIGURES, MISSING  # noqa: E402

CHAPTERS = [f"chapter{i}.md" for i in range(1, 9)]
FIG_RE = re.compile(r"図(\d+\.\d+)")
NUL = "\x00"

# ---------------------------------------------------------------- inline

SPECIALS = {
    "\\": r"\textbackslash{}", "&": r"\&", "%": r"\%", "#": r"\#",
    "_": r"\_", "{": r"\{", "}": r"\}", "~": r"\textasciitilde{}",
    "^": r"\textasciicircum{}", "$": r"\$",
}


def _protect(text, store):
    """数式とインラインコードを退避する（エスケープの対象外にするため）。"""
    def keep(m):
        store.append(m.group(0))
        return f"{NUL}{len(store) - 1}{NUL}"

    text = re.sub(r"\$\$.+?\$\$", keep, text, flags=re.S)
    text = re.sub(r"(?<!\$)\$[^$\n]+?\$", keep, text)
    text = re.sub(r"`[^`\n]+?`", keep, text)
    return text


def _restore(text, store):
    for i, s in enumerate(store):
        if s.startswith("`"):
            body = "".join(SPECIALS.get(c, c) for c in s[1:-1])
            rep = r"\texttt{" + body + "}"
        elif s.startswith("$$"):
            # 段落の途中に置かれた別行立て。`$$ $$` のままだと plain TeX の
            # ディスプレイになり \tag が使えないので環境に直す。
            body = s.strip()[2:-2].strip()
            env = "equation" if "\\tag{" in body else "displaymath"
            rep = f"\n\\begin{{{env}}}\n{body}\n\\end{{{env}}}\n"
        else:
            rep = s                      # インライン数式は TeX のまま通す
        text = text.replace(f"{NUL}{i}{NUL}", rep)
    return text


def inline(text):
    """段落1行分を LaTeX に変換する。"""
    store = []
    text = _protect(text, store)
    text = "".join(SPECIALS.get(c, c) for c in text)
    text = re.sub(r"\*\*(.+?)\*\*", r"\\textbf{\1}", text, flags=re.S)
    text = re.sub(r"(?<!\*)\*([^*]+?)\*(?!\*)", r"\\emph{\1}", text, flags=re.S)
    return _restore(text, store)


# ---------------------------------------------------------------- figures

FIGDIR = os.path.join(OUTDIR, "figs")


def stage_figure(num, src):
    """図を ASCII 名で tex/figs/ に集める。

    `results/` や `docs/論文解説_figs/` は日本語を含む／追跡外のパスにあり、
    extractbb が日本語ディレクトリを読めない（BoundingBox が作れない）。
    提出用に figs/ だけ持ち歩ければよいので、ここで一箇所に集約する。
    """
    os.makedirs(FIGDIR, exist_ok=True)
    ext = os.path.splitext(src)[1].lower()
    dst = os.path.join(FIGDIR, f"fig{num.replace('.', '_')}{ext}")
    if not os.path.exists(dst) or os.path.getmtime(src) > os.path.getmtime(dst):
        shutil.copyfile(src, dst)
    return "figs/" + os.path.basename(dst)


def make_bb():
    """dvipdfmx 用の .xbb を作る。extractbb が無ければ黙って飛ばす。"""
    if not shutil.which("extractbb") or not os.path.isdir(FIGDIR):
        return 0
    n = 0
    for f in sorted(os.listdir(FIGDIR)):
        if f.lower().endswith((".png", ".jpg", ".jpeg")):
            r = subprocess.run(["extractbb", f], cwd=FIGDIR,
                               capture_output=True)
            n += (r.returncode == 0)
    return n


def figure_tex(num):
    src = FIGURES.get(num)
    path = os.path.join(ROOT, src) if src else None
    if not src or not os.path.exists(path):
        why = MISSING.get(num, "画像が手元に無い（results/ は未追跡）")
        return ("\\begin{figure}[htbp]\\centering\n"
                f"\\renewcommand{{\\thefigure}}{{{num}}}\n"
                "\\fbox{\\parbox[c][28mm][c]{0.8\\linewidth}{\\centering\\small "
                f"図{num}（未取得）\\\\ {inline(why)}}}}}\n"
                f"\\caption{{{inline(why)}}}\\label{{fig:{num}}}\n"
                "\\end{figure}")
    rel = stage_figure(num, path)
    return ("\\begin{figure}[htbp]\\centering\n"
            f"\\renewcommand{{\\thefigure}}{{{num}}}\n"
            f"\\includegraphics[width=0.88\\linewidth]{{{rel}}}\n"
            f"\\caption{{}}\\label{{fig:{num}}}\n"
            "\\end{figure}")


# ---------------------------------------------------------------- tables

def split_row(line, store):
    line = _protect(line, store)
    cells = [c.strip() for c in line.strip().strip("|").split("|")]
    return cells


def table_tex(lines, caption, num):
    store = []
    head = split_row(lines[0], store)
    spec = []
    for c in split_row(lines[1], store):
        c = c.strip()
        spec.append("r" if c.endswith(":") and not c.startswith(":")
                    else "c" if c.startswith(":") and c.endswith(":") else "l")
    body = [split_row(r, store) for r in lines[2:]]

    def row(cells):
        cells = [_restore(inline(_restore(c, store)), []) for c in cells]
        return " & ".join(cells) + r" \\"

    out = ["\\begin{table}[htbp]\\centering"]
    if num:
        out.append(f"\\renewcommand{{\\thetable}}{{{num}}}")
    if caption:
        out.append(f"\\caption{{{inline(caption)}}}")
    if num:
        out.append(f"\\label{{tab:{num}}}")
    out += ["\\small", "\\begin{tabular}{" + "".join(spec) + "}", "\\hline",
            row(head), "\\hline"]
    out += [row(r) for r in body]
    out += ["\\hline", "\\end{tabular}", "\\end{table}"]
    return "\n".join(out)


# ---------------------------------------------------------------- blocks

DROP_SECTIONS = ("本章の図表", "著者が確認", "著者が原典で確認")

CAP_RE = re.compile(r"^\*\*表\s*(\d+\.\d+[a-z]?)\s*[:：]?\s*(.+?)\*\*$")
H_RE = re.compile(r"^(#{1,4})\s+(.*)$")
NUM_RE = re.compile(r"^(?:第\d+章\s*|\d+(?:\.\d+)*\.?\s+)")


def convert(md, placed):
    lines = md.split("\n")
    out, i, pending = [], 0, []
    first_quote_dropped = False

    def flush_figs():
        for num in pending:
            out.append(figure_tex(num))
            out.append("")
        pending.clear()

    while i < len(lines):
        line = lines[i]
        s = line.strip()

        if not s:
            flush_figs()
            out.append("")
            i += 1
            continue

        if s == "---":
            i += 1
            continue

        m = H_RE.match(s)
        if m:
            level, raw = len(m.group(1)), m.group(2).strip()
            title = NUM_RE.sub("", raw).strip()
            if any(title.startswith(d) for d in DROP_SECTIONS):
                i += 1                   # 草稿管理用の節は本文ごと落とす
                while i < len(lines) and not re.match(r"^#{1,2} ", lines[i]):
                    i += 1
                continue
            cmd = {1: "chapter", 2: "section", 3: "subsection",
                   4: "subsubsection"}[level]
            num = re.match(r"^(\d+(?:\.\d+)+)\s", raw)
            if level >= 2 and not num:
                out.append(f"\\{cmd}*{{{inline(title)}}}")
            else:
                out.append(f"\\{cmd}{{{inline(title)}}}")
                if num:
                    out.append(f"\\label{{sec:{num.group(1)}}}")
            i += 1
            continue

        if s.startswith("$$"):                       # 数式ブロック
            block = [lines[i]]
            i += 1
            while i < len(lines) and not lines[i].strip().endswith("$$"):
                block.append(lines[i])
                i += 1
            if i < len(lines):
                block.append(lines[i])
                i += 1
            body = "\n".join(block).strip()
            body = body[2:-2].strip() if body.endswith("$$") else body[2:]
            env = "equation" if "\\tag{" in body else "displaymath"
            out.append(f"\\begin{{{env}}}\n{body}\n\\end{{{env}}}")
            continue

        if s.startswith(">"):                        # 引用（注記）
            block = []
            while i < len(lines) and lines[i].strip().startswith(">"):
                block.append(lines[i].strip().lstrip(">").strip())
                i += 1
            text = " ".join(x for x in block if x)
            if not first_quote_dropped and text.startswith("\\textbf{草稿") \
                    or text.startswith("**草稿"):
                first_quote_dropped = True           # 章頭の草稿バナーは落とす
                continue
            out.append("\\begin{quote}\\small\n" + inline(text) + "\n\\end{quote}")
            continue

        if s.startswith("|"):                        # 表（キャプション無し）
            block = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                block.append(lines[i])
                i += 1
            if len(block) >= 2:
                out.append(table_tex(block, None, None))
            continue

        cap = CAP_RE.match(s)
        if cap and i + 1 < len(lines) and lines[i + 1].strip().startswith("|"):
            i += 1
            block = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                block.append(lines[i])
                i += 1
            out.append(table_tex(block, cap.group(2), cap.group(1)))
            continue

        if re.match(r"^[-*] ", s) or re.match(r"^\d+\. ", s):
            ordered = bool(re.match(r"^\d+\. ", s))
            env = "enumerate" if ordered else "itemize"
            items = []
            while i < len(lines) and (re.match(r"^\s*[-*] ", lines[i])
                                      or re.match(r"^\s*\d+\. ", lines[i])
                                      or (items and lines[i].startswith("  ")
                                          and lines[i].strip())):
                t = lines[i].strip()
                t = re.sub(r"^(?:[-*]|\d+\.)\s+", "", t)
                if re.match(r"^\s*(?:[-*]|\d+\.)\s", lines[i]):
                    items.append(t)
                else:
                    items[-1] += " " + t
                i += 1
            out.append(f"\\begin{{{env}}}")
            out += [f"\\item {inline(t)}" for t in items]
            out.append(f"\\end{{{env}}}")
            continue

        # 通常の段落。Markdown が行で折り返されているので、
        # 空行か別ブロックが来るまでを1段落としてまとめてから変換する
        # （そうしないと行をまたぐ `**強調**` が壊れる）。
        buf = []
        while i < len(lines):
            t = lines[i].strip()
            if not t or t == "---" or H_RE.match(t) or t.startswith(
                    ("|", ">", "$$")) or re.match(r"^(?:[-*] |\d+\. )", t):
                break
            buf.append(t)
            i += 1
        para = "\n".join(buf)
        for num in FIG_RE.findall(para):
            if num in placed or (num not in FIGURES and num not in MISSING):
                continue
            placed.add(num)
            pending.append(num)
        out.append(inline(para))

    flush_figs()
    return "\n".join(out)


# ---------------------------------------------------------------- output

PREAMBLE = r"""% 卒論プリアンブル。研究室指定の書式が出たら、このファイルを差し替える。
% ビルド: platex thesis.tex（×2）→ dvipdfmx thesis.dvi
\usepackage{graphicx}
\usepackage{amsmath,amssymb}
\usepackage{bm}
\usepackage{url}
\usepackage[dvipdfmx]{hyperref}
\usepackage{pxjahyper}
\hypersetup{colorlinks=true,linkcolor=black,citecolor=black,urlcolor=blue}
\renewcommand{\baselinestretch}{1.1}
\setcounter{tocdepth}{2}
\setcounter{secnumdepth}{3}
"""

MAIN = r"""%%%% 卒業論文（本文は docs/thesis/chapter*.md から自動生成）
%%%% 再生成: python3 docs/build_thesis_tex.py
%%%% 手で編集すると次回の生成で消えるので、修正は Markdown 側に入れること。
\documentclass[a4paper,11pt,dvipdfmx]{jsbook}
\input{preamble}

\title{%(title)s}
\author{向川 航平（922086）}
\date{2026年12月}

\begin{document}
\maketitle
\tableofcontents
%(includes)s
\end{document}
"""

TITLE = "農業用ロボットの経路追従における\\\\スパース最適制御の実機適用"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="生成後に .toc の節番号が Markdown と一致するか検証")
    args = ap.parse_args()

    os.makedirs(OUTDIR, exist_ok=True)
    placed, includes, nmap = set(), [], []

    for n, name in enumerate(CHAPTERS, 1):
        md = open(os.path.join(HERE, "thesis", name), encoding="utf-8").read()
        for h in re.findall(r"^#{2,3}\s+(\d+\.\d+(?:\.\d+)?)\s", md, re.M):
            nmap.append(h)
        tex = convert(md, placed)
        path = os.path.join(OUTDIR, f"ch{n}.tex")
        open(path, "w", encoding="utf-8").write(tex + "\n")
        includes.append(f"\\include{{ch{n}}}")
        print(f"ch{n}.tex  {len(tex):>7,} bytes  ← {name}")

    open(os.path.join(OUTDIR, "preamble.tex"), "w",
         encoding="utf-8").write(PREAMBLE)
    open(os.path.join(OUTDIR, "thesis.tex"), "w", encoding="utf-8").write(
        MAIN % {"title": TITLE, "includes": "\n".join(includes)})
    open(os.path.join(OUTDIR, "numbers.txt"), "w",
         encoding="utf-8").write("\n".join(nmap))
    print(f"\n出力: {OUTDIR}")
    print(f"図 {len(placed)} 点を配置（未取得 {len(MISSING)} 点を含む）")
    print(f".xbb を {make_bb()} 件生成した")

    if args.check:
        # \include した章の label は ch*.aux 側に出る
        auxes = [os.path.join(OUTDIR, f) for f in sorted(os.listdir(OUTDIR))
                 if f.endswith(".aux")]
        if not auxes:
            print(".aux が無い。先に platex を2回通すこと。")
            return 1
        got = {}
        for a in auxes:
            got.update(dict(re.findall(
                r"\\newlabel\{sec:([\d.]+)\}\{\{([^}]*)\}",
                open(a, encoding="utf-8").read())))
        bad = [(k, v) for k, v in sorted(got.items()) if k != v]
        missing = [n for n in nmap if n not in got]
        if bad or missing:
            print(f"節番号の不一致 {len(bad)} 件 / label 未生成 {len(missing)} 件")
            for k, v in bad[:5]:
                print(f"  Markdown {k} → LaTeX {v}")
            if missing:
                print(f"  未生成: {missing[:5]}")
            return 1
        print(f"節番号 {len(got)} 件すべて Markdown の番号と一致した。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
