# -*- coding: utf-8 -*-
"""L0 と L1 の関係の説明（白黒・A4）: HTML → PDF。

原本は `docs/l0l1_説明.html`（追跡する）。PDF は `.gitignore` の `*.pdf` 対象なので
必要なときにここで作り直す。図はすべて HTML 内のインライン SVG で、外部画像は無い。

変換は Chrome の headless --print-to-pdf を使う。LaTeX（platex）は不要。
用紙・余白・改ページは HTML 側の `@page` と `break-inside` で決めている。

使い方:
    uv run python docs/build_l0l1_pdf.py
    uv run python docs/build_l0l1_pdf.py --open   # 生成後に開く

出力: docs/l0l1_説明.pdf
"""

import argparse
import os
import pathlib
import shutil
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE / "l0l1_説明.html"
OUT = HERE / "l0l1_説明.pdf"

# Chrome の場所（mac の既定インストール先 → PATH 上の名前）
CANDIDATES = [
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    "/Applications/Chromium.app/Contents/MacOS/Chromium",
    "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge",
]


def find_chrome():
    for p in CANDIDATES:
        if os.path.isfile(p):
            return p
    for name in ("google-chrome", "chromium", "chromium-browser", "msedge"):
        p = shutil.which(name)
        if p:
            return p
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--open", action="store_true", help="生成後に PDF を開く")
    args = ap.parse_args()

    if not SRC.is_file():
        sys.exit(f"原本が無い: {SRC}")

    chrome = find_chrome()
    if not chrome:
        sys.exit("Chrome / Chromium / Edge が見つからない。"
                 "どれかを入れるか、HTML をブラウザで開いて Cmd+P → PDF に保存する。")

    cmd = [
        chrome,
        "--headless=new",
        "--disable-gpu",
        "--no-first-run",
        "--no-pdf-header-footer",      # ページ番号・URL の自動ヘッダを出さない
        "--run-all-compositor-stages-before-draw",
        "--virtual-time-budget=10000",  # Web フォントと SVG の描画を待つ
        f"--print-to-pdf={OUT}",
        SRC.as_uri(),                   # 日本語パスを percent-encode する
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)

    if not OUT.is_file():
        sys.stderr.write(r.stdout + r.stderr)
        sys.exit(f"PDF ができていない（Chrome 終了コード {r.returncode}）")

    print(f"出力: {OUT}（{OUT.stat().st_size / 1024:.0f} KB）")
    if args.open:
        subprocess.run(["open", str(OUT)])


if __name__ == "__main__":
    main()
