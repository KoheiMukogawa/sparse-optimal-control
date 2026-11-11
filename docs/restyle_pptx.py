# -*- coding: utf-8 -*-
"""発表スライド（.pptx）の体裁を、機械でできる範囲だけ一括で直す。

方針は `docs/スライドデザイン方針.md`（出典: 久松 2024, 日児腎誌 37:111-116）。
そのうち **手作業では数百箇所になるもの** だけをここで扱う。

  1. 欧文フォント —— 和文フォントで組まれた英数字を Segoe UI にする
     （`<a:latin>` だけを差し替え、和文 `<a:ea>` は Meiryo のまま）。
     **コード用の MS Gothic は対象外**。日本語コメントと ASCII の桁を
     揃えるには和文等幅が要るので、ここだけは原則の例外として残す。
  2. 近似重複色の統合 —— 見た目が変わらないのに色数だけ増えている組を潰す。
  3. テーマフォント —— 新しいテキストボックスが Calibri で出てくるのを直す。

文字数・サイズ・余白・ページ番号は**機械では直せない**（削る判断が要る）。
`--check` で現状を測って出すので、手作業の優先順位づけに使う。

使い方:
    python3 docs/restyle_pptx.py docs/研究会発表_2026-10-22.pptx --check
    python3 docs/restyle_pptx.py docs/研究会発表_2026-10-22.pptx
      → docs/研究会発表_2026-10-22_restyled.pptx を新しく作る（元は触らない）

標準ライブラリだけで動く（python-pptx は不要）。
"""

import argparse
import os
import re
import shutil
import sys
import zipfile
from collections import Counter

# 和文フォントで組まれている欧文を、この欧文フォントへ移す
LATIN = "Segoe UI"
# 等幅が要るので置き換えない和文フォント（コードブロック）
KEEP = {"MS Gothic", "MS Mincho", "Consolas"}

# 近似重複色の統合（残す色 ← 潰す色）。見た目はほぼ変わらない。
MERGE = {
    "23292F": ["222A30", "202A33"],                    # 墨3種 → 文字の基本色1つ
    "4F5A63": ["53606A"],                              # 灰2種 → 補助色1つ
    "F6F8F9": ["F4F7F8", "F4F6F8", "EEF3F5", "EEF4F6"],  # オフ白5種 → 箱の下地1つ
    "E4EFF3": ["E3F0F5"],                              # 淡青2種
    "2E7A60": ["2E8B57"],                              # 緑2種
    "C0433B": ["B14A2E"],                              # 赤系の強調2種
}
# 灰のランプ（#6B7680 / #8A959E / #DCE2E6）は段階として使い分けているので残す。
# 無彩色は色数に数えない（出典 図10b「灰色で色数を抑える」）。
COLOR_MAP = {old: new for new, olds in MERGE.items() for old in olds}

PARTS = re.compile(r"ppt/(slides|slideLayouts|slideMasters|notesSlides)/[^/]+\.xml$")


def restyle_xml(xml):
    """1つのパートを直し、(直した XML, 変更件数の内訳) を返す。"""
    n_font = n_color = 0

    def latin(m):
        nonlocal n_font
        if m.group(1) in KEEP or m.group(1) == LATIN:
            return m.group(0)
        n_font += 1
        return f'<a:latin typeface="{LATIN}"'

    xml = re.sub(r'<a:latin typeface="([^"]*)"', latin, xml)

    def color(m):
        nonlocal n_color
        new = COLOR_MAP.get(m.group(1).upper())
        if not new:
            return m.group(0)
        n_color += 1
        return f'srgbClr val="{new}"'

    xml = re.sub(r'srgbClr val="([0-9A-Fa-f]{6})"', color, xml)
    return xml, (n_font, n_color)


def restyle_theme(xml):
    """テーマの既定フォントを 欧文 Segoe UI / 和文 Meiryo にする。"""
    def fix(m):
        s = m.group(0)
        s = re.sub(r'<a:latin typeface="[^"]*"', f'<a:latin typeface="{LATIN}"', s)
        s = re.sub(r'<a:ea typeface="[^"]*"', '<a:ea typeface="Meiryo"', s)
        return s
    return re.sub(r"<a:(major|minor)Font>.*?</a:\1Font>", fix, xml, flags=re.S)


def check(src):
    """現状を測って出す。書き換えはしない。"""
    fonts, colors, sizes, chars = Counter(), Counter(), Counter(), []
    with zipfile.ZipFile(src) as z:
        slides = [n for n in z.namelist()
                  if re.match(r"ppt/slides/slide\d+\.xml$", n)]
        for n in sorted(slides, key=lambda p: int(re.search(r"\d+", p.split("/")[-1]).group())):
            s = z.read(n).decode("utf-8")
            fonts.update(re.findall(r'<a:latin typeface="([^"]+)"', s))
            colors.update(c.upper() for c in re.findall(r'srgbClr val="([0-9A-Fa-f]{6})"', s))
            sizes.update(int(v) / 100 for v in re.findall(r'sz="(\d+)"', s))
            chars.append(len("".join(re.findall(r"<a:t>(.*?)</a:t>", s, re.S))))

    print(f"スライド {len(chars)} 枚・総 {sum(chars):,} 字")
    print(f"  1枚あたり: 平均 {sum(chars)/len(chars):.0f} 字 / 中央値 "
          f"{sorted(chars)[len(chars)//2]} 字 / 最大 {max(chars)} 字　（方針: 150 字以内）")
    over = [i + 1 for i, c in enumerate(chars) if c > 150]
    print(f"  150 字超: {len(over)} 枚 {over}")

    tot = sum(sizes.values())
    small = sum(v for k, v in sizes.items() if k < 18)
    print(f"\n文字サイズ: 指定のある {int(tot)} run のうち 18 pt 未満が "
          f"{int(small)} ({small/tot*100:.0f} %)　（方針: 本文 18 pt 下限）")
    print("  " + " ".join(f"{k:g}pt×{v}" for k, v in sorted(sizes.items(), reverse=True)))

    print(f"\n欧文フォント指定: " + ", ".join(f"{k}×{v}" for k, v in fonts.most_common()))
    bad = sum(v for k, v in fonts.items() if k not in KEEP and k != LATIN)
    print(f"  → 和文フォントで組まれた欧文 {bad} run（方針: Segoe UI）")

    print(f"\n色 {len(colors)} 種: " + ", ".join(f"#{k}×{v}" for k, v in colors.most_common()))
    dup = [f"#{o}→#{n}" for o, n in COLOR_MAP.items() if o in colors]
    print(f"  → 統合できる近似重複: {', '.join(dup) if dup else 'なし'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", help="入力 .pptx")
    ap.add_argument("-o", "--out", help="出力 .pptx（既定: <入力>_restyled.pptx）")
    ap.add_argument("--check", action="store_true", help="測るだけで書き換えない")
    a = ap.parse_args()

    if not os.path.exists(a.src):
        sys.exit(f"入力が無い: {a.src}")
    if a.check:
        check(a.src)
        return

    out = a.out or re.sub(r"\.pptx$", "_restyled.pptx", a.src)
    if os.path.abspath(out) == os.path.abspath(a.src):
        sys.exit("出力が入力と同じ。元ファイルは上書きしない方針。")

    font = color = 0
    with zipfile.ZipFile(a.src) as zin, \
            zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as zout:
        for item in zin.infolist():
            data = zin.read(item.filename)
            if PARTS.match(item.filename):
                xml, (f, c) = restyle_xml(data.decode("utf-8"))
                font += f
                color += c
                data = xml.encode("utf-8")
            elif re.match(r"ppt/theme/theme\d+\.xml$", item.filename):
                data = restyle_theme(data.decode("utf-8")).encode("utf-8")
            zout.writestr(item, data)

    with zipfile.ZipFile(out) as z:       # 壊れていないことだけ確認する
        broken = z.testzip()
    if broken:
        sys.exit(f"出力が壊れている: {broken}")

    print(f"欧文フォント → {LATIN}: {font} 箇所")
    print(f"近似重複色の統合: {color} 箇所")
    print(f"出力: {out}（{os.path.getsize(out)/1024:.0f} KB）")
    print("\n残りは手作業: 文字を削る / ページ番号を消す / 余白を揃える"
          "（docs/スライドデザイン方針.md の適用手順 3〜6）")


if __name__ == "__main__":
    main()
