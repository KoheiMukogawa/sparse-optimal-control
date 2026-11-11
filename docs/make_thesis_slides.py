# -*- coding: utf-8 -*-
"""卒論発表のスライドを `docs/スライドデザイン方針.md` に従って生成する。

台紙（テーマ・マスタ・レイアウト）だけ `docs/研究会発表_2026-10-22.pptx` から借り、
**スライドは全部この場で組み直す**。python-pptx は使わず標準ライブラリだけで
OOXML を書く（この環境に入れられないため）。

方針の要点だけ再掲する:
  - フォントは BIZ UDPGothic 1本（コードのみ MS Gothic）
  - 色は無彩色＋朱 #B1390A の1色。有彩色は図の中だけ
  - 見出しは**言い切りの文**（assertion-evidence）。体言止めにしない
  - 箇条書きは `・` で3項目まで。1枚 150 字以内
  - 囲みは1枚に最大1つ。角丸・影・色帯・飾りバーを置かない
  - ページ番号はフィールドで入れる（質疑で参照されるため）

数値はすべて `docs/thesis/chapter5.md`・`chapter6.md`・`chapter8.md` から採った。
**原稿を直したらここも直すこと**（二重管理になっている箇所は番号で検索できる）。

使い方:
    python3 docs/make_thesis_slides.py
出力: docs/卒論発表_ドラフト.pptx（元ファイルは触らない）
"""

import os
import re
import struct
import zipfile
from xml.sax.saxutils import escape

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
BASE = os.path.join(HERE, "研究会発表_2026-10-22.pptx")
OUT = os.path.join(HERE, "卒論発表_ドラフト.pptx")

EMU = 914400
SW, SH = 12192000, 6858000          # 13.333 × 7.5 in
FONT = "BIZ UDPGothic"
MONO = "MS Gothic"
INK = "1A1A1A"        # 見出し
BODY = "2D2D2D"       # 本文
MUTED = "757575"      # 注記・ラベル
LINE = "C4C4C4"       # 罫線
ACCENT = "B1390A"     # 唯一の有彩色

L = 0.70              # 左右の余白（in）
CW = 13.333 - 2 * L   # 本文の幅


def I(v):
    return int(round(v * EMU))


# ---------------------------------------------------------------- 低水準
def _runs(text, size, color, bold=False, font=FONT):
    """`**太字**` と `##朱##` だけの簡易記法を実行列に変える。"""
    out = []
    for part in re.split(r"(\*\*.+?\*\*|##.+?##)", text):
        if not part:
            continue
        b, c = bold, color
        if part.startswith("**"):
            part, b = part[2:-2], True
        elif part.startswith("##"):
            part, b, c = part[2:-2], True, ACCENT
        out.append(
            '<a:r><a:rPr lang="ja-JP" altLang="en-US" sz="%d" b="%d" dirty="0">'
            '<a:solidFill><a:srgbClr val="%s"/></a:solidFill>'
            '<a:latin typeface="%s"/><a:ea typeface="%s"/><a:cs typeface="%s"/>'
            "</a:rPr><a:t>%s</a:t></a:r>"
            % (int(size * 100), 1 if b else 0, c, font, font, font, escape(part)))
    return "".join(out)


def para(text, size=14, color=BODY, bold=False, bullet=False, before=0,
         line=150, align="l", font=FONT):
    ind = ('marL="180000" indent="-180000"' if bullet else 'marL="0" indent="0"')
    txt = ("・" + text) if bullet else text
    return ('<a:p><a:pPr algn="%s" %s><a:lnSpc><a:spcPct val="%d"/></a:lnSpc>'
            '<a:spcBef><a:spcPts val="%d"/></a:spcBef><a:buNone/></a:pPr>%s</a:p>'
            % (align, ind, line * 1000, int(before * 100),
               _runs(txt, size, color, bold, font)))


def textbox(sid, x, y, w, h, paras, anchor="t"):
    return ('<p:sp><p:nvSpPr><p:cNvPr id="%d" name="t%d"/><p:cNvSpPr txBox="1"/>'
            '<p:nvPr/></p:nvSpPr><p:spPr><a:xfrm><a:off x="%d" y="%d"/>'
            '<a:ext cx="%d" cy="%d"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/>'
            '</a:prstGeom><a:noFill/></p:spPr><p:txBody>'
            '<a:bodyPr wrap="square" lIns="0" tIns="0" rIns="0" bIns="0" anchor="%s"/>'
            "<a:lstStyle/>%s</p:txBody></p:sp>"
            % (sid, sid, I(x), I(y), I(w), I(h), anchor, "".join(paras)))


def rect(sid, x, y, w, h, fill=None, line=None, lw=0.75):
    f = ('<a:solidFill><a:srgbClr val="%s"/></a:solidFill>' % fill) if fill else "<a:noFill/>"
    ln = ('<a:ln w="%d"><a:solidFill><a:srgbClr val="%s"/></a:solidFill></a:ln>'
          % (int(lw * 12700), line)) if line else "<a:ln><a:noFill/></a:ln>"
    return ('<p:sp><p:nvSpPr><p:cNvPr id="%d" name="r%d"/><p:cNvSpPr/><p:nvPr/>'
            '</p:nvSpPr><p:spPr><a:xfrm><a:off x="%d" y="%d"/><a:ext cx="%d" cy="%d"/>'
            '</a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom>%s%s</p:spPr>'
            '<p:txBody><a:bodyPr/><a:lstStyle/><a:p/></p:txBody></p:sp>'
            % (sid, sid, I(x), I(y), I(w), I(h), f, ln))


def png_size(path):
    with open(path, "rb") as f:
        return struct.unpack(">II", f.read(33)[16:24])


def picture(sid, rid, path, x, y, w, h):
    """(x, y, w, h) の枠に縦横比を保って収め、中央に置く。"""
    pw, ph = png_size(path)
    s = min(w / pw, h / ph)
    dw, dh = pw * s, ph * s
    ox, oy = x + (w - dw) / 2, y + (h - dh) / 2
    return ('<p:pic><p:nvPicPr><p:cNvPr id="%d" name="p%d"/><p:cNvPicPr>'
            '<a:picLocks noChangeAspect="1"/></p:cNvPicPr><p:nvPr/></p:nvPicPr>'
            '<p:blipFill><a:blip r:embed="%s"/><a:stretch><a:fillRect/></a:stretch>'
            '</p:blipFill><p:spPr><a:xfrm><a:off x="%d" y="%d"/><a:ext cx="%d" cy="%d"/>'
            '</a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></p:spPr></p:pic>'
            % (sid, sid, rid, I(ox), I(oy), I(dw), I(dh)))


def table(sid, x, y, widths, rows, size=13, rowh=0.38):
    """横罫だけの表。1行目が見出し。セルは (文字列) か ## で朱。"""
    grid = "".join('<a:gridCol w="%d"/>' % I(w) for w in widths)
    trs = []
    for i, row in enumerate(rows):
        head = (i == 0)
        tcs = []
        for cell in row:
            al = "l" if row.index(cell) == 0 else "ctr"
            p = para(cell, size=size, color=INK if head else BODY, bold=head,
                     align=al, line=100)
            bottom = ('<a:lnB w="%d"><a:solidFill><a:srgbClr val="%s"/></a:solidFill>'
                      "</a:lnB>" % (int((1.25 if head else 0.75) * 12700),
                                    MUTED if head else LINE))
            tcs.append('<a:tc><a:txBody><a:bodyPr/><a:lstStyle/>%s</a:txBody>'
                       '<a:tcPr marL="54000" marR="54000" marT="36000" marB="36000" '
                       'anchor="ctr"><a:lnL><a:noFill/></a:lnL><a:lnR><a:noFill/></a:lnR>'
                       "<a:lnT><a:noFill/></a:lnT>%s<a:noFill/></a:tcPr></a:tc>"
                       % (p, bottom))
        trs.append('<a:tr h="%d">%s</a:tr>' % (I(rowh), "".join(tcs)))
    return ('<p:graphicFrame><p:nvGraphicFramePr><p:cNvPr id="%d" name="tbl%d"/>'
            '<p:cNvGraphicFramePr><a:graphicFrameLocks noGrp="1"/>'
            '</p:cNvGraphicFramePr><p:nvPr/></p:nvGraphicFramePr>'
            '<p:xfrm><a:off x="%d" y="%d"/><a:ext cx="%d" cy="%d"/></p:xfrm>'
            '<a:graphic><a:graphicData uri="http://schemas.openxmlformats.org/'
            'drawingml/2006/table"><a:tbl><a:tblPr/><a:tblGrid>%s</a:tblGrid>%s'
            "</a:tbl></a:graphicData></a:graphic></p:graphicFrame>"
            % (sid, sid, I(x), I(y), I(sum(widths)), I(rowh * len(rows)),
               grid, "".join(trs)))


def pagenum(sid, n):
    fld = ('<a:fld id="{2B4C1E5%d-0000-4000-8000-00000000%04d}" type="slidenum">'
           '<a:rPr lang="ja-JP" altLang="en-US" sz="1000"><a:solidFill>'
           '<a:srgbClr val="%s"/></a:solidFill><a:latin typeface="%s"/>'
           '<a:ea typeface="%s"/></a:rPr><a:t>%d</a:t></a:fld>'
           % (n % 10, n, MUTED, FONT, FONT, n))
    p = ('<a:p><a:pPr algn="r"><a:buNone/></a:pPr>%s</a:p>' % fld)
    return textbox(sid, 11.6, 6.95, 1.03, 0.3, [p])


SLD = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
       '<p:sld xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"'
       ' xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"'
       ' xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">'
       "<p:cSld>%s<p:spTree><p:nvGrpSpPr><p:cNvPr id=\"1\" name=\"\"/>"
       "<p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr><p:grpSpPr><a:xfrm>"
       '<a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/>'
       '<a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>%s</p:spTree></p:cSld>'
       "<p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr></p:sld>")


# ---------------------------------------------------------------- 版面
TOP = 2.00          # 本文の上端
BOT = 6.60          # 本文の下端
COL = 5.35          # 2段組みの左段の幅
RX = L + COL + 0.60  # 右段の左端


def title_slide(nid, title_lines, sub, meta):
    """表紙だけ黒地。院試デッキと同じ作り。"""
    s = [rect(nid(), 0, 0, 13.333, 7.5, fill=INK)]
    ps = [para(t, 36, "FFFFFF", bold=True, line=130) for t in title_lines]
    s.append(textbox(nid(), L + 0.3, 2.45, CW - 0.6, 1.9, ps))
    s.append(textbox(nid(), L + 0.3, 4.35, CW - 0.6, 0.35,
                     [para(sub, 14, ACCENT, line=100)]))
    s.append(textbox(nid(), L + 0.3, 5.55, CW - 0.6, 1.3,
                     [para(m, 12 if i else 16, "C4C4C4" if i else "FFFFFF",
                           bold=(i == 0), line=135) for i, m in enumerate(meta)]))
    return s


def divider(nid, label, title):
    s = [textbox(nid(), L, 3.05, CW, 0.26, [para(label, 11, MUTED, line=100)])]
    s.append(textbox(nid(), L, 3.30, CW, 0.7, [para(title, 30, INK, bold=True)]))
    s.append(rect(nid(), L, 4.15, 2.0, 0.0105, fill=INK))
    return s


def content(nid, img, eyebrow, title, bullets=(), fig=None, note=None,
            box=None, rows=None, widths=None, takeaway=None):
    s = [textbox(nid(), L, 0.42, CW, 0.26, [para(eyebrow, 11, MUTED, line=100)])]
    s.append(textbox(nid(), L, 0.62, CW, 1.18,
                     [para(title, 30, INK, bold=True, line=115)], anchor="b"))
    bot = BOT - (0.45 if note else 0) - (0.75 if takeaway else 0)

    bw = CW if not (fig or box) else COL
    if bullets:
        ps = [para(b, 14, BODY, bullet=True, before=(0 if i == 0 else 10))
              for i, b in enumerate(bullets)]
        s.append(textbox(nid(), L, TOP, bw, bot - TOP, ps))

    if fig:
        path = os.path.join(ROOT, fig)
        if bullets:
            s.append(picture(nid(), img(path), path, RX, TOP, 13.333 - L - RX,
                             bot - TOP))
        else:
            s.append(picture(nid(), img(path), path, L, TOP, CW, bot - TOP))

    if box:
        head, lines, val = box
        bx, by, bwid, bh = RX, TOP, 13.333 - L - RX, 2.5
        s.append(rect(nid(), bx, by, bwid, bh, line=LINE))
        ps = [para(head, 12, INK, bold=True, line=100)]
        if val:
            ps.append(para(val, 28, INK, bold=True, before=14, line=110))
        ps += [para(t, 11, MUTED, before=12, line=135) for t in lines]
        s.append(textbox(nid(), bx + 0.32, by + 0.30, bwid - 0.64, bh - 0.6, ps))

    if rows:
        tw = sum(widths)
        s.append(table(nid(), L + (CW - tw) / 2, TOP, widths, rows))

    if takeaway:
        s.append(rect(nid(), L, bot + 0.18, CW, 0.62, line=LINE))
        s.append(textbox(nid(), L, bot + 0.18, CW, 0.62,
                         [para(takeaway, 15, INK, bold=True, align="ctr", line=100)],
                         anchor="ctr"))
    if note:
        s.append(textbox(nid(), L, BOT + 0.06, CW, 0.4,
                         [para(note, 10.5, MUTED, line=130)]))
    return s


# ---------------------------------------------------------------- 中身
# 数値の出所: chapter5.md / chapter6.md / chapter8.md（表6.4・表6.6・表6.9・8.1）
TITLE = ["農業用ロボットの経路追従における",
         "スパース最適制御の実機適用"]

SLIDES = [
    dict(kind="title", title=TITLE,
         sub="入力遅延下でのチャタリング機序と対策",
         meta=["向川 航平", "大阪工業大学 ロボティクス＆デザイン工学部",
               "卒業研究発表"]),

    dict(eyebrow="1. 背景", title="既存手法は、誤差がゼロでない限り入力を出し続ける",
         bullets=[
             "農業用の小型移動ロボットの経路追従。車載計算機は Raspberry Pi 4 で、"
             "0.1 秒の制御周期内に毎回最適化を解く",
             "古典的な Kanayama 法も標準的な L2-MPC も、微小な操舵を絶え間なく打ち続ける",
             "微小操舵の反復は、アクチュエータの消耗と電力消費につながる"],
         box=("ω ゼロ入力率（実機実測）", [
             "Kanayama 法は走行時間の 35 % にわたり操舵し続けている。"
             "小さな入力を打ち続けるのが既存手法の典型である"],
             "Kanayama 0.650 ／ L2 0.872")),

    dict(eyebrow="1. 問い", title="問いは「L1 が優れているか」ではない",
         bullets=[
             "最大ハンズオフ制御は、##必要なときにだけ大きく動かし、それ以外は完全に止める##",
             "ただし理論はオープンループで、遅延も外乱も無い前提に立っている",
             "実機にはどちらもある。本研究はその差を定量化する"],
         takeaway="スパース制御を実機で成立させる条件は何か"),

    dict(eyebrow="2. 手法", title="入力の大きさではなく、打つ回数を減らす",
         bullets=[
             "L2 は $u^2$ を罰する → 小さな操舵を満遍なく使い、なめらかになる",
             "L1 は |u| を罰する → 多くが厳密に 0 になる（bang-off-bang）",
             "本研究は補正入力 δu に L1 を課し、N=15・0.1 秒周期の MPC として毎周期解き直す"],
         fig="results/2026-08-02_thesis_figs2/fig2_2_penalty_shapes.png"),

    dict(eyebrow="3. 結果", title="素の L1-MPC は、実機のコーナーで振動した",
         bullets=[
             "Σ|u| は ##19.17##（L2 は 4.65。4.1 倍）",
             "ω の符号反転は ##18 回##（L2 は 0 回）。走行時間の 37.6 % を角速度上限に張り付いた",
             "ω ゼロ率は 0.529 で 4 条件中**最低**"],
         fig="results/2026-07-26_thesis_figs/fig6_3_omega_all.png",
         takeaway="スパース制御を素朴に実装すると、最もスパースでない制御になる"),

    dict(eyebrow="4. 原因", title="原因は実機にしかない遅延だった",
         bullets=[
             "理想シミュレーションでは同じ制御則が振動しない（符号反転 0 回）",
             "求解・通信・速度ループから、入力遅延を約 200 ms ＝ 2 ステップと見積もった",
             "sim に遅延 2 ステップを注入すると ##Σ|u| 18.49・符号反転 19 回##。実機（19.17・18 回）と一致"],
         fig="results/2026-08-02_thesis_figs2/fig7_1_limit_cycle.png",
         note="※ 200 ms は推定値である。カメラ真値による直接計測は今後の課題（8.2）。"),

    dict(eyebrow="4. 原因", title="破綻はしきい値的で、壊れるのは L1 だけである",
         bullets=[
             "遅延 1 ステップまでは無事（符号反転 2 回）。2 ステップで急激に崩れる",
             "L2 は遅延 3 ステップまで符号反転 0 回、4 ステップでも 1 回に留まる",
             "実機の遅延は、ちょうどこの境界を越えていた"],
         fig="results/2026-07-26_delay_sweep/delay_sweep.png",
         note="※ 図の横軸は遅延ステップ数（1 ステップ＝0.1 秒）。6 条件 × 遅延 5 点。"),

    dict(eyebrow="5. 対策", title="入力の変化率を罰すると、振動だけが消えた",
         fig="results/2026-07-26_thesis_figs/fig5_6_omega_before_after.png",
         takeaway="符号反転 18 → 1 回、Σ|u| 19.17 → 5.02。ω ゼロ率は 0.882 で 4 条件中最良",
         note="※ 実機バッチ 4 条件 × 3 本、12/12 到達。最大 |ω| は 2.0 rad/s のままで、"
              "変わったのは上限に張り付く時間（37.6 % → 3.9 %）である。"),

    dict(eyebrow="6. 評価", title="総合では L2-MPC が優る。勝つのは 2 軸だけ",
         rows=[["指標", "L2-MPC", "L1＋移動抑制（提案）"],
               ["odom 基準 RMSE [cm]", "**1.83**", "2.20"],
               ["真値基準の終点誤差 [cm]", "**12.67**", "21.17"],
               ["求解時間 p95 [ms]", "**21.1**", "31.0"],
               ["ω ゼロ率", "0.872", "##0.882##"],
               ["最終姿勢 |θ| [°]", "19.0 ± 3.0", "##5.0 ± 4.6##"]],
         widths=[4.6, 3.0, 3.6],
         takeaway="入力の疎さと終端姿勢を重視する用途において選択されるべき手法である"),

    dict(eyebrow="7. 考察", title="計算コストは、可解性ではなく安定性の問題である",
         bullets=[
             "L1 は L2 より求解が重い（L 字経路で p95 約 1.8 倍）",
             "その求解時間は、そのまま##入力遅延の一部##になる",
             "遅い計算が自分自身の安定性を削る。低計算資源の機体に載せる際の一般的な注意点"],
         fig="results/2026-07-26_thesis_figs/fig6_4_solve_time.png"),

    dict(eyebrow="7. 考察", title="オドメトリでは滑りが見えず、評価を誤る",
         bullets=[
             "真値基準の誤差は、odom 基準の ##6.9〜9.6 倍##",
             "直線経路では、両者で手法の優劣が逆転する事例も観測された",
             "AprilTag の俯瞰カメラで真値計測系を構築した（手実測と 2.5 cm・1.6° で一致）"],
         fig="results/2026-07-26_thesis_figs/fig6_6_odom_vs_truth.png"),

    dict(eyebrow="8. 結論", title="遅延への明示的な配慮が、成立の必須条件である",
         bullets=[
             "素朴な L1-MPC は実機で破綻する。最もスパースでない制御になる",
             "原因は入力遅延であり、その機序をシミュレーションで定量的に再現した",
             "入力変化率へのペナルティで、##スパース性を保ったまま##振動を除去できた"],
         fig="results/2026-08-02_thesis_figs2/fig1_3_logic_flow.png"),

    dict(eyebrow="8. 貢献", title="残したのは、障害の同定・計算コストの意味・真値計測系",
         bullets=[
             "**実機適用の障害と解決法**：bang-off-bang が遅延下で限界振動に転じる機序を同定し、最小限の変更で対処できることを実機で示した",
             "**計算コストの位置づけ**：求解時間が遅延を通じて安定性に効くことを示した",
             "**外部真値計測系**：俯瞰カメラとタグの自動サーベイは再利用できる"]),

    dict(eyebrow="8. 課題", title="残るのは外乱下の実機評価と、遅延の直接計測",
         bullets=[
             "**外乱下の実機評価**：積載 0 / 500 / 1000 g の 27 走行。条件は整備済みで、復旧後ただちに実施する",
             "**遅延の直接計測**：200 ms は推定値。θ の立ち上がり時刻差をカメラ真値で測れば主張の核を裏づけられる",
             "以降の実機実験は 1 m L 字に統一する（真値を測れる唯一のコース）"]),

    dict(kind="divider", label="APPENDIX", title="補足"),

    dict(eyebrow="補足 A", title="w_ms = 2.0 は、遅延 3〜4 ステップの余裕を買っている",
         bullets=[
             "λ × w_ms の 2 次元掃引を遅延 2 ステップ下で行い、さらに遅延を 0〜4 ステップまで振って選んだ",
             "w_ms = 0.5 でも遅延 2 ステップでは同等だが、##遅延 4 ステップで崩壊する##（Σ|u| 16.16 対 7.52）",
             "単一条件での最適化は、見かけの最適を掴む"],
         fig="results/2026-07-26_lambda_ms_grid/sweep_ms.png"),

    dict(eyebrow="補足 B", title="シミュレーションが再現するのは平均値だけである",
         bullets=[
             "入力量と符号反転は実機とよく一致する（l1_ms2 は 1 回対 1 回）",
             "ただし ##RMSE は一致しない##。飽和率はシミュレーションが 12〜21 % 過大に出る",
             "決定論的モデルのため、実機のばらつき（σ ≤ 0.25）を説明する能力はない"],
         fig="results/2026-07-26_thesis_figs/fig5_7_real_vs_sim.png"),
]


# ---------------------------------------------------------------- 梱包
CT = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
      '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
      '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.'
      'relationships+xml"/><Default Extension="xml" ContentType="application/xml"/>'
      '<Default Extension="png" ContentType="image/png"/>'
      '<Override PartName="/ppt/presentation.xml" ContentType="application/vnd.'
      'openxmlformats-officedocument.presentationml.presentation.main+xml"/>'
      '<Override PartName="/ppt/slideMasters/slideMaster1.xml" ContentType="application/'
      'vnd.openxmlformats-officedocument.presentationml.slideMaster+xml"/>'
      "%s%s"
      '<Override PartName="/ppt/theme/theme1.xml" ContentType="application/vnd.'
      'openxmlformats-officedocument.theme+xml"/>'
      '<Override PartName="/ppt/presProps.xml" ContentType="application/vnd.'
      'openxmlformats-officedocument.presentationml.presProps+xml"/>'
      '<Override PartName="/ppt/viewProps.xml" ContentType="application/vnd.'
      'openxmlformats-officedocument.presentationml.viewProps+xml"/>'
      '<Override PartName="/ppt/tableStyles.xml" ContentType="application/vnd.'
      'openxmlformats-officedocument.presentationml.tableStyles+xml"/>'
      '<Override PartName="/docProps/core.xml" ContentType="application/vnd.'
      'openxmlformats-package.core-properties+xml"/>'
      '<Override PartName="/docProps/app.xml" ContentType="application/vnd.'
      'openxmlformats-officedocument.extended-properties+xml"/></Types>')

ROOT_RELS = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
             '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/'
             'relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats'
             '.org/officeDocument/2006/relationships/officeDocument" Target="ppt/'
             'presentation.xml"/><Relationship Id="rId2" Type="http://schemas.'
             'openxmlformats.org/package/2006/relationships/metadata/core-properties"'
             ' Target="docProps/core.xml"/><Relationship Id="rId3" Type="http://schemas.'
             'openxmlformats.org/officeDocument/2006/relationships/extended-properties"'
             ' Target="docProps/app.xml"/></Relationships>')

REL = ('<Relationship Id="%s" Type="http://schemas.openxmlformats.org/'
       'officeDocument/2006/relationships/%s" Target="%s"/>')


def build(n, spec):
    """1枚分の (XML, 使う画像のパス) を返す。"""
    counter = [1]
    imgs = []

    def nid():
        counter[0] += 1
        return counter[0]

    def img(path):
        imgs.append(path)
        return "rId%d" % (len(imgs) + 1)

    kind = spec.get("kind", "content")
    bg = ""
    if kind == "title":
        bg = ('<p:bg><p:bgPr><a:solidFill><a:srgbClr val="%s"/></a:solidFill>'
              "<a:effectLst/></p:bgPr></p:bg>" % INK)
        shapes = title_slide(nid, spec["title"], spec["sub"], spec["meta"])
    elif kind == "divider":
        shapes = divider(nid, spec["label"], spec["title"])
        shapes.append(pagenum(nid(), n))
    else:
        shapes = content(nid, img, **{k: v for k, v in spec.items() if k != "kind"})
        shapes.append(pagenum(nid(), n))
    return SLD % (bg, "".join(shapes)), imgs


def main():
    base = zipfile.ZipFile(BASE)
    keep = [n for n in base.namelist()
            if n.startswith(("ppt/slideLayouts/", "ppt/slideMasters/"))
            or n in ("ppt/presProps.xml", "ppt/viewProps.xml", "ppt/tableStyles.xml",
                     "docProps/core.xml")]

    theme = base.read("ppt/theme/theme1.xml").decode("utf-8")
    theme = re.sub(r"<a:(major|minor)Font>.*?</a:\1Font>",
                   lambda m: re.sub(r'<a:(latin|ea) typeface="[^"]*"',
                                    lambda x: '<a:%s typeface="%s"' % (x.group(1), FONT),
                                    m.group(0)),
                   theme, flags=re.S)

    slides, media = [], {}
    for i, spec in enumerate(SLIDES, 1):
        xml, imgs = build(i, spec)
        names = []
        for k, path in enumerate(imgs, 1):
            name = "s%02d_%d.png" % (i, k)
            media[name] = path
            names.append(name)
        slides.append((xml, names))

    n = len(slides)
    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as z:
        for name in keep:
            z.writestr(name, base.read(name))
        z.writestr("ppt/theme/theme1.xml", theme)

        lay = sorted(x for x in keep if re.match(r"ppt/slideLayouts/slideLayout\d+\.xml$", x))
        ct_lay = "".join(
            '<Override PartName="/%s" ContentType="application/vnd.openxmlformats-'
            'officedocument.presentationml.slideLayout+xml"/>' % x for x in lay)
        ct_sld = "".join(
            '<Override PartName="/ppt/slides/slide%d.xml" ContentType="application/vnd.'
            'openxmlformats-officedocument.presentationml.slide+xml"/>' % (i + 1)
            for i in range(n))
        z.writestr("[Content_Types].xml", CT % (ct_lay, ct_sld))
        z.writestr("_rels/.rels", ROOT_RELS)
        z.writestr("docProps/app.xml",
                   '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
                   '<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/'
                   '2006/extended-properties"><Application>Microsoft Office PowerPoint'
                   "</Application><PresentationFormat>ワイド画面</PresentationFormat>"
                   "<Slides>%d</Slides><AppVersion>16.0000</AppVersion></Properties>" % n)

        ids = "".join('<p:sldId id="%d" r:id="rId%d"/>' % (256 + i, i + 2)
                      for i in range(n))
        z.writestr("ppt/presentation.xml",
                   '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
                   '<p:presentation xmlns:a="http://schemas.openxmlformats.org/drawingml'
                   '/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument'
                   '/2006/relationships" xmlns:p="http://schemas.openxmlformats.org/'
                   'presentationml/2006/main" saveSubsetFonts="1">'
                   '<p:sldMasterIdLst><p:sldMasterId id="2147483648" r:id="rId1"/>'
                   "</p:sldMasterIdLst><p:sldIdLst>%s</p:sldIdLst>"
                   '<p:sldSz cx="%d" cy="%d"/><p:notesSz cx="6858000" cy="9144000"/>'
                   "</p:presentation>" % (ids, SW, SH))

        rels = [REL % ("rId1", "slideMaster", "slideMasters/slideMaster1.xml")]
        rels += [REL % ("rId%d" % (i + 2), "slide", "slides/slide%d.xml" % (i + 1))
                 for i in range(n)]
        rels += [REL % ("rId%d" % (n + 2), "presProps", "presProps.xml"),
                 REL % ("rId%d" % (n + 3), "viewProps", "viewProps.xml"),
                 REL % ("rId%d" % (n + 4), "theme", "theme/theme1.xml"),
                 REL % ("rId%d" % (n + 5), "tableStyles", "tableStyles.xml")]
        z.writestr("ppt/_rels/presentation.xml.rels",
                   '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
                   '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/'
                   'relationships">%s</Relationships>' % "".join(rels))

        for i, (xml, names) in enumerate(slides, 1):
            z.writestr("ppt/slides/slide%d.xml" % i, xml)
            r = [REL % ("rId1", "slideLayout", "../slideLayouts/slideLayout1.xml")]
            r += [REL % ("rId%d" % (k + 2), "image", "../media/%s" % nm)
                  for k, nm in enumerate(names)]
            z.writestr("ppt/slides/_rels/slide%d.xml.rels" % i,
                       '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
                       '<Relationships xmlns="http://schemas.openxmlformats.org/package/'
                       '2006/relationships">%s</Relationships>' % "".join(r))

        for name, path in media.items():
            z.write(path, "ppt/media/%s" % name)

    print("スライド %d 枚・図 %d 点 → %s（%.0f KB）"
          % (n, len(media), OUT, os.path.getsize(OUT) / 1024))
    for i, spec in enumerate(SLIDES, 1):
        t = spec.get("title")
        t = t[0] + "…" if isinstance(t, list) else t
        plain = [re.sub(r"\*\*|##", "", b) for b in spec.get("bullets", ())]
        chars = sum(len(b) for b in plain)
        flag = "  ← 150字超" if chars > 150 else ""
        print("  %2d %-9s %s（本文 %d 字）%s"
              % (i, spec.get("eyebrow", spec.get("kind", "")), t, chars, flag))


if __name__ == "__main__":
    main()
