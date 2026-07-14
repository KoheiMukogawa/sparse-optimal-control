# -*- coding: utf-8 -*-
"""カンペ Markdown → 印刷用PDF（日本語フォント埋め込み）。

使い方:  uv run python docs/build_kanpe_pdf.py
依存: reportlab, japanize-matplotlib（IPAexゴシックのttfを借用）。
元のmd（docs/カンペ_第2回研究会発表.md）を読み、同名の .pdf を出力する。
"""
import html, os, re, glob
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.enums import TA_LEFT
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, HRFlowable)
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.colors import HexColor

HERE = os.path.dirname(os.path.abspath(__file__))
MD  = os.path.join(HERE, "カンペ_第2回研究会発表.md")
PDF = os.path.join(HERE, "カンペ_第2回研究会発表.pdf")

# IPAexゴシックの場所（japanize-matplotlib同梱を利用）
cands = glob.glob(os.path.join(HERE, "..", ".venv", "lib", "*", "site-packages",
                               "japanize_matplotlib", "fonts", "ipaexg.ttf"))
FONT = cands[0]
pdfmetrics.registerFont(TTFont("IPA", FONT))
pdfmetrics.registerFontFamily("IPA", normal="IPA", bold="IPA", italic="IPA", boldItalic="IPA")

TEAL="#1F6F8B"; ORANGE="#E07A3F"; INK="#23292F"; GRAY="#6B7680"

def st(name, size, color=INK, lead=None, before=0, after=2, left=0, bold=False):
    return ParagraphStyle(name, fontName="IPA", fontSize=size, textColor=HexColor(color),
                          leading=lead or size*1.45, spaceBefore=before, spaceAfter=after,
                          leftIndent=left, alignment=TA_LEFT)

S = {
 "title": st("title",17,TEAL,before=0,after=8),
 "h2":    st("h2",12.5,TEAL,before=11,after=4),
 "note":  st("note",8.8,GRAY,before=1,after=1,left=2),
 "sub":   st("sub",10,ORANGE,before=4,after=1),
 "b":     st("b",9.6,INK,before=0,after=2,left=10),
 "b2":    st("b2",9.3,INK,before=0,after=2,left=22),
}

def inline(text):
    text = html.escape(text)
    text = re.sub(r"\*\*(.+?)\*\*", r'<font color="%s">\1</font>' % TEAL, text)
    return text

def main():
    lines = open(MD, encoding="utf-8").read().split("\n")
    doc = SimpleDocTemplate(PDF, pagesize=A4, topMargin=15*mm, bottomMargin=15*mm,
                            leftMargin=16*mm, rightMargin=16*mm,
                            title="カンペ 第2回研究会発表")
    flow=[]
    for ln in lines:
        raw=ln.rstrip()
        if not raw.strip():
            continue
        if raw.startswith("# "):
            flow.append(Paragraph(inline(raw[2:]), S["title"]))
            flow.append(HRFlowable(width="100%", thickness=1.2, color=HexColor(TEAL), spaceAfter=4))
        elif raw.startswith("## "):
            flow.append(HRFlowable(width="100%", thickness=0.5, color=HexColor("#D6DEE2"), spaceBefore=8, spaceAfter=2))
            flow.append(Paragraph(inline(raw[3:]), S["h2"]))
        elif raw.startswith("> "):
            flow.append(Paragraph(inline(raw[2:]), S["note"]))
        elif re.match(r"^\*\*.+\*\*$", raw.strip()):
            flow.append(Paragraph(inline(raw.strip().strip("*")), S["sub"]))
        elif raw.lstrip().startswith("- "):
            indent = len(raw)-len(raw.lstrip())
            body = raw.lstrip()[2:]
            sty = S["b2"] if indent>=2 else S["b"]
            flow.append(Paragraph("• "+inline(body), sty))
        elif raw.strip()=="---":
            flow.append(Spacer(1,3))
        else:
            flow.append(Paragraph(inline(raw), S["b"]))
    doc.build(flow)
    print("wrote", PDF)

if __name__ == "__main__":
    main()
