# 研究会 卒論概要（LaTeX）

2026-10 の研究会で提出する**卒論概要**のソースを置く。
発表スライドとは別の成果物である（指導教員からの指示、2026-10-13）。

## 仕様

| 項目 | 内容 |
|---|---|
| 形式 | LaTeX（**platex + dvipdfmx**。pdflatex ではない） |
| 書式 | `jsmepaper_robomec.cls`（日本機械学会ロボメク書式）。**2段組み固定**（cls 394行目に `\twocolumn`） |
| 分量 | **2段組みで1ページまたは2ページ** |
| 用紙 | A4、9pt |

> 指導教員は当初「4ページ〜8ページ」と伝えていたが、**1〜2ページに変更された**（2026-10-13）。

## このディレクトリのファイル

**書式ファイルと参考サンプルは `.gitignore` で追跡していない。**
本体リポジトリが public であり、サンプルには他の学生の氏名・学籍番号・
研究室の写真が含まれるためである。**公開リポジトリに入れないこと。**

追跡しないファイル（指導教員から配布・ローカルにのみ存在）:

- `jsmepaper_robomec.cls` — 書式本体
- `a4jsme.sty` / `jsadd_robomec.sty` / `jsme10.sty` / `jsmefont.sty` — 付属スタイル
- `abstract.tex` / `abstract.pdf` ほか `abstract.*` — **先輩の完成済みサンプル**（書式と中身の実例）
- `fig/` — サンプルが使う図

失った場合は指導教員に再配布を依頼する。

## サンプルから読み取れる構成（1ページに収まっている実例）

題名 → 著者名（学籍番号）と指導教員名 → 本文6節 → 参考文献。

1. 研究目的
2. 提案手法
3. システム概要
4. 実験方法
5. 実験結果
6. おわりに
7. 参考文献

図2点（機体写真・結果グラフ）＋表1点で、本文はおよそ 2000 字。

## ビルド

```sh
cd docs/abstract
platex abstract.tex && platex abstract.tex   # 相互参照のため2回
dvipdfmx abstract.dvi
```

必要な TeX パッケージ（Ubuntu）:

```sh
sudo apt-get install -y texlive-lang-japanese texlive-latex-extra \
    texlive-latex-recommended texlive-fonts-recommended texlive-pictures
```

`abstract.tex` が読み込む主なパッケージ: epic, eepic, graphicx, bm, url, here,
caption, fancyhdr, subfigure, amsmath, ascmac, geometry。

## 図の出所

本研究の図は `rover/fig_style.py`（Okabe-Ito 配色）で生成したものを使う。
卒論本文と同じ図を流用する場合は `results/2026-08-02_thesis_figs2/` 以下を参照する。
