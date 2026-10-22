# 提出物

指導教員・研究室へ実際に提出した成果物を、提出日つきの名前で置く。
**ここのファイルは「提出したもの」の記録なので、提出後は書き換えない。**

| ファイル | 提出日 | 提出先・用途 | ソース |
|---|---|---|---|
| `2026-10-22_研究会_卒論概要.pdf` | 2026-10-22 | 研究会（卒論概要・2段組み2ページ） | `docs/abstract/gaiyou.tex` |

## 更新のしかた

`docs/abstract/gaiyou.tex` を直して出し直す場合は、ビルドしてからここへコピーする。
**ビルド成果物 `docs/abstract/gaiyou.pdf` は追跡していない**ので、
ここのコピーを更新しないと古いままになる。

```sh
cd docs/abstract
platex gaiyou.tex && platex gaiyou.tex && dvipdfmx gaiyou.dvi
cp gaiyou.pdf "../提出物/<提出日>_<提出先>_<内容>.pdf"
```

発表スライド（`docs/研究会発表_2026-10-22.pptx` など）は従来どおり `docs/` 直下に置く。
