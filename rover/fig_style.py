# -*- coding: utf-8 -*-
"""卒論の図の共通スタイル。

図ごとに色やラベルを書くと表記ゆれが出るので、条件名・色・線種・
フォント設定をここ1か所に置く。日本語↔英語ラベルの切り替えもここだけで済む。
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import japanize_matplotlib  # noqa: F401,E402

# 条件の表示名（卒論の本文・表と揃えること）
LABEL = {
    "kanayama": "Kanayama（古典FB）",
    "l2": "L2-MPC",
    "l1": "L1-MPC（λ=0.3）",
    "l1_ms2": "L1-MPC＋移動抑制（$w_{ms}$=2.0）",
}

SHORT = {
    "kanayama": "Kanayama",
    "l2": "L2",
    "l1": "L1（素）",
    "l1_ms2": "L1+抑制",
}

# 配色は Okabe-Ito（色覚多様性に配慮した標準パレット）から4色を採る。
# 意味づけ: 古典FB=橙、標準最適制御=青、破綻している素のL1=朱、対策後=緑。
#
# 旧配色（灰 #8A959E / 青緑 #1F6F8B / 赤 #C1443C / 緑 #4C956C）は
# **素のL1(赤)と対策後(緑)の識別が2型色覚で ΔE 6.7** と不十分だった。
# 図5.6「対策前後の対比」はこの2条件の比較そのものなので致命的。
# 現配色は全ペアで最悪 ΔE 11.0（2型）・normal 15.6 と基準を満たす。
# 検証: dataviz スキルの validate_palette.js（light・--pairs all で全項目 PASS）。
# なお #E69F00 は白地に対するコントラストが 2.19 と 3:1 を下回るため、
# **凡例と直接ラベルを必ず併記する**（本文の表にも同じ数値が載る）。
COLOR = {
    "kanayama": "#E69F00",
    "l2": "#0072B2",
    "l1": "#D55E00",
    "l1_ms2": "#009E73",
}

# 白黒印刷でも区別がつくよう線種も変える（色に頼らない二重符号化）
LINESTYLE = {
    "kanayama": (0, (1, 1.5)),
    "l2": "-",
    "l1": (0, (4, 1.5)),
    "l1_ms2": "-",
}

# 模式図（構成図・フロー図）用のインク。データ系列の色とは用途を分ける。
INK = "#2B2B2B"        # 本文相当の濃さ
INK_MUTED = "#6E6E6E"  # 補助線・注記
FILL = "#EDF2F5"       # ボックスの下地
FILL_ACCENT = "#DCE9F2"  # 強調したいボックス

CONTROL_PERIOD_MS = 100.0  # 制御周期 0.1s（求解時間の判定線）

# 遅延スイープ（sweep_delay.py・図5.4）の6条件。上の4色に Okabe-Ito の
# 2色を足した6色で、全ペア検証 PASS（最悪 ΔE 7.6・二重符号化つきで可）。
DELAY_LABEL = {
    "l2": "L2-MPC",
    "l1_lam0.3": "L1（λ=0.3・素）",
    "l1_lam0.3_ms0.5": "L1 λ=0.3・$w_{ms}$=0.5",
    "l1_lam0.3_ms2": "L1 λ=0.3・$w_{ms}$=2.0（採用値）",
    "l1_lam2": "L1（λ=2・素）",
    "l1_lam2_ms0.5": "L1 λ=2・$w_{ms}$=0.5",
}
DELAY_COLOR = {
    "l2": "#0072B2",
    "l1_lam0.3": "#D55E00",
    "l1_lam0.3_ms0.5": "#E69F00",
    "l1_lam0.3_ms2": "#009E73",
    "l1_lam2": "#CC79A7",
    "l1_lam2_ms0.5": "#56B4E9",
}
DELAY_LINESTYLE = {
    "l2": "-",
    "l1_lam0.3": (0, (4, 1.5)),
    "l1_lam0.3_ms0.5": (0, (1, 1.5)),
    "l1_lam0.3_ms2": "-",
    "l1_lam2": (0, (5, 1.5, 1, 1.5)),
    "l1_lam2_ms0.5": (0, (3, 1, 1, 1, 1, 1)),
}
DELAY_MARKER = {
    "l2": "o", "l1_lam0.3": "s", "l1_lam0.3_ms0.5": "^",
    "l1_lam0.3_ms2": "D", "l1_lam2": "v", "l1_lam2_ms0.5": "x",
}


def lam_ramp(lams):
    """λ は順序を持つ量なので、カテゴリ色でなく単一色相の濃淡で表す。

    薄い=小さいλ、濃い=大きいλ。返り値は {lam: color}。
    """
    import matplotlib.colors as mcolors
    vals = sorted(lams)
    norm = mcolors.Normalize(vmin=-0.35, vmax=len(vals) - 1)
    cmap = matplotlib.colormaps["Blues"]
    return {v: cmap(norm(i)) for i, v in enumerate(vals)}


def setup():
    """論文用の描画設定を適用する。"""
    plt.rcParams.update({
        "figure.dpi": 150,
        "savefig.dpi": 150,
        "savefig.bbox": "tight",
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "legend.fontsize": 9,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "axes.axisbelow": True,
    })


def style(cond):
    """条件に対応する color / linestyle / label をまとめて返す。"""
    return dict(color=COLOR[cond], linestyle=LINESTYLE[cond], label=LABEL[cond])
