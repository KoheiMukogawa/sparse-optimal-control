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

# 色は意味づけを持たせる: ベースライン=灰、標準最適制御=青緑、
# 破綻している素のL1=赤、対策後=緑。
COLOR = {
    "kanayama": "#8A959E",
    "l2": "#1F6F8B",
    "l1": "#C1443C",
    "l1_ms2": "#4C956C",
}

# 白黒印刷でも区別がつくよう線種も変える
LINESTYLE = {
    "kanayama": (0, (1, 1.5)),
    "l2": "-",
    "l1": (0, (4, 1.5)),
    "l1_ms2": "-",
}

CONTROL_PERIOD_MS = 100.0  # 制御周期 0.1s（求解時間の判定線）


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
