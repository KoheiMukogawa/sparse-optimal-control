# -*- coding: utf-8 -*-
"""実験結果のサマリーチャート生成（研究記録用・ダークテーマ）。

出力先は環境変数 OBSIDIAN_CHART_DIR で指定する。未指定なら
results/obsidian_charts/ に出す（PC が変わっても動くようにするため、
以前ハードコードしていた /mnt/c/Users/.../obsidian は既定にしない）。

  OBSIDIAN_CHART_DIR=/mnt/c/Users/<user>/Documents/obsidian/... \
      uv run python scripts/obsidian_charts.py
"""
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT = Path(__file__).resolve().parent.parent
OUT = Path(os.environ.get("OBSIDIAN_CHART_DIR",
                          ROOT / "results" / "obsidian_charts"))
OUT.mkdir(parents=True, exist_ok=True)

# 日本語フォント。japanize_matplotlib があればそれで済む（mac/Linux 共通）。
# 無い場合だけ Windows 側のフォントを順に試す（WSL のときのみ当たる）。
try:
    import japanize_matplotlib  # noqa: F401
except ImportError:
    for f in ["/mnt/c/Windows/Fonts/NotoSansJP-VF.ttf",
              "/mnt/c/Windows/Fonts/meiryo.ttc",
              "/System/Library/Fonts/ヒラギノ角ゴシック W3.ttc"]:
        try:
            font_manager.fontManager.addfont(f)
            plt.rcParams["font.family"] = font_manager.FontProperties(
                fname=f).get_name()
            break
        except Exception:
            continue

# パレット（dark surface #1a1a19 で検証済み）
SURF, INK, SEC, MUT = "#1a1a19", "#ffffff", "#c3c2b7", "#898781"
GRID, BASE = "#2c2c2a", "#383835"
C_KAN, C_L2, C_L1 = "#3987e5", "#199e70", "#c98500"  # 系列色は全チャート固定

plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF,
    "text.color": SEC, "axes.labelcolor": MUT,
    "xtick.color": MUT, "ytick.color": MUT,
    "axes.edgecolor": BASE, "font.size": 11,
})

def style(ax, xgrid=False, ygrid=False):
    for s in ["top", "right", "left"]:
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color(BASE)
    if xgrid:
        ax.grid(axis="x", color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
    if ygrid:
        ax.grid(axis="y", color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
    ax.tick_params(length=0)

METHODS = ["Kanayama", "L2-MPC", "L1-MPC"]
COLORS = [C_KAN, C_L2, C_L1]

# ── 1. 直線2m 横ずれ（外部計測） ──────────────────────────
fig, ax = plt.subplots(figsize=(7, 3.4), dpi=200)
vals = [14.5, 4.0, 0.0]
labels = ["14〜15 cm", "4 cm", "0 cm"]
y = range(len(METHODS))
ax.barh(y, vals, height=0.55, color=COLORS)
ax.set_yticks(y, METHODS)
ax.invert_yaxis()
for i, (v, t) in enumerate(zip(vals, labels)):
    ax.text(v + 0.3, i, t, va="center", color=INK, fontsize=11, fontweight="bold")
ax.set_xlim(0, 17.5)
style(ax, xgrid=True)
ax.spines["bottom"].set_visible(False)
ax.set_xticks([0, 5, 10, 15])
ax.set_xlabel("横ずれ [cm]（小さいほど高精度）")
fig.suptitle("床走行 直線2m — 経路からの横ずれ（外部計測・真値）",
             color=INK, fontsize=13, x=0.005, ha="left")
fig.text(0.005, 0.865, "L1-MPC が物理最良（0cm）。odom基準RMSEとは序列が逆転＝外部計測が必須  ·  2026-06-13, n=1",
         color=MUT, fontsize=9)
fig.tight_layout(rect=(0, 0, 1, 0.82))
fig.savefig(OUT / "直線2m_横ずれ比較.png", facecolor=SURF, bbox_inches="tight")
plt.close(fig)

# ── 2. L字コーナーのチャタリング指標（小型マルチプル） ──────
fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.1), dpi=200)
panels = [
    ("総操舵量 Σ|u|（小さいほど滑らか）", [4.88, 4.54, 20.71], "{:.1f}", 24.5),
    ("操舵ゼロ率 ω0率 [%]（大きいほどスパース）", [65, 88, 50], "{:.0f}%", 100),
]
for ax, (title, vals, fmt, xmax) in zip(axes, panels):
    y = range(len(METHODS))
    ax.barh(y, vals, height=0.55, color=COLORS)
    ax.set_yticks(y, METHODS)
    ax.invert_yaxis()
    for i, v in enumerate(vals):
        ax.text(v + xmax * 0.015, i, fmt.format(v), va="center",
                color=INK, fontsize=10.5, fontweight="bold")
    ax.set_xlim(0, xmax)
    style(ax, xgrid=True)
    ax.spines["bottom"].set_visible(False)
    ax.set_title(title, color=SEC, fontsize=10.5, loc="left", pad=8)
fig.suptitle("床走行 L字 — L1-MPC のチャタリング（bang-bang）指標", color=INK,
             fontsize=13, x=0.005, ha="left")
fig.text(0.005, 0.855, "L1はΣ|u|が約4.6倍・ω符号反転15回（L2は0回）＝コーナーでジグザグ  ·  2026-06-13 bag実測",
         color=MUT, fontsize=9)
fig.tight_layout(rect=(0, 0, 1, 0.83))
fig.savefig(OUT / "L字_チャタ指標比較.png", facecolor=SURF, bbox_inches="tight")
plt.close(fig)

# ── 3. 求解時間 p95（RPi実機・走行中） ────────────────────
fig, ax = plt.subplots(figsize=(7, 3.4), dpi=200)
courses = ["直線2m", "L字"]
l2 = [43.6, 46.6]
l1 = [49.8, 89.7]
x = [0, 1]
w = 0.3
b1 = ax.bar([i - w / 2 - 0.01 for i in x], l2, width=w, color=C_L2, label="L2-MPC")
b2 = ax.bar([i + w / 2 + 0.01 for i in x], l1, width=w, color=C_L1, label="L1-MPC")
for bars in (b1, b2):
    for r in bars:
        ax.text(r.get_x() + r.get_width() / 2, r.get_height() + 2,
                f"{r.get_height():.0f}", ha="center", color=INK,
                fontsize=10.5, fontweight="bold")
ax.axhline(100, color=MUT, linewidth=1.2, linestyle=(0, (4, 3)))
ax.text(1.28, 102, "制御周期 0.1s の上限", color=MUT, fontsize=9, ha="right")
ax.set_xticks(x, courses)
ax.set_ylim(0, 118)
ax.set_ylabel("QP求解時間 p95 [ms]")
style(ax, ygrid=True)
ax.legend(frameon=False, labelcolor=SEC, loc="lower left",
          bbox_to_anchor=(0, 1.0), ncol=2, fontsize=10, borderaxespad=0)
fig.suptitle("走行中のQP求解時間 p95（RPi4 実機）", color=INK, fontsize=13,
             x=0.005, ha="left")
fig.text(0.005, 0.875, "全条件で0.1s周期内＝リアルタイム実行可能。Kanayamaは解析的FBで≈0のため省略  ·  2026-06-13",
         color=MUT, fontsize=9)
fig.tight_layout(rect=(0, 0, 1, 0.80))
fig.savefig(OUT / "求解時間p95比較.png", facecolor=SURF, bbox_inches="tight")
plt.close(fig)

print("done:", sorted(p.name for p in OUT.glob("*.png")))
