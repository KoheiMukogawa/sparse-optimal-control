# -*- coding: utf-8 -*-
"""論文解説スライド用の図を、発表デッキの配色で生成する。

使い方: uv run python docs/make_paper_figs.py
依存: matplotlib, numpy, japanize-matplotlib
出力: docs/論文解説_figs/*.png （スライドに貼った図の元データ）
"""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import japanize_matplotlib  # noqa
from matplotlib.patches import Circle, Polygon

import os
FIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "論文解説_figs")
os.makedirs(FIG, exist_ok=True)

TEAL="#1F6F8B"; TEALDK="#124659"; ORANGE="#E07A3F"
INK="#23292F"; GRAY="#8A959E"; GRID="#DCE2E6"
plt.rcParams.update({"font.size":13,"axes.edgecolor":"#9aa6ae","axes.linewidth":1.0,
                     "axes.titlesize":14,"figure.facecolor":"white","axes.unicode_minus":False})

def despine(ax, keep=("left","bottom")):
    for s in ["top","right","left","bottom"]:
        ax.spines[s].set_visible(s in keep)

# ---------- 1) 差別化：従来(L2) vs スパース(L1) ----------
fig,(a1,a2)=plt.subplots(2,1,figsize=(7.3,4.5),sharex=True)
t=np.linspace(0,5,400)
u2=0.55*np.exp(-0.35*t)*np.cos(2.1*t)+0.12*np.exp(-0.2*t)
a1.axhline(0,color=GRAY,lw=0.8)
a1.plot(t,u2,color=TEAL,lw=2.4); a1.fill_between(t,u2,0,color=TEAL,alpha=0.12)
a1.set_title("従来の最適制御（L2：なめらか）",color=TEAL,fontweight="bold",loc="left")
a1.text(0.98,0.9,"常に少しずつ動かし続ける（停止しない）",transform=a1.transAxes,
        ha="right",va="top",color=INK,fontsize=12)
a1.set_ylabel("操舵 入力",color=INK); a1.set_ylim(-0.7,0.95); a1.set_yticks([]); despine(a1,("left",))
ts=np.arange(0,5,0.1); u1=np.zeros_like(ts)
u1[ts<0.7]=-1.0; u1[(ts>=4.5)&(ts<4.9)]=1.0
a2.axhspan(-0.18,0.18,xmin=0.7/5,xmax=4.5/5,color=TEAL,alpha=0.10)
a2.axhline(0,color=GRAY,lw=0.8)
ml,sl,_=a2.stem(ts,u1,basefmt=" "); plt.setp(sl,color=ORANGE,lw=2.2); plt.setp(ml,color=ORANGE,ms=5)
a2.set_title("スパース最適制御（L1：疎 ＝ 提案手法・論文）",color=ORANGE,fontweight="bold",loc="left")
a2.annotate("ほとんど手放し（操舵ゼロ）",xy=(2.6,0.0),xytext=(2.6,0.62),ha="center",color=TEALDK,
            fontsize=12,arrowprops=dict(arrowstyle="->",color=TEALDK))
a2.annotate("要所だけ\n最大で操舵",xy=(0.35,-1.0),xytext=(1.35,-0.95),ha="left",va="center",
            color=ORANGE,fontsize=11,arrowprops=dict(arrowstyle="->",color=ORANGE))
a2.set_ylabel("操舵 入力",color=INK); a2.set_ylim(-1.45,1.2); a2.set_yticks([-1,0,1])
a2.set_xlabel("時間 →",color=INK); a2.set_xticks([]); despine(a2,("left",))
fig.suptitle("同じ「目標に到達する」でも、入力の使い方が正反対",fontsize=14.5,fontweight="bold",color=INK)
fig.tight_layout(rect=[0,0,1,0.96]); fig.savefig(f"{FIG}/fig_compare.png",dpi=200,bbox_inches="tight"); plt.close(fig)

# ---------- 2) ペナルティ形状 L0 / L1 / L2 ----------
fig,ax=plt.subplots(figsize=(6.6,3.8))
x=np.linspace(-2,2,400)
ax.plot(x,x**2,color=TEAL,lw=2.8,label="L2：u²（2乗）")
ax.plot(x,np.abs(x),color=ORANGE,lw=2.8,label="L1：|u|（絶対値）")
ax.plot([-2,-1e-6],[1,1],color=GRAY,lw=2.4,ls="--",label="L0：動かしたら一律1（手数）")
ax.plot([1e-6,2],[1,1],color=GRAY,lw=2.4,ls="--")
ax.plot(0,0,"o",color=GRAY,ms=8,zorder=5)
ax.annotate("0で『カクッ』と折れる\n→ ぴったり0が選ばれる",xy=(0.03,0.03),xytext=(0.55,1.55),
            color=ORANGE,fontsize=11.5,ha="left",arrowprops=dict(arrowstyle="->",color=ORANGE))
ax.annotate("0付近がなめらか\n→ 完全には0にしない",xy=(-0.38,0.14),xytext=(-1.98,1.6),
            color=TEAL,fontsize=11.5,ha="left",arrowprops=dict(arrowstyle="->",color=TEAL))
ax.set_xlim(-2,2); ax.set_ylim(-0.18,2.25)
ax.set_xlabel("入力の大きさ  u",color=INK); ax.set_ylabel("ペナルティ（罰）",color=INK)
ax.set_title("「動かすこと」をどう罰するか — L0 / L1 / L2",fontweight="bold",color=INK,loc="left")
ax.axhline(0,color=GRAY,lw=0.8); ax.axvline(0,color=GRID,lw=0.8)
ax.legend(loc="upper center",bbox_to_anchor=(0.5,-0.18),ncol=3,fontsize=9.5,frameon=False)
despine(ax)
fig.tight_layout(); fig.savefig(f"{FIG}/fig_norms.png",dpi=200,bbox_inches="tight"); plt.close(fig)

# ---------- 3) なぜL1で0が増えるか：L2(円) vs L1(ひし形) の幾何 ----------
fig,axes=plt.subplots(1,2,figsize=(8.4,4.2))
def base(ax,title,color):
    ax.set_xlim(-1.5,2.3); ax.set_ylim(-1.5,1.8); ax.set_aspect("equal")
    ax.axhline(0,color=GRAY,lw=0.9); ax.axvline(0,color=GRAY,lw=0.9)
    xs=np.array([-0.6,2.2]); ax.plot(xs,(2-xs)/2,color=INK,lw=1.8,ls="-")  # 制約 u1+2u2=2
    ax.text(2.15,-0.18,"目標到達の制約\n(Φz=ζ)",color=INK,fontsize=9.5,ha="right",va="top")
    ax.set_title(title,color=color,fontweight="bold")
    ax.set_xticks([]); ax.set_yticks([])
    ax.text(2.25,0.06,"u1",color=GRAY,fontsize=11,va="bottom",ha="right")
    ax.text(0.06,1.72,"u2",color=GRAY,fontsize=11,ha="left",va="top")
# L2
ax=axes[0]; base(ax,"L2（円）","#1F6F8B")
r=2/np.sqrt(5); ax.add_patch(Circle((0,0),r,fill=True,fc=TEAL,ec=TEAL,alpha=0.16,lw=2.2))
ax.add_patch(Circle((0,0),r,fill=False,ec=TEAL,lw=2.4))
px,py=0.4,0.8; ax.plot(px,py,"o",color=TEAL,ms=9,zorder=6)
ax.plot([px,px],[0,py],ls=":",color=TEAL,lw=1.6); ax.plot([0,px],[py,py],ls=":",color=TEAL,lw=1.6)
ax.annotate("接点は軸の外\n→ u1,u2 とも 0 でない",xy=(px,py),xytext=(-1.45,-1.35),
            color=TEAL,fontsize=10.5,ha="left",arrowprops=dict(arrowstyle="->",color=TEAL))
# L1
ax=axes[1]; base(ax,"L1（ひし形）","#E07A3F")
dia=Polygon([(1,0),(0,1),(-1,0),(0,-1)],closed=True,fc=ORANGE,ec=ORANGE,alpha=0.16,lw=2.2)
ax.add_patch(dia); ax.add_patch(Polygon([(1,0),(0,1),(-1,0),(0,-1)],closed=True,fill=False,ec=ORANGE,lw=2.4))
ax.plot(0,1,"o",color=ORANGE,ms=10,zorder=6)
ax.annotate("接点が『角』＝軸上\n→ u1 が厳密に 0（疎）",xy=(0,1),xytext=(-1.45,-1.35),
            color=ORANGE,fontsize=10.5,ha="left",arrowprops=dict(arrowstyle="->",color=ORANGE))
for a in axes:
    for s in ["top","right","left","bottom"]: a.spines[s].set_visible(False)
fig.suptitle("『一番近い点』が、L1ではちょうど角（＝軸の上）で当たる",fontsize=13.5,fontweight="bold",color=INK)
fig.tight_layout(rect=[0,0,1,0.95]); fig.savefig(f"{FIG}/fig_geom.png",dpi=200,bbox_inches="tight"); plt.close(fig)

# ---------- 4) ソフト閾値（不感帯）：小さい入力は0に潰れる ----------
fig,ax=plt.subplots(figsize=(5.2,3.6))
x=np.linspace(-3,3,400); lam=1.0
S=np.sign(x)*np.maximum(np.abs(x)-lam,0)
ax.plot(x,x,ls="--",color=GRAY,lw=1.8,label="そのまま（y=x）")
ax.plot(x,S,color=ORANGE,lw=2.8,label="L1の答え（ソフト閾値）")
ax.axvspan(-lam,lam,color=TEAL,alpha=0.10)
ax.text(0,2.3,"不感帯\n小さい入力は0",color=TEALDK,fontsize=10.5,ha="center",va="center")
ax.annotate("",xy=(-lam,-2.6),xytext=(lam,-2.6),arrowprops=dict(arrowstyle="<->",color=TEALDK))
ax.axhline(0,color=GRAY,lw=0.8); ax.axvline(0,color=GRID,lw=0.8)
ax.set_xlim(-3,3); ax.set_ylim(-3,3); ax.set_xlabel("入力（生の値）",color=INK); ax.set_ylabel("制御に使う値",color=INK)
ax.set_title("L1は小さい入力を『0に潰す』",fontweight="bold",color=INK,loc="left")
ax.legend(loc="lower right",fontsize=9.5,framealpha=0.95); despine(ax)
fig.tight_layout(); fig.savefig(f"{FIG}/fig_soft.png",dpi=200,bbox_inches="tight"); plt.close(fig)

# ---------- 5) bang-off-bang：最適解は ±最大 か 0 ----------
fig,ax=plt.subplots(figsize=(7.2,3.6))
t=np.linspace(0,5,1000)
u=np.zeros_like(t)
u[t<1.0]=1.0; u[(t>=3.4)&(t<4.2)]=-1.0
ax.axhspan(-0.12,0.12,xmin=1.0/5,xmax=3.4/5,color=TEAL,alpha=0.12)
ax.axhspan(-0.12,0.12,xmin=4.2/5,xmax=1.0,color=TEAL,alpha=0.12)
ax.plot(t,u,color=ORANGE,lw=3,drawstyle="steps-post")
ax.fill_between(t,u,0,step="post",color=ORANGE,alpha=0.15)
for y,lab in [(1,"＋最大"),(0,"停止 0"),(-1,"−最大")]:
    ax.axhline(y,color=GRID,lw=0.8); ax.text(5.05,y,lab,va="center",ha="left",color=INK,fontsize=11)
ax.text(2.2,0.4,"手放し（hands-off）",color=TEALDK,fontsize=12,ha="center",fontweight="bold")
ax.text(4.55,0.4,"手放し",color=TEALDK,fontsize=11,ha="center")
ax.set_xlim(0,5.0); ax.set_ylim(-1.5,1.5); ax.set_yticks([-1,0,1])
ax.set_xlabel("時間 →",color=INK); ax.set_ylabel("最適な操舵",color=INK); ax.set_xticks([])
ax.set_title("最適なスパース制御 ＝ ±最大 か 0 の3値（bang-off-bang）",fontweight="bold",color=INK,loc="left")
despine(ax,("left","bottom"))
fig.tight_layout(); fig.savefig(f"{FIG}/fig_bob.png",dpi=200,bbox_inches="tight"); plt.close(fig)

print("OK: fig_compare, fig_norms, fig_geom, fig_soft, fig_bob")
