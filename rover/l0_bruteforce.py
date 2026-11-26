# -*- coding: utf-8 -*-
"""L0 総当たり解と L1 解の支持集合の照合（卒論 S8・8.2(6b)）。

2.5 節の等価性は定理1（正規性＝可到達性＋A の正則性）からは出ない
（本研究の A は冪零で det A = 0）。代わりに参考文献 [6] の
「L1 最適解が bang-off-bang なら L0 最適解でもある」に依拠している。
本モジュールはその依拠を**自分の系の数値で検証する**。

定式化は 2.6 節（`openloop_sparse.py`）と同一にする:

    min ||z||_1  s.t.  Phi z = zeta,  ||z||_inf <= u_max

L0 側は「アクチュエータを動かしている**ステップ数**の最小化」とする
（永原の L0 ＝ 入力がゼロでない時間の測度に対応。δv・δω のどちらかが
非ゼロならそのステップは「動かしている」と数える）。これを支持集合の
総当たりで厳密に解き、L1 解の支持集合と濃度を比較する。

    各濃度 k について S ⊂ {0..n-1}, |S|=k を全列挙し、
    S の外をゼロに固定した実行可能性（LP）を判定する。
    最初に実行可能になった k が L0 最小値である。

実行: uv run python rover/l0_bruteforce.py
出力: results/<日付>_l0_support/{l0_support.csv, notes 用の表}
"""

import argparse
import csv
import itertools
import math
import os

import cvxpy as cp
import numpy as np
from scipy.optimize import linprog

from openloop_sparse import discretize, reachability

ZERO_TOL = 1e-6   # これ未満を「厳密にゼロ」とみなす（openloop_sparse と同値）
BANG_TOL = 1e-4   # |u| が u_max からこれ以内なら「飽和（bang）」とみなす


def target(x0, n, h, V):
    """Phi と zeta を返す（2.6 節と同一の終端条件 x_n = 0）。"""
    Ad, Bd = discretize(h, V)
    Phi = reachability(Ad, Bd, n)
    zeta = -np.linalg.matrix_power(Ad, n) @ np.array(x0, dtype=float)
    return Phi, zeta


def solve_l1(Phi, zeta, n, u_max=1.0):
    """L1 最小解と、その支持・bang-off-bang 性を返す。"""
    z = cp.Variable(2 * n)
    prob = cp.Problem(cp.Minimize(cp.norm1(z)),
                      [Phi @ z == zeta, cp.norm_inf(z) <= u_max])
    prob.solve()
    if z.value is None:
        raise RuntimeError(f"L1 求解失敗: status={prob.status}")
    zv = np.asarray(z.value).ravel()
    dv, dw = zv[0::2], zv[1::2]

    def bang_off_bang(u):
        """各成分が 0 か ±u_max のみかを判定する。"""
        return bool(np.all((np.abs(u) < ZERO_TOL)
                           | (np.abs(np.abs(u) - u_max) < BANG_TOL)))

    step_support = [k for k in range(n)
                    if max(abs(dv[k]), abs(dw[k])) >= ZERO_TOL]
    return dict(
        l1_value=float(prob.value),
        dv=dv, dw=dw,
        step_support=step_support,
        n_active_steps=len(step_support),
        dv_zero=int(np.sum(np.abs(dv) < ZERO_TOL)),
        dw_zero=int(np.sum(np.abs(dw) < ZERO_TOL)),
        dv_bob=bang_off_bang(dv),
        dw_bob=bang_off_bang(dw),
    )


def l1_on_support(Phi, zeta, steps, n, u_max=1.0):
    """支持を steps に制限したときの L1 最小値を返す（実行不能なら None）。

    **これが本モジュールの決め手である。** L1 問題は最適値が一意でも
    最適解は一意でないことがあり、ソルバが返す解が疎でなくても
    「同じ L1 値を達成する疎な解が存在する」場合がある。制限付きの
    最小値が制限なしの最適値と一致すれば、L0 最小の支持の上に
    L1 最適解が存在する＝等価性は「存在する」意味で成り立つ。
    """
    cols = [2 * k + j for k in steps for j in (0, 1)]
    if not cols:
        return 0.0 if np.allclose(zeta, 0.0, atol=ZERO_TOL) else None
    y = cp.Variable(len(cols))
    prob = cp.Problem(cp.Minimize(cp.norm1(y)),
                      [Phi[:, cols] @ y == zeta, cp.norm_inf(y) <= u_max])
    prob.solve()
    if y.value is None:
        return None
    return float(prob.value)


def active_steps_lower_bound(x0, n, h=0.1, V=1.0, u_max=1.0):
    """活性ステップ数の厳密な下界（総当たりできない大きい n 用）。

    δv は x にしか効かず（Bd の第1列は [h,0,0]）、終端 δx=0 には
    Σ_k δv_k · h = x0_x が必要。|δv_k| <= u_max なので
    活性ステップ数 >= ceil(|x0_x| / (h · u_max)) が常に成り立つ。
    """
    return math.ceil(abs(float(x0[0])) / (h * u_max) - 1e-12)


def feasible_on(Phi, zeta, steps, u_max=1.0):
    """steps の外をゼロに固定したとき解が存在するかを LP で判定する。"""
    cols = [2 * k + j for k in steps for j in (0, 1)]
    if not cols:
        return bool(np.allclose(zeta, 0.0, atol=ZERO_TOL))
    A_eq = Phi[:, cols]
    res = linprog(c=np.zeros(len(cols)), A_eq=A_eq, b_eq=zeta,
                  bounds=[(-u_max, u_max)] * len(cols), method="highs")
    return bool(res.success)


def brute_force_l0(Phi, zeta, n, u_max=1.0, k_max=None,
                   budget=2_000_000, max_supports=400):
    """動かすステップ数の最小値と、その濃度の支持を**すべて**返す。

    濃度 k を小さい順に試し、実行可能なものが1つ以上見つかった k で止める。
    そのうえで **同じ濃度 k の実行可能な支持を全列挙する**（max_supports で上限）。
    「L0 最小の支持の上に L1 最適解が存在するか」は支持ごとに変わるため、
    最初に見つかった1つだけで判定すると誤る。
    """
    k_max = n if k_max is None else min(k_max, n)
    tried = 0
    for k in range(0, k_max + 1):
        found = []
        for S in itertools.combinations(range(n), k):
            tried += 1
            if tried > budget:
                return dict(k_min=None, supports=[], tried=tried,
                            reason=f"列挙が budget={budget} を超えた")
            if feasible_on(Phi, zeta, S, u_max):
                found.append(list(S))
                if len(found) >= max_supports:
                    break
        if found:
            return dict(k_min=k, supports=found, tried=tried, reason="")
    return dict(k_min=None, supports=[], tried=tried,
                reason="どの濃度でも実行可能解が無い（n 不足）")


def run_case(x0, n, h=0.1, V=1.0, u_max=1.0, k_max=None):
    """1 事例について L1 と総当たり L0 を解き、比較結果を返す。"""
    Phi, zeta = target(x0, n, h, V)
    try:
        l1 = solve_l1(Phi, zeta, n, u_max)
    except RuntimeError as e:
        return dict(x0=x0, n=n, l1_value=float("nan"), l1_active=None,
                    l1_dv_zero=None, l1_dw_zero=None,
                    l1_dv_bob=None, l1_dw_bob=None,
                    l0_k_min=None, l0_support=None, n_l0_supports=0,
                    l1_support=[],
                    l1_on_l0=None, gap=None, exists_sparse=None,
                    lower_bound=active_steps_lower_bound(x0, n, h, V, u_max),
                    agree=None, tried=0,
                    reason=f"L1 実行不能（n が不足）: {e}")
    l0 = brute_force_l0(Phi, zeta, n, u_max, k_max=k_max)
    k_min = l0["k_min"]
    # L0 最小の支持を**すべて**試し、L1 最適値に一致するものがあるかを見る
    l1_on_l0, best_support = None, None
    for S in l0["supports"]:
        v = l1_on_support(Phi, zeta, S, n, u_max)
        if v is not None and (l1_on_l0 is None or v < l1_on_l0):
            l1_on_l0, best_support = v, S
    gap = (None if l1_on_l0 is None else l1_on_l0 - l1["l1_value"])
    return dict(
        x0=x0, n=n,
        l1_value=l1["l1_value"],
        l1_active=l1["n_active_steps"],
        l1_dv_zero=l1["dv_zero"],
        l1_dw_zero=l1["dw_zero"],
        l1_dv_bob=l1["dv_bob"],
        l1_dw_bob=l1["dw_bob"],
        l0_k_min=k_min,
        l0_support=best_support,
        n_l0_supports=len(l0["supports"]),
        l1_support=l1["step_support"],
        l1_on_l0=l1_on_l0,
        gap=gap,
        exists_sparse=(gap is not None and abs(gap) < 1e-6),
        lower_bound=active_steps_lower_bound(x0, n, h, V, u_max),
        agree=(k_min is not None and l1["n_active_steps"] == k_min),
        tried=l0["tried"],
        reason=l0["reason"],
    )


CASES = [
    # (x0, n): n が小さすぎると終端条件に到達できず L1 自体が実行不能になる。
    # 到達可能な範囲で、x 誤差あり/なしの両方を並べる
    ((0.05, 0.05, math.radians(5)), 6),
    ((0.05, 0.05, math.radians(5)), 10),
    ((0.10, 0.10, math.radians(10)), 10),
    ((0.00, 0.20, math.radians(20)), 15),
    ((0.10, 0.20, math.radians(15)), 20),
    ((0.00, 0.10, math.radians(0)), 20),
]

# 2.6 節（図2.5）が等価性の論拠として引いている事例。n=50 は総当たり不能
# （2^50 通り）なので、L1 解の性質と活性ステップ数の厳密な下界だけを見る。
BIG_CASE = ((2.0, 1.0, math.radians(30)), 50)

CSV_FIELDS = ["x0_x", "x0_y", "x0_theta_deg", "n", "l1_value",
              "l1_active_steps", "l0_k_min", "lower_bound",
              "n_l0_minimal_supports", "agree",
              "l1_value_on_l0_support", "gap", "exists_sparse_l1_optimum",
              "l1_dv_zero", "l1_dw_zero", "l1_dv_bob", "l1_dw_bob",
              "l1_support", "l0_support", "tried", "reason"]


def write_outputs(rows, outdir):
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, "l0_support.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_FIELDS)
        for r in rows:
            w.writerow([
                r["x0"][0], r["x0"][1], math.degrees(r["x0"][2]), r["n"],
                f"{r['l1_value']:.4f}", r["l1_active"], r["l0_k_min"],
                r["lower_bound"], r["n_l0_supports"], r["agree"],
                ("" if r["l1_on_l0"] is None else f"{r['l1_on_l0']:.4f}"),
                ("" if r["gap"] is None else f"{r['gap']:.2e}"),
                r["exists_sparse"],
                r["l1_dv_zero"], r["l1_dw_zero"],
                r["l1_dv_bob"], r["l1_dw_bob"],
                " ".join(map(str, r["l1_support"])),
                " ".join(map(str, r["l0_support"] or [])),
                r["tried"], r["reason"],
            ])
    return path


def main():
    ap = argparse.ArgumentParser(description="L0 総当たりと L1 支持集合の照合")
    ap.add_argument("--outdir", default="results/2026-11-26_l0_support")
    ap.add_argument("--k-max", type=int, default=None,
                    help="総当たりする濃度の上限（既定は n）")
    args = ap.parse_args()

    rows = []
    # 2.6 節の事例（総当たり不能・L1 の性質と下界のみ）
    x0b, nb = BIG_CASE
    Phi, zeta = target(x0b, nb, 0.1, 1.0)
    lb = solve_l1(Phi, zeta, nb, 1.0)
    lbound = active_steps_lower_bound(x0b, nb, 0.1, 1.0, 1.0)
    print(f"[2.6節の事例] x0=({x0b[0]},{x0b[1]},30deg) n={nb}  "
          f"L1値={lb['l1_value']:.2f}  活性={lb['n_active_steps']}step  "
          f"活性の下界={lbound}  "
          f"δvゼロ={lb['dv_zero']}/{nb} δωゼロ={lb['dw_zero']}/{nb}  "
          f"δv:bob={lb['dv_bob']} δω:bob={lb['dw_bob']}")
    rows.append(dict(
        x0=x0b, n=nb, l1_value=lb["l1_value"],
        l1_active=lb["n_active_steps"],
        l1_dv_zero=lb["dv_zero"], l1_dw_zero=lb["dw_zero"],
        l1_dv_bob=lb["dv_bob"], l1_dw_bob=lb["dw_bob"],
        l0_k_min=None, l0_support=None, n_l0_supports=0,
        l1_support=lb["step_support"], l1_on_l0=None, gap=None,
        exists_sparse=None, lower_bound=lbound, agree=None, tried=0,
        reason="総当たり不能（n=50）。下界と bang-off-bang 判定のみ"))

    for x0, n in CASES:
        r = run_case(x0, n, k_max=args.k_max)
        rows.append(r)
        print(f"x0=({x0[0]:.2f},{x0[1]:.2f},{math.degrees(x0[2]):.0f}deg) "
              f"n={n:>2}  L1活性 {str(r['l1_active']):>4}step  "
              f"L0最小 {str(r['l0_k_min']):>4}  "
              f"下界 {r['lower_bound']:>2}  "
              f"最小支持 {r['n_l0_supports']:>3}通り  "
              f"疎な最適解の存在 {r['exists_sparse']}  "
              f"δv:bob={r['l1_dv_bob']} δω:bob={r['l1_dw_bob']}"
              + (f"  ← {r['reason']}" if r["reason"] else ""))
    print("\n出力:", write_outputs(rows, args.outdir))


if __name__ == "__main__":
    main()
