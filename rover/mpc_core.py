# -*- coding: utf-8 -*-
"""ローリングホライズン MPC の純粋ロジック（ROS非依存）。

follower_core.py（経路射影・Kanayama誤差・ゴール判定）と組み合わせて使う。
L2（二次コスト）と L1（λ‖u‖₁、Maximum Hands-off）を reg で切替。
求解時間は bench_qp.py で実機検証済み（N≤30 が 0.1s 周期内）。

設計:
  - 状態 z = (x_e, y_e, θ_e)  … follower_core.tracking_error（ロボット座標系）
  - 入力 δu = (v - v_r, ω - w_r) … 参照速度まわりの偏差
  - 誤差ダイナミクス（参照軌道まわりの線形化）を 1次オイラーで離散化:
      ẋ_e =  w_r·y_e - δv
      ẏ_e = -w_r·x_e + v_r·θ_e
      θ̇_e = -δω
  - 入力制約は実速度 (v, ω) に課す: |v_r+δv|≤v_max, |w_r+δw|≤w_max
  - 終端は等式制約でなくソフトコスト Qf（実行不可能回避）

毎ステップ A,B,z0,(v_r,w_r) をパラメータ更新して同一問題を再求解（DPP）。
"""

import numpy as np

try:
    import cvxpy as cp
except ImportError:  # pragma: no cover
    raise SystemExit("cvxpy が必要です: pip install --user cvxpy osqp")


def error_dynamics(v_r, w_r, ts):
    """参照速度 (v_r,w_r)・周期 ts での離散化誤差ダイナミクス (A,B)。"""
    A_c = np.array([[0.0, w_r, 0.0],
                    [-w_r, 0.0, v_r],
                    [0.0, 0.0, 0.0]])
    B_c = np.array([[-1.0, 0.0],
                    [0.0, 0.0],
                    [0.0, -1.0]])
    A = np.eye(3) + ts * A_c
    B = ts * B_c
    return A, B


class MPCFollower:
    """経路追従 MPC。command() に現在誤差と参照速度を渡すと (v, ω) を返す。"""

    def __init__(self, N=15, ts=0.1, reg="l2", lam=1.0,
                 v_max=0.15, w_max=2.0,
                 q=(10.0, 10.0, 1.0), r=(1.0, 1.0), qf_scale=10.0,
                 move_suppress=0.0):
        self.N = N
        self.ts = ts
        self.reg = reg
        self.v_max = v_max
        self.w_max = w_max
        self.move_suppress = move_suppress

        nx, nu = 3, 2
        # --- パラメータ（毎ステップ更新） ---
        self.z0 = cp.Parameter(nx)
        self.A = cp.Parameter((nx, nx))
        self.B = cp.Parameter((nx, nu))
        self.u_nom = cp.Parameter(nu)        # (v_r, w_r)
        # 移動抑制（Δu率ペナルティ）用: 前ステップ実適用の δu。実機の求解/通信
        # 遅延下で L1 の bang-bang が限界振動（チャタ）化するのを抑える。
        self.du_prev = cp.Parameter(nu, value=np.zeros(nu)) if move_suppress > 0 else None
        self._last_du = np.zeros(nu)
        # --- 変数 ---
        z = cp.Variable((nx, N + 1))
        du = cp.Variable((nu, N))            # 参照まわりの入力偏差

        Q = np.diag(q)
        R = np.diag(r)
        Qf = qf_scale * Q
        u_max = np.array([v_max, w_max])

        cost = 0
        constr = [z[:, 0] == self.z0]
        for k in range(N):
            cost += cp.quad_form(z[:, k], Q)
            if reg == "l1":
                # Maximum Hands-off: 参照速度からの補正 δu の L1 を最小化。
                # → 補正をゼロに張り付かせ、巡航は維持したまま「補正を打つ
                #   瞬間だけ動かす」スパース操舵を得る（オープンループ版78%ゼロと整合）。
                cost += lam * cp.norm1(du[:, k])
            else:
                cost += cp.quad_form(du[:, k], R)
            if move_suppress > 0:
                # Δu = δu_k - δu_{k-1}（k=0 は前回適用値）の二次ペナルティ
                prev = self.du_prev if k == 0 else du[:, k - 1]
                cost += move_suppress * cp.sum_squares(du[:, k] - prev)
            constr += [z[:, k + 1] == self.A @ z[:, k] + self.B @ du[:, k]]
            # 実速度の box 制約
            constr += [cp.abs(self.u_nom + du[:, k]) <= u_max]
        cost += cp.quad_form(z[:, N], Qf)

        self.prob = cp.Problem(cp.Minimize(cost), constr)
        self.du = du
        self.z = z
        self.last_solve_s = 0.0

    def command(self, x_e, y_e, th_e, v_r, w_r=0.0):
        """現在の誤差と参照速度から最適な (v, ω) を返す。求解失敗時は None。"""
        import time
        self.z0.value = np.array([x_e, y_e, th_e])
        self.u_nom.value = np.array([v_r, w_r])
        A, B = error_dynamics(v_r, w_r, self.ts)
        self.A.value = A
        self.B.value = B
        if self.du_prev is not None:
            self.du_prev.value = self._last_du
        t0 = time.perf_counter()
        try:
            self.prob.solve(solver=cp.OSQP, warm_start=True)
        except cp.error.SolverError:
            self.last_solve_s = time.perf_counter() - t0
            return None
        self.last_solve_s = time.perf_counter() - t0
        if self.du.value is None:
            return None
        dv, dw = self.du.value[:, 0]
        self._last_du = np.array([dv, dw])
        v = float(np.clip(v_r + dv, -self.v_max, self.v_max))
        w = float(np.clip(w_r + dw, -self.w_max, self.w_max))
        return v, w
