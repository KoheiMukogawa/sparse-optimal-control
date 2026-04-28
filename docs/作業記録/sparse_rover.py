
スパース最適制御 シミュレーション作業報告書
作業日：2026年4月28日
---
概要
ライトローバー（Raspberry Pi 4搭載）の自立走行を目標に、
センサなしの状態でできる取り組みとして、
スパース最適制御のシミュレーションを実施した。
---
1. 理論整理
1.1 ロボットの運動モデル（非線形）
差動二輪ロボットの運動方程式：
$$\dot{x} = v\cos\theta$$
$$\dot{y} = v\sin\theta$$
$$\dot{\theta} = \omega$$
状態変数：$\mathbf{x} = [\delta x,\ \delta y,\ \delta\theta]^\top$（目標軌道からのずれ）
制御入力：$\mathbf{u} = [\delta v,\ \delta\omega]^\top$（速度・角速度の補正）
1.2 線形化
動作点（$\theta_0 = 0$、速度 $V$ でまっすぐ走っている状態）まわりでテイラー展開：
$$\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}$$
$$A = \begin{bmatrix} 0 & 0 & 0 \ 0 & 0 & V \ 0 & 0 & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 1 & 0 \ 0 & 0 \ 0 & 1 \end{bmatrix}$$
非線形になる理由：
$\cos\theta$、$\sin\theta$ が曲線（直線ではない）
$v \cdot \cos\theta$ が変数×変数の掛け算
1.3 離散時間への変換
サンプリング時間 $h$ を使って連続時間モデルを離散化：
$$\mathbf{x}[k+1] = A_d\mathbf{x}[k] + B_d\mathbf{u}[k]$$
$$A_d = \begin{bmatrix} 1 & 0 & 0 \ 0 & 1 & h \ 0 & 0 & 1 \end{bmatrix}, \quad B_d = \begin{bmatrix} h & 0 \ 0 & h^2/2 \ 0 & h \end{bmatrix}$$
1.4 Φ と ζ の構築
状態遷移を $n$ ステップ展開し、終端条件 $\mathbf{x}[n] = 0$ を適用：
$$\Phi\mathbf{z} = \zeta$$
$$\Phi = \begin{bmatrix} A_d^{n-1}B_d & A_d^{n-2}B_d & \cdots & B_d \end{bmatrix}$$
$$\zeta = -A_d^n\mathbf{x}[0]$$
記号	意味
$\Phi$	各ステップの入力が最終状態に与える影響
$\zeta$	初期状態の自然な流れを打ち消す目標値
$\mathbf{z}$	全ステップの制御入力をまとめたベクトル
1.5 スパース最適制御問題
$$\min_{\mathbf{z}} |\mathbf{z}|1 \quad \text{s.t.} \quad \Phi\mathbf{z} = \zeta,\quad |\mathbf{z}|\infty \leq 1$$
項	役割
$\min |\mathbf{z}|_1$	L1ノルム最小化→制御入力をスパースに
$\Phi\mathbf{z} = \zeta$	目標地点への到達を保証
$|\mathbf{z}|_\infty \leq 1$	モータの飽和制約
---
2. シミュレーション
2.1 シミュレーション条件
パラメータ	値
サンプリング時間 $h$	0.1 秒
基準速度 $V$	1.0 m/s
ステップ数 $n$	50（5秒間）
初期状態 $\delta x$	2.0 m
初期状態 $\delta y$	1.0 m
初期状態 $\delta\theta$	30度
目標状態	$(0, 0, 0)$
2.2 シミュレーション結果
項目	結果
最終状態 $\delta x$	≈ 0.0000 m
最終状態 $\delta y$	≈ 0.0000 m
最終状態 $\delta\theta$	≈ 0.0000 deg
$\delta\omega$ ゼロ入力ステップ数	39 / 50（78%）
L1ノルム（目的関数値）	30.65
![シミュレーション結果](sparse_rover_simulation.png)
2.3 考察
角速度入力 $\delta\omega$ の78%がゼロ → スパース性を確認
目標地点に誤差ほぼ0で到達 → 制御の有効性を確認
今回はオープンループ制御のため、外乱があるとエラーが蓄積する
解決策としてモデル予測制御（MPC）との組み合わせが有効
---
3. 実装コード
`sparse_rover.py` 参照
使用ライブラリ
ライブラリ	用途
numpy	行列計算
cvxpy	L1最小化（最適化ソルバー）
matplotlib	グラフ描画
実行方法
```bash
pip install numpy cvxpy matplotlib
python sparse_rover.py
```
---
4. 今後の課題
[ ] L2制御との比較（スパース性の違いを可視化）
[ ] MPC（モデル予測制御）との組み合わせ実装
[ ] 実機（ライトローバー）への実装
[ ] センサ（LiDAR等）を追加して自己位置推定と組み合わせ
---
参考文献
永原正章，「最適制御とスパースモデリング」，電子情報通信学会 基礎・境界ソサイエティ Fundamentals Review，Vol.10，No.3，2017
