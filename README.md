NHK2023でやろうとした、EKF（拡張カルマンフィルタ）を用いた自己位置推定に関して。
間違っている部分があるかもしれません。

# 問題設定
LRFを用いてロボットの位置 $\boldsymbol{x}=[x, y, \theta]^\top$ が得られる。
2つの接地エンコーダとジャイロセンサから $\dot{\boldsymbol{x}}=[v_x, v_y, \omega]^\top$ が得られる。
これら2つの情報を統合して、最も尤もらしい位置 $\boldsymbol{x}$ を推定したい。

# 理論
#### 状態空間モデル（非線形）
```math
\begin{align}
\boldsymbol{x}(k+1) & = f(\boldsymbol{x}(k), \boldsymbol{u}(k)) + \boldsymbol{v}(k) \\
\boldsymbol{y}(k) & = h(\boldsymbol{x}(k)) + \boldsymbol{w}(k)
\end{align}
```
[全方向移動台車のオドメトリ](https://naokichi-tech.hatenablog.com/entry/2023/02/09/232746)「機体中心を向いた接地エンコーダ2個」より、
```math
\begin{align}
v_x & = v_1\cos\theta - v_2\sin\theta \\
v_y & = v_1\sin\theta + v_2\cos\theta
\end{align}
```
これより、
```math
\begin{align}
\boldsymbol{x}(k+1) & = \boldsymbol{x}(k) +
\begin{bmatrix}
v_1\cos\theta - v_2\sin\theta \\
v_1\sin\theta + v_2\cos\theta \\
\omega
\end{bmatrix}
\Delta t + \boldsymbol{v}(k)\\
\boldsymbol{y}(k) & = \boldsymbol{x}(k) + \boldsymbol{w}(k)
\end{align}
```
ここで、$\boldsymbol{v}_t$ はシステム雑音（姿勢 from オドメトリの雑音）、$\boldsymbol{w}_t$ は観測雑音（姿勢 from LRFの雑音）である。
```math
\begin{align}
\boldsymbol{v}_t & \sim \mathcal{N}(\boldsymbol{0}, Q) \\
\boldsymbol{w}_t & \sim \mathcal{N}(\boldsymbol{0}, R)
\end{align}
```
$Q$、$R$は共分散行列である。それぞれロボット停止時の姿勢データから計算して求めることとする。
ヤコビ行列を計算しておく。
```math
\begin{align}
\dfrac{\partial f(\boldsymbol{x}, \boldsymbol{u})}{\partial \boldsymbol{x}} & =
\begin{bmatrix}
1 & 0 & -v_1\sin\theta-v_2\cos\theta \\
0 & 1 & v_1\cos\theta-v_2\sin\theta \\
0 & 0 & 1
\end{bmatrix} \\
\dfrac{\partial h(\boldsymbol{x})}{\partial \boldsymbol{x}} & =
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
= I
\end{align}
```

#### 予測ステップ
```math
\begin{align}
\text{事前状態推定値：} \quad\hat{\boldsymbol{x}}^-(k) & = f(\hat{\boldsymbol{x}}(k-1), \boldsymbol{u}(k-1)) \\
\text{線形近似：} \quad A(k-1) & = \left.\dfrac{\partial f(\boldsymbol{x}, \boldsymbol{u})}{\partial \boldsymbol{x}}\right|_{\boldsymbol{x}=\hat{\boldsymbol{x}}(k-1), \boldsymbol{u}=\boldsymbol{u}(k-1)} \\
C(k) & = \left.\dfrac{\partial h(\boldsymbol{x})}{\partial \boldsymbol{x}}\right|_{\boldsymbol{x}=\hat{\boldsymbol{x}}(k)} = I \\
\text{事前誤差共分散行列：} \quad P^-(k) & = A(k-1)P(k-1)A^\top(k-1) + Q(k-1)
\end{align}
```

#### フィルタリングステップ
```math
\begin{align}
\text{カルマンゲイン行列：} \quad G(k) & = P^-(k)C^\top(k)(C(k)P^-(k)C^\top(k)+R(k))^{-1} \\
& = P^-(k)(P^-(k)+R(k))^{-1} \\
\text{状態推定値：} \quad\hat{\boldsymbol{x}}(k) & = \hat{\boldsymbol{x}}^-(k) + G(k)(\boldsymbol{y}(k)-h(\hat{\boldsymbol{x}}^-(k))) \\
& = \hat{\boldsymbol{x}}^-(k) + G(k)(\boldsymbol{y}(k)-\hat{\boldsymbol{x}}^-(k)) \\
& = (I-G(k))\hat{\boldsymbol{x}}^-(k) + G(k)\boldsymbol{y}(k) \\
\text{事後誤差共分散行列} \quad P(k) & = (I-G(k)C)P^-(k) \\
& = (I-G(k))P^-(k)
\end{align}
```

# 疑問点など
- 今回の場合、入力 $\boldsymbol{u}$ は何に相当するか。
- 今回の場合 、$Q$ や $R$ は時間に対して不変として良いか。
- 姿勢 from オドメトリの雑音や姿勢 from LRFの雑音は本当に白色か。
- LRFのみ、オドメトリのみに比べてどう性能が良くなったか、定量的な検討の余地あり。

# 参考文献
- 足立修一・丸田一郎、カルマンフィルタの基礎。
