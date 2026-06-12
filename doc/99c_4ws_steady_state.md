# 四轮转向（$\delta_r = k_r\,\delta_f$）闭环稳态跟踪误差与前馈转角推导

## 1 问题设定

基于 `02b_error_cg_4ws.md` 第 5 节的四轮转向误差状态空间：

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B_2\,\mathbf{u} + G\,\dot\theta_{\text{ref}}
$$

状态向量 $\mathbf{x} = [e_1,\;\dot{e}_1,\;e_2,\;\dot{e}_2]^T$，控制量 $\mathbf{u} = [\delta_f,\;\delta_r]^T$。

### 1.1 后轮转向比例约束

$$
\delta_r = k_r\,\delta_f
$$

代入后，等效输入矩阵为：

$$
B_2 \begin{bmatrix} \delta_f \\ k_r\,\delta_f \end{bmatrix}
= \left(B_2 \begin{bmatrix} 1 \\ k_r \end{bmatrix}\right) \delta_f
= B_{eq}\,\delta_f
$$

其中等效输入矩阵：

$$
B_{eq} = \begin{bmatrix}
0 \\[6pt]
\dfrac{C_f + k_r C_r}{m} \\[6pt]
0 \\[6pt]
\dfrac{l_f C_f - k_r l_r C_r}{I_z}
\end{bmatrix}
$$

### 1.2 状态反馈控制律

$$
\delta_f = -K\mathbf{x} + \delta_{ff} = -k_1 e_1 - k_2 \dot{e}_1 - k_3 e_2 - k_4 \dot{e}_2 + \delta_{ff}
$$

### 1.3 闭环系统

$$
\dot{\mathbf{x}} = (A - B_{eq}K)\,\mathbf{x} + B_{eq}\,\delta_{ff} + G\,\dot\theta_{\text{ref}}
$$

---

## 2 定圆稳态条件

- 曲率恒定：$\kappa = 1/R$
- $\dot\theta_{\text{ref}} = v_x / R$
- 稳态：$\dot{\mathbf{x}} = 0$，$\mathbf{x}_{ss} = [e_{1,ss},\;0,\;e_{2,ss},\;0]^T$

---

## 3 稳态误差求解

### 3.1 稳态方程

令 $\dot{\mathbf{x}} = 0$，$\dot{e}_1 = \dot{e}_2 = 0$，有效方程为第 2 行和第 4 行。


**第 2 行**（$\ddot{e}_1 = 0$）：

$$
0 = \frac{C_f + C_r}{m}\,e_{2,ss}
- \frac{C_f + k_r C_r}{m}\,k_1\,e_{1,ss}
- \frac{C_f + k_r C_r}{m}\,k_3\,e_{2,ss}
+ \frac{C_f + k_r C_r}{m}\,\delta_{ff}
+ \left(-v_x - \frac{l_f C_f - l_r C_r}{m\,v_x}\right)\frac{v_x}{R}
$$

**第 4 行**（$\ddot{e}_2 = 0$）：

$$
0 = \frac{l_f C_f - l_r C_r}{I_z}\,e_{2,ss}
- \frac{l_f C_f - k_r l_r C_r}{I_z}\,k_1\,e_{1,ss}
- \frac{l_f C_f - k_r l_r C_r}{I_z}\,k_3\,e_{2,ss}
+ \frac{l_f C_f - k_r l_r C_r}{I_z}\,\delta_{ff}
- \frac{l_f^2 C_f + l_r^2 C_r}{I_z\,v_x}\cdot\frac{v_x}{R}
$$

### 3.2 化简

**第 2 行** 乘以 $m/(C_f + k_r C_r)$：

$$
0 = \left(\frac{C_f + C_r}{C_f + k_r C_r} - k_3\right) e_{2,ss}
- k_1\,e_{1,ss}
+ \delta_{ff}
+ \frac{m}{C_f + k_r C_r}\left(-v_x - \frac{l_f C_f - l_r C_r}{m\,v_x}\right)\frac{v_x}{R}
$$

扰动项化简：

$$
\frac{m}{C_f + k_r C_r}\left(-v_x - \frac{l_f C_f - l_r C_r}{m\,v_x}\right)\frac{v_x}{R}
= -\frac{m v_x^2}{(C_f + k_r C_r) R} - \frac{l_f C_f - l_r C_r}{(C_f + k_r C_r) R}
$$

**第 4 行** 乘以 $I_z/(l_f C_f - k_r l_r C_r)$：

$$
0 = \left(\frac{l_f C_f - l_r C_r}{l_f C_f - k_r l_r C_r} - k_3\right) e_{2,ss}
- k_1\,e_{1,ss}
+ \delta_{ff}
- \frac{l_f^2 C_f + l_r^2 C_r}{(l_f C_f - k_r l_r C_r) R}
$$

### 3.3 两方程相减消去 $e_{1,ss}$ 和 $\delta_{ff}$

两方程均含 $-k_1 e_{1,ss} + \delta_{ff}$，相减（第 2 行 $-$ 第 4 行）：

$$
0 = \left[\frac{C_f + C_r}{C_f + k_r C_r} - \frac{l_f C_f - l_r C_r}{l_f C_f - k_r l_r C_r}\right] e_{2,ss}
+ \left[-\frac{m v_x^2}{(C_f + k_r C_r) R} - \frac{l_f C_f - l_r C_r}{(C_f + k_r C_r) R}\right]
+ \frac{l_f^2 C_f + l_r^2 C_r}{(l_f C_f - k_r l_r C_r) R}
$$

### 3.4 $e_{2,ss}$ 系数化简

定义 $P = C_f + k_r C_r$，$Q = l_f C_f - k_r l_r C_r$。

$$
\frac{C_f + C_r}{P} - \frac{l_f C_f - l_r C_r}{Q}
= \frac{(C_f + C_r)Q - (l_f C_f - l_r C_r)P}{PQ}
$$

展开分子：

$$
(C_f + C_r)(l_f C_f - k_r l_r C_r) - (l_f C_f - l_r C_r)(C_f + k_r C_r)
$$

$$
= C_f l_f C_f - C_f k_r l_r C_r + C_r l_f C_f - C_r k_r l_r C_r
$$
$$
\quad - l_f C_f^2 - k_r l_f C_f C_r + l_r C_r C_f + k_r l_r C_r^2
$$

$$
= \cancel{C_f^2 l_f} - k_r C_f C_r l_r + C_f C_r l_f - k_r C_r^2 l_r
- \cancel{C_f^2 l_f} - k_r C_f C_r l_f + C_f C_r l_r + k_r C_r^2 l_r
$$

$$
= C_f C_r(l_f + l_r) - k_r C_f C_r(l_r + l_f)
= C_f C_r L (1 - k_r)
$$

因此 $e_{2,ss}$ 系数为：

$$
\frac{C_f C_r L (1 - k_r)}{PQ}
$$

### 3.5 常数项化简

$$
-\frac{m v_x^2}{PR} - \frac{l_f C_f - l_r C_r}{PR} + \frac{l_f^2 C_f + l_r^2 C_r}{QR}
$$

$$
= \frac{1}{R}\left[-\frac{m v_x^2 + l_f C_f - l_r C_r}{P} + \frac{l_f^2 C_f + l_r^2 C_r}{Q}\right]
$$

$$
= \frac{1}{R}\cdot\frac{-(m v_x^2 + l_f C_f - l_r C_r)Q + (l_f^2 C_f + l_r^2 C_r)P}{PQ}
$$

展开分子中与 $mv_x^2$ 无关的部分：

$$
-(l_f C_f - l_r C_r)(l_f C_f - k_r l_r C_r) + (l_f^2 C_f + l_r^2 C_r)(C_f + k_r C_r)
$$

$$
= -(l_f^2 C_f^2 - k_r l_f l_r C_f C_r - l_f l_r C_f C_r + k_r l_r^2 C_r^2)
$$
$$
\quad + l_f^2 C_f^2 + k_r l_f^2 C_f C_r + l_r^2 C_f C_r + k_r l_r^2 C_r^2
$$

$$
= k_r l_f l_r C_f C_r + l_f l_r C_f C_r - \cancel{k_r l_r^2 C_r^2}
+ k_r l_f^2 C_f C_r + l_r^2 C_f C_r + \cancel{k_r l_r^2 C_r^2}
$$

$$
= C_f C_r [l_f l_r(1 + k_r) + k_r l_f^2 + l_r^2]
= C_f C_r [l_f l_r + k_r l_f l_r + k_r l_f^2 + l_r^2]
$$

$$
= C_f C_r [l_r(l_f + l_r) + k_r l_f(l_f + l_r)]
= C_f C_r L (l_r + k_r l_f)
$$

加上 $-mv_x^2 Q$ 项：

分子 $= -m v_x^2 Q + C_f C_r L(l_r + k_r l_f)$

$$
= -m v_x^2 (l_f C_f - k_r l_r C_r) + C_f C_r L(l_r + k_r l_f)
$$

### 3.6 求解 $e_{2,ss}$

$$
\frac{C_f C_r L(1-k_r)}{PQ}\,e_{2,ss} = \frac{m v_x^2 (l_f C_f - k_r l_r C_r) - C_f C_r L(l_r + k_r l_f)}{PQR}
$$

$$
e_{2,ss} = \frac{m v_x^2 (l_f C_f - k_r l_r C_r) - C_f C_r L(l_r + k_r l_f)}{C_f C_r L(1-k_r) R}
$$

分离各项：

$$
\boxed{e_{2,ss} = \frac{m v_x^2 (l_f C_f - k_r l_r C_r)}{C_f C_r L(1-k_r) R} - \frac{l_r + k_r l_f}{(1-k_r) R}}
$$


---

## 4 $k_r = 0$ 时退化验证

令 $k_r = 0$：

$$
e_{2,ss}\big|_{k_r=0} = \frac{m v_x^2 l_f C_f}{C_f C_r L R} - \frac{l_r}{R}
= \frac{l_f m v_x^2}{C_r L R} - \frac{l_r}{R}
$$

与 99b 文档结果完全一致 ✓

---

## 5 前馈转角 $\delta_{ff}$ 的求解

### 5.1 目标：令 $e_{1,ss} = 0$

将 $e_{1,ss} = 0$ 代入第 4 行化简后的方程：

$$
0 = \left(\frac{l_f C_f - l_r C_r}{Q} - k_3\right) e_{2,ss}
+ \delta_{ff}
- \frac{l_f^2 C_f + l_r^2 C_r}{Q R}
$$

解得：

$$
\delta_{ff} = \frac{l_f^2 C_f + l_r^2 C_r}{(l_f C_f - k_r l_r C_r) R}
- \left(\frac{l_f C_f - l_r C_r}{l_f C_f - k_r l_r C_r} - k_3\right) e_{2,ss}
$$

### 5.2 化简

定义 $Q = l_f C_f - k_r l_r C_r$，展开：

$$
\delta_{ff} = \frac{l_f^2 C_f + l_r^2 C_r}{QR}
- \frac{l_f C_f - l_r C_r}{Q}\,e_{2,ss}
+ k_3\,e_{2,ss}
$$

代入 $e_{2,ss}$ 的表达式：

$$
\frac{l_f C_f - l_r C_r}{Q}\,e_{2,ss}
= \frac{(l_f C_f - l_r C_r)[m v_x^2 Q - C_f C_r L(l_r + k_r l_f)]}{C_f C_r L(1-k_r) Q R}
$$

$$
= \frac{(l_f C_f - l_r C_r) m v_x^2}{C_f C_r L(1-k_r) R}
- \frac{(l_f C_f - l_r C_r)(l_r + k_r l_f)}{(1-k_r) Q R}
$$

因此：

$$
\delta_{ff} = \frac{l_f^2 C_f + l_r^2 C_r}{QR}
- \frac{(l_f C_f - l_r C_r) m v_x^2}{C_f C_r L(1-k_r) R}
+ \frac{(l_f C_f - l_r C_r)(l_r + k_r l_f)}{(1-k_r) Q R}
+ k_3\,e_{2,ss}
$$

合并第 1 项和第 3 项（公分母 $(1-k_r)QR$）：

$$
\frac{(1-k_r)(l_f^2 C_f + l_r^2 C_r) + (l_f C_f - l_r C_r)(l_r + k_r l_f)}{(1-k_r) Q R}
$$

展开分子：

$$
(l_f^2 C_f + l_r^2 C_r) - k_r(l_f^2 C_f + l_r^2 C_r)
+ l_f l_r C_f + k_r l_f^2 C_f - l_r^2 C_r - k_r l_f l_r C_r
$$

$$
= l_f^2 C_f + l_r^2 C_r - k_r l_f^2 C_f - k_r l_r^2 C_r
+ l_f l_r C_f + k_r l_f^2 C_f - l_r^2 C_r - k_r l_f l_r C_r
$$

$$
= l_f^2 C_f + l_f l_r C_f - k_r l_r^2 C_r - k_r l_f l_r C_r
$$

$$
= l_f C_f(l_f + l_r) - k_r l_r C_r(l_r + l_f)
= L(l_f C_f - k_r l_r C_r) = LQ
$$

因此第 1+3 项合并为：

$$
\frac{LQ}{(1-k_r) Q R} = \frac{L}{(1-k_r) R}
$$

最终：

$$
\boxed{\delta_{ff} = \frac{L}{(1-k_r)R} - \frac{(l_f C_f - l_r C_r) m v_x^2}{C_f C_r L(1-k_r) R} + k_3\,e_{2,ss}}
$$

等价形式：

$$
\delta_{ff} = \frac{1}{1-k_r}\left[\frac{L}{R} + \frac{m v_x^2}{R}\left(\frac{l_r}{C_f L} - \frac{l_f}{C_r L}\right)\right] + k_3\,e_{2,ss}
$$

> 注：$-(l_f C_f - l_r C_r)/(C_f C_r L) = l_r/(C_f L) - l_f/(C_r L)$


---

## 6 $k_r = 0$ 时退化验证

令 $k_r = 0$：

$$
\delta_{ff}\big|_{k_r=0} = \frac{L}{R} + \frac{m v_x^2}{R}\left(\frac{l_r}{C_f L} - \frac{l_f}{C_r L}\right) + k_3\,e_{2,ss}\big|_{k_r=0}
$$

与 99b 文档结果完全一致 ✓

---

## 7 物理解读

### 7.1 稳态航向误差

$$
e_{2,ss} = \frac{m v_x^2 (l_f C_f - k_r l_r C_r)}{C_f C_r L(1-k_r) R} - \frac{l_r + k_r l_f}{(1-k_r) R}
$$

| 特性 | 说明 |
|---|---|
| 与 $K$ 无关 | 状态反馈无法消除稳态航向误差（单自由度约束 $\delta_r = k_r\delta_f$） |
| $k_r$ 的影响 | 后轮同向转向（$k_r > 0$）可减小航向误差 |
| $k_r = l_r C_r / (l_f C_f)$ 时 | 动力学项分子为零，航向误差仅含几何项 |

### 7.2 前馈转角

$$
\delta_{ff} = \frac{1}{1-k_r}\left[\frac{L}{R} + K_{us}\frac{v_x^2}{R}\right] + k_3\,e_{2,ss}
$$

其中 $K_{us} = \dfrac{m}{L}\left(\dfrac{l_r}{C_f} - \dfrac{l_f}{C_r}\right)$ 为不足转向梯度。

| 项 | 含义 |
|---|---|
| $\dfrac{L}{(1-k_r)R}$ | 等效 Ackermann 转角（后轮同向转向减小前轮需求） |
| $\dfrac{K_{us} v_x^2}{(1-k_r)R}$ | 不足转向补偿（被 $1-k_r$ 缩放） |
| $k_3\,e_{2,ss}$ | 航向误差反馈补偿 |

### 7.3 实际前后轮转角

$$
\delta_f = \delta_{ff} - K\mathbf{x}, \qquad \delta_r = k_r\,\delta_f
$$

稳态时（$e_{1,ss} = 0$）：

$$
\delta_{f,ss} = \delta_{ff} - k_3\,e_{2,ss} = \frac{1}{1-k_r}\left[\frac{L}{R} + K_{us}\frac{v_x^2}{R}\right]
$$

$$
\delta_{r,ss} = k_r\,\delta_{f,ss} = \frac{k_r}{1-k_r}\left[\frac{L}{R} + K_{us}\frac{v_x^2}{R}\right]
$$

---

## 8 一般情况：$e_{1,ss}$ 的表达式

从第 4 行方程（$e_{1,ss} \neq 0$）：

$$
\boxed{e_{1,ss} = \frac{1}{k_1}\left(\delta_{ff} - \delta_{ff}^*\right)}
$$

其中理想前馈：

$$
\delta_{ff}^* = \frac{L}{(1-k_r)R} + \frac{K_{us}\,v_x^2}{(1-k_r)R} + k_3\,e_{2,ss}
$$

---

## 9 总结

### 9.1 稳态航向误差（与反馈增益 $K$ 无关）

$$
\boxed{e_{2,ss} = \frac{m v_x^2 (l_f C_f - k_r l_r C_r)}{C_f C_r L(1-k_r) R} - \frac{l_r + k_r l_f}{(1-k_r) R}}
$$

$k_r = 0$ 时：$e_{2,ss} = \dfrac{l_f m v_x^2}{C_r L R} - \dfrac{l_r}{R}$

### 9.2 前馈转角（使 $e_{1,ss} = 0$）

$$
\boxed{\delta_{ff}^* = \frac{1}{1-k_r}\left[\frac{L}{R} + \frac{m v_x^2}{R}\left(\frac{l_r}{C_f L} - \frac{l_f}{C_r L}\right)\right] + k_3\,e_{2,ss}}
$$

$k_r = 0$ 时：$\delta_{ff}^* = \dfrac{L}{R} + K_{us}\dfrac{v_x^2}{R} + k_3\,e_{2,ss}$

### 9.3 稳态横向位置误差（一般前馈 $\delta_{ff}$）

$$
\boxed{e_{1,ss} = \frac{1}{k_1}\left(\delta_{ff} - \delta_{ff}^*\right)
= \frac{1}{k_1}\left[\delta_{ff} - \frac{1}{1-k_r}\left(\frac{L}{R} + \frac{m v_x^2}{R}\left(\frac{l_r}{C_f L} - \frac{l_f}{C_r L}\right)\right) - k_3\,e_{2,ss}\right]}
$$

$k_r = 0$ 时：$e_{1,ss} = \dfrac{1}{k_1}\left[\delta_{ff} - \dfrac{L}{R} - K_{us}\dfrac{v_x^2}{R} - k_3\,e_{2,ss}\right]$

### 9.4 汇总表

| 量 | 完整表达式 | 依赖 $K$？ |
|---|---|---|
| $e_{2,ss}$ | $\dfrac{m v_x^2(l_f C_f - k_r l_r C_r)}{C_f C_r L(1-k_r)R} - \dfrac{l_r + k_r l_f}{(1-k_r)R}$ | **否** |
| $\delta_{ff}^*$ | $\dfrac{1}{1-k_r}\left[\dfrac{L}{R} + \dfrac{m v_x^2}{R}\left(\dfrac{l_r}{C_f L} - \dfrac{l_f}{C_r L}\right)\right] + k_3\,e_{2,ss}$ | 仅 $k_3$ |
| $e_{1,ss}$ | $\dfrac{1}{k_1}\left(\delta_{ff} - \delta_{ff}^*\right)$ | $k_1, k_3$ |

其中 $K_{us} = \dfrac{m}{L}\left(\dfrac{l_r}{C_f} - \dfrac{l_f}{C_r}\right)$，$L = l_f + l_r$。

---

## 10 SymPy 验证脚本

```python
"""
验证四轮转向（delta_r = kr * delta_f）稳态跟踪误差和前馈转角。
运行: python3 doc/verify_99c_4ws_steady_state.py
"""
from sympy import symbols, Matrix, simplify, solve, factor, cancel, expand

# 定义符号
Cf, Cr, lf, lr, m, Iz, vx, R = symbols('Cf Cr lf lr m Iz vx R', positive=True)
k1, k2, k3, k4, kr, delta_ff = symbols('k1 k2 k3 k4 kr delta_ff')
L = lf + lr

# 4WS 系统矩阵 (02b 第5节)
A = Matrix([
    [0, 1, 0, 0],
    [0, -(Cf + Cr)/(m*vx), (Cf + Cr)/m, -(lf*Cf - lr*Cr)/(m*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf - lr*Cr)/(Iz*vx), (lf*Cf - lr*Cr)/Iz, -(lf**2*Cf + lr**2*Cr)/(Iz*vx)]
])

# 4WS 输入矩阵 (4x2)
B2 = Matrix([
    [0, 0],
    [Cf/m, Cr/m],
    [0, 0],
    [lf*Cf/Iz, -lr*Cr/Iz]
])

# 扰动矩阵
G = Matrix([0, -vx - (lf*Cf - lr*Cr)/(m*vx), 0, -(lf**2*Cf + lr**2*Cr)/(Iz*vx)])

# 等效输入矩阵: B_eq = B2 * [1; kr]
B_eq = B2 * Matrix([1, kr])

# 反馈增益
K = Matrix([[k1, k2, k3, k4]])

# 闭环矩阵
A_cl = A - B_eq * K

# 稳态条件
dot_theta_ref = vx / R
e1_ss, e2_ss = symbols('e1_ss e2_ss')
x_ss = Matrix([e1_ss, 0, e2_ss, 0])

# 稳态方程
steady_eq = A_cl * x_ss + B_eq * delta_ff + G * dot_theta_ref

# 有效方程: 第2行和第4行
eq2 = steady_eq[1]
eq4 = steady_eq[3]

print("=" * 60)
print("验证 1: e2_ss 与反馈增益 K 无关")
print("=" * 60)

sol = solve([eq2, eq4], [e1_ss, e2_ss])
e2_ss_expr = sol[e2_ss]

has_k = any(e2_ss_expr.has(ki) for ki in [k1, k2, k3, k4])
print(f"e2_ss 含有反馈增益 K: {has_k}")

# 预期 e2_ss
e2_ss_expected = (m*vx**2*(lf*Cf - kr*lr*Cr)/(Cf*Cr*L*(1-kr)*R)
                  - (lr + kr*lf)/((1-kr)*R))
diff_e2 = simplify(e2_ss_expr - e2_ss_expected)
print(f"e2_ss - 预期 = {diff_e2}")

print("\n" + "=" * 60)
print("验证 2: kr=0 时 e2_ss 退化为 99b 结果")
print("=" * 60)

e2_ss_kr0 = e2_ss_expected.subs(kr, 0)
e2_ss_99b = lf*m*vx**2/(Cr*L*R) - lr/R
diff_kr0 = simplify(e2_ss_kr0 - e2_ss_99b)
print(f"e2_ss(kr=0) - 99b = {diff_kr0}")

print("\n" + "=" * 60)
print("验证 3: delta_ff 使 e1_ss = 0")
print("=" * 60)

e1_ss_expr = sol[e1_ss]
delta_ff_sol = solve(e1_ss_expr, delta_ff)[0]

# 预期 delta_ff
delta_ff_expected = (L/((1-kr)*R)
                     + m*vx**2/((1-kr)*R) * (lr/(Cf*L) - lf/(Cr*L))
                     + k3*e2_ss_expected)

diff_ff = simplify(delta_ff_sol - delta_ff_expected)
print(f"delta_ff(e1=0) - 预期 = {diff_ff}")

print("\n" + "=" * 60)
print("验证 4: kr=0 时 delta_ff 退化为 99b 结果")
print("=" * 60)

delta_ff_kr0 = delta_ff_expected.subs(kr, 0)
delta_ff_99b = L/R + m*vx**2/R*(lr/(Cf*L) - lf/(Cr*L)) + k3*(lf*m*vx**2/(Cr*L*R) - lr/R)
diff_ff_kr0 = simplify(delta_ff_kr0 - delta_ff_99b)
print(f"delta_ff(kr=0) - 99b = {diff_ff_kr0}")

print("\n" + "=" * 60)
print("验证 5: 矩阵逆直接验证")
print("=" * 60)

# 代入 delta_ff_expected, 求 x_ss = -A_cl^{-1} * (B_eq*delta_ff + G*dot_theta_ref)
rhs = B_eq * delta_ff_expected + G * dot_theta_ref
x_ss_full = -A_cl.inv() * rhs

e1_check = simplify(x_ss_full[0])
print(f"e1_ss (矩阵逆) = {e1_check}")

e2_check = simplify(x_ss_full[2] - e2_ss_expected)
print(f"e2_ss (矩阵逆) - 预期 = {e2_check}")

print(f"dot_e1_ss (矩阵逆) = {simplify(x_ss_full[1])}")
print(f"dot_e2_ss (矩阵逆) = {simplify(x_ss_full[3])}")

print("\n" + "=" * 60)
print("验证 6: e1_ss 一般表达式")
print("=" * 60)

e1_ss_general = (1/k1) * (delta_ff - delta_ff_expected.subs(delta_ff, delta_ff))
# 直接验证 solve 结果
e1_ss_check = (1/k1) * (delta_ff - delta_ff_sol)  # 应该 = -e1_ss_expr... 不对
# 正确方式: e1_ss_expr 应该 = (1/k1)*(delta_ff - delta_ff_star)
delta_ff_star = delta_ff_expected  # 这就是使 e1=0 的前馈
e1_ss_formula = (1/k1) * (delta_ff - delta_ff_star)
diff_e1 = simplify(e1_ss_expr - e1_ss_formula)
print(f"e1_ss - (1/k1)*(delta_ff - delta_ff*) = {diff_e1}")

print("\n" + "=" * 60)
print("所有验证完成")
print("=" * 60)
```
