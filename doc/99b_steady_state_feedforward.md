# 纯前轮转向闭环稳态跟踪误差与前馈转角推导

## 1 问题设定

基于 `02b_error_cg_4ws.md` 第 8 节的纯前轮转向误差状态空间：

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B\,\delta_f + G\,\dot\theta_{\text{ref}}
$$

状态向量 $\mathbf{x} = [e_1,\;\dot{e}_1,\;e_2,\;\dot{e}_2]^T$。

### 1.1 状态反馈控制律

$$
\delta_f = -K\mathbf{x} + \delta_{ff} = -k_1 e_1 - k_2 \dot{e}_1 - k_3 e_2 - k_4 \dot{e}_2 + \delta_{ff}
$$

其中 $K = [k_1,\;k_2,\;k_3,\;k_4]$ 为反馈增益，$\delta_{ff}$ 为前馈转角。

### 1.2 闭环系统

代入控制律：

$$
\dot{\mathbf{x}} = (A - BK)\,\mathbf{x} + B\,\delta_{ff} + G\,\dot\theta_{\text{ref}}
$$

---

## 2 定圆稳态条件

定圆转向：车辆以恒定速度 $v_x$ 沿半径 $R$ 的圆弧行驶。

- 曲率恒定：$\kappa = 1/R$
- 参考航向变化率恒定：$\dot\theta_{\text{ref}} = \kappa\,v_x = v_x / R$
- 稳态条件：$\dot{\mathbf{x}} = 0$

---

## 3 稳态误差求解

### 3.1 稳态方程

令 $\dot{\mathbf{x}} = 0$：

$$
0 = (A - BK)\,\mathbf{x}_{ss} + B\,\delta_{ff} + G\,\dot\theta_{\text{ref}}
$$

解得：

$$
\mathbf{x}_{ss} = -(A - BK)^{-1}\left(B\,\delta_{ff} + G\,\dot\theta_{\text{ref}}\right)
$$


### 3.2 稳态时的额外约束

稳态时 $\dot{e}_1 = 0$，$\dot{e}_2 = 0$，因此状态向量简化为：

$$
\mathbf{x}_{ss} = [e_{1,ss},\;0,\;e_{2,ss},\;0]^T
$$

这意味着只需关注第 2 行和第 4 行方程（对应 $\ddot{e}_1 = 0$ 和 $\ddot{e}_2 = 0$）。

### 3.3 展开闭环方程

将 $A - BK$ 展开，利用 $\dot{e}_1 = \dot{e}_2 = 0$，第 2 行和第 4 行给出：

**第 2 行**（$\ddot{e}_1 = 0$）：

$$
0 = \left(\frac{C_f + C_r}{m} - \frac{C_f}{m}k_3\right) e_{2,ss}
- \frac{C_f}{m} k_1\, e_{1,ss}
+ \frac{C_f}{m}\,\delta_{ff}
+ \left(-v_x - \frac{l_f C_f - l_r C_r}{m\,v_x}\right)\dot\theta_{\text{ref}}
$$

**第 4 行**（$\ddot{e}_2 = 0$）：

$$
0 = \left(\frac{l_f C_f - l_r C_r}{I_z} - \frac{l_f C_f}{I_z}k_3\right) e_{2,ss}
- \frac{l_f C_f}{I_z} k_1\, e_{1,ss}
+ \frac{l_f C_f}{I_z}\,\delta_{ff}
- \frac{l_f^2 C_f + l_r^2 C_r}{I_z\,v_x}\,\dot\theta_{\text{ref}}
$$

### 3.4 化简

定义辅助量：
- $L = l_f + l_r$（轴距）
- $\dot\theta_{\text{ref}} = v_x / R$

**第 2 行** 乘以 $m/C_f$：

$$
0 = \left(\frac{C_f + C_r}{C_f} - k_3\right) e_{2,ss}
- k_1\, e_{1,ss}
+ \delta_{ff}
+ \frac{m}{C_f}\left(-v_x - \frac{l_f C_f - l_r C_r}{m\,v_x}\right)\frac{v_x}{R}
$$

化简扰动项：

$$
\frac{m}{C_f}\left(-v_x - \frac{l_f C_f - l_r C_r}{m\,v_x}\right)\frac{v_x}{R}
= -\frac{m v_x^2}{C_f R} - \frac{l_f C_f - l_r C_r}{C_f R}
$$

$$
= -\frac{m v_x^2}{C_f R} - \frac{l_f}{R} + \frac{l_r C_r}{C_f R}
$$

**第 4 行** 乘以 $I_z / (l_f C_f)$：

$$
0 = \left(1 - \frac{l_r C_r}{l_f C_f} - k_3\right) e_{2,ss}
- k_1\, e_{1,ss}
+ \delta_{ff}
- \frac{l_f^2 C_f + l_r^2 C_r}{l_f C_f\,v_x}\cdot\frac{v_x}{R}
$$

化简扰动项：

$$
-\frac{l_f^2 C_f + l_r^2 C_r}{l_f C_f R}
= -\frac{l_f}{R} - \frac{l_r^2 C_r}{l_f C_f R}
$$


### 3.5 两方程相减消去 $e_{1,ss}$ 和 $\delta_{ff}$

两方程均含 $-k_1 e_{1,ss} + \delta_{ff}$，相减（第 2 行 $-$ 第 4 行）：

$$
0 = \left[\frac{C_f + C_r}{C_f} - k_3 - \left(1 - \frac{l_r C_r}{l_f C_f} - k_3\right)\right] e_{2,ss}
+ \left[-\frac{m v_x^2}{C_f R} - \frac{l_f}{R} + \frac{l_r C_r}{C_f R}\right]
- \left[-\frac{l_f}{R} - \frac{l_r^2 C_r}{l_f C_f R}\right]
$$

化简系数：

$$
\frac{C_f + C_r}{C_f} - 1 + \frac{l_r C_r}{l_f C_f}
= 1 + \frac{C_r}{C_f} - 1 + \frac{l_r C_r}{l_f C_f}
= \frac{C_r}{C_f} + \frac{l_r C_r}{l_f C_f}
= \frac{C_r(l_f + l_r)}{l_f C_f}
= \frac{C_r L}{l_f C_f}
$$

化简常数项：

$$
-\frac{m v_x^2}{C_f R} - \frac{l_f}{R} + \frac{l_r C_r}{C_f R} + \frac{l_f}{R} + \frac{l_r^2 C_r}{l_f C_f R}
= -\frac{m v_x^2}{C_f R} + \frac{l_r C_r}{C_f R} + \frac{l_r^2 C_r}{l_f C_f R}
$$

$$
= -\frac{m v_x^2}{C_f R} + \frac{l_r C_r (l_f + l_r)}{l_f C_f R}
= -\frac{m v_x^2}{C_f R} + \frac{l_r C_r L}{l_f C_f R}
$$

因此：

$$
\frac{C_r L}{l_f C_f}\, e_{2,ss} = \frac{m v_x^2}{C_f R} - \frac{l_r C_r L}{l_f C_f R}
$$

$$
\boxed{e_{2,ss} = \frac{m v_x^2 l_f}{C_r L R} - \frac{l_r}{R}}
$$

---

## 4 稳态航向误差的物理解读

$$
e_{2,ss} = \frac{l_f m v_x^2}{C_r L R} - \frac{l_r}{R}
$$

| 项 | 表达式 | 含义 |
|---|---|---|
| 动力学项 | $\dfrac{l_f m v_x^2}{C_r L R}$ | 离心力导致后轮侧偏，产生航向偏差 |
| 几何项 | $-\dfrac{l_r}{R}$ | 质心相对后轴的几何角度关系 |

**关键观察**：
- $e_{2,ss}$ **与反馈增益 $K$ 无关**——状态反馈无法消除稳态航向误差
- $e_{2,ss}$ 仅取决于车辆参数和运动状态
- 这是因为系统只有 1 个控制输入（$\delta_f$），无法同时将 $e_1$ 和 $e_2$ 都控制为零


---

## 5 前馈转角 $\delta_{ff}$ 的求解

### 5.1 目标：令 $e_{1,ss} = 0$

设计前馈转角使稳态横向位置误差为零。将 $e_{1,ss} = 0$ 代入第 4 行方程：

$$
0 = \left(1 - \frac{l_r C_r}{l_f C_f} - k_3\right) e_{2,ss}
+ \delta_{ff}
- \frac{l_f}{R} - \frac{l_r^2 C_r}{l_f C_f R}
$$

解得：

$$
\delta_{ff} = \frac{l_f}{R} + \frac{l_r^2 C_r}{l_f C_f R}
- \left(1 - \frac{l_r C_r}{l_f C_f} - k_3\right) e_{2,ss}
$$

代入 $e_{2,ss}$：

$$
\delta_{ff} = \frac{l_f}{R} + \frac{l_r^2 C_r}{l_f C_f R}
- \left(1 - \frac{l_r C_r}{l_f C_f} - k_3\right)\left(\frac{l_f m v_x^2}{C_r L R} - \frac{l_r}{R}\right)
$$

### 5.2 化简

先整理 $\left(1 - \frac{l_r C_r}{l_f C_f}\right)$：

$$
1 - \frac{l_r C_r}{l_f C_f} = \frac{l_f C_f - l_r C_r}{l_f C_f}
$$

展开乘积：

$$
\frac{l_f C_f - l_r C_r}{l_f C_f}\left(\frac{l_f m v_x^2}{C_r L R} - \frac{l_r}{R}\right)
= \frac{(l_f C_f - l_r C_r) l_f m v_x^2}{l_f C_f C_r L R} - \frac{(l_f C_f - l_r C_r) l_r}{l_f C_f R}
$$

$$
= \frac{(l_f C_f - l_r C_r) m v_x^2}{C_f C_r L R} - \frac{l_r}{R} + \frac{l_r^2 C_r}{l_f C_f R}
$$

因此：

$$
\delta_{ff} = \frac{l_f}{R} + \frac{l_r^2 C_r}{l_f C_f R}
- \frac{(l_f C_f - l_r C_r) m v_x^2}{C_f C_r L R} + \frac{l_r}{R} - \frac{l_r^2 C_r}{l_f C_f R}
+ k_3\left(\frac{l_f m v_x^2}{C_r L R} - \frac{l_r}{R}\right)
$$

$$
= \frac{l_f + l_r}{R} - \frac{(l_f C_f - l_r C_r) m v_x^2}{C_f C_r L R}
+ k_3\left(\frac{l_f m v_x^2}{C_r L R} - \frac{l_r}{R}\right)
$$

$$
= \frac{L}{R} + \frac{m v_x^2}{R}\left(\frac{l_r}{C_f L} - \frac{l_f}{C_r L}\right)
+ k_3\left(\frac{l_f m v_x^2}{C_r L R} - \frac{l_r}{R}\right)
$$

$$
\boxed{\delta_{ff} = \frac{L}{R} + \frac{m v_x^2}{R}\left(\frac{l_r}{C_f L} - \frac{l_f}{C_r L}\right) + k_3\,e_{2,ss}}
$$

其中 $e_{2,ss} = \dfrac{l_f m v_x^2}{C_r L R} - \dfrac{l_r}{R}$。

### 5.3 物理解读

$$
\delta_{ff} = \underbrace{\frac{L}{R}}_{\text{Ackermann}} + \underbrace{\frac{m v_x^2}{R}\left(\frac{l_r}{C_f L} - \frac{l_f}{C_r L}\right)}_{\text{不足转向补偿}} + \underbrace{k_3\,e_{2,ss}}_{\text{航向误差反馈补偿}}
$$

| 项 | 含义 |
|---|---|
| $L/R$ | 低速纯几何 Ackermann 转角 |
| 不足转向项 | 高速时前后轮侧偏刚度差异导致的额外转角需求 |
| $k_3 e_{2,ss}$ | 补偿稳态航向误差对横向位置的影响 |

**注意**：当 $k_3 = 0$ 时，前馈退化为经典的稳态转向公式：

$$
\delta_{ff}\big|_{k_3=0} = \frac{L}{R} + K_{us}\,\frac{v_x^2}{R}
$$

其中 $K_{us} = \dfrac{m}{L}\left(\dfrac{l_r}{C_f} - \dfrac{l_f}{C_r}\right)$ 为不足转向梯度。


---

## 6 完整稳态误差向量

当 $\delta_{ff}$ 按上式选取时：

$$
\mathbf{x}_{ss} = \begin{bmatrix} 0 \\ 0 \\ e_{2,ss} \\ 0 \end{bmatrix}
= \begin{bmatrix} 0 \\ 0 \\ \dfrac{l_f m v_x^2}{C_r L R} - \dfrac{l_r}{R} \\ 0 \end{bmatrix}
$$

- 横向位置误差 $e_{1,ss} = 0$（通过前馈消除）
- 航向误差 $e_{2,ss} \neq 0$（单输入系统的固有限制）
- 速度误差 $\dot{e}_{1,ss} = \dot{e}_{2,ss} = 0$（稳态）

---

## 7 一般情况：$e_{1,ss}$ 的表达式

若不指定 $e_{1,ss} = 0$ 的约束，从第 4 行方程可得 $e_{1,ss}$ 与 $\delta_{ff}$ 的关系：

$$
e_{1,ss} = \frac{1}{k_1}\left[\delta_{ff} - \frac{l_f}{R} - \frac{l_r^2 C_r}{l_f C_f R}
+ \left(1 - \frac{l_r C_r}{l_f C_f} - k_3\right) e_{2,ss}\right]
$$

整理：

$$
e_{1,ss} = \frac{1}{k_1}\left[\delta_{ff} - \frac{L}{R} - \frac{m v_x^2}{R}\left(\frac{l_r}{C_f L} - \frac{l_f}{C_r L}\right) - k_3\,e_{2,ss}\right]
$$

即：

$$
\boxed{e_{1,ss} = \frac{1}{k_1}\left(\delta_{ff} - \delta_{ff}^*\right)}
$$

其中 $\delta_{ff}^* = \dfrac{L}{R} + \dfrac{m v_x^2}{R}\left(\dfrac{l_r}{C_f L} - \dfrac{l_f}{C_r L}\right) + k_3\,e_{2,ss}$ 为使 $e_{1,ss} = 0$ 的理想前馈。

---

## 8 总结

| 量 | 表达式 | 是否依赖 $K$ |
|---|---|---|
| $e_{2,ss}$ | $\dfrac{l_f m v_x^2}{C_r L R} - \dfrac{l_r}{R}$ | **否** |
| $\delta_{ff}$（使 $e_1=0$） | $\dfrac{L}{R} + \dfrac{m v_x^2}{R}\left(\dfrac{l_r}{C_f L} - \dfrac{l_f}{C_r L}\right) + k_3\,e_{2,ss}$ | 仅依赖 $k_3$ |
| $e_{1,ss}$（一般） | $\dfrac{1}{k_1}\left(\delta_{ff} - \delta_{ff}^*\right)$ | 依赖 $k_1, k_3$ |

---

## 9 SymPy 验证脚本

```python
"""
验证稳态跟踪误差和前馈转角的推导。
运行: python3 doc/99b_steady_state_feedforward.md  (提取脚本后运行)
或: python3 doc/verify_steady_state_feedforward.py
"""
from sympy import symbols, Matrix, simplify, solve, Rational, factor, cancel

# 定义符号
Cf, Cr, lf, lr, m, Iz, vx, R = symbols('Cf Cr lf lr m Iz vx R', positive=True)
k1, k2, k3, k4, delta_ff = symbols('k1 k2 k3 k4 delta_ff')
L = lf + lr

# 定义矩阵
A = Matrix([
    [0, 1, 0, 0],
    [0, -(Cf + Cr)/(m*vx), (Cf + Cr)/m, -(lf*Cf - lr*Cr)/(m*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf - lr*Cr)/(Iz*vx), (lf*Cf - lr*Cr)/Iz, -(lf**2*Cf + lr**2*Cr)/(Iz*vx)]
])

B = Matrix([0, Cf/m, 0, lf*Cf/Iz])

G = Matrix([0, -vx - (lf*Cf - lr*Cr)/(m*vx), 0, -(lf**2*Cf + lr**2*Cr)/(Iz*vx)])

K = Matrix([[k1, k2, k3, k4]])

# 闭环矩阵
A_cl = A - B * K

# 稳态条件: dot_x = 0
# 0 = A_cl * x_ss + B * delta_ff + G * dot_theta_ref
# dot_theta_ref = vx / R
dot_theta_ref = vx / R

# 稳态时 dot_e1 = dot_e2 = 0, 所以 x_ss = [e1_ss, 0, e2_ss, 0]
e1_ss, e2_ss = symbols('e1_ss e2_ss')
x_ss = Matrix([e1_ss, 0, e2_ss, 0])

# 稳态方程 (4个方程，但第1行和第3行是 0=0+0+0+0 自动满足)
steady_eq = A_cl * x_ss + B * delta_ff + G * dot_theta_ref

# 第1行: 0 = 0 (自动满足，因为 dot_e1 = e1_dot = x[1] = 0)
# 第3行: 0 = 0 (自动满足，因为 dot_e2 = e2_dot = x[3] = 0)
# 有效方程是第2行和第4行
eq2 = steady_eq[1]  # ddot_e1 = 0
eq4 = steady_eq[3]  # ddot_e2 = 0

print("=" * 60)
print("验证 1: 稳态航向误差 e2_ss 与 K 无关")
print("=" * 60)

# 解两个方程求 e1_ss 和 e2_ss
sol = solve([eq2, eq4], [e1_ss, e2_ss])
e2_ss_expr = sol[e2_ss]

# 验证 e2_ss 不含 k1, k2, k3, k4
has_k = any(e2_ss_expr.has(ki) for ki in [k1, k2, k3, k4])
print(f"e2_ss 含有反馈增益 K: {has_k}")

# 验证解析表达式
e2_ss_expected = lf*m*vx**2/(Cr*L*R) - lr/R
diff_e2 = simplify(e2_ss_expr - e2_ss_expected)
print(f"e2_ss - 预期 = {diff_e2}")

print("\n" + "=" * 60)
print("验证 2: e1_ss 表达式")
print("=" * 60)

e1_ss_expr = sol[e1_ss]

# 预期: e1_ss = (1/k1) * (delta_ff - delta_ff_star)
delta_ff_star = L/R + m*vx**2/R * (lr/(Cf*L) - lf/(Cr*L)) + k3*e2_ss_expected
e1_ss_expected = (1/k1) * (delta_ff - delta_ff_star)

diff_e1 = simplify(e1_ss_expr - e1_ss_expected)
print(f"e1_ss - 预期 = {diff_e1}")

print("\n" + "=" * 60)
print("验证 3: 前馈转角 delta_ff 使 e1_ss = 0")
print("=" * 60)

# 令 e1_ss = 0，解 delta_ff
delta_ff_sol = solve(e1_ss_expr, delta_ff)[0]

# 预期前馈
delta_ff_expected = L/R + m*vx**2/R * (lr/(Cf*L) - lf/(Cr*L)) + k3*e2_ss_expected

diff_ff = simplify(delta_ff_sol - delta_ff_expected)
print(f"delta_ff(e1=0) - 预期 = {diff_ff}")

print("\n" + "=" * 60)
print("验证 4: 直接用矩阵逆验证")
print("=" * 60)

# 用完整矩阵逆求解 x_ss = -A_cl^{-1} * (B*delta_ff + G*dot_theta_ref)
# 代入 delta_ff = delta_ff_expected
rhs = B * delta_ff_expected + G * dot_theta_ref
x_ss_full = -A_cl.inv() * rhs

# 验证 x_ss[0] = 0 (e1 = 0)
e1_check = simplify(x_ss_full[0])
print(f"e1_ss (矩阵逆) = {e1_check}")

# 验证 x_ss[2] = e2_ss_expected
e2_check = simplify(x_ss_full[2] - e2_ss_expected)
print(f"e2_ss (矩阵逆) - 预期 = {e2_check}")

# 验证 dot_e1 = dot_e2 = 0
print(f"dot_e1_ss (矩阵逆) = {simplify(x_ss_full[1])}")
print(f"dot_e2_ss (矩阵逆) = {simplify(x_ss_full[3])}")

print("\n" + "=" * 60)
print("验证 5: k3=0 时退化为经典公式")
print("=" * 60)

delta_ff_k3_0 = delta_ff_expected.subs(k3, 0)
delta_ff_classic = L/R + m*vx**2/(R*L) * (lr/Cf - lf/Cr)
diff_classic = simplify(delta_ff_k3_0 - delta_ff_classic)
print(f"k3=0 时 delta_ff - 经典公式 = {diff_classic}")

print("\n" + "=" * 60)
print("所有验证完成")
print("=" * 60)
```
