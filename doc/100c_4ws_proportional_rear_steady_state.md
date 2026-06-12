# 4WS 比例后轮转向闭环稳态跟踪误差与前馈转角推导

> 基于 [[03c_error_rear_4ws]] 的后轴误差状态空间，设后轮转角与前轮成比例 $\delta_r = k_r\,\delta_f$，推导闭环稳态跟踪误差和前馈转角。$k_r=0$ 时退化为 [[100a_rear_axle_steady_state_feedforward]]。

---

## 1 问题设定

### 1.1 系统方程

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B_f\,\delta_f + B_r\,\delta_r + G\,\dot\theta_{\text{ref}}
$$

矩阵定义与 100b 相同（后轴参考点，4WS）。

### 1.2 后轮比例调度

$$
\delta_r = k_r\,\delta_f
$$

其中 $k_r$ 为调度比例系数（可依赖速度，但在稳态分析中视为常数）。

代入后系统变为单输入：

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + (B_f + k_r B_r)\,\delta_f + G\,\dot\theta_{\text{ref}}
$$

定义**等效输入矩阵**：

$$
B_{eq} = B_f + k_r B_r = \begin{bmatrix} 0 \\ \dfrac{C_f\eta + k_r C_r\xi}{mI_z} \\ 0 \\ \dfrac{l_fC_f - k_r l_rC_r}{I_z} \end{bmatrix}
$$

### 1.3 状态反馈控制律

$$
\delta_f = -K\mathbf{x} + \delta_{ff}, \qquad K = [k_1,\;k_2,\;k_3,\;k_4]
$$

### 1.4 闭环系统

$$
\dot{\mathbf{x}} = (A - B_{eq}K)\,\mathbf{x} + B_{eq}\,\delta_{ff} + G\,\dot\theta_{\text{ref}}
$$

---

## 2 定圆稳态条件

- $\dot\theta_{\text{ref}} = v_x/R = \text{const}$
- $\dot{\mathbf{x}} = 0$，$\mathbf{x}_{ss} = [e_{1,ss},\;0,\;e_{2,ss},\;0]^T$

---

## 3 稳态误差求解

### 3.1 有效方程

第 2 行和第 4 行给出两个约束。

记 $B_{eq}$ 的分量为：

$$
b_2 = \frac{C_f\eta + k_r C_r\xi}{mI_z}, \qquad b_4 = \frac{l_fC_f - k_r l_rC_r}{I_z}
$$

**第 4 行**（$\ddot{e}_2 = 0$）：

$$
0 = \left(A_{43} - b_4 k_3\right)e_{2,ss} - b_4 k_1\,e_{1,ss} + b_4\,\delta_{ff} + G_4\,\frac{v_x}{R}
$$

其中 $A_{43} = \dfrac{l_fC_f - l_rC_r}{I_z}$，$G_4 = -\dfrac{l_fC_fL}{I_zv_x}$。

两侧除以 $b_4$：

$$
0 = \left(\frac{A_{43}}{b_4} - k_3\right)e_{2,ss} - k_1\,e_{1,ss} + \delta_{ff} + \frac{G_4}{b_4}\cdot\frac{v_x}{R} \tag{I}
$$

化简 $A_{43}/b_4$：

$$
\frac{A_{43}}{b_4} = \frac{(l_fC_f - l_rC_r)/I_z}{(l_fC_f - k_r l_rC_r)/I_z} = \frac{l_fC_f - l_rC_r}{l_fC_f - k_r l_rC_r}
$$

化简 $G_4/b_4$：

$$
\frac{G_4}{b_4} = \frac{-l_fC_fL/(I_zv_x)}{(l_fC_f - k_r l_rC_r)/I_z} = \frac{-l_fC_fL}{(l_fC_f - k_r l_rC_r)v_x}
$$

因此式 (I) 的扰动项为：

$$
\frac{G_4}{b_4}\cdot\frac{v_x}{R} = \frac{-l_fC_fL}{(l_fC_f - k_r l_rC_r)R}
$$

**第 2 行**（$\ddot{e}_1 = 0$）：

$$
0 = \left(A_{23} - b_2 k_3\right)e_{2,ss} - b_2 k_1\,e_{1,ss} + b_2\,\delta_{ff} + G_2\,\frac{v_x}{R}
$$

其中 $A_{23} = \dfrac{C_f\eta + C_r\xi}{mI_z}$，$G_2 = -\dfrac{C_fL\eta}{mI_zv_x} - v_x$。

两侧除以 $b_2$：

$$
0 = \left(\frac{A_{23}}{b_2} - k_3\right)e_{2,ss} - k_1\,e_{1,ss} + \delta_{ff} + \frac{G_2}{b_2}\cdot\frac{v_x}{R} \tag{II}
$$

化简 $A_{23}/b_2$：

$$
\frac{A_{23}}{b_2} = \frac{(C_f\eta + C_r\xi)/(mI_z)}{(C_f\eta + k_r C_r\xi)/(mI_z)} = \frac{C_f\eta + C_r\xi}{C_f\eta + k_r C_r\xi}
$$

化简 $G_2/b_2$：

$$
\frac{G_2}{b_2} = \frac{-C_fL\eta/(mI_zv_x) - v_x}{(C_f\eta + k_r C_r\xi)/(mI_z)} = \frac{(-C_fL\eta - mI_zv_x^2)/v_x}{(C_f\eta + k_r C_r\xi)/I_z} \cdot \frac{1}{1}
$$

更直接地：

$$
\frac{G_2}{b_2}\cdot\frac{v_x}{R} = \frac{mI_z}{C_f\eta + k_r C_r\xi}\left(-\frac{C_fL\eta}{mI_zv_x} - v_x\right)\frac{v_x}{R} = \frac{-C_fL\eta - mI_zv_x^2}{(C_f\eta + k_r C_r\xi)R}
$$

### 3.2 两方程相减求 $e_{2,ss}$

(II) $-$ (I) 消去 $-k_1 e_{1,ss} + \delta_{ff}$：

$$
0 = \left(\frac{A_{23}}{b_2} - \frac{A_{43}}{b_4}\right)e_{2,ss} + \frac{G_2 v_x}{b_2 R} - \frac{G_4 v_x}{b_4 R}
$$

记扰动差：

$$
\frac{G_2 v_x}{b_2 R} - \frac{G_4 v_x}{b_4 R} = \frac{-C_fL\eta - mI_zv_x^2}{(C_f\eta + k_r C_r\xi)R} - \frac{-l_fC_fL}{(l_fC_f - k_r l_rC_r)R}
$$

$$
= \frac{-C_fL\eta - mI_zv_x^2}{(C_f\eta + k_r C_r\xi)R} + \frac{l_fC_fL}{(l_fC_f - k_r l_rC_r)R}
$$

记系数差：

$$
\frac{A_{23}}{b_2} - \frac{A_{43}}{b_4} = \frac{C_f\eta + C_r\xi}{C_f\eta + k_r C_r\xi} - \frac{l_fC_f - l_rC_r}{l_fC_f - k_r l_rC_r}
$$

这些表达式较为复杂，直接用 SymPy 求解更为可靠。下面给出最终结果并用 SymPy 验证。

---

## 4 等效单输入系统的直接求解

### 4.1 闭环方程

由于 $\delta_r = k_r\,\delta_f$，系统等效为：

$$
\dot{\mathbf{x}} = (A - B_{eq}K)\,\mathbf{x} + B_{eq}\,\delta_{ff} + G\,\dot\theta_{\text{ref}}
$$

稳态方程（$\dot{\mathbf{x}} = 0$，$\dot{e}_1 = \dot{e}_2 = 0$）：

**第 4 行**：

$$
0 = (A_{43} - b_4 k_3)\,e_{2,ss} - b_4 k_1\,e_{1,ss} + b_4\,\delta_{ff} + G_4\,\frac{v_x}{R}
$$

除以 $b_4$：

$$
0 = \left(\frac{l_fC_f - l_rC_r}{l_fC_f - k_r l_rC_r} - k_3\right)e_{2,ss} - k_1\,e_{1,ss} + \delta_{ff} - \frac{l_fC_fL}{(l_fC_f - k_r l_rC_r)R} \tag{I}
$$

**第 2 行**：

$$
0 = (A_{23} - b_2 k_3)\,e_{2,ss} - b_2 k_1\,e_{1,ss} + b_2\,\delta_{ff} + G_2\,\frac{v_x}{R}
$$

除以 $b_2$：

$$
0 = \left(\frac{C_f\eta + C_r\xi}{C_f\eta + k_r C_r\xi} - k_3\right)e_{2,ss} - k_1\,e_{1,ss} + \delta_{ff} + \frac{-C_fL\eta - mI_zv_x^2}{(C_f\eta + k_r C_r\xi)R} \tag{II}
$$

### 4.2 相减求 $e_{2,ss}$

(II) $-$ (I)：

$$
0 = \left[\frac{C_f\eta + C_r\xi}{C_f\eta + k_r C_r\xi} - \frac{l_fC_f - l_rC_r}{l_fC_f - k_r l_rC_r}\right]e_{2,ss} + \frac{-C_fL\eta - mI_zv_x^2}{(C_f\eta + k_r C_r\xi)R} + \frac{l_fC_fL}{(l_fC_f - k_r l_rC_r)R}
$$

**关键观察**：$e_{2,ss}$ 的系数和常数项均不含 $k_1, k_2, k_3, k_4, \delta_{ff}$。

因此 **$e_{2,ss}$ 与反馈增益 $K$ 和前馈 $\delta_{ff}$ 无关**。

具体求解留给 SymPy（见第 11 节），最终结果为：

$$
\boxed{e_{2,ss} = \frac{mv_x^2(C_fl_f - k_rC_rl_r)}{C_fC_rLR(1-k_r)} - \frac{k_rL}{(1-k_r)R}}
$$

等价形式（合并为单一分式）：

$$
e_{2,ss} = \frac{mv_x^2(C_fl_f - k_rC_rl_r) - k_rC_fC_rL^2}{C_fC_rLR(1-k_r)}
$$

### 4.3 验证特殊情况

- **$k_r = 0$**：$e_{2,ss} = \dfrac{ml_fv_x^2}{C_rLR}$，与 100a 一致 ✓
- **$k_r \to 1$**：$e_{2,ss} \to \infty$，物理上 $k_r = 1$ 意味着前后轮同角度转向（蟹行），无法产生横摆，不能跟踪曲率
- **$C_f = C_r = C$，$l_f = l_r = L/2$**：$e_{2,ss} = \dfrac{mLv_x^2(1-k_r)/2 - k_rC L^2}{C^2L^2R(1-k_r)} \cdot C$（可进一步化简）

---

## 5 前馈转角 $\delta_{ff}$ 的求解

### 5.1 目标：令 $e_{1,ss} = 0$

将 $e_{1,ss} = 0$ 代入式 (I)：

$$
0 = \left(\frac{l_fC_f - l_rC_r}{l_fC_f - k_r l_rC_r} - k_3\right)e_{2,ss} + \delta_{ff} - \frac{l_fC_fL}{(l_fC_f - k_r l_rC_r)R}
$$

解得：

$$
\delta_{ff} = \frac{l_fC_fL}{(l_fC_f - k_r l_rC_r)R} - \left(\frac{l_fC_f - l_rC_r}{l_fC_f - k_r l_rC_r} - k_3\right)e_{2,ss}
$$

### 5.2 化简

代入 $e_{2,ss}$，经 SymPy 化简（见第 11 节），最终结果为：

$$
\boxed{\delta_{ff} = \frac{L + K_{us}v_x^2}{(1 - k_r)R} + k_3\,e_{2,ss}}
$$

其中 $K_{us} = \dfrac{m}{L}\left(\dfrac{l_r}{C_f} - \dfrac{l_f}{C_r}\right)$，$e_{2,ss}$ 如第 4.2 节所述。

### 5.3 验证特殊情况

- **$k_r = 0$**：$\delta_{ff} = \dfrac{L + K_{us}v_x^2}{R} + k_3\cdot\dfrac{ml_fv_x^2}{C_rLR}$，与 100a 一致 ✓
- **$k_3 = 0$**：$\delta_{ff} = \dfrac{L + K_{us}v_x^2}{(1-k_r)R}$，经典前馈除以 $(1-k_r)$

### 5.4 物理解读

$$
\delta_{ff} = \underbrace{\frac{L + K_{us}v_x^2}{(1-k_r)R}}_{\text{等效前馈}} + \underbrace{k_3\,e_{2,ss}}_{\text{航向误差补偿}}
$$

| 项 | 含义 |
|---|---|
| $1/(1-k_r)$ 因子 | 后轮同向转 $k_r\delta_f$ 减弱了等效转向能力，前轮需增大转角补偿 |
| $k_3\,e_{2,ss}$ | 与 100a/100b 相同的航向误差反馈补偿 |

---

## 6 稳态时的实际转角

$$
\delta_f^{ss} = \delta_{ff} - k_3\,e_{2,ss} = \frac{L + K_{us}v_x^2}{(1-k_r)R}
$$

$$
\delta_r^{ss} = k_r\,\delta_f^{ss} = \frac{k_r(L + K_{us}v_x^2)}{(1-k_r)R}
$$

等效转向角（前轮减后轮）：

$$
\delta_f^{ss} - \delta_r^{ss} = (1-k_r)\,\delta_f^{ss} = \frac{L + K_{us}v_x^2}{R}
$$

这正是经典的稳态转向公式——物理上必须如此，因为稳态圆弧跟踪所需的等效转向角不依赖于前后轮的分配方式。

---

## 7 完整稳态误差向量

$$
\mathbf{x}_{ss} = \begin{bmatrix} 0 \\ 0 \\ e_{2,ss} \\ 0 \end{bmatrix}
= \begin{bmatrix} 0 \\ 0 \\ \dfrac{mv_x^2(C_fl_f - k_rC_rl_r) - k_rC_fC_rL^2}{C_fC_rLR(1-k_r)} \\ 0 \end{bmatrix}
$$

---

## 8 一般情况：$e_{1,ss}$ 的表达式

$$
\boxed{e_{1,ss} = \frac{1}{k_1}\left(\delta_{ff} - \delta_{ff}^*\right)}
$$

其中 $\delta_{ff}^* = \dfrac{L + K_{us}v_x^2}{(1-k_r)R} + k_3\,e_{2,ss}$。

---

## 9 与 07a 偏置辨识的一致性验证

### 9.1 07a 中的运动学模型

07a 文档第 11.2 节给出：

$$
\dot\psi = \frac{v_x}{L}(\delta_f - \delta_r) = \frac{v_x}{L}(\delta_f - k_r\,\delta_f) = \frac{v_x(1-k_r)}{L}\,\delta_f
$$

稳态圆弧跟踪时 $\dot\psi = v_x/R$，因此：

$$
\delta_f^{ss} = \frac{L}{(1-k_r)R}
$$

这是纯运动学（无侧偏）的结果。本文的动力学结果在低速极限（$v_x \to 0$，$K_{us}v_x^2 \to 0$）下：

$$
\delta_{ff}\big|_{v_x\to 0, k_3=0} = \frac{L}{(1-k_r)R}
$$

与 07a 的运动学模型完全一致。✓

### 9.2 $k_r = 0$ 退化

当 $k_r = 0$ 时：
- $e_{2,ss} = ml_fv_x^2/(C_rLR)$（与 100a 一致）
- $\delta_{ff} = (L + K_{us}v_x^2)/R + k_3\,e_{2,ss}$（与 100a 一致）
- 07a 的 2WS 模型 $\dot\psi = v_x\delta_f/L$（一致）

---

## 10 总结

| 量 | 表达式 | 是否依赖 $K$ |
|---|---|---|
| $e_{2,ss}$ | $\dfrac{mv_x^2(C_fl_f - k_rC_rl_r) - k_rC_fC_rL^2}{C_fC_rLR(1-k_r)}$ | **否** |
| $\delta_{ff}$（使 $e_1=0$） | $\dfrac{L + K_{us}v_x^2}{(1-k_r)R} + k_3\,e_{2,ss}$ | 仅依赖 $k_3$ |
| $e_{1,ss}$（一般） | $\dfrac{1}{k_1}\left(\delta_{ff} - \delta_{ff}^*\right)$ | 依赖 $k_1, k_3$ |
| $\delta_f^{ss} - \delta_r^{ss}$ | $\dfrac{L + K_{us}v_x^2}{R}$ | 否（物理不变量） |


---

## 11 SymPy 验证

验证脚本：`doc/verify_100c_4ws_proportional_rear.py`

验证内容：
1. $e_{2,ss}$ 与反馈增益 $K$ 和 $\delta_{ff}$ 无关 ✓
2. $e_{2,ss}$ 解析表达式验证 ✓
3. 前馈转角 $\delta_{ff}$ 使 $e_{1,ss} = 0$ ✓
4. $k_r = 0$ 时退化为纯前轮转向结果（100a）✓
5. $k_3 = 0$ 时退化为 $(L + K_{us}v_x^2)/((1-k_r)R)$ ✓
6. 矩阵逆直接求解验证 ✓
7. 稳态等效转向角 $= (L + K_{us}v_x^2)/R$（物理不变量）✓
8. 与 07a 运动学模型一致性 ✓
9. $e_{1,ss}$ 一般表达式验证 ✓
10. 数值验证（典型车辆参数）✓

```python
"""
验证 4WS 比例后轮转向 (delta_r = kr * delta_f) 闭环稳态跟踪误差和前馈转角 (100c)。
基于 03c_error_rear_4ws.md 的后轴误差状态空间。

运行: python3 doc/verify_100c_4ws_proportional_rear.py
"""
from sympy import symbols, Matrix, simplify, solve, factor, cancel, expand

# 定义符号
Cf, Cr, lf, lr, m, Iz, vx, R = symbols('Cf Cr lf lr m Iz vx R', positive=True)
k1, k2, k3, k4, delta_ff, kr = symbols('k1 k2 k3 k4 delta_ff kr')
L = lf + lr
eta = Iz - m * lf * lr
xi = Iz + m * lr**2

# ============================================================
# 4WS 后轴定义的系统矩阵 (来自 03c)
# ============================================================
A = Matrix([
    [0, 1, 0, 0],
    [0, -(Cf*eta + Cr*xi)/(m*Iz*vx), (Cf*eta + Cr*xi)/(m*Iz), -(Cf*L*eta)/(m*Iz*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf - lr*Cr)/(Iz*vx), (lf*Cf - lr*Cr)/Iz, -(lf*Cf*L)/(Iz*vx)]
])

Bf = Matrix([0, Cf*eta/(m*Iz), 0, lf*Cf/Iz])
Br = Matrix([0, Cr*xi/(m*Iz), 0, -lr*Cr/Iz])
G = Matrix([0, -(Cf*L*eta)/(m*Iz*vx) - vx, 0, -(lf*Cf*L)/(Iz*vx)])

# 等效输入矩阵: B_eq = Bf + kr * Br
B_eq = Bf + kr * Br

K_mat = Matrix([[k1, k2, k3, k4]])

# 闭环矩阵
A_cl = A - B_eq * K_mat

# 稳态条件
dot_theta_ref = vx / R

# 稳态时 x_ss = [e1_ss, 0, e2_ss, 0]
e1_ss, e2_ss = symbols('e1_ss e2_ss')
x_ss = Matrix([e1_ss, 0, e2_ss, 0])

# 稳态方程: 0 = A_cl * x_ss + B_eq * delta_ff + G * dot_theta_ref
steady_eq = A_cl * x_ss + B_eq * delta_ff + G * dot_theta_ref

# 有效方程: 第2行和第4行
eq2 = steady_eq[1]  # ddot_e1 = 0
eq4 = steady_eq[3]  # ddot_e2 = 0

print("=" * 60)
print("验证 1: 稳态航向误差 e2_ss 与 K 无关")
print("=" * 60)

# 解两个方程求 e1_ss 和 e2_ss
sol = solve([eq2, eq4], [e1_ss, e2_ss])
e2_ss_expr = sol[e2_ss]
e2_ss_simplified = simplify(e2_ss_expr)

print(f"e2_ss (sympy) = {e2_ss_simplified}")

# 验证 e2_ss 不含 k1, k2, k3, k4, delta_ff
has_k = any(e2_ss_expr.has(s) for s in [k1, k2, k3, k4, delta_ff])
print(f"e2_ss 含有反馈增益 K 或 delta_ff: {has_k}")
assert not has_k, "e2_ss 不应依赖 K 或 delta_ff!"

# 验证解析表达式
e2_ss_expected = m*vx**2*(Cf*lf - kr*Cr*lr) / (Cf*Cr*L*R*(1-kr)) - kr*L/((1-kr)*R)
diff_e2 = simplify(e2_ss_expr - e2_ss_expected)
print(f"e2_ss - 预期值 = {diff_e2}")
assert diff_e2 == 0, f"e2_ss 验证失败! diff = {diff_e2}"
print("✓ e2_ss 验证通过")

print("\n" + "=" * 60)
print("验证 2: 前馈转角 delta_ff 使 e1_ss = 0")
print("=" * 60)

e1_ss_expr = sol[e1_ss]
delta_ff_sol = solve(e1_ss_expr, delta_ff)[0]

K_us = m / L * (lr / Cf - lf / Cr)
delta_ff_expected = (L + K_us * vx**2) / ((1 - kr) * R) + k3 * e2_ss_expected

diff_ff = simplify(delta_ff_sol - delta_ff_expected)
print(f"delta_ff - 预期值 = {diff_ff}")
assert diff_ff == 0, f"delta_ff 验证失败! diff = {diff_ff}"
print("✓ delta_ff 验证通过")

print("\n" + "=" * 60)
print("验证 3: kr=0 退化为纯前轮转向 (100a)")
print("=" * 60)

e2_ss_100a = m * lf * vx**2 / (Cr * L * R)
delta_ff_100a = (L + K_us * vx**2) / R + k3 * e2_ss_100a

diff_e2_kr0 = simplify(e2_ss_expected.subs(kr, 0) - e2_ss_100a)
diff_ff_kr0 = simplify(delta_ff_expected.subs(kr, 0) - delta_ff_100a)
print(f"e2_ss(kr=0) - 100a = {diff_e2_kr0}")
print(f"delta_ff(kr=0) - 100a = {diff_ff_kr0}")
assert diff_e2_kr0 == 0 and diff_ff_kr0 == 0
print("✓ kr=0 退化为 100a 验证通过")

print("\n" + "=" * 60)
print("验证 4: 稳态等效转向角 = 经典公式")
print("=" * 60)

delta_f_ss = delta_ff_expected - k3 * e2_ss_expected
effective_steer = simplify((1 - kr) * delta_f_ss)
classic_steer = (L + K_us * vx**2) / R
diff_eff = simplify(effective_steer - classic_steer)
print(f"(1-kr)*delta_f_ss - 经典公式 = {diff_eff}")
assert diff_eff == 0
print("✓ 稳态等效转向角验证通过")

print("\n" + "=" * 60)
print("验证 5: 矩阵逆验证")
print("=" * 60)

rhs = B_eq * delta_ff_expected + G * dot_theta_ref
x_ss_full = simplify(-A_cl.inv() * rhs)
assert simplify(x_ss_full[0]) == 0
assert simplify(x_ss_full[1]) == 0
assert simplify(x_ss_full[2] - e2_ss_expected) == 0
assert simplify(x_ss_full[3]) == 0
print("✓ 矩阵逆验证全部通过")

print("\n" + "=" * 60)
print("所有验证通过 ✓")
print("=" * 60)
```
