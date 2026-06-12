# 4WS 后轴参考点闭环稳态跟踪误差与前馈转角推导

> 基于 [[03c_error_rear_4ws]] 的四轮转向后轴误差状态空间，推导状态反馈闭环下定圆稳态跟踪误差和前馈转角。纯前轮转向特例见 [[100a_rear_axle_steady_state_feedforward]]。

---

## 1 问题设定

基于 `03c_error_rear_4ws.md` 第 6 节的 4WS 后轴误差状态空间：

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B_f\,\delta_f + B_r\,\delta_r + G\,\dot\theta_{\text{ref}}
$$

状态向量 $\mathbf{x} = [e_1,\;\dot{e}_1,\;e_2,\;\dot{e}_2]^T$，误差定义在后轴中心。

### 1.1 系统矩阵（后轴定义，4WS）

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\[6pt]
0 & -\dfrac{C_f\eta + C_r\xi}{mI_zv_x} & \dfrac{C_f\eta + C_r\xi}{mI_z} & -\dfrac{C_fL\eta}{mI_zv_x} \\[6pt]
0 & 0 & 0 & 1 \\[6pt]
0 & -\dfrac{l_fC_f - l_rC_r}{I_zv_x} & \dfrac{l_fC_f - l_rC_r}{I_z} & -\dfrac{l_fC_fL}{I_zv_x}
\end{bmatrix}
$$

$$
B_f = \begin{bmatrix} 0 \\ \dfrac{C_f\eta}{mI_z} \\ 0 \\ \dfrac{l_fC_f}{I_z} \end{bmatrix}, \qquad
B_r = \begin{bmatrix} 0 \\ \dfrac{C_r\xi}{mI_z} \\ 0 \\ -\dfrac{l_rC_r}{I_z} \end{bmatrix}, \qquad
G = \begin{bmatrix} 0 \\ -\dfrac{C_fL\eta}{mI_zv_x} - v_x \\ 0 \\ -\dfrac{l_fC_fL}{I_zv_x} \end{bmatrix}
$$

其中 $\eta = I_z - ml_fl_r$，$\xi = I_z + ml_r^2$，$L = l_f + l_r$。

### 1.2 控制律

4WS 系统有两个控制输入。设后轮转角由独立的调度器给定（$\delta_r$ 为已知量），前轮转角采用状态反馈 + 前馈：

$$
\delta_f = -K\mathbf{x} + \delta_{ff} = -k_1 e_1 - k_2 \dot{e}_1 - k_3 e_2 - k_4 \dot{e}_2 + \delta_{ff}
$$

其中 $K = [k_1,\;k_2,\;k_3,\;k_4]$ 为前轮反馈增益，$\delta_{ff}$ 为前轮前馈转角。

### 1.3 闭环系统

代入控制律：

$$
\dot{\mathbf{x}} = (A - B_fK)\,\mathbf{x} + B_f\,\delta_{ff} + B_r\,\delta_r + G\,\dot\theta_{\text{ref}}
$$

---

## 2 定圆稳态条件

- 曲率恒定：$\kappa = 1/R$
- 参考航向变化率恒定：$\dot\theta_{\text{ref}} = \kappa v_x = v_x/R$
- 后轮转角恒定：$\delta_r = \text{const}$
- 稳态条件：$\dot{\mathbf{x}} = 0$

稳态时 $\dot{e}_1 = \dot{e}_2 = 0$，状态向量简化为：

$$
\mathbf{x}_{ss} = [e_{1,ss},\;0,\;e_{2,ss},\;0]^T
$$

---

## 3 稳态误差求解

### 3.1 有效方程

令 $\dot{\mathbf{x}} = 0$，第 1、3 行自动满足。有效约束来自第 2 行和第 4 行。

**第 4 行**（$\ddot{e}_2 = 0$）：

$$
0 = \left(\frac{l_fC_f - l_rC_r}{I_z} - \frac{l_fC_f}{I_z}k_3\right)e_{2,ss} - \frac{l_fC_f}{I_z}k_1\,e_{1,ss} + \frac{l_fC_f}{I_z}\,\delta_{ff} - \frac{l_rC_r}{I_z}\,\delta_r - \frac{l_fC_fL}{I_zv_x}\cdot\frac{v_x}{R}
$$

两侧乘以 $I_z/(l_fC_f)$：

$$
0 = \left(1 - \frac{l_rC_r}{l_fC_f} - k_3\right)e_{2,ss} - k_1\,e_{1,ss} + \delta_{ff} - \frac{l_rC_r}{l_fC_f}\,\delta_r - \frac{L}{R} \tag{I}
$$

**第 2 行**（$\ddot{e}_1 = 0$）：

$$
0 = \left(\frac{C_f\eta + C_r\xi}{mI_z} - \frac{C_f\eta}{mI_z}k_3\right)e_{2,ss} - \frac{C_f\eta}{mI_z}k_1\,e_{1,ss} + \frac{C_f\eta}{mI_z}\,\delta_{ff} + \frac{C_r\xi}{mI_z}\,\delta_r + \left(-\frac{C_fL\eta}{mI_zv_x} - v_x\right)\frac{v_x}{R}
$$

两侧乘以 $mI_z/(C_f\eta)$（假设 $\eta \neq 0$）：

$$
0 = \left(1 + \frac{C_r\xi}{C_f\eta} - k_3\right)e_{2,ss} - k_1\,e_{1,ss} + \delta_{ff} + \frac{C_r\xi}{C_f\eta}\,\delta_r - \frac{L}{R} - \frac{mI_zv_x^2}{C_f\eta R} \tag{II}
$$

### 3.2 两方程相减求 $e_{2,ss}$

式 (II) $-$ 式 (I)：

$$
0 = \left(\frac{C_r\xi}{C_f\eta} + \frac{l_rC_r}{l_fC_f}\right)e_{2,ss} + \left(\frac{C_r\xi}{C_f\eta} + \frac{l_rC_r}{l_fC_f}\right)\delta_r - \frac{mI_zv_x^2}{C_f\eta R}
$$

提取公因子。先化简 $\dfrac{C_r\xi}{C_f\eta} + \dfrac{l_rC_r}{l_fC_f}$：

通分 $l_fC_f\eta$：

$$
\frac{C_r\xi \cdot l_f + l_rC_r\eta}{l_fC_f\eta} = \frac{C_r(l_f\xi + l_r\eta)}{l_fC_f\eta}
$$

展开 $l_f\xi + l_r\eta$：

$$
l_f(I_z + ml_r^2) + l_r(I_z - ml_fl_r) = l_fI_z + ml_fl_r^2 + l_rI_z - ml_fl_r^2 = (l_f + l_r)I_z = LI_z
$$

因此：

$$
\frac{C_r\xi}{C_f\eta} + \frac{l_rC_r}{l_fC_f} = \frac{C_rLI_z}{l_fC_f\eta}
$$

代入相减后的方程：

$$
\frac{C_rLI_z}{l_fC_f\eta}(e_{2,ss} + \delta_r) = \frac{mI_zv_x^2}{C_f\eta R}
$$

消去 $I_z/(C_f\eta)$：

$$
C_rL(e_{2,ss} + \delta_r) = \frac{ml_fv_x^2}{R}
$$

$$
\boxed{e_{2,ss} = \frac{ml_fv_x^2}{C_rLR} - \delta_r}
$$

---

## 4 稳态航向误差的物理解读

$$
e_{2,ss} = \frac{ml_fv_x^2\kappa}{C_rL} - \delta_r
$$

### 4.1 各项含义

| 项 | 表达式 | 含义 |
|---|---|---|
| 动力学项 | $\dfrac{ml_fv_x^2\kappa}{C_rL}$ | 后轮侧偏角（与纯前轮转向相同） |
| 后轮转角补偿 | $-\delta_r$ | 后轮主动转向直接减小航向误差 |

### 4.2 关键性质

- $e_{2,ss}$ **与反馈增益 $K$ 无关**（单输入系统的固有限制）
- 后轮转角 $\delta_r$ **直接减小**稳态航向误差
- 当 $\delta_r = ml_fv_x^2\kappa/(C_rL)$ 时，$e_{2,ss} = 0$——4WS 可以完全消除稳态航向误差

### 4.3 与纯前轮转向的对比

| | 纯前轮转向（100a） | 4WS（本文） |
|---|---|---|
| $e_{2,ss}$ | $\dfrac{ml_fv_x^2}{C_rLR}$ | $\dfrac{ml_fv_x^2}{C_rLR} - \delta_r$ |
| 可否消除 | 否（单输入限制） | 是（通过选择 $\delta_r$） |

---

## 5 前馈转角 $\delta_{ff}$ 的求解

### 5.1 目标：令 $e_{1,ss} = 0$

将 $e_{1,ss} = 0$ 代入式 (I)：

$$
0 = \left(1 - \frac{l_rC_r}{l_fC_f} - k_3\right)e_{2,ss} + \delta_{ff} - \frac{l_rC_r}{l_fC_f}\,\delta_r - \frac{L}{R}
$$

解得：

$$
\delta_{ff} = \frac{L}{R} + \frac{l_rC_r}{l_fC_f}\,\delta_r - \left(1 - \frac{l_rC_r}{l_fC_f} - k_3\right)e_{2,ss}
$$

### 5.2 展开化简

代入 $e_{2,ss} = \dfrac{ml_fv_x^2}{C_rLR} - \delta_r$：

$$
\delta_{ff} = \frac{L}{R} + \frac{l_rC_r}{l_fC_f}\,\delta_r - \left(\frac{l_fC_f - l_rC_r}{l_fC_f} - k_3\right)\left(\frac{ml_fv_x^2}{C_rLR} - \delta_r\right)
$$

展开乘积：

$$
= \frac{L}{R} + \frac{l_rC_r}{l_fC_f}\,\delta_r - \frac{l_fC_f - l_rC_r}{l_fC_f}\cdot\frac{ml_fv_x^2}{C_rLR} + \frac{l_fC_f - l_rC_r}{l_fC_f}\,\delta_r + k_3\left(\frac{ml_fv_x^2}{C_rLR} - \delta_r\right)
$$

合并 $\delta_r$ 项：

$$
\frac{l_rC_r}{l_fC_f}\,\delta_r + \frac{l_fC_f - l_rC_r}{l_fC_f}\,\delta_r = \frac{l_rC_r + l_fC_f - l_rC_r}{l_fC_f}\,\delta_r = \delta_r
$$

化简不足转向项：

$$
-\frac{l_fC_f - l_rC_r}{l_fC_f}\cdot\frac{ml_fv_x^2}{C_rLR} = -\frac{(l_fC_f - l_rC_r)mv_x^2}{C_fC_rLR} = \frac{mv_x^2}{R}\left(\frac{l_r}{C_fL} - \frac{l_f}{C_rL}\right)
$$

因此：

$$
\delta_{ff} = \frac{L}{R} + \frac{mv_x^2}{R}\left(\frac{l_r}{C_fL} - \frac{l_f}{C_rL}\right) + \delta_r + k_3\,e_{2,ss}
$$

利用不足转向梯度 $K_{us} = \dfrac{m}{L}\left(\dfrac{l_r}{C_f} - \dfrac{l_f}{C_r}\right)$：

$$
\boxed{\delta_{ff} = \frac{L + K_{us}v_x^2}{R} + \delta_r + k_3\,e_{2,ss}}
$$

其中 $e_{2,ss} = \dfrac{ml_fv_x^2}{C_rLR} - \delta_r$。

### 5.3 等价形式

将 $e_{2,ss}$ 代入展开：

$$
\delta_{ff} = \frac{L + K_{us}v_x^2}{R} + (1 - k_3)\delta_r + k_3\cdot\frac{ml_fv_x^2}{C_rLR}
$$

### 5.4 物理解读

$$
\delta_{ff} = \underbrace{\frac{L + K_{us}v_x^2}{R}}_{\text{经典前馈}} + \underbrace{\delta_r}_{\text{后轮转角补偿}} + \underbrace{k_3\,e_{2,ss}}_{\text{航向误差反馈补偿}}
$$

| 项 | 含义 |
|---|---|
| $(L + K_{us}v_x^2)/R$ | Ackermann + 不足转向补偿（与纯前轮转向相同） |
| $\delta_r$ | 后轮转向产生的等效前轮转角需求 |
| $k_3\,e_{2,ss}$ | 补偿稳态航向误差对反馈律的影响 |

**$\delta_r$ 项的来源**：后轮转角 $\delta_r$ 产生侧向力改变了稳态力平衡，前轮需要额外转角来维持圆弧跟踪。

---

## 6 特殊情况

### 6.1 $\delta_r = 0$（退化为纯前轮转向）

$$
e_{2,ss}\big|_{\delta_r=0} = \frac{ml_fv_x^2}{C_rLR}
$$

$$
\delta_{ff}\big|_{\delta_r=0} = \frac{L + K_{us}v_x^2}{R} + k_3\cdot\frac{ml_fv_x^2}{C_rLR}
$$

与 100a 的结果完全一致。

### 6.2 $k_3 = 0$（无航向误差反馈）

$$
\delta_{ff}\big|_{k_3=0} = \frac{L + K_{us}v_x^2}{R} + \delta_r
$$

前馈为经典公式加上后轮转角。

### 6.3 $e_{2,ss} = 0$（完全消除航向误差）

条件：$\delta_r = \dfrac{ml_fv_x^2}{C_rLR}$

此时：

$$
\delta_{ff} = \frac{L + K_{us}v_x^2}{R} + \frac{ml_fv_x^2}{C_rLR}
$$

---

## 7 完整稳态误差向量

当 $\delta_{ff}$ 按第 5 节选取时：

$$
\mathbf{x}_{ss} = \begin{bmatrix} 0 \\ 0 \\ e_{2,ss} \\ 0 \end{bmatrix}
= \begin{bmatrix} 0 \\ 0 \\ \dfrac{ml_fv_x^2}{C_rLR} - \delta_r \\ 0 \end{bmatrix}
$$

---

## 8 一般情况：$e_{1,ss}$ 的表达式

若不指定 $e_{1,ss} = 0$，由式 (I)：

$$
e_{1,ss} = \frac{1}{k_1}\left[\delta_{ff} - \frac{l_rC_r}{l_fC_f}\,\delta_r - \frac{L}{R} + \left(1 - \frac{l_rC_r}{l_fC_f} - k_3\right)e_{2,ss}\right]
$$

即：

$$
\boxed{e_{1,ss} = \frac{1}{k_1}\left(\delta_{ff} - \delta_{ff}^*\right)}
$$

其中 $\delta_{ff}^* = \dfrac{L + K_{us}v_x^2}{R} + \delta_r + k_3\,e_{2,ss}$ 为使 $e_{1,ss} = 0$ 的理想前馈。

---

## 9 总结

| 量 | 表达式 | 是否依赖 $K$ |
|---|---|---|
| $e_{2,ss}$ | $\dfrac{ml_fv_x^2}{C_rLR} - \delta_r$ | **否** |
| $\delta_{ff}$（使 $e_1=0$） | $\dfrac{L + K_{us}v_x^2}{R} + \delta_r + k_3\,e_{2,ss}$ | 仅依赖 $k_3$ |
| $e_{1,ss}$（一般） | $\dfrac{1}{k_1}\left(\delta_{ff} - \delta_{ff}^*\right)$ | 依赖 $k_1, k_3$ |

**4WS 的核心优势**：通过选择 $\delta_r = ml_fv_x^2\kappa/(C_rL)$，可以使 $e_{2,ss} = 0$，从而实现零稳态航向误差——这是纯前轮转向无法做到的。



---

## 10 SymPy 验证

验证脚本：`doc/verify_100b_4ws_rear_steady_state.py`

验证内容：
1. $e_{2,ss}$ 与反馈增益 $K$ 无关 ✓
2. $e_{2,ss} = ml_fv_x^2/(C_rLR) - \delta_r$ ✓
3. 前馈转角 $\delta_{ff}$ 使 $e_{1,ss} = 0$ ✓
4. $\delta_r = 0$ 时退化为纯前轮转向结果（100a）✓
5. $k_3 = 0$ 时退化为经典公式 + $\delta_r$ ✓
6. 矩阵逆直接求解验证 ✓
7. $e_{1,ss}$ 一般表达式验证 ✓
8. $e_{2,ss} = 0$ 的条件验证（4WS 优势）✓
9. 数值验证（典型车辆参数）✓

```python
"""
验证 4WS 后轴参考点闭环稳态跟踪误差和前馈转角的推导 (100b)。
基于 03c_error_rear_4ws.md 的后轴误差状态空间。

运行: python3 doc/verify_100b_4ws_rear_steady_state.py
"""
from sympy import symbols, Matrix, simplify, solve, factor, cancel

# 定义符号
Cf, Cr, lf, lr, m, Iz, vx, R = symbols('Cf Cr lf lr m Iz vx R', positive=True)
k1, k2, k3, k4, delta_ff, delta_r = symbols('k1 k2 k3 k4 delta_ff delta_r')
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

K = Matrix([[k1, k2, k3, k4]])

# 闭环矩阵 (仅前轮反馈)
A_cl = A - Bf * K

# 稳态条件
dot_theta_ref = vx / R

# 稳态时 x_ss = [e1_ss, 0, e2_ss, 0]
e1_ss, e2_ss = symbols('e1_ss e2_ss')
x_ss = Matrix([e1_ss, 0, e2_ss, 0])

# 稳态方程: 0 = A_cl * x_ss + Bf * delta_ff + Br * delta_r + G * dot_theta_ref
steady_eq = A_cl * x_ss + Bf * delta_ff + Br * delta_r + G * dot_theta_ref

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

# 验证 e2_ss 不含 k1, k2, k3, k4
has_k = any(e2_ss_expr.has(ki) for ki in [k1, k2, k3, k4])
print(f"e2_ss 含有反馈增益 K: {has_k}")

# 验证解析表达式: e2_ss = m*lf*vx^2/(Cr*L*R) - delta_r
e2_ss_expected = m * lf * vx**2 / (Cr * L * R) - delta_r
diff_e2 = simplify(e2_ss_expr - e2_ss_expected)
print(f"e2_ss - 预期值 = {diff_e2}")
assert diff_e2 == 0, f"e2_ss 验证失败! diff = {diff_e2}"
print("✓ e2_ss = ml_f v_x^2/(C_r L R) - δ_r 验证通过")

print("\n" + "=" * 60)
print("验证 2: 前馈转角 delta_ff 使 e1_ss = 0")
print("=" * 60)

# 令 e1_ss = 0，解 delta_ff
e1_ss_expr = sol[e1_ss]
delta_ff_sol = solve(e1_ss_expr, delta_ff)[0]
delta_ff_simplified = simplify(delta_ff_sol)

print(f"delta_ff(e1=0) = {delta_ff_simplified}")

# 预期前馈: (L + K_us*vx^2)/R + delta_r + k3 * e2_ss
K_us = m / L * (lr / Cf - lf / Cr)
delta_ff_expected = (L + K_us * vx**2) / R + delta_r + k3 * e2_ss_expected

diff_ff = simplify(delta_ff_sol - delta_ff_expected)
print(f"delta_ff - 预期值 = {diff_ff}")
assert diff_ff == 0, f"delta_ff 验证失败! diff = {diff_ff}"
print("✓ delta_ff 验证通过")

print("\n" + "=" * 60)
print("验证 3: delta_r=0 退化为纯前轮转向 (100a)")
print("=" * 60)

e2_ss_dr0 = e2_ss_expected.subs(delta_r, 0)
delta_ff_dr0 = delta_ff_expected.subs(delta_r, 0)

# 100a 的结果
e2_ss_100a = m * lf * vx**2 / (Cr * L * R)
delta_ff_100a = (L + K_us * vx**2) / R + k3 * e2_ss_100a

diff_e2_dr0 = simplify(e2_ss_dr0 - e2_ss_100a)
diff_ff_dr0 = simplify(delta_ff_dr0 - delta_ff_100a)
print(f"e2_ss(δ_r=0) - 100a 结果 = {diff_e2_dr0}")
print(f"delta_ff(δ_r=0) - 100a 结果 = {diff_ff_dr0}")
assert diff_e2_dr0 == 0, "δ_r=0 退化验证失败 (e2)!"
assert diff_ff_dr0 == 0, "δ_r=0 退化验证失败 (delta_ff)!"
print("✓ δ_r=0 退化为 100a 验证通过")

print("\n" + "=" * 60)
print("验证 4: k3=0 时退化为经典公式 + delta_r")
print("=" * 60)

delta_ff_k3_0 = delta_ff_expected.subs(k3, 0)
delta_ff_classic_4ws = (L + K_us * vx**2) / R + delta_r
diff_classic = simplify(delta_ff_k3_0 - delta_ff_classic_4ws)
print(f"k3=0 时 delta_ff - 经典公式 = {diff_classic}")
assert diff_classic == 0, "经典公式退化验证失败!"
print("✓ k3=0 退化验证通过")

print("\n" + "=" * 60)
print("验证 5: 直接用矩阵逆验证")
print("=" * 60)

# 代入 delta_ff = delta_ff_expected，用矩阵逆求 x_ss
rhs = Bf * delta_ff_expected + Br * delta_r + G * dot_theta_ref
x_ss_full = simplify(-A_cl.inv() * rhs)

# 验证 e1 = 0
e1_check = simplify(x_ss_full[0])
print(f"e1_ss (矩阵逆) = {e1_check}")
assert e1_check == 0, f"矩阵逆验证 e1=0 失败! got {e1_check}"
print("✓ e1_ss = 0 验证通过")

# 验证 e2 = e2_ss_expected
e2_check = simplify(x_ss_full[2] - e2_ss_expected)
print(f"e2_ss (矩阵逆) - 预期 = {e2_check}")
assert e2_check == 0, f"矩阵逆验证 e2 失败! got {e2_check}"
print("✓ e2_ss 验证通过")

# 验证 dot_e1 = dot_e2 = 0
de1_check = simplify(x_ss_full[1])
de2_check = simplify(x_ss_full[3])
print(f"dot_e1_ss (矩阵逆) = {de1_check}")
print(f"dot_e2_ss (矩阵逆) = {de2_check}")
assert de1_check == 0, "矩阵逆验证 dot_e1=0 失败!"
assert de2_check == 0, "矩阵逆验证 dot_e2=0 失败!"
print("✓ dot_e1 = dot_e2 = 0 验证通过")

print("\n" + "=" * 60)
print("验证 6: e1_ss 一般表达式")
print("=" * 60)

# 一般情况: e1_ss = (1/k1) * (delta_ff - delta_ff_star)
delta_ff_star = (L + K_us * vx**2) / R + delta_r + k3 * e2_ss_expected
e1_ss_general_expected = (delta_ff - delta_ff_star) / k1

diff_e1 = simplify(e1_ss_expr - e1_ss_general_expected)
print(f"e1_ss - 预期 (1/k1)(delta_ff - delta_ff*) = {diff_e1}")
assert diff_e1 == 0, f"e1_ss 一般表达式验证失败! diff = {diff_e1}"
print("✓ e1_ss 一般表达式验证通过")

print("\n" + "=" * 60)
print("验证 7: e2_ss=0 的条件")
print("=" * 60)

# 当 delta_r = m*lf*vx^2/(Cr*L*R) 时 e2_ss = 0
delta_r_zero_e2 = m * lf * vx**2 / (Cr * L * R)
e2_check_zero = simplify(e2_ss_expected.subs(delta_r, delta_r_zero_e2))
print(f"e2_ss(δ_r = ml_fv_x^2/(C_rLR)) = {e2_check_zero}")
assert e2_check_zero == 0, "e2_ss=0 条件验证失败!"
print("✓ 4WS 可完全消除稳态航向误差验证通过")

print("\n" + "=" * 60)
print("验证 8: 数值验证（典型车辆参数）")
print("=" * 60)

import math

# 典型参数
params = {
    Cf: 80000,   # N/rad
    Cr: 80000,   # N/rad
    lf: 1.2,     # m
    lr: 1.5,     # m
    m: 1500,     # kg
    Iz: 3000,    # kg*m^2
    vx: 20,      # m/s
    R: 100,      # m
    k1: 0.05,
    k2: 0.1,
    k3: 1.0,
    k4: 0.5,
    delta_r: 0.01,  # rad
}

# 数值计算
e2_num = float(e2_ss_expected.subs(params))
delta_ff_num = float(delta_ff_expected.subs(params))

print(f"车辆参数: Cf={params[Cf]}, Cr={params[Cr]}, lf={params[lf]}, lr={params[lr]}")
print(f"          m={params[m]}, Iz={params[Iz]}, vx={params[vx]}, R={params[R]}")
print(f"反馈增益: k1={params[k1]}, k2={params[k2]}, k3={params[k3]}, k4={params[k4]}")
print(f"后轮转角: delta_r={params[delta_r]} rad = {params[delta_r]*180/math.pi:.4f} deg")
print(f"")
print(f"e2_ss = {e2_num:.6f} rad = {e2_num*180/math.pi:.4f} deg")
print(f"delta_ff = {delta_ff_num:.6f} rad = {delta_ff_num*180/math.pi:.4f} deg")
print(f"经典前馈+δ_r (k3=0) = {float(((L+K_us*vx**2)/R + delta_r).subs(params)):.6f} rad")

# 数值矩阵逆验证
A_num = A.subs(params)
Bf_num = Bf.subs(params)
Br_num = Br.subs(params)
G_num = G.subs(params)
K_num = K.subs(params)
A_cl_num = A_num - Bf_num * K_num

rhs_num = Bf_num * delta_ff_num + Br_num * params[delta_r] + G_num * float(dot_theta_ref.subs(params))
x_ss_num = -A_cl_num.inv() * rhs_num

print(f"\n数值矩阵逆验证:")
print(f"  e1_ss  = {float(x_ss_num[0]):.2e} (应为 0)")
print(f"  de1_ss = {float(x_ss_num[1]):.2e} (应为 0)")
print(f"  e2_ss  = {float(x_ss_num[2]):.6f} (预期 {e2_num:.6f})")
print(f"  de2_ss = {float(x_ss_num[3]):.2e} (应为 0)")

# 对比纯前轮转向
params_no_4ws = dict(params)
params_no_4ws[delta_r] = 0
e2_no_4ws = float(e2_ss_expected.subs(params_no_4ws))
print(f"\n对比: 纯前轮转向 e2_ss = {e2_no_4ws:.6f} rad = {e2_no_4ws*180/math.pi:.4f} deg")
print(f"       4WS (δ_r={params[delta_r]}) e2_ss = {e2_num:.6f} rad")
print(f"       4WS 减小了 {(e2_no_4ws - e2_num)/e2_no_4ws*100:.1f}% 的航向误差")

print("\n" + "=" * 60)
print("所有验证通过 ✓")
print("=" * 60)
```
