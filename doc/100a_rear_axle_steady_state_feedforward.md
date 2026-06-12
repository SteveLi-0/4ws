# 后轴参考点闭环稳态跟踪误差与前馈转角推导

> 基于 [[03a_error_rear_front_steer]] 的后轴误差状态空间，推导状态反馈闭环下定圆稳态跟踪误差和前馈转角。

---

## 1 问题设定

基于 `03a_error_rear_front_steer.md` 第 6 节的后轴误差状态空间：

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B\,\delta_f + G\,\dot\theta_{\text{ref}}
$$

状态向量 $\mathbf{x} = [e_1,\;\dot{e}_1,\;e_2,\;\dot{e}_2]^T$，其中 $e_1$、$e_2$ 定义在后轴中心。

### 1.1 系统矩阵（后轴定义）

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\[6pt]
0 & -\dfrac{C_f\eta + C_r\xi}{mI_zv_x} & \dfrac{C_f\eta + C_r\xi}{mI_z} & -\dfrac{C_fL\eta}{mI_zv_x} \\[6pt]
0 & 0 & 0 & 1 \\[6pt]
0 & -\dfrac{l_fC_f - l_rC_r}{I_zv_x} & \dfrac{l_fC_f - l_rC_r}{I_z} & -\dfrac{l_fC_fL}{I_zv_x}
\end{bmatrix}
$$

$$
B = \begin{bmatrix} 0 \\ \dfrac{C_f\eta}{mI_z} \\ 0 \\ \dfrac{l_fC_f}{I_z} \end{bmatrix}, \qquad
G = \begin{bmatrix} 0 \\ -\dfrac{C_fL\eta}{mI_zv_x} - v_x \\ 0 \\ -\dfrac{l_fC_fL}{I_zv_x} \end{bmatrix}
$$

其中 $\eta = I_z - ml_fl_r$，$\xi = I_z + ml_r^2$，$L = l_f + l_r$。

### 1.2 状态反馈控制律

$$
\delta_f = -K\mathbf{x} + \delta_{ff} = -k_1 e_1 - k_2 \dot{e}_1 - k_3 e_2 - k_4 \dot{e}_2 + \delta_{ff}
$$

### 1.3 闭环系统

$$
\dot{\mathbf{x}} = (A - BK)\,\mathbf{x} + B\,\delta_{ff} + G\,\dot\theta_{\text{ref}}
$$

---

## 2 定圆稳态条件

- 曲率恒定：$\kappa = 1/R$
- 参考航向变化率恒定：$\dot\theta_{\text{ref}} = \kappa v_x = v_x/R$
- 稳态条件：$\dot{\mathbf{x}} = 0$

稳态时 $\dot{e}_1 = \dot{e}_2 = 0$，状态向量简化为：

$$
\mathbf{x}_{ss} = [e_{1,ss},\;0,\;e_{2,ss},\;0]^T
$$

---

## 3 稳态误差求解

### 3.1 有效方程

第 1 行和第 3 行在 $\dot{e}_1 = \dot{e}_2 = 0$ 时自动满足。有效约束来自第 2 行（$\ddot{e}_1 = 0$）和第 4 行（$\ddot{e}_2 = 0$）。

**第 4 行**（$\ddot{e}_2 = 0$）：

$$
0 = \left(\frac{l_fC_f - l_rC_r}{I_z} - \frac{l_fC_f}{I_z}k_3\right)e_{2,ss} - \frac{l_fC_f}{I_z}k_1\,e_{1,ss} + \frac{l_fC_f}{I_z}\,\delta_{ff} - \frac{l_fC_fL}{I_zv_x}\cdot\frac{v_x}{R}
$$

两侧乘以 $I_z/(l_fC_f)$：

$$
0 = \left(1 - \frac{l_rC_r}{l_fC_f} - k_3\right)e_{2,ss} - k_1\,e_{1,ss} + \delta_{ff} - \frac{L}{R} \tag{I}
$$

**第 2 行**（$\ddot{e}_1 = 0$）：

$$
0 = \left(\frac{C_f\eta + C_r\xi}{mI_z} - \frac{C_f\eta}{mI_z}k_3\right)e_{2,ss} - \frac{C_f\eta}{mI_z}k_1\,e_{1,ss} + \frac{C_f\eta}{mI_z}\,\delta_{ff} + \left(-\frac{C_fL\eta}{mI_zv_x} - v_x\right)\frac{v_x}{R}
$$

两侧乘以 $mI_z/(C_f\eta)$（假设 $\eta \neq 0$）：

$$
0 = \left(\frac{C_f\eta + C_r\xi}{C_f\eta} - k_3\right)e_{2,ss} - k_1\,e_{1,ss} + \delta_{ff} + \frac{mI_z}{C_f\eta}\left(-\frac{C_fL\eta}{mI_zv_x} - v_x\right)\frac{v_x}{R}
$$

化简系数：

$$
\frac{C_f\eta + C_r\xi}{C_f\eta} = 1 + \frac{C_r\xi}{C_f\eta}
$$

化简扰动项：

$$
\frac{mI_z}{C_f\eta}\left(-\frac{C_fL\eta}{mI_zv_x} - v_x\right)\frac{v_x}{R} = \left(-\frac{L}{R}\right) + \left(-\frac{mI_zv_x^2}{C_f\eta R}\right) = -\frac{L}{R} - \frac{mI_zv_x^2}{C_f\eta R}
$$

因此第 2 行化简为：

$$
0 = \left(1 + \frac{C_r\xi}{C_f\eta} - k_3\right)e_{2,ss} - k_1\,e_{1,ss} + \delta_{ff} - \frac{L}{R} - \frac{mI_zv_x^2}{C_f\eta R} \tag{II}
$$

### 3.2 两方程相减求 $e_{2,ss}$

式 (II) $-$ 式 (I)：

$$
0 = \frac{C_r\xi}{C_f\eta}\,e_{2,ss} - \frac{mI_zv_x^2}{C_f\eta R} + \frac{L}{R} - \frac{L}{R}
$$

等等，让我重新仔细计算。式 (I) 的扰动项为 $-L/R$，式 (II) 的扰动项为 $-L/R - mI_zv_x^2/(C_f\eta R)$。

(II) $-$ (I)：

$$
0 = \left[\left(1 + \frac{C_r\xi}{C_f\eta} - k_3\right) - \left(1 - \frac{l_rC_r}{l_fC_f} - k_3\right)\right]e_{2,ss} + \left[-\frac{L}{R} - \frac{mI_zv_x^2}{C_f\eta R}\right] - \left[-\frac{L}{R}\right]
$$

系数化简：

$$
\frac{C_r\xi}{C_f\eta} + \frac{l_rC_r}{l_fC_f} = \frac{C_r(I_z + ml_r^2)}{C_f(I_z - ml_fl_r)} + \frac{l_rC_r}{l_fC_f}
$$

常数项化简：

$$
-\frac{mI_zv_x^2}{C_f\eta R}
$$

因此：

$$
\left(\frac{C_r\xi}{C_f\eta} + \frac{l_rC_r}{l_fC_f}\right)e_{2,ss} = \frac{mI_zv_x^2}{C_f\eta R}
$$

化简左侧系数，通分 $l_fC_f\eta$：

$$
\frac{C_r\xi \cdot l_f + l_rC_r\eta}{l_fC_f\eta} = \frac{C_r(l_f\xi + l_r\eta)}{l_fC_f\eta}
$$

展开 $l_f\xi + l_r\eta$：

$$
l_f(I_z + ml_r^2) + l_r(I_z - ml_fl_r) = l_fI_z + ml_fl_r^2 + l_rI_z - ml_fl_r^2 = (l_f + l_r)I_z = LI_z
$$

因此：

$$
\frac{C_r \cdot LI_z}{l_fC_f\eta}\,e_{2,ss} = \frac{mI_zv_x^2}{C_f\eta R}
$$

消去 $I_z/(C_f\eta)$：

$$
\frac{C_rL}{l_f}\,e_{2,ss} = \frac{mv_x^2}{R}
$$

$$
\boxed{e_{2,ss} = \frac{ml_fv_x^2}{C_rLR}}
$$

---

## 4 稳态航向误差的物理解读

$$
e_{2,ss} = \frac{ml_fv_x^2}{C_rLR} = \frac{ml_fv_x^2\kappa}{C_rL}
$$

这正是后轮稳态侧偏角 $\alpha_{r,ss}$（见 03a 第 9.1 节）。

**关键性质**：
- $e_{2,ss}$ **与反馈增益 $K$ 无关**
- 低速时 $e_{2,ss} \to 0$（后轴定义的优势）
- 与质心定义的 $e_{2,ss} = ml_fv_x^2/(C_rLR) - l_r/R$ 相比，少了几何项 $-l_r/R$

| | 质心定义 | 后轴定义 |
|---|---------|---------|
| $e_{2,ss}$ | $\dfrac{ml_fv_x^2}{C_rLR} - \dfrac{l_r}{R}$ | $\dfrac{ml_fv_x^2}{C_rLR}$ |
| 低速极限 | $-l_r/R \neq 0$ | $0$ |
| 物理来源 | 质心侧偏角 | 后轮侧偏角 |

---

## 5 前馈转角 $\delta_{ff}$ 的求解

### 5.1 目标：令 $e_{1,ss} = 0$

将 $e_{1,ss} = 0$ 代入式 (I)：

$$
0 = \left(1 - \frac{l_rC_r}{l_fC_f} - k_3\right)e_{2,ss} + \delta_{ff} - \frac{L}{R}
$$

解得：

$$
\delta_{ff} = \frac{L}{R} - \left(1 - \frac{l_rC_r}{l_fC_f} - k_3\right)e_{2,ss}
$$

### 5.2 展开化简

$$
\left(1 - \frac{l_rC_r}{l_fC_f}\right)e_{2,ss} = \frac{l_fC_f - l_rC_r}{l_fC_f}\cdot\frac{ml_fv_x^2}{C_rLR} = \frac{(l_fC_f - l_rC_r)mv_x^2}{C_fC_rLR}
$$

$$
= \frac{mv_x^2}{R}\left(\frac{l_f}{C_rL} - \frac{l_r}{C_fL}\right)
= \frac{mv_x^2}{R}\left(\frac{l_f}{C_rL} - \frac{l_r}{C_fL}\right)
$$

注意这里的符号：$\dfrac{l_f}{C_rL} - \dfrac{l_r}{C_fL} = -\left(\dfrac{l_r}{C_fL} - \dfrac{l_f}{C_rL}\right) = -K_{us}/m \cdot L$

更直接地：

$$
\delta_{ff} = \frac{L}{R} - \frac{(l_fC_f - l_rC_r)mv_x^2}{C_fC_rLR} + k_3\,e_{2,ss}
$$

整理不足转向项：

$$
-\frac{(l_fC_f - l_rC_r)m v_x^2}{C_fC_rLR} = \frac{mv_x^2}{R}\cdot\frac{l_rC_r - l_fC_f}{C_fC_rL} = \frac{mv_x^2}{R}\left(\frac{l_r}{C_fL} - \frac{l_f}{C_rL}\right)
$$

因此：

$$
\boxed{\delta_{ff} = \frac{L}{R} + \frac{mv_x^2}{R}\left(\frac{l_r}{C_fL} - \frac{l_f}{C_rL}\right) + k_3\,e_{2,ss}}
$$

其中 $e_{2,ss} = \dfrac{ml_fv_x^2}{C_rLR}$。

### 5.3 等价形式

利用不足转向梯度 $K_{us} = \dfrac{m}{L}\left(\dfrac{l_r}{C_f} - \dfrac{l_f}{C_r}\right)$：

$$
\delta_{ff} = \frac{L + K_{us}v_x^2}{R} + k_3\,e_{2,ss}
$$

当 $k_3 = 0$ 时退化为经典 Ackermann + 不足转向补偿：

$$
\delta_{ff}\big|_{k_3=0} = \frac{L + K_{us}v_x^2}{R} = (L + K_{us}v_x^2)\kappa
$$

与 03a 第 9 节的前馈公式一致。

### 5.4 物理解读

$$
\delta_{ff} = \underbrace{\frac{L}{R}}_{\text{Ackermann}} + \underbrace{K_{us}\frac{v_x^2}{R}}_{\text{不足转向补偿}} + \underbrace{k_3\,e_{2,ss}}_{\text{航向误差反馈补偿}}
$$

$k_3\,e_{2,ss}$ 项的作用：稳态时存在非零航向误差 $e_{2,ss}$，反馈律中 $-k_3 e_2$ 会产生额外的转角修正，前馈需要预补偿这个量以维持 $e_{1,ss} = 0$。

---

## 6 完整稳态误差向量

当 $\delta_{ff}$ 按上式选取时：

$$
\mathbf{x}_{ss} = \begin{bmatrix} 0 \\ 0 \\ e_{2,ss} \\ 0 \end{bmatrix}
= \begin{bmatrix} 0 \\ 0 \\ \dfrac{ml_fv_x^2}{C_rLR} \\ 0 \end{bmatrix}
$$

---

## 7 一般情况：$e_{1,ss}$ 的表达式

若不指定 $e_{1,ss} = 0$，由式 (I)：

$$
e_{1,ss} = \frac{1}{k_1}\left[\delta_{ff} - \frac{L}{R} + \left(1 - \frac{l_rC_r}{l_fC_f} - k_3\right)e_{2,ss}\right]
$$

即：

$$
\boxed{e_{1,ss} = \frac{1}{k_1}\left(\delta_{ff} - \delta_{ff}^*\right)}
$$

其中 $\delta_{ff}^* = \dfrac{L}{R} + K_{us}\dfrac{v_x^2}{R} + k_3\,e_{2,ss}$ 为使 $e_{1,ss} = 0$ 的理想前馈。

---

## 8 与质心定义结果的对比

| 量 | 质心定义（99b） | 后轴定义（本文） |
|---|---|---|
| $e_{2,ss}$ | $\dfrac{ml_fv_x^2}{C_rLR} - \dfrac{l_r}{R}$ | $\dfrac{ml_fv_x^2}{C_rLR}$ |
| $\delta_{ff}$（$e_1=0$） | $\dfrac{L}{R} + K_{us}\dfrac{v_x^2}{R} + k_3\,e_{2,ss}^{\text{CG}}$ | $\dfrac{L}{R} + K_{us}\dfrac{v_x^2}{R} + k_3\,e_{2,ss}^{\text{rear}}$ |
| $k_3=0$ 时 $\delta_{ff}$ | $(L + K_{us}v_x^2)/R$ | $(L + K_{us}v_x^2)/R$ |

**关键观察**：
1. 两种定义下 $k_3=0$ 时的前馈转角**完全相同**（物理上必须如此）
2. $e_{2,ss}$ 不同是因为参考点不同导致的几何关系差异
3. 后轴定义在低速时 $e_{2,ss} \to 0$，控制器设计更简洁

---

## 9 总结

| 量 | 表达式 | 是否依赖 $K$ |
|---|---|---|
| $e_{2,ss}$ | $\dfrac{ml_fv_x^2}{C_rLR}$ | **否** |
| $\delta_{ff}$（使 $e_1=0$） | $\dfrac{L + K_{us}v_x^2}{R} + k_3\,e_{2,ss}$ | 仅依赖 $k_3$ |
| $e_{1,ss}$（一般） | $\dfrac{1}{k_1}\left(\delta_{ff} - \delta_{ff}^*\right)$ | 依赖 $k_1, k_3$ |



---

## 10 SymPy 验证

验证脚本：`doc/verify_100a_rear_axle_steady_state.py`

验证内容：
1. $e_{2,ss}$ 与反馈增益 $K$ 无关 ✓
2. $e_{2,ss} = ml_fv_x^2/(C_rLR)$ ✓
3. 前馈转角 $\delta_{ff}$ 使 $e_{1,ss} = 0$ ✓
4. $k_3 = 0$ 时退化为经典 Ackermann + 不足转向公式 ✓
5. 矩阵逆直接求解验证 ✓
6. $e_{1,ss}$ 一般表达式验证 ✓
7. 与质心定义的差值为 $l_r/R$ ✓
8. 数值验证（典型车辆参数）✓

```python
"""
验证后轴参考点闭环稳态跟踪误差和前馈转角的推导 (100a)。
基于 03a_error_rear_front_steer.md 的后轴误差状态空间。

运行: python3 doc/verify_100a_rear_axle_steady_state.py
"""
from sympy import symbols, Matrix, simplify, solve, factor, cancel, pprint

# 定义符号
Cf, Cr, lf, lr, m, Iz, vx, R = symbols('Cf Cr lf lr m Iz vx R', positive=True)
k1, k2, k3, k4, delta_ff = symbols('k1 k2 k3 k4 delta_ff')
L = lf + lr
eta = Iz - m * lf * lr
xi = Iz + m * lr**2

# ============================================================
# 后轴定义的系统矩阵 (来自 03a)
# ============================================================
A = Matrix([
    [0, 1, 0, 0],
    [0, -(Cf*eta + Cr*xi)/(m*Iz*vx), (Cf*eta + Cr*xi)/(m*Iz), -(Cf*L*eta)/(m*Iz*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf - lr*Cr)/(Iz*vx), (lf*Cf - lr*Cr)/Iz, -(lf*Cf*L)/(Iz*vx)]
])

B = Matrix([0, Cf*eta/(m*Iz), 0, lf*Cf/Iz])

G = Matrix([0, -(Cf*L*eta)/(m*Iz*vx) - vx, 0, -(lf*Cf*L)/(Iz*vx)])

K = Matrix([[k1, k2, k3, k4]])

# 闭环矩阵
A_cl = A - B * K

# 稳态条件
dot_theta_ref = vx / R

# 稳态时 x_ss = [e1_ss, 0, e2_ss, 0]
e1_ss, e2_ss = symbols('e1_ss e2_ss')
x_ss = Matrix([e1_ss, 0, e2_ss, 0])

# 稳态方程
steady_eq = A_cl * x_ss + B * delta_ff + G * dot_theta_ref

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

# 验证解析表达式
e2_ss_expected = m * lf * vx**2 / (Cr * L * R)
diff_e2 = simplify(e2_ss_expr - e2_ss_expected)
print(f"e2_ss - 预期值 (ml_f v_x^2 / (C_r L R)) = {diff_e2}")
assert diff_e2 == 0, "e2_ss 验证失败!"
print("✓ e2_ss 验证通过")

print("\n" + "=" * 60)
print("验证 2: 前馈转角 delta_ff 使 e1_ss = 0")
print("=" * 60)

# 令 e1_ss = 0，解 delta_ff
e1_ss_expr = sol[e1_ss]
delta_ff_sol = solve(e1_ss_expr, delta_ff)[0]
delta_ff_simplified = simplify(delta_ff_sol)

print(f"delta_ff(e1=0) = {delta_ff_simplified}")

# 预期前馈: L/R + K_us * vx^2/R + k3 * e2_ss
K_us = m / L * (lr / Cf - lf / Cr)
delta_ff_expected = L / R + K_us * vx**2 / R + k3 * e2_ss_expected

diff_ff = simplify(delta_ff_sol - delta_ff_expected)
print(f"delta_ff - 预期值 = {diff_ff}")
assert diff_ff == 0, "delta_ff 验证失败!"
print("✓ delta_ff 验证通过")

print("\n" + "=" * 60)
print("验证 3: k3=0 时退化为经典公式")
print("=" * 60)

delta_ff_k3_0 = delta_ff_expected.subs(k3, 0)
delta_ff_classic = L / R + K_us * vx**2 / R
diff_classic = simplify(delta_ff_k3_0 - delta_ff_classic)
print(f"k3=0 时 delta_ff - 经典公式 = {diff_classic}")
assert diff_classic == 0, "经典公式退化验证失败!"
print("✓ k3=0 退化验证通过")

print("\n" + "=" * 60)
print("验证 4: 直接用矩阵逆验证")
print("=" * 60)

# 代入 delta_ff = delta_ff_expected，用矩阵逆求 x_ss
rhs = B * delta_ff_expected + G * dot_theta_ref
x_ss_full = simplify(-A_cl.inv() * rhs)

# 验证 e1 = 0
e1_check = simplify(x_ss_full[0])
print(f"e1_ss (矩阵逆) = {e1_check}")
assert e1_check == 0, "矩阵逆验证 e1=0 失败!"
print("✓ e1_ss = 0 验证通过")

# 验证 e2 = e2_ss_expected
e2_check = simplify(x_ss_full[2] - e2_ss_expected)
print(f"e2_ss (矩阵逆) - 预期 = {e2_check}")
assert e2_check == 0, "矩阵逆验证 e2 失败!"
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
print("验证 5: e1_ss 一般表达式")
print("=" * 60)

# 一般情况: e1_ss = (1/k1) * (delta_ff - delta_ff_star)
delta_ff_star = L / R + K_us * vx**2 / R + k3 * e2_ss_expected
e1_ss_general_expected = (delta_ff - delta_ff_star) / k1

diff_e1 = simplify(e1_ss_expr - e1_ss_general_expected)
print(f"e1_ss - 预期 (1/k1)(delta_ff - delta_ff*) = {diff_e1}")
assert diff_e1 == 0, "e1_ss 一般表达式验证失败!"
print("✓ e1_ss 一般表达式验证通过")

print("\n" + "=" * 60)
print("验证 6: 与质心定义的对比")
print("=" * 60)

# 质心定义的 e2_ss
e2_ss_cg = m * lf * vx**2 / (Cr * L * R) - lr / R
print(f"e2_ss (质心) = ml_f v_x^2/(C_r L R) - l_r/R")
print(f"e2_ss (后轴) = ml_f v_x^2/(C_r L R)")
print(f"差值 = e2_rear - e2_cg = {simplify(e2_ss_expected - e2_ss_cg)} = l_r/R")
print("✓ 差值为 l_r/R，即后轴侧偏角 β_r 的几何分量")

print("\n" + "=" * 60)
print("验证 7: 数值验证（典型车辆参数）")
print("=" * 60)

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
}

# 数值计算
e2_num = float(e2_ss_expected.subs(params))
delta_ff_num = float(delta_ff_expected.subs(params))

print(f"车辆参数: Cf={params[Cf]}, Cr={params[Cr]}, lf={params[lf]}, lr={params[lr]}")
print(f"          m={params[m]}, Iz={params[Iz]}, vx={params[vx]}, R={params[R]}")
print(f"反馈增益: k1={params[k1]}, k2={params[k2]}, k3={params[k3]}, k4={params[k4]}")
print(f"")
print(f"e2_ss = {e2_num:.6f} rad = {e2_num*180/3.14159:.4f} deg")
print(f"delta_ff = {delta_ff_num:.6f} rad = {delta_ff_num*180/3.14159:.4f} deg")
print(f"经典前馈 (k3=0) = {float((L/R + K_us*vx**2/R).subs(params)):.6f} rad")

# 数值矩阵逆验证
A_num = A.subs(params)
B_num = B.subs(params)
G_num = G.subs(params)
K_num = K.subs(params)
A_cl_num = A_num - B_num * K_num

rhs_num = B_num * delta_ff_num + G_num * float(dot_theta_ref.subs(params))
x_ss_num = -A_cl_num.inv() * rhs_num

print(f"\n数值矩阵逆验证:")
print(f"  e1_ss  = {float(x_ss_num[0]):.2e} (应为 0)")
print(f"  de1_ss = {float(x_ss_num[1]):.2e} (应为 0)")
print(f"  e2_ss  = {float(x_ss_num[2]):.6f} (预期 {e2_num:.6f})")
print(f"  de2_ss = {float(x_ss_num[3]):.2e} (应为 0)")

print("\n" + "=" * 60)
print("所有验证通过 ✓")
print("=" * 60)
```
