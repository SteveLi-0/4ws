# 四轮转向扰动观测器——右手系独立推导验证（06b）

> 本文在严格右手坐标系下独立推导 4WS bicycle model 状态空间方程，与 06a 文档交叉验证。

---

## 1. 正方向约定（右手系）

坐标系：$x$ 前、$y$ 左、$z$ 上，角度方向遵循右手定则。

| 物理量 | 正方向 | 右手定则 |
| --- | --- | --- |
| $v_y$ | **向左** | $y$ 轴正方向 |
| $r = \dot\psi$ | **逆时针** | 右手绕 $z$ 轴 |
| $\delta_f, \delta_r$ | **向左转** | 右手绕 $z$ 轴 |
| $\phi$ | **左高右低** | 右手绕 $x$ 轴：$+y$（左）转向 $+z$（上）→ 左侧抬高 |
| $a_y$ | **向左** | $y$ 轴正方向 |
| $F_{yf}, F_{yr}$ | **向左** | $y$ 轴正方向 |

> $\phi > 0$（左高右低）→ 重力分量指向**右**（$-y$ 方向）。

---

## 2. 运动学

### 2.1 前后轴侧向速度（$v_y$ 向左为正）

左转（$r > 0$）时车头向左摆，前轴左向速度增大；车尾向右摆，后轴左向速度减小：

$$v_{y,f} = v_y + l_f r, \quad v_{y,r} = v_y - l_r r$$

### 2.2 速度方向角（向左为正）

$$\zeta_f = \frac{v_y + l_f r}{v_x}, \quad \zeta_r = \frac{v_y - l_r r}{v_x}$$

### 2.3 侧偏角（01 文档正值定义 $\alpha = \delta - \zeta$）

$$\alpha_f = \delta_f - \frac{v_y + l_f r}{v_x} \tag{S1}$$

$$\alpha_r = \delta_r - \frac{v_y - l_r r}{v_x} \tag{S2}$$

**校验**：$\delta = r = 0$，$v_y > 0$（车向左漂）→ $\alpha_f = -v_y/v_x < 0$ → $F_{yf} = C_f \alpha_f < 0$（向右）→ 恢复力 ✓

### 2.4 轮胎侧偏力（向左为正）

$$F_{yf} = C_f \alpha_f = C_f\!\left(\delta_f - \frac{v_y + l_f r}{v_x}\right) \tag{F1}$$

$$F_{yr} = C_r \alpha_r = C_r\!\left(\delta_r - \frac{v_y - l_r r}{v_x}\right) \tag{F2}$$

---

## 3. 动力学方程

### 3.1 侧向加速度

$$a_y = \dot{v}_y + v_x r$$

**校验**：匀速左转（$r > 0$，$\dot{v}_y = 0$）→ $a_y = v_x r > 0$（向左 = 向心方向） ✓

### 3.2 侧向力平衡

$$m\,a_y = F_{yf} + F_{yr} + F_\phi$$

$\phi > 0$（右手定则：左高右低）→ 重力分量向**右** → $F_\phi = -mg\phi$（$y$ 轴负方向）：

$$m(\dot{v}_y + v_x r) = F_{yf} + F_{yr} - mg\phi \tag{1}$$

**校验**：$\phi > 0$（左高右低）→ $-mg\phi$ 使 $a_y$ 减小（向右推）→ 物理正确 ✓

### 3.3 横摆力矩平衡

$$I_z \dot{r} = l_f F_{yf} - l_r F_{yr} \tag{2}$$

**校验**：$\delta_f > 0$（左转），$v_y = r = 0$ → $F_{yf} = C_f\delta_f > 0$（向左）→ 力矩 $l_f F_{yf} > 0$（逆时针） ✓

---

## 4. 代入化简

### 4.1 侧向方程

将 (F1)、(F2) 代入 (1)，移项 $mv_xr$：

$$m\dot{v}_y = C_f\!\left(\delta_f - \frac{v_y + l_f r}{v_x}\right) + C_r\!\left(\delta_r - \frac{v_y - l_r r}{v_x}\right) - mg\phi - mv_xr$$

展开归类：

$$m\dot{v}_y = -\frac{C_f+C_r}{v_x}\,v_y + \frac{-C_f l_f + C_r l_r}{v_x}\,r + C_f\delta_f + C_r\delta_r - mg\phi - mv_xr$$

两边除以 $m$：

$$\boxed{\dot{v}_y = -\frac{C_f+C_r}{mv_x}\,v_y + \left(\frac{-C_f l_f + C_r l_r}{mv_x} - v_x\right)r + \frac{C_f}{m}\,\delta_f + \frac{C_r}{m}\,\delta_r - g\,\phi} \tag{3}$$

### 4.2 横摆方程

将 (F1)、(F2) 代入 (2)：

$$l_f F_{yf} = C_f l_f\delta_f - \frac{C_f l_f}{v_x}(v_y + l_f r)$$

$$l_r F_{yr} = C_r l_r\delta_r - \frac{C_r l_r}{v_x}(v_y - l_r r)$$

作差，两边除以 $I_z$：

$$\boxed{\dot{r} = \frac{-C_f l_f + C_r l_r}{I_z v_x}\,v_y - \frac{C_f l_f^2 + C_r l_r^2}{I_z v_x}\,r + \frac{C_f l_f}{I_z}\,\delta_f - \frac{C_r l_r}{I_z}\,\delta_r} \tag{4}$$

---

## 5. 状态空间模型（右手系，$v_y$ 向左为正）

增广扰动状态 $\delta_d$（随机游走 $\dot\delta_d = 0$）：

$$\mathbf{x} = \begin{bmatrix} v_y \\ r \\ \delta_d \end{bmatrix}, \quad \mathbf{u} = \begin{bmatrix} \delta_f \\ \delta_r \\ \phi \end{bmatrix}, \quad y = r_{\text{meas}}$$

$$A = \begin{bmatrix}
-\dfrac{C_f + C_r}{m v_x} & \dfrac{-C_f l_f + C_r l_r}{m v_x} - v_x & \dfrac{C_f}{m} \\[8pt]
\dfrac{-C_f l_f + C_r l_r}{I_z v_x} & -\dfrac{C_f l_f^2 + C_r l_r^2}{I_z v_x} & \dfrac{C_f l_f}{I_z} \\[8pt]
0 & 0 & 0
\end{bmatrix} \tag{A-RH}$$

$$B = \begin{bmatrix}
\dfrac{C_f}{m} & \dfrac{C_r}{m} & -g \\[8pt]
\dfrac{C_f l_f}{I_z} & -\dfrac{C_r l_r}{I_z} & 0 \\[8pt]
0 & 0 & 0
\end{bmatrix} \tag{B-RH}$$

$$C = \begin{bmatrix} 0 & 1 & 0 \end{bmatrix}$$

---

## 6. 与 06a §3 (A-RH, B-RH) 对比

06a §3 采用与本文相同的右手坐标系和 $\phi$ 约定（右手绕 $x$ 轴，$\phi > 0$ 左高右低）。逐元素比对：

| 位置 | 本文 (RH) | 06a §3 (A-RH, B-RH) | 一致？ |
| --- | --- | --- | --- |
| $A$ 全部元素 | 同右 | 同左 | ✓ |
| $B_{11}$ | $+C_f/m$ | $+C_f/m$ | ✓ |
| $B_{12}$ | $+C_r/m$ | $+C_r/m$ | ✓ |
| $B_{13}$ | $-g$ | $-g$ | ✓ |
| $B_{21}$ | $+C_fl_f/I_z$ | $+C_fl_f/I_z$ | ✓ |
| $B_{22}$ | $-C_rl_r/I_z$ | $-C_rl_r/I_z$ | ✓ |

**右手系形式完全一致**，两篇文档的 $\phi$ 约定相同（右手定则，$\phi > 0$ 左高右低，重力向右即 $-y$），$B_{13} = -g$。

---

## 7. 变换到 06 文档 $v_y$ 方向

### 7.1 仅翻转 $v_y$

右手系中 $\phi$ 是由坐标轴定义的几何角度，不随 $v_y$ 方向翻转。因此只需状态变换，无需输入变换：

$$\tilde{v}_y = -v_y, \quad T = \mathrm{diag}(-1,\;1,\;1)$$

$$\tilde{A} = TAT^{-1}, \quad \tilde{B} = TB$$

### 7.2 变换后的矩阵

$$\tilde{A} = \begin{bmatrix}
-\dfrac{C_f + C_r}{m v_x} & \dfrac{C_f l_f - C_r l_r}{m v_x} + v_x & -\dfrac{C_f}{m} \\[8pt]
\dfrac{C_f l_f - C_r l_r}{I_z v_x} & -\dfrac{C_f l_f^2 + C_r l_r^2}{I_z v_x} & \dfrac{C_f l_f}{I_z} \\[8pt]
0 & 0 & 0
\end{bmatrix} \tag{A-06}$$

$$\tilde{B} = \begin{bmatrix}
-\dfrac{C_f}{m} & -\dfrac{C_r}{m} & g \\[8pt]
\dfrac{C_f l_f}{I_z} & -\dfrac{C_r l_r}{I_z} & 0 \\[8pt]
0 & 0 & 0
\end{bmatrix} \tag{B-06}$$

> $\tilde{B}_{13} = (-1) \times (-g) = +g$ ✓

### 7.3 与 06a §4.2、06c、06 文档的最终对照

| 位置 | 本文 (A-06)/(B-06) | 06a §4.2 | 06c | 06 文档 | 一致？ |
| --- | --- | --- | --- | --- | --- |
| $A_{11}$ | $-(C_f+C_r)/(mv_x)$ | 同左 | 同左 | 同左 | ✓ |
| $A_{12}$ | $(C_fl_f-C_rl_r)/(mv_x)+v_x$ | 同左 | 同左 | 同左 | ✓ |
| $A_{13}$ | $-C_f/m$ | 同左 | 同左 | 同左 | ✓ |
| $A_{21}$ | $(C_fl_f-C_rl_r)/(I_zv_x)$ | 同左 | 同左 | 同左 | ✓ |
| $A_{22}$ | $-(C_fl_f^2+C_rl_r^2)/(I_zv_x)$ | 同左 | 同左 | 同左 | ✓ |
| $A_{23}$ | $+C_fl_f/I_z$ | 同左 | 同左 | 同左 | ✓ |
| $B_{11}$ | $-C_f/m$ | 同左 | 同左 | 同左 | ✓ |
| $B_{12}$ | $-C_r/m$ | 同左 | 同左 | — | ✓ |
| $B_{13}$ | $+g$ | $+g$ | $+g$ | $+g$ | ✓ |
| $B_{21}$ | $+C_fl_f/I_z$ | 同左 | 同左 | 同左 | ✓ |
| $B_{22}$ | $-C_rl_r/I_z$ | 同左 | 同左 | — | ✓ |

**四篇文档的 06 形矩阵完全一致。**

---

## 8. 结论

三条独立推导路径殊途同归：

| 文档 | 出发约定 | $v_y$ 方向 | $\phi$ 约定 | 到定位系的变换 | $B_{13}$ 路径 |
| --- | --- | --- | --- | --- | --- |
| **06a** | 右手系 | 向左 | 右手绕 $x$（左高右低 +） | $\tilde{B}=TB$ | $(-1)(-g)=+g$ |
| **06b**（本文） | 右手系 | 向左 | 同上 | $\tilde{B}=TB$ | $(-1)(-g)=+g$ |
| **06c** | 混合约定 | **向右** | 左高右低 + | 无需变换 | 直接 $+g$ |

> **结论**：06a 与 06b 采用完全相同的右手系约定，右手系形式 (A-RH, B-RH) 完全一致（$B_{13}=-g$）。变换到定位坐标系（$\tilde{v}_y$ 向右为正）后，$\tilde{B}_{13} = +g$ 恒成立，全部 11 个矩阵元素三篇文档均一致。
