# 四轮转向扰动观测器——独立推导验证（06c）

> 本文以明确的正方向约定为起点，独立推导 4WS bicycle model 状态空间方程，与 06a 文档的结果进行交叉验证。

---

## 1. 正方向约定

| 物理量 | 正方向 | 说明 |
| --- | --- | --- |
| $v_y$ | **向右** | 质心侧向速度 |
| $\phi$ | **左高右低** | 道路横滚角；$\phi > 0$ 时重力分量向右，与 $v_y$ 正方向一致 |
| $a_y$ | **向左** | 车体坐标系惯性侧向加速度 |
| $r = \dot\psi$ | **逆时针** | 横摆角速度（从上方看） |
| $\delta_f, \delta_r$ | **向左** | 前/后轮转角（逆时针为正） |
| $F_{yf}, F_{yr}$ | **向左** | 轮胎侧偏力（几何力，与 01 文档一致） |

> 关键：$v_y$ 与 $a_y$ 正方向**相反**。这是本推导的核心特征。

---

## 2. 运动学关系

### 2.1 前后轴侧向速度

$v_y$ 向右为正，$r > 0$ 为逆时针（左转）。左转时车头向**左**摆、车尾向**右**摆，因此：

- 前轴右向速度比质心**小**（车头摆离右方）：$v_{y,f} = v_y - l_f r$
- 后轴右向速度比质心**大**（车尾摆向右方）：$v_{y,r} = v_y + l_r r$

### 2.2 速度方向角

速度方向角定义为速度偏离纵轴的角度，**向左为正**（与 01 文档一致）。右向速度分量需取反投影到左向：

$$\zeta_f = \frac{-v_{y,f}}{v_x} = \frac{-(v_y - l_f r)}{v_x} = \frac{-v_y + l_f r}{v_x}$$

$$\zeta_r = \frac{-v_{y,r}}{v_x} = \frac{-(v_y + l_r r)}{v_x} = \frac{-v_y - l_r r}{v_x}$$

### 2.3 侧偏角（01 文档正值定义 $\alpha = \delta - \zeta$）

$$\alpha_f = \delta_f - \zeta_f = \delta_f - \frac{-v_y + l_f r}{v_x} = \delta_f + \frac{v_y - l_f r}{v_x} \tag{S1}$$

$$\alpha_r = \delta_r - \zeta_r = \delta_r - \frac{-v_y - l_r r}{v_x} = \delta_r + \frac{v_y + l_r r}{v_x} \tag{S2}$$

**物理校验**：$v_y > 0$（车向右移），$\delta = r = 0$ 时 $\alpha_f = v_y/v_x > 0$，轮胎力 $F = C\alpha > 0$（向左），恢复力方向正确 ✓

### 2.4 轮胎侧偏力（向左为正）

$$F_{yf} = C_f \alpha_f = C_f\!\left(\delta_f + \frac{v_y - l_f r}{v_x}\right) \tag{F1}$$

$$F_{yr} = C_r \alpha_r = C_r\!\left(\delta_r + \frac{v_y + l_r r}{v_x}\right) \tag{F2}$$

---

## 3. 动力学方程

### 3.1 侧向加速度

车体坐标系侧向加速度（$a_y$ 向左为正，01 文档 §7）：

$$a_y = -\dot{v}_y + v_x r$$

> 推导：01 文档中 $a_y^{(L)} = \dot{v}_y^{(L)} + v_x r$。以 $v_y^{(L)} = -v_y$（左 = 负右）代入得 $a_y = -\dot{v}_y + v_x r$。
>
> **物理校验**：匀速圆周左转（$r > 0$，$\dot{v}_y = 0$），$a_y = v_x r > 0$（向左，即向心方向） ✓

### 3.2 侧向力平衡

牛顿第二定律（$a_y$、$F_y$ 均向左为正）：

$$m\,a_y = F_{yf} + F_{yr} + F_\phi$$

道路横滚力（$\phi > 0$ 左高右低，重力向**右**，即向左分量为负）：

$$F_\phi = -mg\phi$$

代入：

$$m(-\dot{v}_y + v_x r) = F_{yf} + F_{yr} - mg\phi$$

移项得 $\dot{v}_y$（$v_y$ 向右为正）：

$$m\dot{v}_y = -(F_{yf} + F_{yr}) + mg\phi + mv_x r \tag{1}$$

> **物理校验**：$\phi > 0$（重力向右）→ $+mg\phi$ 使 $\dot{v}_y$ 增大（向右加速） ✓

### 3.3 横摆力矩平衡

横摆力矩使用向左为正的几何力，$r$ 逆时针为正：

$$I_z \dot{r} = l_f F_{yf} - l_r F_{yr} \tag{2}$$

> **物理校验**：$\delta_f > 0$（左转），$v_y = r = 0$ 时 $F_{yf} = C_f \delta_f > 0$（向左），力矩 $l_f F_{yf} > 0$（逆时针） ✓

---

## 4. 代入化简

### 4.1 侧向方程

将 (F1)、(F2) 代入 (1)，展开 $-(F_{yf} + F_{yr})$：

$$-(F_{yf} + F_{yr}) = -C_f\delta_f - \frac{C_f}{v_x}v_y + \frac{C_f l_f}{v_x}r - C_r\delta_r - \frac{C_r}{v_x}v_y - \frac{C_r l_r}{v_x}r$$

$$= -\frac{C_f + C_r}{v_x}\,v_y + \frac{C_f l_f - C_r l_r}{v_x}\,r - C_f\delta_f - C_r\delta_r$$

合并 (1) 中全部 $r$ 项，两边除以 $m$：

$$\boxed{\dot{v}_y = -\frac{C_f + C_r}{m v_x}\,v_y + \left(\frac{C_f l_f - C_r l_r}{m v_x} + v_x\right) r - \frac{C_f}{m}\,\delta_f - \frac{C_r}{m}\,\delta_r + g\,\phi} \tag{3}$$

### 4.2 横摆方程

将 (F1)、(F2) 代入 (2)：

$$l_f F_{yf} = C_f l_f\delta_f + \frac{C_f l_f}{v_x}\,v_y - \frac{C_f l_f^2}{v_x}\,r$$

$$l_r F_{yr} = C_r l_r\delta_r + \frac{C_r l_r}{v_x}\,v_y + \frac{C_r l_r^2}{v_x}\,r$$

作差，两边除以 $I_z$：

$$\boxed{\dot{r} = \frac{C_f l_f - C_r l_r}{I_z v_x}\,v_y - \frac{C_f l_f^2 + C_r l_r^2}{I_z v_x}\,r + \frac{C_f l_f}{I_z}\,\delta_f - \frac{C_r l_r}{I_z}\,\delta_r} \tag{4}$$

---

## 5. 状态空间模型

### 增广状态（扰动 $\delta_d$ 建模为随机游走 $\dot\delta_d = 0$）

$$\mathbf{x} = \begin{bmatrix} v_y \\ r \\ \delta_d \end{bmatrix}, \quad \mathbf{u} = \begin{bmatrix} \delta_f \\ \delta_r \\ \phi \end{bmatrix}, \quad y = r_{\text{meas}}$$

### 连续时间模型 $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}$

$$A = \begin{bmatrix}
-\dfrac{C_f + C_r}{m v_x} & \dfrac{C_f l_f - C_r l_r}{m v_x} + v_x & -\dfrac{C_f}{m} \\[8pt]
\dfrac{C_f l_f - C_r l_r}{I_z v_x} & -\dfrac{C_f l_f^2 + C_r l_r^2}{I_z v_x} & \dfrac{C_f l_f}{I_z} \\[8pt]
0 & 0 & 0
\end{bmatrix}$$

$$B = \begin{bmatrix}
-\dfrac{C_f}{m} & -\dfrac{C_r}{m} & g \\[8pt]
\dfrac{C_f l_f}{I_z} & -\dfrac{C_r l_r}{I_z} & 0 \\[8pt]
0 & 0 & 0
\end{bmatrix}$$

$$C = \begin{bmatrix} 0 & 1 & 0 \end{bmatrix}$$

---

## 6. 与 06a 文档交叉验证

### 6.1 与 06a §4.2 (A-06)、(B-06) 逐元素比对

| 位置 | 本文（独立推导） | 06a (A-06)/(B-06) | 一致？ |
| --- | --- | --- | --- |
| $A_{11}$ | $-\dfrac{C_f+C_r}{mv_x}$ | $-\dfrac{C_f+C_r}{mv_x}$ | ✓ |
| $A_{12}$ | $\dfrac{C_fl_f-C_rl_r}{mv_x}+v_x$ | $\dfrac{C_fl_f-C_rl_r}{mv_x}+v_x$ | ✓ |
| $A_{13}$ | $-\dfrac{C_f}{m}$ | $-\dfrac{C_f}{m}$ | ✓ |
| $A_{21}$ | $\dfrac{C_fl_f-C_rl_r}{I_zv_x}$ | $\dfrac{C_fl_f-C_rl_r}{I_zv_x}$ | ✓ |
| $A_{22}$ | $-\dfrac{C_fl_f^2+C_rl_r^2}{I_zv_x}$ | $-\dfrac{C_fl_f^2+C_rl_r^2}{I_zv_x}$ | ✓ |
| $A_{23}$ | $+\dfrac{C_fl_f}{I_z}$ | $+\dfrac{C_fl_f}{I_z}$ | ✓ |
| $B_{11}$ | $-\dfrac{C_f}{m}$ | $-\dfrac{C_f}{m}$ | ✓ |
| $B_{12}$ | $-\dfrac{C_r}{m}$ | $-\dfrac{C_r}{m}$ | ✓ |
| $B_{13}$ | $+g$ | $+g$ | ✓ |
| $B_{21}$ | $+\dfrac{C_fl_f}{I_z}$ | $+\dfrac{C_fl_f}{I_z}$ | ✓ |
| $B_{22}$ | $-\dfrac{C_rl_r}{I_z}$ | $-\dfrac{C_rl_r}{I_z}$ | ✓ |

**全部 11 个独立元素一致。**

### 6.2 $B_{13} = +g$ 的独立验证

本文推导路径：

1. $\phi > 0$（左高右低）→ 重力分量向**右** → $F_\phi^{(left)} = -mg\phi$（向左为负）
2. 侧向方程 (1)：$m\dot{v}_y = -(F_{yf}+F_{yr}) + mg\phi + mv_xr$，其中 $+mg\phi$ 来自 $-(-mg\phi)$
3. $\dot{v}_y$ 向右为正，$+g\phi$ 使车向右加速 ✓
4. 结论：$B_{13} = +g$，无需依赖坐标变换论证

> 06a §4.1 通过 $\tilde{B} = TB$（$T = \mathrm{diag}(-1,1,1)$）将右手系 $B_{13}=-g$ 变换为 $+g$，与本文直接推导结论一致，两条独立路径互相印证。

### 6.3 与 06 文档（前轮转向）的兼容性

令 $\delta_r = 0$，$B$ 退化为：

$$B\big|_{\delta_r=0} = \begin{bmatrix}
-\dfrac{C_f}{m} & g \\[8pt]
\dfrac{C_f l_f}{I_z} & 0 \\[8pt]
0 & 0
\end{bmatrix}$$

与 06 文档的 $B$ 矩阵完全一致 ✓
