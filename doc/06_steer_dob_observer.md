# 转向扰动观测器（Steer DOB Observer）核心模型

> 基于线性二自由度 bicycle model，增广一个扰动状态，构成 3 阶 Luenberger 观测器，用于估计等效转向扰动。

---

## 1. 符号定义

| 符号       | 含义                                   |
| ---------- | -------------------------------------- |
| $v_y$      | 质心侧向速度（**代码约定正方向向右**） |
| $r$        | 横摆角速度，$r = \dot\psi$             |
| $\delta_d$ | 扰动等效前轮转角（待估计）             |
| $\delta_f$ | 前轮转角（由方向盘转角换算）           |
| $\phi$     | 道路横滚角                             |
| $v_x$      | 纵向速度                               |
| $C_f$      | 前轴侧偏刚度（$>0$）                   |
| $C_r$      | 后轴侧偏刚度（$>0$）                   |
| $l_f, l_r$ | 质心到前/后轴距离                      |
| $m$        | 车辆质量                               |
| $I_z$      | 横摆转动惯量                           |
| $g$        | 重力加速度（9.8）                      |
| $T_s$      | 控制周期                               |

---

## 2. 从 bicycle model 到状态空间——公式推导

### 2.1 标准教科书推导（$v_y$ 正方向向左）

以质心为原点，$x$ 轴沿车辆纵向向前，$y$ 轴向左。前后轮侧偏角（车轮指向角 − 速度方向角）：

$$
\alpha_f = \delta_f - \frac{v_y + l_f r}{v_x}, \quad
\alpha_r = -\frac{v_y - l_r r}{v_x}
$$

轮胎侧向力（$C_\alpha > 0$，力与侧偏角同向）：

$$
F_{yf} = C_f \alpha_f, \quad F_{yr} = C_r \alpha_r
$$

质心侧向牛顿方程与横摆力矩方程：

$$
m(\dot{v}_y + v_x r) = F_{yf} + F_{yr}
$$

$$
I_z \dot{r} = l_f F_{yf} - l_r F_{yr}
$$

展开得标准 2-DOF 方程：

$$
\dot{v}_y = -\frac{C_f + C_r}{m v_x} v_y
+\left(\frac{-C_f l_f + C_r l_r}{m v_x} - v_x\right) r
+\frac{C_f}{m} \delta_f
\tag{std-1}
$$

$$
\dot{r} = \frac{-C_f l_f + C_r l_r}{I_z v_x} v_y
-\frac{C_f l_f^2 + C_r l_r^2}{I_z v_x} r
+\frac{C_f l_f}{I_z} \delta_f
\tag{std-2}
$$

写成矩阵形式：

$$
A_{\text{std}} = \begin{bmatrix}
-\dfrac{C_f + C_r}{m v_x} & \dfrac{-C_f l_f + C_r l_r}{m v_x} - v_x \\[6pt]
\dfrac{-C_f l_f + C_r l_r}{I_z v_x} & -\dfrac{C_f l_f^2 + C_r l_r^2}{I_z v_x}
\end{bmatrix}, \quad
B_{\text{std}} = \begin{bmatrix}
\dfrac{C_f}{m} \\[6pt]
\dfrac{C_f l_f}{I_z}
\end{bmatrix}
$$

### 2.2 代码的符号约定：$\tilde{v}_y = -v_{y,\text{std}}$（正方向向右）

代码中 A/B 矩阵的若干项与标准教科书存在符号差异。通过逐项比对可以发现，**代码将侧向速度正方向定义为向右**，即：

$$
\tilde{v}_y = -v_{y,\text{std}}
$$

引入坐标变换矩阵（仅翻转第一个状态分量）：

$$
T = \text{diag}(-1,\; 1) = \begin{bmatrix} -1 & 0 \\ 0 & 1 \end{bmatrix}
$$

变换关系：

$$
A_{\text{code}} = T \, A_{\text{std}} \, T^{-1}, \quad
B_{\text{code}} = T \, B_{\text{std}}
$$

逐项展开（$T^{-1} = T$）：

$$
A_{\text{code}} =
\begin{bmatrix} -1 & 0 \\ 0 & 1 \end{bmatrix}
\begin{bmatrix} a_{11} & a_{12} \\ a_{21} & a_{22} \end{bmatrix}
\begin{bmatrix} -1 & 0 \\ 0 & 1 \end{bmatrix}
= \begin{bmatrix} a_{11} & -a_{12} \\ -a_{21} & a_{22} \end{bmatrix}
$$

代入标准系数：

| 项    | 标准 $A_{\text{std}}$               | 变换后 $A_{\text{code}}$            | 代码实际值 |
| ----- | ----------------------------------- | ----------------------------------- | ---------- |
| (0,0) | $-\frac{C_f+C_r}{mv_x}$             | $-\frac{C_f+C_r}{mv_x}$             | ✓ 一致     |
| (0,1) | $\frac{-C_fl_f+C_rl_r}{mv_x}-v_x$   | $\frac{C_fl_f-C_rl_r}{mv_x}+v_x$    | ✓ 一致     |
| (1,0) | $\frac{-C_fl_f+C_rl_r}{I_zv_x}$     | $\frac{C_fl_f-C_rl_r}{I_zv_x}$      | ✓ 一致     |
| (1,1) | $-\frac{C_fl_f^2+C_rl_r^2}{I_zv_x}$ | $-\frac{C_fl_f^2+C_rl_r^2}{I_zv_x}$ | ✓ 一致     |

$$
B_{\text{code}} = T \, B_{\text{std}} =
\begin{bmatrix} -C_f/m \\ C_f l_f / I_z \end{bmatrix}
$$

| 项    | 标准 $B_{\text{std}}$ | 变换后 $B_{\text{code}}$ | 代码实际值 |
| ----- | --------------------- | ------------------------ | ---------- |
| (0,0) | $+C_f/m$              | $-C_f/m$                 | ✓ 一致     |
| (1,0) | $+C_fl_f/I_z$         | $+C_fl_f/I_z$            | ✓ 一致     |

**横滚项**也一致：标准下 $B_{\text{std}}(0,1) = -g$（重力向右为负），变换后 $B_{\text{code}}(0,1) = +g$ ✓

> **结论**：代码 $B(0,0)=-C_f/m$ 是正确的，对应 $\tilde{v}_y$ 向右为正的约定。整个 A/B 矩阵与标准教科书通过坐标变换 $T$ 严格等价。

### 2.3 增广扰动状态

将扰动 $\delta_d$ 建模为**等效的附加前轮转角**，作为第 3 个状态增广：

$$
\tilde{\mathbf{x}} = \begin{bmatrix} \tilde{v}_y \\ r \\ \delta_d \end{bmatrix}, \quad
\mathbf{u} = \begin{bmatrix} \delta_f \\ \phi \end{bmatrix}
$$
 
扰动假设为随机游走（$\dot\delta_d = 0$），因此第 3 行全零。扰动对 $\tilde{v}_y$ 和 $r$ 的耦合系数与 $\delta_f$ 相同（因为物理上就是一个附加前轮角）：

$$
A_{13} = B_{10} = -\frac{C_f}{m}, \quad
A_{23} = B_{20} = +\frac{C_f l_f}{I_z}
$$

最终增广模型：

$$
A = \begin{bmatrix}
-\dfrac{C_f + C_r}{m v_x} & \dfrac{C_f l_f - C_r l_r}{m v_x} + v_x & -\dfrac{C_f}{m} \\[8pt]
\dfrac{C_f l_f - C_r l_r}{I_z v_x} & -\dfrac{C_f l_f^2 + C_r l_r^2}{I_z v_x} & \dfrac{C_f l_f}{I_z} \\[8pt]
0 & 0 & 0
\end{bmatrix}
$$

$$
B = \begin{bmatrix}
-\dfrac{C_f}{m} & g \\[8pt]
\dfrac{C_f l_f}{I_z} & 0 \\[8pt]
0 & 0
\end{bmatrix}, \quad
C = \begin{bmatrix} 0 & 1 & 0 \end{bmatrix}
$$

### 离散化（前向欧拉）

$$
A_d = I + A T_s, \quad B_d = B T_s
$$

---

## 3. 观测器更新

### 预测

$$
\hat{\mathbf{x}}^- = A_d \hat{\mathbf{x}} + B_d \mathbf{u}
$$

### 新息（innovation）

$$
e = r_{\text{meas}} - \hat{r}^-
$$

### 校正（基础 Luenberger）

$$
\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + L \cdot e
$$

增益 $L$ 为 3×1 向量，其中 $L_3$（扰动通道）动态缩放：

$$
L_3^{\text{actual}} = L_3^{\text{base}} \cdot f(v) \cdot \max(k_{\text{lc}},\; k_{\text{mhe}})
$$

- $f(v)$：速度插值表
- $k_{\text{lc}}$：变道附加增益（高速变道时提高响应）
- $k_{\text{mhe}}$：MHE 辨识工作时的附加增益

---

## 4. 快速扰动隔离（可选）

将新息分为高频与低频，分别处理：

$$
\hat{r}' = \hat{r}^- + d_{r,\text{last}}
$$

$$
e_{\text{total}} = r_{\text{meas}} - \hat{r}'
$$

**高通滤波**（一阶递推）：

$$
e_{\text{HP}} = \frac{\tau}{\tau + T_s}\left(e_{\text{HP,last}} + e_{\text{total}} - e_{\text{total,last}}\right)
$$

**低通 = 残差 − 高通**：

$$
e_{\text{LP}} = e_{\text{total}} - e_{\text{HP}}
$$

- $e_{\text{LP}}$ → 驱动主观测器：$\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + L \cdot e_{\text{LP}}$
- $e_{\text{HP}}$ → 驱动独立的横摆扰动项：

$$
d_r = e^{-T_s/\tau} \cdot d_{r,\text{last}} + k_{\text{rapid}} \cdot e_{\text{HP}}, \quad |d_r| \le d_{\max}
$$

**目的**：路面冲击等瞬态激励走高频通道快速衰减，不污染主扰动估计 $\delta_d$。

---

## 5. 输出

$$
\delta_{\text{disturb}} = \text{Clamp}\!\left(\text{DeltaAngleToStrAng}(\hat\delta_d),\; \pm \delta_{\max}\right)
$$

将扰动等效前轮角转换回方向盘角度后输出。
