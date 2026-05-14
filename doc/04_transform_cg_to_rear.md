# 从质心误差模型到后轴误差模型的坐标变换（4WS）

> 本文从 4WS 质心误差状态方程出发，通过坐标变换推导后轴误差状态方程。质心误差模型见 [[02b_error_cg_4ws]]，后轴直接推导版本见 [[03a_error_rear_front_steer]]。

---

## 1 出发点：质心误差模型（4WS）

状态向量 $\mathbf{x}_{CG} = [e_{1c},\;\dot e_{1c},\;e_2,\;\dot e_2]^T$，其中 $e_{1c}$ 为质心到路径的横向距离，$e_2 = \psi - \theta_{\text{ref}}$ 为航向误差。

$$
\dot{\mathbf{x}}_{CG} = A_{CG}\,\mathbf{x}_{CG} + B_{f,CG}\,\delta_f + B_{r,CG}\,\delta_r + G_{CG}\,\dot\theta_{\text{ref}}
$$

$$
A_{CG} = \begin{bmatrix}
0 & 1 & 0 & 0 \\[6pt]
0 & -\dfrac{C_f+C_r}{mv_x} & \dfrac{C_f+C_r}{m} & -\dfrac{l_fC_f - l_rC_r}{mv_x} \\[6pt]
0 & 0 & 0 & 1 \\[6pt]
0 & -\dfrac{l_fC_f - l_rC_r}{I_zv_x} & \dfrac{l_fC_f - l_rC_r}{I_z} & -\dfrac{l_f^2C_f + l_r^2C_r}{I_zv_x}
\end{bmatrix}
$$

$$
B_{f,CG} = \begin{bmatrix} 0 \\ C_f/m \\ 0 \\ l_fC_f/I_z \end{bmatrix}, \quad
B_{r,CG} = \begin{bmatrix} 0 \\ C_r/m \\ 0 \\ -l_rC_r/I_z \end{bmatrix}, \quad
G_{CG} = \begin{bmatrix} 0 \\ -v_x - \dfrac{l_fC_f - l_rC_r}{mv_x} \\ 0 \\ -\dfrac{l_f^2C_f + l_r^2C_r}{I_zv_x} \end{bmatrix}
$$

---

## 2 后轴误差运动学

### 2.1 后轴横向误差率

后轴侧向速度 $v_{yr} = v_y - l_r r$，后轴横向误差率由 Frenet 关系直接给出：

$$
\dot e_{1r} = v_{yr} + v_x\,e_2 = (v_y - l_r r) + v_x\,e_2
$$

而质心横向误差率为：

$$
\dot e_{1c} = v_y + v_x\,e_2
$$

两者之差：

$$
\boxed{\dot e_{1r} = \dot e_{1c} - l_r r}
$$

利用 $r = \dot e_2 + \dot\theta_{\text{ref}}$：

$$
\dot e_{1r} = \dot e_{1c} - l_r\dot e_2 - l_r\dot\theta_{\text{ref}} \tag{1}
$$

### 2.2 航向误差

航向误差 $e_2 = \psi - \theta_{\text{ref}}$ 及其导数 $\dot e_2 = r - \dot\theta_{\text{ref}}$ 与参考点选择无关（均为车身航向与路径切线的夹角），因此：

$$
e_{2,R} = e_{2,CG} = e_2, \qquad \dot e_{2,R} = \dot e_{2,CG} = \dot e_2
$$

> 严格来说，$\theta_{\text{ref}}$ 在质心投影点和后轴投影点处略有不同（因路径曲率）。在缓变曲率假设下，差异为 $O(\kappa l_r)$，可忽略。

---

## 3 变换关系

### 3.1 状态反解

由式 (1) 可得质心状态用后轴状态表达：

$$
\dot e_{1c} = \dot e_{1r} + l_r\dot e_2 + l_r\dot\theta_{\text{ref}} \tag{2}
$$

### 3.2 二阶动力学关系

对 $\dot e_{1r} = \dot e_{1c} - l_r r$ 求导：

$$
\ddot e_{1r} = \ddot e_{1c} - l_r\dot r
$$

在 $\ddot\theta_{\text{ref}} \approx 0$ 假设下，$\dot r = \ddot e_2$，因此：

$$
\boxed{\ddot e_{1r} = \ddot e_{1c} - l_r\ddot e_2} \tag{3}
$$

这是变换的核心关系：**后轴横向加速度 = 质心横向加速度 $-$ 横摆加速度的力臂项**。

---

## 4 变换 $\ddot e_2$ 方程

质心模型的 $\ddot e_2$ 方程：

$$
\ddot e_2 = -\frac{l_fC_f - l_rC_r}{I_zv_x}\,\dot e_{1c} + \frac{l_fC_f - l_rC_r}{I_z}\,e_2 - \frac{l_f^2C_f + l_r^2C_r}{I_zv_x}\,\dot e_2 + \frac{l_fC_f}{I_z}\delta_f - \frac{l_rC_r}{I_z}\delta_r - \frac{l_f^2C_f + l_r^2C_r}{I_zv_x}\dot\theta_{\text{ref}}
$$

用式 (2) 替换 $\dot e_{1c} = \dot e_{1r} + l_r\dot e_2 + l_r\dot\theta_{\text{ref}}$：

$$
\ddot e_2 = -\frac{l_fC_f - l_rC_r}{I_zv_x}(\dot e_{1r} + l_r\dot e_2 + l_r\dot\theta_{\text{ref}}) + \frac{l_fC_f - l_rC_r}{I_z}\,e_2 - \frac{l_f^2C_f + l_r^2C_r}{I_zv_x}\,\dot e_2 + \cdots
$$

整理 $\dot e_2$ 系数：

$$
-\frac{l_r(l_fC_f - l_rC_r)}{I_zv_x} - \frac{l_f^2C_f + l_r^2C_r}{I_zv_x} = -\frac{l_f^2C_f + l_r^2C_r + l_rl_fC_f - l_r^2C_r}{I_zv_x} = -\frac{l_fC_f(l_f + l_r)}{I_zv_x} = -\frac{l_fC_fL}{I_zv_x}
$$

整理 $\dot\theta_{\text{ref}}$ 系数（同理）：

$$
-\frac{l_r(l_fC_f - l_rC_r)}{I_zv_x} - \frac{l_f^2C_f + l_r^2C_r}{I_zv_x} = -\frac{l_fC_fL}{I_zv_x}
$$

最终：

$$
\boxed{\ddot e_2 = -\frac{l_fC_f - l_rC_r}{I_zv_x}\,\dot e_{1r} + \frac{l_fC_f - l_rC_r}{I_z}\,e_2 - \frac{l_fC_fL}{I_zv_x}\,\dot e_2 + \frac{l_fC_f}{I_z}\delta_f - \frac{l_rC_r}{I_z}\delta_r - \frac{l_fC_fL}{I_zv_x}\dot\theta_{\text{ref}}}
$$

> **变化**：$\dot e_1$ 和 $e_2$ 的系数不变，$\dot e_2$ 的阻尼系数从 $-(l_f^2C_f + l_r^2C_r)/(I_zv_x)$ 变为 $-l_fC_fL/(I_zv_x)$，扰动系数同样变化。输入系数不变。

---

## 5 变换 $\ddot e_{1r}$ 方程

由式 (3)，$\ddot e_{1r} = \ddot e_{1c} - l_r\ddot e_2$。先将两个方程在质心状态下展开，再替换为后轴状态。

### 5.1 合并 $\ddot e_{1c} - l_r\ddot e_2$（质心状态表达）

$$
\ddot e_{1r} = \left(-\frac{C_f+C_r}{mv_x} + \frac{l_r(l_fC_f - l_rC_r)}{I_zv_x}\right)\dot e_{1c} + \left(\frac{C_f+C_r}{m} - \frac{l_r(l_fC_f - l_rC_r)}{I_z}\right)e_2
$$

$$
+\left(-\frac{l_fC_f - l_rC_r}{mv_x} + \frac{l_r(l_f^2C_f + l_r^2C_r)}{I_zv_x}\right)\dot e_2
$$

$$
+\left(\frac{C_f}{m} - \frac{l_rl_fC_f}{I_z}\right)\delta_f + \left(\frac{C_r}{m} + \frac{l_r^2C_r}{I_z}\right)\delta_r
$$

$$
+\left(-v_x - \frac{l_fC_f - l_rC_r}{mv_x} + \frac{l_r(l_f^2C_f + l_r^2C_r)}{I_zv_x}\right)\dot\theta_{\text{ref}}
$$

### 5.2 替换 $\dot e_{1c} \to \dot e_{1r} + l_r\dot e_2 + l_r\dot\theta_{\text{ref}}$

记 $\dot e_{1c}$ 的系数为 $p$，则替换后 $\dot e_2$ 增加 $pl_r$，$\dot\theta_{\text{ref}}$ 增加 $pl_r$。

定义 $\eta = I_z - ml_fl_r$，$\xi = I_z + ml_r^2$，化简各系数：

**$\dot e_{1r}$ 系数**：

$$
p = -\frac{(C_f+C_r)I_z - ml_r(l_fC_f - l_rC_r)}{mI_zv_x} = -\frac{C_f\eta + C_r\xi}{mI_zv_x}
$$

**$e_2$ 系数**：

$$
\frac{(C_f+C_r)I_z - ml_r(l_fC_f - l_rC_r)}{mI_z} = \frac{C_f\eta + C_r\xi}{mI_z}
$$

**$\dot e_2$ 系数**（含 $pl_r$ 修正）：

$$
-\frac{(l_fC_f - l_rC_r)I_z - ml_r(l_f^2C_f + l_r^2C_r)}{mI_zv_x} + l_r p
$$

化简分子第一项：$(l_fC_f - l_rC_r)I_z - ml_r(l_f^2C_f + l_r^2C_r) = l_fC_f(I_z - ml_rl_f) - l_rC_r(I_z + ml_r^2) = l_fC_f\eta - l_rC_r\xi$

加上 $l_r p$ 的贡献：

$$
-\frac{l_fC_f\eta - l_rC_r\xi}{mI_zv_x} - \frac{l_r(C_f\eta + C_r\xi)}{mI_zv_x} = -\frac{(l_f + l_r)C_f\eta}{mI_zv_x} = -\frac{C_fL\eta}{mI_zv_x}
$$

**$\delta_f$ 系数**：

$$
\frac{C_f}{m} - \frac{l_rl_fC_f}{I_z} = \frac{C_f(I_z - ml_fl_r)}{mI_z} = \frac{C_f\eta}{mI_z}
$$

**$\delta_r$ 系数**：

$$
\frac{C_r}{m} + \frac{l_r^2C_r}{I_z} = \frac{C_r(I_z + ml_r^2)}{mI_z} = \frac{C_r\xi}{mI_z}
$$

**$\dot\theta_{\text{ref}}$ 系数**（含 $pl_r$ 修正）：类似 $\dot e_2$ 的化简过程，最终为：

$$
-\frac{C_fL\eta}{mI_zv_x} - v_x
$$

### 5.3 最终结果

$$
\boxed{\ddot e_{1r} = -\frac{C_f\eta + C_r\xi}{mI_zv_x}\,\dot e_{1r} + \frac{C_f\eta + C_r\xi}{mI_z}\,e_2 - \frac{C_fL\eta}{mI_zv_x}\,\dot e_2 + \frac{C_f\eta}{mI_z}\delta_f + \frac{C_r\xi}{mI_z}\delta_r + \left(-\frac{C_fL\eta}{mI_zv_x} - v_x\right)\dot\theta_{\text{ref}}}
$$

---

## 6 后轴误差状态方程（4WS）

$$
\dot{\mathbf{x}}_R = A_R\,\mathbf{x}_R + B_{f,R}\,\delta_f + B_{r,R}\,\delta_r + G_R\,\dot\theta_{\text{ref}}
$$

其中 $\mathbf{x}_R = [e_{1r},\;\dot e_{1r},\;e_2,\;\dot e_2]^T$，$\eta = I_z - ml_fl_r$，$\xi = I_z + ml_r^2$。

### 6.1 系统矩阵 $A_R$

$$
A_R = \begin{bmatrix}
0 & 1 & 0 & 0 \\[8pt]
0 & -\dfrac{C_f\eta + C_r\xi}{mI_zv_x} & \dfrac{C_f\eta + C_r\xi}{mI_z} & -\dfrac{C_fL\eta}{mI_zv_x} \\[8pt]
0 & 0 & 0 & 1 \\[8pt]
0 & -\dfrac{l_fC_f - l_rC_r}{I_zv_x} & \dfrac{l_fC_f - l_rC_r}{I_z} & -\dfrac{l_fC_fL}{I_zv_x}
\end{bmatrix}
$$

> $A_R$ 与选择 2WS 或 4WS 无关——A 矩阵只含状态系数，不依赖输入 $\delta_f,\delta_r$。

### 6.2 输入矩阵

$$
B_{f,R} = \begin{bmatrix} 0 \\ \dfrac{C_f\eta}{mI_z} \\ 0 \\ \dfrac{l_fC_f}{I_z} \end{bmatrix}, \qquad
B_{r,R} = \begin{bmatrix} 0 \\ \dfrac{C_r\xi}{mI_z} \\ 0 \\ -\dfrac{l_rC_r}{I_z} \end{bmatrix}
$$

> **关键区别**：质心模型中前后轮的横向加速度增益分别为 $C_f/m$ 和 $C_r/m$（仅与质量有关），而后轴模型中变为 $C_f\eta/(mI_z)$ 和 $C_r\xi/(mI_z)$（同时涉及惯量耦合参数 $\eta$ 和 $\xi$）。

### 6.3 扰动矩阵

$$
G_R = \begin{bmatrix}
0 \\[6pt]
-\dfrac{C_fL\eta}{mI_zv_x} - v_x \\[6pt]
0 \\[6pt]
-\dfrac{l_fC_fL}{I_zv_x}
\end{bmatrix}
$$

---

## 7 变换总结

### 7.1 变换公式

状态替换关系（非纯线性变换，含扰动偏移）：

$$
\dot e_{1c} = \dot e_{1r} + l_r\dot e_2 + l_r\dot\theta_{\text{ref}}
$$

动力学变换关系：

$$
\ddot e_{1r} = \ddot e_{1c} - l_r\ddot e_2
$$

### 7.2 矩阵元素变化规律

| 矩阵元素 | 质心定义 | 后轴定义 | 变化原因 |
|----------|---------|---------|---------|
| $A_{22}$ | $-\dfrac{C_f+C_r}{mv_x}$ | $-\dfrac{C_f\eta+C_r\xi}{mI_zv_x}$ | $\ddot e_{1c} - l_r\ddot e_2$ + 状态替换 |
| $A_{24}$ | $-\dfrac{l_fC_f-l_rC_r}{mv_x}$ | $-\dfrac{C_fL\eta}{mI_zv_x}$ | 同上 |
| $A_{44}$ | $-\dfrac{l_f^2C_f+l_r^2C_r}{I_zv_x}$ | $-\dfrac{l_fC_fL}{I_zv_x}$ | 状态替换 |
| $B_{f,2}$ | $C_f/m$ | $C_f\eta/(mI_z)$ | $\ddot e_{1c} - l_r\ddot e_2$ |
| $B_{r,2}$ | $C_r/m$ | $C_r\xi/(mI_z)$ | 同上 |

### 7.3 物理参数

$$
\eta = I_z - ml_fl_r, \qquad \xi = I_z + ml_r^2
$$

| 参数 | 物理意义 |
|------|---------|
| $\eta$ | 惯量耦合参数：$\eta > 0$ 时前轮转角对后轴横向加速度有正增益 |
| $\xi$ | 后轴等效惯量参数：$\xi > 0$ 恒成立，后轮转角对后轴横向加速度始终有正增益 |

> $B_{r,2} = C_r\xi/(mI_z) > 0$ 恒为正，意味着在后轴误差模型中，后轮转角对后轴横向加速度始终有正增益，这符合直觉——后轮直接控制后轴处的侧向力。

---

## 8 与直接推导的一致性

当 $\delta_r = 0$ 时，$B_{r,R}$ 项消失，方程退化为：

$$
\dot{\mathbf{x}}_R = A_R\,\mathbf{x}_R + B_{f,R}\,\delta_f + G_R\,\dot\theta_{\text{ref}}
$$

此结果与 [[03a_error_rear_front_steer]] 中从后轴 Frenet 关系直接推导的结果完全一致（$A_R$、$B_{f,R}$、$G_R$ 逐元素相同），可由 SymPy 符号计算验证。

---

## 9 使用前提

1. 小角度近似：$\delta_f$、$\delta_r$、$e_2$、$\beta_r$ 均较小
2. 纵向速度近似恒定：$\dot v_x \approx 0$
3. 线性轮胎模型：$F_{yf} = C_f\alpha_f$，$F_{yr} = C_r\alpha_r$，$C_f, C_r > 0$
4. 参考航向变化率缓慢：$\ddot\theta_{\text{ref}} \approx 0$
5. 忽略纵向力对侧向/横摆的影响
6. 质心与后轴投影点处曲率近似相等（缓变曲率）
