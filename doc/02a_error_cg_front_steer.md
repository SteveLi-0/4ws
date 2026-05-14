# 纯前轮转向 Frenet 坐标系下横向误差方程

> 本文从自行车模型动力学出发，在 $\delta_r=0$ 条件下推导横向控制的误差状态空间方程。符号约定与 [[01_bicycle_model]] 一致，完整 4WS 推导见 [[02b_error_cg_4ws]]。

---

## 1 符号定义

| 符号 | 含义 |
|------|------|
| $e_1$ | 横向位置误差（质心到参考路径的垂直距离） |
| $e_2$ | 航向误差 $\psi - \theta_{\text{ref}}$ |
| $\delta_f$ | 前轮转角（控制量） |
| $v_x$ | 纵向速度（近似常值） |
| $v_y$ | 侧向速度 |
| $\omega$ | 横摆角速度 $\dot\psi$ |
| $m$ | 整车质量 |
| $I_z$ | 绕 $z$ 轴转动惯量 |
| $l_f,\;l_r$ | 质心到前、后轴距离 |
| $C_f,\;C_r$ | 前、后轮侧偏刚度（$>0$） |
| $\dot\theta_{\text{ref}}$ | 参考航向变化率（扰动），$\dot\theta_{\text{ref}} = \kappa\,v_{\text{ref}}$ |

---

## 2 车辆动力学（$\delta_r = 0$）

### 2.1 轮胎侧偏角

$$
\alpha_f = \delta_f - \frac{v_y + l_f\,\omega}{v_x}, \qquad
\alpha_r = -\frac{v_y - l_r\,\omega}{v_x}
$$

### 2.2 侧向与横摆动力学

$$
m\,\dot{v}_y = -m\,v_x\,\omega
+C_f\!\left(\delta_f - \frac{v_y + l_f\omega}{v_x}\right)
+C_r\!\left(-\frac{v_y - l_r\omega}{v_x}\right)
$$

$$
I_z\,\dot{\omega} =
l_f C_f\!\left(\delta_f - \frac{v_y + l_f\omega}{v_x}\right)
+l_r C_r\!\cdot\frac{v_y - l_r\omega}{v_x}
$$

展开整理：

$$
\dot{v}_y = \frac{1}{m}\left[C_f\delta_f - \frac{C_f+C_r}{v_x}v_y - \frac{l_fC_f - l_rC_r}{v_x}\omega\right] - v_x\omega
$$

$$
\dot{\omega} = \frac{1}{I_z}\left[l_fC_f\delta_f - \frac{l_fC_f - l_rC_r}{v_x}v_y - \frac{l_f^2C_f + l_r^2C_r}{v_x}\omega\right]
$$

---

## 3 Frenet 坐标系下的误差定义

### 3.1 误差运动学（小角度近似）

$$
\dot{e}_1 = v_y + v_x\,e_2
$$

$$
\dot{e}_2 = \omega - \dot\theta_{\text{ref}}
$$

由此得到状态替换关系：

$$
v_y = \dot{e}_1 - v_x\,e_2, \qquad
\omega = \dot{e}_2 + \dot\theta_{\text{ref}}
$$

---

## 4 误差动力学推导

### 4.1 $\ddot{e}_1$ 的推导

$$
\ddot{e}_1 = \dot{v}_y + v_x\,\dot{e}_2
$$

将 $\dot{v}_y$ 代入，利用 $\dot{v}_y + v_x\omega = \frac{1}{m}[C_f\delta_f - \frac{C_f+C_r}{v_x}v_y - \frac{l_fC_f-l_rC_r}{v_x}\omega]$，再加上 $v_x\dot{e}_2$：

$$
\ddot{e}_1 = \frac{1}{m}\left[C_f\delta_f - \frac{C_f+C_r}{v_x}v_y - \frac{l_fC_f-l_rC_r}{v_x}\omega\right] - v_x\dot\theta_{\text{ref}}
$$

> **关键消去**：$\dot{v}_y$ 中的 $-v_x\omega$ 与 $v_x\dot{e}_2 = v_x(\omega - \dot\theta_{\text{ref}})$ 中的 $v_x\omega$ 相消，仅留扰动项 $-v_x\dot\theta_{\text{ref}}$。

代入 $v_y = \dot{e}_1 - v_xe_2$，$\omega = \dot{e}_2 + \dot\theta_{\text{ref}}$：

$$
\ddot{e}_1 =
-\frac{C_f + C_r}{m\,v_x}\,\dot{e}_1
+\frac{C_f + C_r}{m}\,e_2
-\frac{l_f C_f - l_r C_r}{m\,v_x}\,\dot{e}_2
+\frac{C_f}{m}\,\delta_f
+\left[-v_x - \frac{l_f C_f - l_r C_r}{m\,v_x}\right]\dot\theta_{\text{ref}}
$$

### 4.2 $\ddot{e}_2$ 的推导

$$
\ddot{e}_2 = \dot{\omega} - \ddot\theta_{\text{ref}}
$$

假设 $\ddot\theta_{\text{ref}} \approx 0$，代入 $\dot\omega$ 并替换：

$$
\ddot{e}_2 =
-\frac{l_f C_f - l_r C_r}{I_z\,v_x}\,\dot{e}_1
+\frac{l_f C_f - l_r C_r}{I_z}\,e_2
-\frac{l_f^2 C_f + l_r^2 C_r}{I_z\,v_x}\,\dot{e}_2
+\frac{l_f C_f}{I_z}\,\delta_f
-\frac{l_f^2 C_f + l_r^2 C_r}{I_z\,v_x}\,\dot\theta_{\text{ref}}
$$

---

## 5 状态空间形式

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B\,\delta_f + G\,\dot\theta_{\text{ref}}
$$

其中 $\mathbf{x} = [e_1,\;\dot{e}_1,\;e_2,\;\dot{e}_2]^T$。

### 5.1 系统矩阵 $A$

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\[6pt]
0 & -\dfrac{C_f+C_r}{m\,v_x} & \dfrac{C_f+C_r}{m} & -\dfrac{l_f C_f - l_r C_r}{m\,v_x} \\[6pt]
0 & 0 & 0 & 1 \\[6pt]
0 & -\dfrac{l_f C_f - l_r C_r}{I_z\,v_x} & \dfrac{l_f C_f - l_r C_r}{I_z} & -\dfrac{l_f^2 C_f + l_r^2 C_r}{I_z\,v_x}
\end{bmatrix}
$$

### 5.2 输入矩阵 $B$

$$
B = \begin{bmatrix}
0 \\[6pt]
\dfrac{C_f}{m} \\[6pt]
0 \\[6pt]
\dfrac{l_f C_f}{I_z}
\end{bmatrix}
$$

### 5.3 扰动矩阵 $G$

$$
G = \begin{bmatrix}
0 \\[6pt]
-v_x - \dfrac{l_f C_f - l_r C_r}{m\,v_x} \\[6pt]
0 \\[6pt]
-\dfrac{l_f^2 C_f + l_r^2 C_r}{I_z\,v_x}
\end{bmatrix}
$$

---

## 6 物理解读

| 矩阵元素 | 物理意义 |
|---|---|
| $A_{22} = -\dfrac{C_f+C_r}{mv_x}$ | 前后轮侧偏力对横向速度的阻尼 |
| $A_{23} = \dfrac{C_f+C_r}{m}$ | 航向偏差通过侧偏力产生的横向加速度 |
| $A_{24} = -\dfrac{C_fl_f - C_rl_r}{mv_x}$ | 前后轮力矩不对称对横向运动的耦合 |
| $A_{42} = -\dfrac{l_fC_f - l_rC_r}{I_zv_x}$ | 横向速度通过力矩差对横摆的耦合 |
| $A_{43} = \dfrac{l_fC_f - l_rC_r}{I_z}$ | 航向偏差产生的横摆力矩 |
| $A_{44} = -\dfrac{l_f^2C_f + l_r^2C_r}{I_zv_x}$ | 轮胎侧偏力对横摆的阻尼 |
| $B_2 = \dfrac{C_f}{m}$ | 前轮转角对横向加速度的控制增益 |
| $B_4 = \dfrac{l_fC_f}{I_z}$ | 前轮转角对横摆加速度的控制增益 |
| $G_2 = -v_x - \dfrac{l_fC_f - l_rC_r}{mv_x}$ | 路径曲率对横向加速度的扰动（含离心项 $v_x$） |
| $G_4 = -\dfrac{l_f^2C_f + l_r^2C_r}{I_zv_x}$ | 路径曲率对横摆加速度的扰动 |

---

## 7 前馈转角推导

### 7.1 稳态圆弧跟踪条件

设车辆以恒定速度 $v_x$ 跟踪曲率为 $\kappa$ 的圆弧路径。稳态时：

$$
\dot v_y = 0, \quad \dot\omega = 0, \quad \omega_{ss} = v_x\kappa = \dot\theta_{\text{ref}}
$$

### 7.2 从车辆动力学推导

稳态侧向力平衡（$\delta_r=0$）：

$$
m v_x \omega_{ss} = C_f \alpha_{f,ss} + C_r \alpha_{r,ss}
$$

稳态横摆力矩平衡：

$$
0 = l_f C_f \alpha_{f,ss} - l_r C_r \alpha_{r,ss}
$$

由力矩平衡解得前后轮侧偏角之比：

$$
\alpha_{f,ss} = \frac{l_r C_r}{l_f C_f} \alpha_{r,ss}
$$

代入侧向力平衡：

$$
m v_x \omega_{ss} = \frac{l_r C_r}{l_f} \alpha_{r,ss} + C_r \alpha_{r,ss} = \frac{C_r L}{l_f} \alpha_{r,ss}
$$

解得：

$$
\alpha_{r,ss} = \frac{m v_x \omega_{ss} l_f}{C_r L} = \frac{m l_f v_x^2 \kappa}{C_r L}
$$

$$
\alpha_{f,ss} = \frac{m l_r v_x^2 \kappa}{C_f L}
$$

### 7.3 恢复前轮转角

由侧偏角定义 $\alpha_f = \delta_f - \zeta_f$，需要恢复前轮速度方向角。利用后轮侧偏角 $\alpha_r = -v_{y,ss}/v_x + l_r\omega_{ss}/v_x$（因为 $\delta_r=0$），解得稳态侧向速度：

$$
v_{y,ss} = (l_r \omega_{ss} - \alpha_{r,ss} v_x) = l_r v_x \kappa - \frac{m l_f v_x^3 \kappa}{C_r L}
$$

前轮速度方向角：

$$
\zeta_{f,ss} = \frac{v_{y,ss} + l_f \omega_{ss}}{v_x} = L\kappa - \frac{m l_f v_x^2 \kappa}{C_r L}
$$

因此前馈转角为：

$$
\delta_{f,ff} = \alpha_{f,ss} + \zeta_{f,ss} = \frac{m l_r v_x^2 \kappa}{C_f L} + L\kappa - \frac{m l_f v_x^2 \kappa}{C_r L}
$$

整理得：

$$
\boxed{\delta_{f,ff} = L\kappa + \frac{m v_x^2}{L}\left(\frac{l_r}{C_f} - \frac{l_f}{C_r}\right)\kappa}
$$

### 7.4 不足转向梯度

定义不足转向梯度：

$$
K_{us} = \frac{m}{L}\left(\frac{l_r}{C_f} - \frac{l_f}{C_r}\right)
$$

则前馈转角可简写为：

$$
\delta_{f,ff} = (L + K_{us} v_x^2)\,\kappa
$$

或等价地（利用侧向加速度 $a_y = v_x^2\kappa$）：

$$
\delta_{f,ff} = L\kappa + K_{us}\,a_y
$$

| 条件 | 含义 |
|------|------|
| $K_{us} > 0$（$l_r/C_f > l_f/C_r$） | 不足转向，需增大转角补偿 |
| $K_{us} = 0$ | 中性转向 |
| $K_{us} < 0$（$l_r/C_f < l_f/C_r$） | 过度转向，需减小转角 |

**物理意义**：前馈转角由两部分组成：
- $L\kappa$：纯运动学转角（低速 Ackermann 转角）
- $K_{us}v_x^2\kappa$：动力学修正项，补偿高速时轮胎侧偏导致的不足/过度转向

---

## 8 稳态误差分析

### 8.1 稳态误差的来源

尽管前馈转角 $\delta_{f,ff}$ 保证了车辆能够在曲率为 $\kappa$ 的路径上稳态行驶，但在误差状态空间中，**稳态航向误差 $e_{2,ss}$ 一般不为零**。

原因：稳态圆弧行驶时，侧向速度 $v_{y,ss}\neq 0$（车辆存在质心侧偏角 $\beta_{ss}$）。而误差运动学要求：

$$
\dot e_1 = 0 \implies v_{y,ss} + v_x \, e_{2,ss} = 0
$$

因此：

$$
e_{2,ss} = -\frac{v_{y,ss}}{v_x} = -\beta_{ss}
$$

### 8.2 稳态侧偏角

由第 7.3 节的 $v_{y,ss}$：

$$
\beta_{ss} = \frac{v_{y,ss}}{v_x} = l_r\kappa - \frac{ml_f v_x^2 \kappa}{C_r L}
$$

因此稳态航向误差为：

$$
\boxed{e_{2,ss} = -l_r\kappa + \frac{ml_f v_x^2 \kappa}{C_r L} = \left(\frac{ml_f v_x^2}{C_r L} - l_r\right)\kappa}
$$

**讨论**：

- 低速时（$v_x \to 0$）：$e_{2,ss} \approx -l_r\kappa$，航向误差由纯几何决定
- 高速时：$e_{2,ss}$ 随 $v_x^2$ 增大，且方向可能反转
- $e_{2,ss} = 0$ 的临界速度为 $v_x^* = \sqrt{l_r C_r L / (m l_f)}$

### 8.3 稳态横向误差

稳态时 $\dot e_2 = 0 \implies \omega_{ss} = \dot\theta_{\text{ref}}$（满足），且由 $A$ 矩阵第一列全为零可知 $e_1$ 不出现在动力学中。

在仅使用前馈（无反馈）的情况下，$e_1$ 是一个**积分器状态**——任何初始横向偏差都不会自然收敛。因此：

$$
e_{1,ss} = \text{取决于初始条件（无反馈时不收敛）}
$$

### 8.4 总结

| 误差 | 仅前馈 | 前馈 + 反馈 |
|------|--------|-------------|
| $e_1$ | 不收敛（开环积分器） | 反馈消除 |
| $e_2$ | 存在稳态偏差 $-\beta_{ss}$ | 反馈可大幅减小但不完全消除；需积分项才能消除 |
| $\dot e_1$ | 0（稳态） | 0 |
| $\dot e_2$ | 0（稳态） | 0 |

---

## 9 完整控制律

### 9.1 反馈 + 前馈结构

$$
\delta_f = \delta_{f,fb} + \delta_{f,ff}
$$

其中：
- **反馈**：$\delta_{f,fb} = -K\mathbf{x} = -[k_1\;k_2\;k_3\;k_4]\begin{bmatrix}e_1\\\dot e_1\\e_2\\\dot e_2\end{bmatrix}$（由 LQR 或极点配置确定）
- **前馈**：$\delta_{f,ff} = (L + K_{us}v_x^2)\,\kappa$

### 9.2 从状态方程直接推导前馈

在状态方程中，令 $\dot{\mathbf{x}} = 0$，$\mathbf{x} = 0$（期望完美跟踪）：

$$
0 = B\,\delta_{f,ff} + G\,\dot\theta_{\text{ref}}
$$

由于 $B\in\mathbb{R}^{4\times1}$，$G\in\mathbb{R}^{4\times1}$，此方程在一般情况下**超定无解**（1 个自由度无法同时满足 2 个约束）。

工程中取**横摆通道**（第 4 行）来确定前馈：

$$
0 = \frac{l_f C_f}{I_z}\,\delta_{f,ff} - \frac{l_f^2 C_f + l_r^2 C_r}{I_z v_x}\,\dot\theta_{\text{ref}}
$$

解得：

$$
\delta_{f,ff} = \frac{l_f^2 C_f + l_r^2 C_r}{l_f C_f \, v_x}\,\dot\theta_{\text{ref}}
$$

利用 $\dot\theta_{\text{ref}} = v_x\kappa$ 代入：

$$
\delta_{f,ff} = \frac{l_f^2 C_f + l_r^2 C_r}{l_f C_f}\,\kappa
$$

展开验证与第 7 节结果一致：

$$
\frac{l_f^2 C_f + l_r^2 C_r}{l_f C_f} = l_f + \frac{l_r^2 C_r}{l_f C_f} = L + \frac{l_r^2 C_r - l_r l_f C_f}{... }
$$

直接验证：

$$
\frac{l_f^2C_f + l_r^2C_r}{l_fC_f} = l_f + \frac{l_r^2C_r}{l_fC_f}
$$

$$
L + K_{us}v_x^2 = (l_f+l_r) + \frac{m v_x^2}{L}\left(\frac{l_r}{C_f}-\frac{l_f}{C_r}\right)
$$

两者在 $v_x$ 依赖性上不同——这是因为状态方程方法假设 $\mathbf{x}=0$（含 $e_2=0$），而车辆动力学方法允许 $e_2 = -\beta_{ss}\neq 0$。两种前馈各有侧重：

| 方法 | 前馈表达式 | 优化目标 |
|------|-----------|---------|
| 车辆动力学法 | $(L+K_{us}v_x^2)\kappa$ | 实现稳态圆弧行驶（允许 $e_2=-\beta_{ss}$） |
| 横摆通道法 | $\dfrac{l_f^2C_f+l_r^2C_r}{l_fC_f}\kappa$ | 消除横摆通道稳态扰动（$\ddot e_2 = 0$ 时 $\dot e_2$ 无偏） |

实际工程中两种方法均被使用，差异由反馈环节补偿。

---

## 10 使用前提

1. $\delta_r = 0$（纯前轮转向）
2. 小角度近似：$\delta_f$、$e_2$、$\beta$ 均较小
3. 纵向速度近似恒定：$\dot v_x \approx 0$
4. 线性轮胎模型：$F_{yf}=C_f\alpha_f$，$F_{yr}=C_r\alpha_r$，$C_f,C_r > 0$
5. 参考航向变化率缓慢：$\ddot\theta_{\text{ref}}\approx 0$
6. 忽略纵向力对侧向/横摆的影响
