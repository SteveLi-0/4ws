# 四轮转向 Frenet 坐标系下横向误差方程推导

## 1 符号定义

| 符号 | 含义 |
|------|------|
| $e_1$ | 横向位置误差 |
| $e_2$ | 航向误差 $\theta - \theta_{\text{ref}}$ |
| $\delta_f,\;\delta_r$ | 前、后轮转角（控制量） |
| $v_x$ | 纵向速度（近似常值） |
| $v_y$ | 侧向速度 |
| $\omega$ | 横摆角速度 |
| $m$ | 整车质量 |
| $I_z$ | 绕 $z$ 轴转动惯量 |
| $l_f,\;l_r$ | 质心到前、后轴距离 |
| $C_f,\;C_r$ | 前、后轮侧偏刚度 |
| $\kappa$ | 参考路径曲率 |
| $\dot\theta_{\text{ref}}$ | 参考航向变化率（扰动），$\dot\theta_{\text{ref}} = \kappa\,v_{\text{ref}}$ |

---

## 2 四轮转向自行车模型

### 2.1 轮胎侧偏角

$$
\alpha_f = \delta_f - \frac{v_y + l_f\,\omega}{v_x}, \qquad
\alpha_r = \delta_r - \frac{v_y - l_r\,\omega}{v_x}
$$

### 2.2 侧向与横摆动力学

$$
m\,\dot{v}_y = -m\,v_x\,\omega + C_f\,\alpha_f + C_r\,\alpha_r
$$

$$
I_z\,\dot{\omega} = l_f\,C_f\,\alpha_f - l_r\,C_r\,\alpha_r
$$

展开后：

$$
m\,\dot{v}_y = -m\,v_x\,\omega
+C_f\!\left(\delta_f - \frac{v_y + l_f\omega}{v_x}\right)
+C_r\!\left(\delta_r - \frac{v_y - l_r\omega}{v_x}\right)
$$

$$
I_z\,\dot{\omega} =
l_f C_f\!\left(\delta_f - \frac{v_y + l_f\omega}{v_x}\right)
-l_r C_r\!\left(\delta_r - \frac{v_y - l_r\omega}{v_x}\right)
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

由此可得中间变量的状态替换关系：

$$
v_y = \dot{e}_1 - v_x\,e_2, \qquad
\omega = \dot{e}_2 + \dot\theta_{\text{ref}}
$$

---

## 4 误差状态方程推导

### 4.1 $\ddot{e}_1$ 的推导

$$
\ddot{e}_1 = \dot{v}_y + v_x\,\dot{e}_2
$$

将 $\dot{v}_y$ 的动力学方程代入，并用 $v_y = \dot{e}_1 - v_x e_2$、$\omega = \dot{e}_2 + \dot\theta_{\text{ref}}$ 替换，整理得：

$$
\ddot{e}_1 =
-\frac{C_f + C_r}{m\,v_x}\,\dot{e}_1
+\frac{C_f + C_r}{m}\,e_2
-\frac{l_f C_f - l_r C_r}{m\,v_x}\,\dot{e}_2
+\frac{C_f}{m}\,\delta_f
+\frac{C_r}{m}\,\delta_r
+\left[-v_x - \frac{l_f C_f - l_r C_r}{m\,v_x}\right]\dot\theta_{\text{ref}}
$$

> **推导细节**：$\dot{v}_y + v_x\dot{e}_2$ 中 $-mv_x\omega$ 项与 $v_x\dot{e}_2$ 合并时，$v_x(\dot{e}_2 + \dot\theta_{\text{ref}})$ 的 $v_x\dot{e}_2$ 部分相消，剩余 $-v_x\dot\theta_{\text{ref}}$ 进入扰动项。

### 4.2 $\ddot{e}_2$ 的推导

$$
\ddot{e}_2 = \dot{\omega} - \ddot\theta_{\text{ref}}
$$

假设参考曲率变化缓慢（$\ddot\theta_{\text{ref}} \approx 0$），代入 $\dot\omega$ 并替换中间变量：

$$
\ddot{e}_2 =
-\frac{l_f C_f - l_r C_r}{I_z\,v_x}\,\dot{e}_1
+\frac{l_f C_f - l_r C_r}{I_z}\,e_2
-\frac{l_f^2 C_f + l_r^2 C_r}{I_z\,v_x}\,\dot{e}_2
+\frac{l_f C_f}{I_z}\,\delta_f
-\frac{l_r C_r}{I_z}\,\delta_r
-\frac{l_f^2 C_f + l_r^2 C_r}{I_z\,v_x}\,\dot\theta_{\text{ref}}
$$

---

## 5 状态空间形式

定义状态向量 $\mathbf{x} = [e_1,\;\dot{e}_1,\;e_2,\;\dot{e}_2]^T$，控制量 $\mathbf{u} = [\delta_f,\;\delta_r]^T$：

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B\,\mathbf{u} + C\,\dot\theta_{\text{ref}}
$$

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
0 & 0 \\[6pt]
\dfrac{C_f}{m} & \dfrac{C_r}{m} \\[6pt]
0 & 0 \\[6pt]
\dfrac{l_f C_f}{I_z} & -\dfrac{l_r C_r}{I_z}
\end{bmatrix}
$$

### 5.3 扰动矩阵 $C$

$$
C = \begin{bmatrix}
0 \\[6pt]
-v_x - \dfrac{l_f C_f - l_r C_r}{m\,v_x} \\[6pt]
0 \\[6pt]
-\dfrac{l_f^2 C_f + l_r^2 C_r}{I_z\,v_x}
\end{bmatrix}
$$

---

## 6 与纯前轮转向的对比

| | 纯前轮转向 ($\delta_r = 0$) | 四轮转向 |
|---|---|---|
| 控制量 | $\delta_f$ 标量 | $[\delta_f,\;\delta_r]^T$ 向量 |
| $B$ 矩阵 | 单列：$[0,\;C_f/m,\;0,\;l_fC_f/I_z]^T$ | 双列：后轮通过 $C_r/m$ 和 $-l_rC_r/I_z$ 提供额外控制自由度 |
| 控制自由度 | 1 DOF — 无法同时独立控制 $e_1$ 和 $e_2$ | 2 DOF — 可同时独立控制横向误差和航向误差 |
| 零侧偏条件 | 不可能 | 选取 $\delta_r / \delta_f = l_r / l_f$ 可实现低速零侧偏转向 |

---

## 7 备注

- **$A$ 矩阵与前轮转向完全相同**，四轮转向的优势完全体现在 $B$ 矩阵多出的一列，即后轮的控制通道。
- 扰动项 $\dot\theta_{\text{ref}} = \kappa\,v_{\text{ref}}$ 在跟踪高曲率路径时不可忽略，是前馈补偿的核心。
- 以上推导基于小角度线性化假设（$\sin\alpha \approx \alpha$，$\cos\alpha \approx 1$），适用于正常行驶工况。

---

## 8 纯前轮转向误差方程（$\delta_r = 0$）

令 $\delta_r = 0$，控制量退化为标量 $u = \delta_f$，扰动为 $w = \dot\theta_{\text{ref}}$。

### 8.1 直接令 $\delta_r = 0$

将第 4 节结果中所有 $\delta_r$ 置零即可。

**$\ddot{e}_1$ 方程**：

$$
\ddot{e}_1 =
-\frac{C_f + C_r}{m\,v_x}\,\dot{e}_1
+\frac{C_f + C_r}{m}\,e_2
-\frac{l_f C_f - l_r C_r}{m\,v_x}\,\dot{e}_2
+\frac{C_f}{m}\,\delta_f
+\left[-v_x - \frac{l_f C_f - l_r C_r}{m\,v_x}\right]\dot\theta_{\text{ref}}
$$

**$\ddot{e}_2$ 方程**：

$$
\ddot{e}_2 =
-\frac{l_f C_f - l_r C_r}{I_z\,v_x}\,\dot{e}_1
+\frac{l_f C_f - l_r C_r}{I_z}\,e_2
-\frac{l_f^2 C_f + l_r^2 C_r}{I_z\,v_x}\,\dot{e}_2
+\frac{l_f C_f}{I_z}\,\delta_f
-\frac{l_f^2 C_f + l_r^2 C_r}{I_z\,v_x}\,\dot\theta_{\text{ref}}
$$

### 8.2 状态空间形式

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B\,\delta_f + G\,\dot\theta_{\text{ref}}
$$

其中 $\mathbf{x} = [e_1,\;\dot{e}_1,\;e_2,\;\dot{e}_2]^T$。

**系统矩阵 $A$**（与 4WS 完全相同）：

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\[6pt]
0 & -\dfrac{C_f+C_r}{m\,v_x} & \dfrac{C_f+C_r}{m} & -\dfrac{l_f C_f - l_r C_r}{m\,v_x} \\[6pt]
0 & 0 & 0 & 1 \\[6pt]
0 & -\dfrac{l_f C_f - l_r C_r}{I_z\,v_x} & \dfrac{l_f C_f - l_r C_r}{I_z} & -\dfrac{l_f^2 C_f + l_r^2 C_r}{I_z\,v_x}
\end{bmatrix}
$$

**输入矩阵 $B$**（$B$ 为 4WS 的第一列）：

$$
B = \begin{bmatrix}
0 \\[6pt]
\dfrac{C_f}{m} \\[6pt]
0 \\[6pt]
\dfrac{l_f C_f}{I_z}
\end{bmatrix}
$$

**扰动矩阵 $G$**（与 4WS 的 $C$ 矩阵相同）：

$$
G = \begin{bmatrix}
0 \\[6pt]
-v_x - \dfrac{l_f C_f - l_r C_r}{m\,v_x} \\[6pt]
0 \\[6pt]
-\dfrac{l_f^2 C_f + l_r^2 C_r}{I_z\,v_x}
\end{bmatrix}
$$

### 8.3 物理解读

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

### 8.4 使用前提

1. $\delta_r = 0$（纯前轮转向）
2. 小角度近似：$\delta_f$、$e_2$、$\beta$ 均较小
3. 纵向速度近似恒定：$\dot v_x \approx 0$
4. 线性轮胎模型：$F_{yf}=C_f\alpha_f$，$F_{yr}=C_r\alpha_r$，$C_f,C_r > 0$
5. 参考航向变化率缓慢：$\ddot\theta_{\text{ref}}\approx 0$
6. 忽略纵向力对侧向/横摆的影响