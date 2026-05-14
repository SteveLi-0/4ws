# 后轴参考点定义下的横向误差方程

> 本文将误差参考点从质心改为**后轴中心**，推导 $\delta_r=0$ 条件下的误差状态空间方程。符号约定与 [[01_bicycle_model]] 一致，质心参考点版本见 [[02a_error_cg_front_steer]]。

---

## 1 符号定义

| 符号 | 含义 |
|------|------|
| $e_1$ | 后轴中心到参考路径的横向距离 |
| $e_2$ | 车身朝向与后轴投影点处路径切线的夹角，$e_2 = \psi - \theta_{\text{ref}}$ |
| $\delta_f$ | 前轮转角（控制量） |
| $v_x$ | 纵向速度（近似常值） |
| $v_y$ | 质心侧向速度 |
| $r$ | 横摆角速度 $\dot\psi$ |
| $m$ | 整车质量 |
| $I_z$ | 绕质心的横摆转动惯量 |
| $l_f,\;l_r$ | 质心到前、后轴距离；$L=l_f+l_r$ |
| $C_f,\;C_r$ | 前、后轮侧偏刚度（$>0$） |
| $\kappa$ | 后轴投影点处的参考路径曲率 |
| $\dot\theta_{\text{ref}}$ | 后轴投影点处的参考航向变化率（扰动） |

---

## 2 后轴运动学

### 2.1 后轴速度

刚体运动关系给出后轴中心在车体坐标系下的速度分量：

$$
v_{x,r} = v_x, \qquad v_{y,r} = v_y - l_r r
$$

因此后轴中心速度方向相对车身纵向的偏角为：

$$
\beta_r = \arctan\frac{v_{y,r}}{v_x} \approx \frac{v_y - l_r r}{v_x}
$$

### 2.2 后轴全局速度

后轴中心在全局坐标系下的速度为：

$$
\dot X_r = v_x\cos\psi - (v_y - l_r r)\sin\psi
$$

$$
\dot Y_r = v_x\sin\psi + (v_y - l_r r)\cos\psi
$$

---

## 3 误差运动学

### 3.1 横向误差 $e_1$ 的变化率

$e_1$ 的变化率为后轴速度在路径法线方向的投影。设路径切线方向角为 $\theta_{\text{ref}}$，则：

$$
\dot e_1 = \dot X_r\sin\theta_{\text{ref}} - \dot Y_r\cos\theta_{\text{ref}}
$$

> 注：此处取路径左侧为 $e_1$ 正方向。

展开并利用 $e_2 = \psi - \theta_{\text{ref}}$：

$$
\dot e_1 = v_x\sin e_2 + (v_y - l_r r)\cos e_2
$$

小角度近似（$e_2$ 较小）：

$$
\boxed{\dot e_1 \approx (v_y - l_r r) + v_x\,e_2}
$$

**与质心定义的关键区别**：$v_y$ 被替换为后轴侧向速度 $v_y - l_r r$。

### 3.2 航向误差 $e_2$ 的变化率

$$
\dot e_2 = \dot\psi - \dot\theta_{\text{ref}}
$$

其中 $\dot\theta_{\text{ref}} = \kappa\cdot\dot s$，而 $\dot s$ 为后轴沿路径的推进速率。对于小误差：

$$
\dot s \approx v_x\cos e_2 - (v_y - l_r r)\sin e_2 \approx v_x
$$

因此：

$$
\boxed{\dot e_2 \approx r - \dot\theta_{\text{ref}}}
$$

其中 $\dot\theta_{\text{ref}} \approx \kappa v_x$。

### 3.3 状态替换关系

由误差运动学反解车辆状态：

$$
v_y - l_r r = \dot e_1 - v_x\,e_2
$$

$$
r = \dot e_2 + \dot\theta_{\text{ref}}
$$

进而：

$$
v_y = (\dot e_1 - v_x\,e_2) + l_r(\dot e_2 + \dot\theta_{\text{ref}})
$$

---

## 4 车辆动力学（$\delta_r = 0$）

侧向力平衡：

$$
m(\dot v_y + v_x r) = F_{yf} + F_{yr}
$$

横摆力矩平衡：

$$
I_z\dot r = l_f F_{yf} - l_r F_{yr}
$$

线性轮胎力（用后轴侧向速度 $v_{yr} = v_y - l_r r$ 改写）：

$$
F_{yf} = C_f\!\left(\delta_f - \frac{v_{yr} + Lr}{v_x}\right), \qquad
F_{yr} = -\frac{C_r\,v_{yr}}{v_x}
$$

---

## 5 误差动力学推导

### 5.1 $\ddot e_2$ 的推导

$$
\ddot e_2 = \dot r \quad (\text{假设}\;\ddot\theta_{\text{ref}} \approx 0)
$$

将 $v_y$ 和 $r$ 代入 $\dot r$ 表达式：

$$
I_z\dot r = l_f C_f\!\left(\delta_f - \frac{v_{yr}+Lr}{v_x}\right) + l_r C_r\frac{v_{yr}}{v_x}
$$

其中 $v_{yr} = \dot e_1 - v_x e_2$，$r = \dot e_2 + \dot\theta_{\text{ref}}$，因此：

$$
v_{yr} + Lr = (\dot e_1 - v_x e_2) + L(\dot e_2 + \dot\theta_{\text{ref}})
$$

代入并按状态分量整理：

$$
\ddot e_2 =
\frac{l_f C_f}{I_z}\,\delta_f
-\frac{l_f C_f - l_r C_r}{I_z v_x}\,\dot e_1
+\frac{l_f C_f - l_r C_r}{I_z}\,e_2
-\frac{l_f C_f L}{I_z v_x}\,\dot e_2
-\frac{l_f C_f L}{I_z v_x}\,\dot\theta_{\text{ref}}
$$

### 5.2 $\ddot e_1$ 的推导

$$
\ddot e_1 = \dot v_{yr} + v_x\dot e_2 = (\dot v_y - l_r\dot r) + v_x(r - \dot\theta_{\text{ref}})
$$

利用动力学方程，$\dot v_y + v_x r = (F_{yf}+F_{yr})/m$，因此：

$$
\dot v_y = \frac{F_{yf}+F_{yr}}{m} - v_x r
$$

代入：

$$
\ddot e_1 = \frac{F_{yf}+F_{yr}}{m} - v_x r - l_r\dot r + v_x r - v_x\dot\theta_{\text{ref}}
= \frac{F_y}{m} - l_r\frac{M_z}{I_z} - v_x\dot\theta_{\text{ref}}
$$

> **关键消去**：$-v_x r + v_x r = 0$，与质心定义相同。但此处多出 $-l_r\dot r = -l_r M_z/I_z$ 项，这是后轴参考点与质心参考点的核心区别。

将轮胎力按系数分配：

$$
\ddot e_1 = F_{yf}\!\left(\frac{1}{m}-\frac{l_rl_f}{I_z}\right)
+F_{yr}\!\left(\frac{1}{m}+\frac{l_r^2}{I_z}\right)
-v_x\dot\theta_{\text{ref}}
$$

定义惯量耦合参数 $\eta = I_z - ml_fl_r$，则：

$$
\frac{1}{m}-\frac{l_rl_f}{I_z} = \frac{\eta}{mI_z}, \qquad
\frac{1}{m}+\frac{l_r^2}{I_z} = \frac{I_z+ml_r^2}{mI_z}
$$

代入轮胎力并用误差状态表示：

$$
\ddot e_1 =
\frac{C_f\eta}{mI_z}\,\delta_f
-\frac{C_f\eta + C_r(I_z+ml_r^2)}{mI_zv_x}\,\dot e_1
+\frac{C_f\eta + C_r(I_z+ml_r^2)}{mI_z}\,e_2
-\frac{C_fL\eta}{mI_zv_x}\,\dot e_2
-\left(\frac{C_fL\eta}{mI_zv_x}+v_x\right)\dot\theta_{\text{ref}}
$$

---

## 6 状态空间形式

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B\,\delta_f + G\,\dot\theta_{\text{ref}}
$$

其中 $\mathbf{x} = [e_1,\;\dot e_1,\;e_2,\;\dot e_2]^T$。

### 6.1 系统矩阵 $A$

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\[8pt]
0 & -\dfrac{C_f\eta + C_r(I_z+ml_r^2)}{mI_zv_x} & \dfrac{C_f\eta + C_r(I_z+ml_r^2)}{mI_z} & -\dfrac{C_fL\eta}{mI_zv_x} \\[8pt]
0 & 0 & 0 & 1 \\[8pt]
0 & -\dfrac{l_fC_f - l_rC_r}{I_zv_x} & \dfrac{l_fC_f - l_rC_r}{I_z} & -\dfrac{l_fC_fL}{I_zv_x}
\end{bmatrix}
$$

其中 $\eta = I_z - ml_fl_r$。

### 6.2 输入矩阵 $B$

$$
B = \begin{bmatrix}
0 \\[6pt]
\dfrac{C_f\eta}{mI_z} \\[6pt]
0 \\[6pt]
\dfrac{l_fC_f}{I_z}
\end{bmatrix}
$$

### 6.3 扰动矩阵 $G$

$$
G = \begin{bmatrix}
0 \\[6pt]
-\dfrac{C_fL\eta}{mI_zv_x} - v_x \\[6pt]
0 \\[6pt]
-\dfrac{l_fC_fL}{I_zv_x}
\end{bmatrix}
$$

---

## 7 惯量耦合参数 $\eta$ 的物理意义

$$
\eta = I_z - ml_fl_r
$$

| 条件 | 含义 |
|------|------|
| $\eta > 0$ | 转动惯量较大（质量分布在轴距外侧），前轮转角对后轴横向加速度有正增益 |
| $\eta = 0$ | 前轮转角对后轴横向加速度**无直接影响**（$B_2=0$），后轴横向运动完全由轮胎阻尼和航向耦合驱动 |
| $\eta < 0$ | 极端集中质量分布，前轮转角对后轴产生反向横向效应（实际车辆中罕见） |

> 对于均匀杆状车辆 $I_z = mL^2/12$，此时 $\eta = mL^2/12 - ml_fl_r$。当 $l_f = l_r = L/2$ 时，$\eta = mL^2/12 - mL^2/4 = -mL^2/6 < 0$。实际车辆的 $I_z$ 通常较大（$I_z > ml_fl_r$），故 $\eta > 0$。

---

## 8 与质心参考点定义的对比

质心定义的状态方程（详见 [[02a_error_cg_front_steer]]）：

$$
A_{\text{CG}} = \begin{bmatrix}
0 & 1 & 0 & 0 \\[6pt]
0 & -\dfrac{C_f+C_r}{mv_x} & \dfrac{C_f+C_r}{m} & -\dfrac{l_fC_f-l_rC_r}{mv_x} \\[6pt]
0 & 0 & 0 & 1 \\[6pt]
0 & -\dfrac{l_fC_f-l_rC_r}{I_zv_x} & \dfrac{l_fC_f-l_rC_r}{I_z} & -\dfrac{l_f^2C_f+l_r^2C_r}{I_zv_x}
\end{bmatrix}
$$

### 8.1 逐行对比

**第 4 行（$\ddot e_2$）——仅阻尼项不同**：

| 项 | 质心定义 | 后轴定义 |
|----|---------|---------|
| $A_{42}$（$\dot e_1$ 耦合） | $-\dfrac{l_fC_f - l_rC_r}{I_zv_x}$ | 相同 |
| $A_{43}$（$e_2$ 耦合） | $\dfrac{l_fC_f - l_rC_r}{I_z}$ | 相同 |
| $A_{44}$（$\dot e_2$ 阻尼） | $-\dfrac{l_f^2C_f + l_r^2C_r}{I_zv_x}$ | $-\dfrac{l_fC_fL}{I_zv_x}$ |
| $B_4$（转角增益） | $\dfrac{l_fC_f}{I_z}$ | 相同 |
| $G_4$（曲率扰动） | $-\dfrac{l_f^2C_f + l_r^2C_r}{I_zv_x}$ | $-\dfrac{l_fC_fL}{I_zv_x}$ |

差异来源：$v_y + l_fr = v_{yr} + Lr$，后轴定义中 $r$ 的系数从 $l_f$ 变为 $L$。

**第 2 行（$\ddot e_1$）——结构性变化**：

| 项 | 质心定义 | 后轴定义 |
|----|---------|---------|
| $A_{22}$ | $-\dfrac{C_f+C_r}{mv_x}$ | $-\dfrac{C_f\eta + C_r(I_z+ml_r^2)}{mI_zv_x}$ |
| $A_{23}$ | $\dfrac{C_f+C_r}{m}$ | $\dfrac{C_f\eta + C_r(I_z+ml_r^2)}{mI_z}$ |
| $A_{24}$ | $-\dfrac{l_fC_f - l_rC_r}{mv_x}$ | $-\dfrac{C_fL\eta}{mI_zv_x}$ |
| $B_2$ | $\dfrac{C_f}{m}$ | $\dfrac{C_f\eta}{mI_z}$ |

差异来源：$\ddot e_{1,\text{rear}} = \ddot e_{1,\text{CG}} - l_r\ddot e_2$，后轴横向加速度比质心多减去一个横摆加速度的力臂项。

### 8.2 等价性验证

两组方程在 $A_{22}$ 和 $A_{23}$ 上满足 $A_{23} = -v_x\cdot A_{22}$（后轴定义同样满足），这保证了物理一致性：$\dot e_1$ 的阻尼和 $e_2$ 的耦合来自同一个力。

---

## 9 前馈转角

稳态圆弧跟踪条件（$\dot v_y = 0$，$\dot r = 0$，$r = v_x\kappa$）不依赖于误差定义的参考点，因此前馈转角与质心定义相同：

$$
\delta_{f,ff} = (L + K_{us}v_x^2)\,\kappa
$$

其中不足转向梯度 $K_{us} = \dfrac{m}{L}\!\left(\dfrac{l_r}{C_f}-\dfrac{l_f}{C_r}\right)$。

### 9.1 稳态误差

稳态时 $\dot e_1 = 0$ 要求后轴侧向速度为零：

$$
v_{y,ss} - l_r r_{ss} = 0
$$

这意味着稳态时后轴的速度方向恰好沿着车身纵轴——而这正是 $\delta_r = 0$ 时后轮无侧偏（$\alpha_r = 0$）的条件。

由轮胎力平衡，稳态时：

$$
\alpha_{r,ss} = \frac{ml_fv_x^2\kappa}{C_rL}
$$

因此稳态后轴侧向速度为：

$$
v_{yr,ss} = -\alpha_{r,ss}\cdot v_x = -\frac{ml_fv_x^3\kappa}{C_rL} \neq 0
$$

故稳态横向误差率不为零，需要通过稳态航向误差补偿：

$$
e_{2,ss} = -\frac{v_{yr,ss}}{v_x} = \frac{ml_fv_x^2\kappa}{C_rL} = \alpha_{r,ss}
$$

**物理意义**：稳态时后轴存在侧偏，车身需偏转一个等于后轮侧偏角的航向角来保持后轴在路径上。

### 9.2 与质心定义的稳态误差对比

| | 质心定义 | 后轴定义 |
|---|---------|---------|
| $e_{2,ss}$ | $-\beta_{ss} = -l_r\kappa + \dfrac{ml_fv_x^2\kappa}{C_rL}$ | $\alpha_{r,ss} = \dfrac{ml_fv_x^2\kappa}{C_rL}$ |
| 低速极限 | $-l_r\kappa$（纯几何项） | $0$（后轮无侧偏） |
| 物理来源 | 质心侧偏角 $\beta$ | 后轮侧偏角 $\alpha_r$ |

> 后轴定义在**低速**时稳态航向误差趋于零，这是该定义在低速跟踪场景中的优势。

---

## 10 使用前提

1. $\delta_r = 0$（纯前轮转向）
2. 小角度近似：$\delta_f$、$e_2$、$\beta_r$ 均较小
3. 纵向速度近似恒定：$\dot v_x \approx 0$
4. 线性轮胎模型：$F_{yf}=C_f\alpha_f$，$F_{yr}=C_r\alpha_r$，$C_f,C_r > 0$
5. 参考航向变化率缓慢：$\ddot\theta_{\text{ref}}\approx 0$
6. 忽略纵向力对侧向/横摆的影响
7. 后轴投影点与质心投影点处的曲率近似相等
