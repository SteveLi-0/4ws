# 四轮转向扰动观测器（4WS Steer DOB Observer）核心模型

> 在 06 文档（前轮转向 DOB）基础上，引入后轮转向角 $\delta_r$。从微分方程出发推导增广 3 阶 Luenberger 观测器。
>
> **符号约定**：侧偏角采用 01 文档的正值定义（$\alpha = \delta - \zeta$，$C_f, C_r > 0$，$F_y = C_\alpha \alpha$）。

---

## 1. 坐标系与正方向

### 1.1 车体坐标系（右手系）

$x$ 前、$y$ 左、$z$ 上，角度遵循右手定则。本文推导在此坐标系下进行。

| 物理量 | 正方向 | 右手定则 |
| --- | --- | --- |
| $v_y$ | **向左** | $y$ 轴正方向 |
| $r = \dot\psi$ | **逆时针** | 右手绕 $z$ 轴 |
| $\delta_f, \delta_r$ | **向左转** | 右手绕 $z$ 轴 |
| $\phi$ | **左高右低** | 右手绕 $x$ 轴 |
| $a_y$ | **向左** | $y$ 轴正方向 |
| $F_{yf}, F_{yr}$ | **向左** | $y$ 轴正方向 |

> $\phi > 0$（左高右低）→ 重力分量指向**右**（$-y$ 方向）。

### 1.2 定位坐标系

定位坐标系取 $v_y$ **向右为正**（$\tilde{v}_y = -v_y$）。06 文档的状态空间模型即采用此方向。§4 给出从右手系到定位坐标系的变换及对应矩阵。

### 1.3 符号定义

| 符号 | 含义 |
| --- | --- |
| $v_y$ | 质心侧向速度 |
| $r$ | 横摆角速度，$r = \dot\psi$ |
| $\delta_d$ | 扰动等效前轮转角（待估计） |
| $\delta_f$ | 前轮转角 |
| $\delta_r$ | **后轮转角（新增）** |
| $\phi$ | 道路横滚角 |
| $v_x$ | 纵向速度 |
| $C_f, C_r$ | 前/后轴侧偏刚度（$> 0$） |
| $l_f, l_r$ | 质心到前/后轴距离 |
| $m$ | 车辆质量 |
| $I_z$ | 横摆转动惯量 |
| $g$ | 重力加速度（9.8） |
| $T_s$ | 控制周期 |

---

## 2. 微分方程推导（右手系，$v_y$ 向左为正）

### 2.1 侧偏角与侧偏力

前、后轮速度方向角（01 文档 §11，小角度近似）：

$$\zeta_f = \frac{v_y + l_f r}{v_x}, \quad \zeta_r = \frac{v_y - l_r r}{v_x}$$

侧偏角（01 文档 §12，正值定义 $\alpha = \delta - \zeta$）：

$$\alpha_f = \delta_f - \frac{v_y + l_f r}{v_x} \tag{S1}$$

$$\alpha_r = \delta_r - \frac{v_y - l_r r}{v_x} \tag{S2}$$

轮胎侧偏力（01 文档 §13，向左为正）：

$$F_{yf} = C_f \alpha_f = C_f\!\left(\delta_f - \frac{v_y + l_f r}{v_x}\right) \tag{F1}$$

$$F_{yr} = C_r \alpha_r = C_r\!\left(\delta_r - \frac{v_y - l_r r}{v_x}\right) \tag{F2}$$

> **校验**：$v_y > 0$（车向左漂），$\delta = r = 0$ → $\alpha_f < 0$ → $F_{yf} < 0$（向右恢复力） ✓

### 2.2 运动方程

侧向加速度（01 文档 §7）：$a_y = \dot{v}_y + v_x r$

**侧向力平衡**（$\phi > 0$ 左高右低，重力向右即 $-y$）：

$$m(\dot{v}_y + v_x r) = F_{yf} + F_{yr} - mg\phi \tag{1}$$

**横摆力矩平衡**（01 文档 §10，小角度近似）：

$$I_z \dot{r} = l_f F_{yf} - l_r F_{yr} \tag{2}$$

### 2.3 代入化简

将 (F1)、(F2) 代入 (1)，移项 $mv_xr$，两边除以 $m$：

$$\boxed{\dot{v}_y = -\frac{C_f+C_r}{mv_x}\,v_y + \left(\frac{-C_f l_f + C_r l_r}{mv_x} - v_x\right)r + \frac{C_f}{m}\,\delta_f + \frac{C_r}{m}\,\delta_r - g\,\phi} \tag{3}$$

将 (F1)、(F2) 代入 (2)，两边除以 $I_z$：

$$\boxed{\dot{r} = \frac{-C_f l_f + C_r l_r}{I_z v_x}\,v_y - \frac{C_f l_f^2 + C_r l_r^2}{I_z v_x}\,r + \frac{C_f l_f}{I_z}\,\delta_f - \frac{C_r l_r}{I_z}\,\delta_r} \tag{4}$$

---

## 3. 状态空间模型（右手系）

增广扰动状态 $\delta_d$（随机游走 $\dot\delta_d = 0$），输入从 06 文档的 $[\delta_f,\,\phi]^T$ 增至 $[\delta_f,\,\delta_r,\,\phi]^T$：

$$\mathbf{x} = \begin{bmatrix} v_y \\ r \\ \delta_d \end{bmatrix}, \quad \mathbf{u} = \begin{bmatrix} \delta_f \\ \delta_r \\ \phi \end{bmatrix}, \quad y = r_{\text{meas}}, \quad \dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}$$

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

**物理解读**：

| 输入 | $\dot{v}_y$ 系数 | $\dot{r}$ 系数 | 含义 |
| --- | --- | --- | --- |
| $\delta_f > 0$（左转） | $+C_f/m$（向左） | $+C_fl_f/I_z$（逆时针） | 前轮向左力 + 逆时针力矩 |
| $\delta_r > 0$（左转） | $+C_r/m$（向左） | $-C_rl_r/I_z$（**顺时针**） | 后轮向左力 + **反向**力矩 |
| $\phi > 0$（左高右低） | $-g$（**向右**推） | $0$ | 重力沿坡面向右 |

---

## 4. 定位坐标系形式（$\tilde{v}_y$ 向右为正）

### 4.1 坐标变换

定位坐标系取 $\tilde{v}_y = -v_y$（向右为正）。右手系中 $\phi$ 由坐标轴定义，不随 $v_y$ 翻转，因此仅作状态变换：

$$T = \mathrm{diag}(-1,\;1,\;1), \quad \tilde{A} = TAT^{-1}, \quad \tilde{B} = TB$$

### 4.2 变换后的矩阵

$$\tilde{A} = \begin{bmatrix}
-\dfrac{C_f + C_r}{m v_x} & \dfrac{C_f l_f - C_r l_r}{m v_x} + v_x & -\dfrac{C_f}{m} \\[8pt]
\dfrac{C_f l_f - C_r l_r}{I_z v_x} & -\dfrac{C_f l_f^2 + C_r l_r^2}{I_z v_x} & \dfrac{C_f l_f}{I_z} \\[8pt]
0 & 0 & 0
\end{bmatrix} \tag{A-loc}$$

$$\tilde{B} = \begin{bmatrix}
-\dfrac{C_f}{m} & -\dfrac{C_r}{m} & g \\[8pt]
\dfrac{C_f l_f}{I_z} & -\dfrac{C_r l_r}{I_z} & 0 \\[8pt]
0 & 0 & 0
\end{bmatrix} \tag{B-loc}$$

> $\tilde{B}_{13} = (-1) \times (-g) = +g$：右手系中 $\phi > 0$ 重力向右（$B_{13} = -g$），翻转 $v_y$ 后向右为正方向，$+g$ ✓

### 4.3 逐元素对照

| 位置 | 右手系 (RH) | 定位系 (loc) | 变换 |
| --- | --- | --- | --- |
| $A_{11}$ | $-(C_f+C_r)/(mv_x)$ | 同左 | 不变 |
| $A_{12}$ | $(-C_fl_f+C_rl_r)/(mv_x)-v_x$ | $(C_fl_f-C_rl_r)/(mv_x)+v_x$ | 取反 |
| $A_{13}$ | $+C_f/m$ | $-C_f/m$ | 取反 |
| $A_{21}$ | $(-C_fl_f+C_rl_r)/(I_zv_x)$ | $(C_fl_f-C_rl_r)/(I_zv_x)$ | 取反 |
| $A_{22}$ | $-(C_fl_f^2+C_rl_r^2)/(I_zv_x)$ | 同左 | 不变 |
| $A_{23}$ | $+C_fl_f/I_z$ | 同左 | 不变 |
| $B_{11}$ | $+C_f/m$ | $-C_f/m$ | 取反 |
| $B_{12}$ | $+C_r/m$ | $-C_r/m$ | 取反 |
| $B_{13}$ | $-g$ | $+g$ | 取反 |
| $B_{21}, B_{22}$ | 同右 | 同左 | 不变 |

> 规律：$T$ 取反 $A$ 第 1 行非对角 + 第 1 列非对角 + $B$ 第 1 行。横摆行（第 2 行）不受影响。

### 4.4 与 06 文档一致性

令 $\delta_r = 0$，(A-loc) 和 (B-loc) 退化为 06 文档的 $A$、$B$ 矩阵 ✓

---

## 5. 后轮转向的物理含义

| 转向模式 | 条件 | 侧向力 | 横摆力矩 | 典型场景 |
| --- | --- | --- | --- | --- |
| 同相位 | $\delta_r$ 与 $\delta_f$ 同号 | 叠加增强 | 相互抵消 | 高速变道（蟹行） |
| 反相位 | $\delta_r$ 与 $\delta_f$ 异号 | 部分抵消 | 叠加增强 | 低速转弯（灵活） |

> $B$ 第 1 列（前轮）与第 2 列（后轮）：侧向分量同号，横摆分量异号——两套矩阵形式均满足此规律。

---

## 6. 可观测性

观测器仅量测 $y = r$，扰动 $\delta_d$ 通过间接链路可观：

$$\delta_d \xrightarrow{A_{13}} v_y \xrightarrow{A_{21}} r \xrightarrow{C} y$$

只要 $C_f \neq 0$ 且 $l_f \neq 0$，观测矩阵 $\mathcal{O} = [C;\;CA;\;CA^2]$ 满秩。

---

## 7. 观测器设计

> 以下公式对两套坐标方向均成立，仅 $A_d$、$B_d$ 取对应版本（RH 或 loc）。

### 7.1 离散化（前向欧拉）

$$A_d = I + A T_s, \quad B_d = B T_s$$

### 7.2 Luenberger 观测器

离散系统 $\mathbf{x}_{k+1} = A_d \mathbf{x}_k + B_d \mathbf{u}_k$，$y_k = C \mathbf{x}_k$，观测器：

$$\hat{\mathbf{x}}_{k+1} = A_d \hat{\mathbf{x}}_k + B_d \mathbf{u}_k + L(y_k - C \hat{\mathbf{x}}_k)$$

误差动力学：$\mathbf{e}_{k+1} = (A_d - LC)\,\mathbf{e}_k$。选择 $L$ 使特征值在单位圆内。

### 7.3 预测—校正实现

**预测**（$\mathbf{u} = [\delta_f,\,\delta_r,\,\phi]^T$）：

$$\hat{\mathbf{x}}^- = A_d \hat{\mathbf{x}} + B_d \mathbf{u}$$

**校正**：

$$e = r_{\text{meas}} - \hat{r}^-, \quad \hat{\mathbf{x}} = \hat{\mathbf{x}}^- + L \cdot e$$

### 7.4 增益调度

$L$ 为 $3 \times 1$，扰动通道 $L_3$ 动态缩放：

$$L_3^{\text{actual}} = L_3^{\text{base}} \cdot f(v) \cdot \max(k_{\text{lc}},\; k_{\text{mhe}})$$

---

## 8. 快速扰动隔离（可选）

> 同 06 文档，不因 4WS 而改变。

$$\hat{r}' = \hat{r}^- + d_{r,\text{last}}, \quad e_{\text{total}} = r_{\text{meas}} - \hat{r}'$$

高通：$e_{\text{HP}} = \dfrac{\tau}{\tau + T_s}\left(e_{\text{HP,last}} + e_{\text{total}} - e_{\text{total,last}}\right)$

低通：$e_{\text{LP}} = e_{\text{total}} - e_{\text{HP}}$

- $e_{\text{LP}}$ → 主观测器
- $e_{\text{HP}}$ → 横摆扰动项：$d_r = e^{-T_s/\tau} \cdot d_{r,\text{last}} + k_{\text{rapid}} \cdot e_{\text{HP}},\; |d_r| \le d_{\max}$

---

## 9. 输出

$$\delta_{\text{disturb}} = \text{Clamp}\!\left(\text{DeltaAngleToStrAng}(\hat\delta_d),\; \pm \delta_{\max}\right)$$

---

## 附录：三条独立推导路径汇总

本文结果经三条独立路径交叉验证（详见 06b、06c 文档），最终定位坐标系矩阵完全一致：

| 文档 | 起始约定 | $v_y$ 方向 | $\phi$ 约定 | 到定位系的变换 | $B_{13}$ 路径 |
| --- | --- | --- | --- | --- | --- |
| **06a**（本文 §2–3） | 右手系 | 向左 | 右手绕 $x$（左高右低 +） | $\tilde{B}=TB$ | $(-1)(-g)=+g$ |
| **06b** | 右手系 | 向左 | 同上 | 同上 | 同上 |
| **06c** | 混合约定 | **向右** | 左高右低 + | 无需变换 | 直接 $+g$ |

> **结论**：无论出发约定如何，定位坐标系（$\tilde{v}_y$ 向右为正）下 $\tilde{B}_{13} = +g$ 恒成立，全部 11 个矩阵元素均一致。
