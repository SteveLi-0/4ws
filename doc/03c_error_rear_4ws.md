# 后轴误差状态方程的正向推导（4WS）

> 本文直接从后轴 Frenet 误差定义出发，正向推导四轮转向车辆的横向误差状态方程。不依赖质心误差模型，也不经过坐标变换。坐标变换方法见 [[04_transform_cg_to_rear]]，$\delta_r=0$ 特例见 [[03a_error_rear_front_steer]]。

---

## 1 误差定义与状态向量

| 符号 | 含义 |
|------|------|
| $e_1$ | 后轴中心到参考路径的横向距离 |
| $e_2$ | 车身航向与后轴投影点处路径切线的夹角，$e_2 = \psi - \theta_{\text{ref}}$ |
| $\delta_f,\;\delta_r$ | 前、后轮转角（控制量） |
| $v_x$ | 纵向速度（近似恒定） |
| $v_y$ | 质心侧向速度 |
| $r$ | 横摆角速度 $\dot\psi$ |
| $v_{yr}$ | 后轴侧向速度，$v_{yr} = v_y - l_r r$ |
| $\eta$ | $I_z - ml_fl_r$（惯量耦合参数） |
| $\xi$ | $I_z + ml_r^2$（后轴等效惯量参数） |

状态向量 $\mathbf{x} = [e_1,\;\dot e_1,\;e_2,\;\dot e_2]^T$。

---

## 2 后轴误差运动学

后轴中心在 Frenet 坐标系下的误差运动学（小角度近似）：

$$
\dot e_1 = v_{yr} + v_x\,e_2
$$

$$
\dot e_2 = r - \dot\theta_{\text{ref}}
$$

反解车辆状态：

$$
v_{yr} = \dot e_1 - v_x\,e_2 \tag{1}
$$

$$
r = \dot e_2 + \dot\theta_{\text{ref}} \tag{2}
$$

进而恢复质心侧向速度：

$$
v_y = v_{yr} + l_r r = (\dot e_1 - v_x\,e_2) + l_r(\dot e_2 + \dot\theta_{\text{ref}}) \tag{3}
$$

---

## 3 4WS 车辆动力学

### 3.1 轮胎侧向力

用后轴侧向速度 $v_{yr}$ 改写前轴速度方向角：

$$
v_y + l_f r = v_{yr} + (l_f + l_r)r = v_{yr} + Lr
$$

因此轮胎力为：

$$
F_{yf} = C_f\!\left(\delta_f - \frac{v_{yr} + Lr}{v_x}\right) \tag{4}
$$

$$
F_{yr} = C_r\!\left(\delta_r - \frac{v_{yr}}{v_x}\right) \tag{5}
$$

### 3.2 力和力矩平衡

$$
m(\dot v_y + v_x r) = F_{yf} + F_{yr} \tag{6}
$$

$$
I_z\dot r = l_f F_{yf} - l_r F_{yr} \tag{7}
$$

---

## 4 推导 $\ddot e_2$

$$
\ddot e_2 = \dot r \quad (\text{假设}\;\ddot\theta_{\text{ref}} \approx 0)
$$

将式 (4)(5) 代入式 (7)：

$$
I_z\ddot e_2 = l_f C_f\!\left(\delta_f - \frac{v_{yr} + Lr}{v_x}\right) - l_r C_r\!\left(\delta_r - \frac{v_{yr}}{v_x}\right)
$$

用式 (1)(2) 替换 $v_{yr} = \dot e_1 - v_x e_2$，$r = \dot e_2 + \dot\theta_{\text{ref}}$：

$$
v_{yr} + Lr = (\dot e_1 - v_x e_2) + L(\dot e_2 + \dot\theta_{\text{ref}})
$$

展开并按状态分量整理：

$$
I_z\ddot e_2 = l_fC_f\delta_f - l_rC_r\delta_r + \frac{l_rC_r - l_fC_f}{v_x}\dot e_1 + (l_fC_f - l_rC_r)e_2 - \frac{l_fC_fL}{v_x}\dot e_2 - \frac{l_fC_fL}{v_x}\dot\theta_{\text{ref}}
$$

两侧除以 $I_z$：

$$
\boxed{\ddot e_2 = \frac{l_fC_f}{I_z}\delta_f - \frac{l_rC_r}{I_z}\delta_r - \frac{l_fC_f - l_rC_r}{I_zv_x}\dot e_1 + \frac{l_fC_f - l_rC_r}{I_z}e_2 - \frac{l_fC_fL}{I_zv_x}\dot e_2 - \frac{l_fC_fL}{I_zv_x}\dot\theta_{\text{ref}}}
$$

---

## 5 推导 $\ddot e_1$

### 5.1 建立表达式

$$
\ddot e_1 = \dot v_{yr} + v_x\dot e_2
$$

其中：

$$
\dot v_{yr} = \dot v_y - l_r\dot r
$$

由式 (6)：$\dot v_y = (F_{yf} + F_{yr})/m - v_x r$，代入：

$$
\ddot e_1 = \frac{F_{yf} + F_{yr}}{m} - v_x r - l_r\dot r + v_x(r - \dot\theta_{\text{ref}})
$$

### 5.2 关键消去

$$
-v_x r + v_x r = 0
$$

因此：

$$
\ddot e_1 = \frac{F_{yf} + F_{yr}}{m} - l_r\frac{l_fF_{yf} - l_rF_{yr}}{I_z} - v_x\dot\theta_{\text{ref}}
$$

### 5.3 按轮胎力分配系数

$$
\ddot e_1 = F_{yf}\!\left(\frac{1}{m} - \frac{l_rl_f}{I_z}\right) + F_{yr}\!\left(\frac{1}{m} + \frac{l_r^2}{I_z}\right) - v_x\dot\theta_{\text{ref}}
$$

利用惯量参数：

$$
\frac{1}{m} - \frac{l_rl_f}{I_z} = \frac{\eta}{mI_z}, \qquad \frac{1}{m} + \frac{l_r^2}{I_z} = \frac{\xi}{mI_z}
$$

因此：

$$
\ddot e_1 = \frac{\eta}{mI_z}\,F_{yf} + \frac{\xi}{mI_z}\,F_{yr} - v_x\dot\theta_{\text{ref}} \tag{8}
$$

> **物理解读**：后轴横向加速度由前后轮力通过不同的惯量耦合系数贡献。前轮力的增益 $\eta/(mI_z)$ 可能为零或负（取决于 $I_z$ 与 $ml_fl_r$ 的大小关系），而后轮力的增益 $\xi/(mI_z)$ 恒正。

### 5.4 代入轮胎力

将式 (4)(5) 代入式 (8)：

$$
\ddot e_1 = \frac{\eta}{mI_z}\,C_f\!\left(\delta_f - \frac{v_{yr} + Lr}{v_x}\right) + \frac{\xi}{mI_z}\,C_r\!\left(\delta_r - \frac{v_{yr}}{v_x}\right) - v_x\dot\theta_{\text{ref}}
$$

用式 (1)(2) 替换 $v_{yr}$ 和 $r$，展开：

**$\delta_f$ 项**：$\dfrac{C_f\eta}{mI_z}\delta_f$

**$\delta_r$ 项**：$\dfrac{C_r\xi}{mI_z}\delta_r$

**$\dot e_1$ 项**：前轮贡献 $-\dfrac{C_f\eta}{mI_zv_x}$，后轮贡献 $-\dfrac{C_r\xi}{mI_zv_x}$，合计 $-\dfrac{C_f\eta + C_r\xi}{mI_zv_x}$

**$e_2$ 项**：前轮贡献 $\dfrac{C_f\eta}{mI_z}$，后轮贡献 $\dfrac{C_r\xi}{mI_z}$，合计 $\dfrac{C_f\eta + C_r\xi}{mI_z}$

**$\dot e_2$ 项**：仅前轮贡献（$Lr$ 项中的 $L\dot e_2$），为 $-\dfrac{C_fL\eta}{mI_zv_x}$

**$\dot\theta_{\text{ref}}$ 项**：前轮贡献 $-\dfrac{C_fL\eta}{mI_zv_x}$，加上直接项 $-v_x$，合计 $-\dfrac{C_fL\eta}{mI_zv_x} - v_x$

最终：

$$
\boxed{\ddot e_1 = \frac{C_f\eta}{mI_z}\delta_f + \frac{C_r\xi}{mI_z}\delta_r - \frac{C_f\eta + C_r\xi}{mI_zv_x}\dot e_1 + \frac{C_f\eta + C_r\xi}{mI_z}e_2 - \frac{C_fL\eta}{mI_zv_x}\dot e_2 + \left(-\frac{C_fL\eta}{mI_zv_x} - v_x\right)\dot\theta_{\text{ref}}}
$$

---

## 6 状态空间形式

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B_f\,\delta_f + B_r\,\delta_r + G\,\dot\theta_{\text{ref}}
$$

### 6.1 系统矩阵 $A$

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\[8pt]
0 & -\dfrac{C_f\eta + C_r\xi}{mI_zv_x} & \dfrac{C_f\eta + C_r\xi}{mI_z} & -\dfrac{C_fL\eta}{mI_zv_x} \\[8pt]
0 & 0 & 0 & 1 \\[8pt]
0 & -\dfrac{l_fC_f - l_rC_r}{I_zv_x} & \dfrac{l_fC_f - l_rC_r}{I_z} & -\dfrac{l_fC_fL}{I_zv_x}
\end{bmatrix}
$$

### 6.2 输入矩阵

$$
B_f = \begin{bmatrix} 0 \\ \dfrac{C_f\eta}{mI_z} \\ 0 \\ \dfrac{l_fC_f}{I_z} \end{bmatrix}, \qquad
B_r = \begin{bmatrix} 0 \\ \dfrac{C_r\xi}{mI_z} \\ 0 \\ -\dfrac{l_rC_r}{I_z} \end{bmatrix}
$$

### 6.3 扰动矩阵

$$
G = \begin{bmatrix}
0 \\[6pt]
-\dfrac{C_fL\eta}{mI_zv_x} - v_x \\[6pt]
0 \\[6pt]
-\dfrac{l_fC_fL}{I_zv_x}
\end{bmatrix}
$$

---

## 7 推导要点总结

整个推导的关键步骤：

1. **后轴 Frenet 运动学**：$\dot e_1 = v_{yr} + v_x e_2$ 而非 $v_y + v_x e_2$（核心区别）
2. **后轴速度统一表达轮胎力**：$v_y + l_f r = v_{yr} + Lr$，前轮侧偏角中 $r$ 的力臂从 $l_f$ 变为 $L$
3. **$\ddot e_1$ 的力臂分解**：$\dot v_{yr} = \dot v_y - l_r\dot r$，使得 $\ddot e_1$ 中前后轮力的系数分别为 $\eta/(mI_z)$ 和 $\xi/(mI_z)$
4. **$v_x r$ 消去**：$\ddot e_1 = \dot v_{yr} + v_x\dot e_2$ 中 $-v_xr + v_xr = 0$，仅留扰动 $-v_x\dot\theta_{\text{ref}}$
5. **4WS 的 $\delta_r$ 入口**：$\delta_r$ 通过 $F_{yr}$ 进入 $\ddot e_1$（系数 $C_r\xi/(mI_z)$）和 $\ddot e_2$（系数 $-l_rC_r/I_z$）

---

## 8 物理一致性检验

### 8.1 $A_{23} = -v_x \cdot A_{22}$

$$
A_{22} = -\frac{C_f\eta + C_r\xi}{mI_zv_x}, \qquad A_{23} = \frac{C_f\eta + C_r\xi}{mI_z} = -v_x \cdot A_{22}
$$

来源：$\dot e_1$ 和 $e_2$ 的系数来自同一个力（$-\frac{C_f\eta + C_r\xi}{mI_zv_x}(\dot e_1 - v_xe_2)$），因此 $A_{23}/A_{22} = -v_x$。

### 8.2 $A_{43} = -v_x \cdot A_{42}$

$$
A_{42} = -\frac{l_fC_f - l_rC_r}{I_zv_x}, \qquad A_{43} = \frac{l_fC_f - l_rC_r}{I_z} = -v_x \cdot A_{42}
$$

同理，来自 $v_{yr} = \dot e_1 - v_xe_2$ 的结构。

### 8.3 $B_r$ 第 2 元素恒正

$$
B_{r,2} = \frac{C_r\xi}{mI_z} = \frac{C_r(I_z + ml_r^2)}{mI_z} > 0
$$

因为 $\xi = I_z + ml_r^2 > 0$ 恒成立。后轮转角对后轴横向加速度始终有正增益。

---

## 9 使用前提

1. 小角度近似：$\delta_f$、$\delta_r$、$e_2$、$\beta_r$ 均较小
2. 纵向速度近似恒定：$\dot v_x \approx 0$
3. 线性轮胎模型：$F_{yf} = C_f\alpha_f$，$F_{yr} = C_r\alpha_r$，$C_f, C_r > 0$
4. 参考航向变化率缓慢：$\ddot\theta_{\text{ref}} \approx 0$
5. 忽略纵向力对侧向/横摆的影响
6. 后轴投影点与质心投影点处曲率近似相等
