# 4WS 前后轮偏置独立辨识：基于运动学模型的 EKF

> 本文基于 4WS 自行车运动学模型，构建 EKF 同时辨识前轮偏置 $\Delta_f$ 和后轮偏置 $\Delta_r$。
>
> 前置文档：`01_bicycle_model.md`、`07_4ws_steering_offset_identification.md`、`07a_4ws_offset_estimation_detail.md`

---

## 1. 动机

07a 仅能辨识 $\Delta_{diff} = \Delta_f - \Delta_r$（航向角变化率仅依赖差值）。要独立辨识需引入第二个观测通道：

- $\dot\psi$ 依赖 $\Delta_f - \Delta_r$（差值）
- $v_y$ 依赖 $l_r\Delta_f + l_f\Delta_r$（加权和）

两者线性无关，可解耦。

---

## 2. 系统模型

### 2.1 状态向量

$$
x = \begin{bmatrix} \psi \\ v_y \\ \Delta_f \\ \Delta_r \end{bmatrix}
$$

### 2.2 含偏置的运动学

$$
\delta_f^{real} = \delta_f^{sensor} + \Delta_f, \quad \delta_r^{real} = \delta_r^{cmd} + \Delta_r
$$

$$
\dot\psi = \frac{v_x}{L}(\delta_f^{sensor} + \Delta_f - \delta_r^{cmd} - \Delta_r)
$$

$$
v_y = \frac{v_x}{L}\left[l_r(\delta_f^{sensor} + \Delta_f) + l_f(\delta_r^{cmd} + \Delta_r)\right] \quad (\text{稳态})
$$

### 2.3 状态方程

$$
\dot\psi = \frac{v_x}{L}(\delta_f^{sensor} - \delta_r^{cmd}) + \frac{v_x}{L}\Delta_f - \frac{v_x}{L}\Delta_r
$$

$$
\dot v_y = 0, \quad \dot\Delta_f = 0, \quad \dot\Delta_r = 0
$$

### 2.4 离散化

$$
F = \begin{bmatrix}
1 & 0 & \frac{v_x}{L}\Delta t & -\frac{v_x}{L}\Delta t \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}, \quad
G = \begin{bmatrix}
\frac{v_x}{L}\Delta t & -\frac{v_x}{L}\Delta t \\
0 & 0 \\
0 & 0 \\
0 & 0
\end{bmatrix}
$$

$$
\hat x_{k+1|k} = F \hat x_{k|k} + G u_k, \quad u = [\delta_f^{sensor},\ \delta_r^{cmd}]^T
$$

---

## 3. 观测模型（组合观测矩阵）

观测向量：

$$
z = \begin{bmatrix} \psi_{meas} \\ v_{y,meas} \\ z_3 \end{bmatrix}
$$

其中：
- $\psi_{meas}$：GPS 航向角
- $v_{y,meas}$：GPS 侧向速度（$= V_{GPS}\sin(\chi_{GPS} - \psi)$）
- $z_3 = \frac{v_x}{L}(l_r \delta_f^{sensor} + l_f \delta_r^{cmd})$：运动学伪观测（由输入计算的已知量）

组合观测矩阵：

$$
H = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 1 & -\frac{v_x l_r}{L} & -\frac{v_x l_f}{L}
\end{bmatrix}
$$

**第 3 行的含义：** 运动学稳态约束 $v_y = \frac{v_x}{L}[l_r(\delta_f^{sensor}+\Delta_f) + l_f(\delta_r^{cmd}+\Delta_r)]$ 整理为 $v_y - \frac{v_x l_r}{L}\Delta_f - \frac{v_x l_f}{L}\Delta_r = z_3$，即 $H_3 x = z_3$。

---

## 4. 可观测性

偏置子空间的信息来自两个独立通道：

| 来源 | 提供的偏置组合 |
| ---- | -------------- |
| $\psi$ 观测 + F 传播 | $\Delta_f - \Delta_r$ |
| $H_3$（伪观测） | $l_r\Delta_f + l_f\Delta_r$ |

系数矩阵：

$$
\begin{bmatrix} 1 & -1 \\ l_r & l_f \end{bmatrix}, \quad \det = l_f + l_r = L \neq 0
$$

**系统完全可观（$v_x \neq 0$）。**

---

## 5. 算法流程

**预测：**

$$
\hat x_{k+1|k} = F \hat x_{k|k} + G u_k, \quad P_{k+1|k} = F P_{k|k} F^T + Q
$$

**更新：**

$$
z_3 = \frac{v_x}{L}(l_r \delta_f^{sensor} + l_f \delta_r^{cmd})
$$

$$
\tilde y = z - H \hat x_{k+1|k} \quad (\psi\text{ 分量角度归一化})
$$

$$
S = H P_{k+1|k} H^T + R, \quad K = P_{k+1|k} H^T S^{-1}
$$

$$
\hat x_{k+1|k+1} = \hat x_{k+1|k} + K \tilde y, \quad P_{k+1|k+1} = (I - KH) P_{k+1|k}
$$

---

## 6. 参数

### 过程噪声

$$
Q = \text{diag}\left((0.001)^2,\ (0.05)^2,\ (10^{-6})^2,\ (10^{-6})^2\right)
$$

### 观测噪声

$$
R = \text{diag}\left((0.005)^2,\ (0.1)^2,\ r_{pseudo}\right)
$$

$r_{pseudo}$：稳态时取 $(0.05)^2$，非稳态时增大（×100）或禁用第 3 行。

### 自适应启用条件

伪观测仅在 $|\dot\delta_f| < 0.5°/s$ 且 $|r| < 0.005$ rad/s 时启用。

---

## 7. 工况条件

| 条件 | 阈值 |
| ---- | ---- |
| 车速 | 30~120 kph |
| 纵向加速度 | \|a_x\| ≤ 0.3 m/s² |
| 横摆角速度 | \|r\| < 0.005 rad/s |
| GPS 精度 | RTK 固定解 |
| 横向坡度 | < 1% |

---

## 8. 与 07a 的对比

| | 07a | 07b |
|---|---|---|
| 观测 | GPS 航向角 | GPS 航向角 + $v_y$ + 伪观测 |
| 辨识目标 | $\Delta_{diff}$ | $\Delta_f$、$\Delta_r$ 独立 |
| 收敛时间 | 5~15 s | 10~20 s |
| GPS 要求 | 航向角 | 航向角 + 速度方向角 |
| 补偿效果 | 消除横摆偏差 | 同时消除横摆 + 侧偏角偏差 |

---

## 9. 补偿

$$
\delta_f^{comp} = \delta_f^{cmd} - \Delta_f, \quad \delta_r^{comp} = \delta_r^{cmd} - \Delta_r
$$
