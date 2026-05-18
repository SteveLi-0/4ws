# 4WS 转向偏置（offset）辨识与补偿

> 本文讨论 4WS 车辆前后轮转向中位偏置的辨识问题。符号约定与 01\_bicycle\_model.md 一致。

---

## 1. 问题背景

在 2WS 场景中，前轮转角存在机械中位偏差，可通过直线行驶时的横摆角速度残差辨识出 offset 并进行补偿。当车辆为 4WS 时，前后轮各自存在转向偏置：

$$
\delta_f^{real} = \delta_f^{cmd} + \Delta_f
$$

$$
\delta_r^{real} = \delta_r^{cmd} + \Delta_r
$$

其中 $\Delta_f$ 和 $\Delta_r$ 分别为前轮和后轮的转向中位偏置。

---

## 2. 偏置在运动学中的耦合关系

将含偏置的真实转角代入小角度运动学关系：

**横摆角速度：**

$$
r \approx \frac{v_x}{L}\left[(\delta_f^{cmd} - \delta_r^{cmd}) + (\Delta_f - \Delta_r)\right]
$$

**质心侧偏角：**

$$
\beta \approx \frac{l_r(\delta_f^{cmd} + \Delta_f) + l_f(\delta_r^{cmd} + \Delta_r)}{L}
$$

由此可见：

* 横摆角速度 $r$ 仅能观测到偏置差值 $\Delta_f - \Delta_r$；
* 质心侧偏角 $\beta$ 能观测到偏置加权和 $l_r\Delta_f + l_f\Delta_r$。

若同时有 $r$ 和 $\beta$ 的观测，可联立求解：

$$
\Delta_f - \Delta_r = \frac{L \cdot r_{residual}}{v_x}
$$

$$
l_r\Delta_f + l_f\Delta_r = L \cdot \beta_{residual}
$$

解为：

$$
\Delta_f = \frac{l_f \cdot \frac{L \cdot r_{residual}}{v_x} + L \cdot \beta_{residual}}{L}
$$

$$
\Delta_r = \Delta_f - \frac{L \cdot r_{residual}}{v_x}
$$

但实际中 $\beta$ 难以直接测量，因此仅依赖横摆角速度无法将 $\Delta_f$ 和 $\Delta_r$ 分别辨识。

---

## 3. 可观测性分析

在直线行驶工况（$\delta_f^{cmd}=0$，$\delta_r^{cmd}=0$）下，可用观测量与偏置的关系如下：

| 观测量 | 可辨识的偏置组合 | 能否分别辨识 |
| ------ | ---------------- | ------------ |
| 横摆角速度 $r$ | $\Delta_f - \Delta_r$ | 否 |
| 质心侧偏角 $\beta$ | $l_r\Delta_f + l_f\Delta_r$ | 否 |
| $r$ 与 $\beta$ 联合 | $\Delta_f$ 和 $\Delta_r$ | 是 |
| 侧向位移漂移 | $l_r\Delta_f + l_f\Delta_r$（间接） | 否 |

**结论：仅凭横摆角速度一个观测量，只能辨识差值 $\Delta_f - \Delta_r$，无法分别得到两个偏置。**

---

## 4. 辨识方案

### 4.1 方案 A：差值辨识 + 统一补偿（推荐，工程最简）

仅辨识偏置差值 $\Delta_{diff} = \Delta_f - \Delta_r$，将补偿量全部施加在前轮或按比例分配：

1. 直线行驶工况下（$\delta_f^{cmd}=0$，$\delta_r^{cmd}=0$），测量横摆角速度残差 $r_{residual}$；
2. 计算偏置差值：

$$
\Delta_{diff} = \frac{L \cdot r_{residual}}{v_x}
$$

3. 补偿策略（以下二选一）：
   * 全部补偿在前轮：$\delta_f^{cmd} \leftarrow \delta_f^{cmd} - \Delta_{diff}$
   * 前后各半：$\delta_f^{cmd} \leftarrow \delta_f^{cmd} - \frac{\Delta_{diff}}{2}$，$\delta_r^{cmd} \leftarrow \delta_r^{cmd} + \frac{\Delta_{diff}}{2}$

该方案能消除直线行驶时的横摆偏差，但可能在质心侧偏角上留有少量残差。

---

### 4.2 方案 B：分阶段单轴辨识

利用不同工况下的约束解耦前后偏置：

**阶段 1**——锁定后轮（$\delta_r^{cmd}=0$），直线行驶：

$$
r_1 = \frac{v_x}{L}(\Delta_f - \Delta_r)
$$

**阶段 2**——锁定前轮（$\delta_f^{cmd}=0$），给后轮一个已知的非零角度 $\delta_r^{cmd}=\delta_0$，观察实际曲率是否与预期一致：

$$
r_2 = \frac{v_x}{L}(\Delta_f - \delta_0 - \Delta_r)
$$

两式相减：

$$
r_1 - r_2 = \frac{v_x}{L}\delta_0
$$

此式为恒等关系，不提供新的偏置信息。**说明仅改变输入工况但只用 $r$ 观测，仍无法解耦。**

---

### 4.3 方案 C：扩展卡尔曼滤波联合辨识（工程最实用）

将 $\Delta_f$ 和 $\Delta_r$ 作为缓变状态扩展到状态向量中：

$$
x_{aug} = \begin{bmatrix} v_y \\ r \\ \Delta_f \\ \Delta_r \end{bmatrix}
$$

观测量包括：

* IMU 提供的横摆角速度 $r$；
* GPS 提供的侧向速度或航向角变化率（间接反映 $\beta$）；
* 侧向加速度 $a_y = \dot v_y + v_x r$。

通过多传感器融合，在丰富工况（转向、直行交替）下可逐步收敛 $\Delta_f$ 和 $\Delta_r$。

---

### 4.4 方案 D：机械零位标定

在前后转向机构上安装绝对位置传感器，直接物理测量零位偏差。精度最高，但需要额外硬件支持。

---

## 5. 方案对比

| 方案 | 能否分别辨识 | 所需传感器 | 工程复杂度 | 适用场景 |
| ---- | ------------ | ---------- | ---------- | -------- |
| A. 差值辨识 + 统一补偿 | 否（仅差值） | IMU | 低 | 对侧偏角精度要求不高 |
| B. 分阶段单轴辨识 | 否（仍耦合） | IMU | 低 | 不推荐，无法解耦 |
| C. EKF 联合辨识 | 是 | IMU + GPS | 中 | 需要精确分别补偿 |
| D. 机械零位标定 | 是 | 绝对位置传感器 | 硬件改动 | 精度要求最高 |
