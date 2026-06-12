# 4WS 偏置辨识：后轮随动控制约束下的方案

> 后轮转角由调度表随动控制，满足 $\delta_r^{cmd} = k(v)\cdot\delta_f^{sensor}$（固定速度下为线性关系）。本文分析此约束对偏置辨识的影响，并给出相应的估计与补偿策略。
>
> 前置文档：`07_4ws_steering_offset_identification.md`、`07b_4ws_offset_ekf_dynamic.md`、`08_steering_scheduler_analysis.md`

---

## 1. 后轮随动控制模型

### 1.1 调度表线性关系

在固定速度 $v_x$ 下，后轮期望转角与前轮传感器读数满足线性关系（未饱和区域）：

$$
\delta_r^{cmd} = k(v_x) \cdot \delta_f^{sensor}
$$

其中 $k(v_x)$ 为速度依赖的增益：

$$
k(v_x) = \begin{cases}
-1/6 & v_x \leq 15 \text{ kph} \\
\approx \frac{1}{6}\cdot\frac{v_x - 30}{50} & 15 < v_x < 80 \text{ kph} \\
+1/6 & v_x \geq 80 \text{ kph}
\end{cases}
$$

### 1.2 偏置定义与信号流

**偏置定义（与 07 一致）：** 传感器零位有偏差，传感器读数为零时真实转角不为零：

$$
\delta_f^{real} = \delta_f^{sensor} + \Delta_f
$$

**控制架构：** 控制器基于传感器反馈闭环，目标是让传感器读数等于指令值 $\delta_f^{sensor} \to \delta_f^{cmd}$。

**后轮随动：** ECU 读取前轮传感器值，乘以增益，作为后轮指令。后轮执行机构存在偏置 $\Delta_r$：

$$
\delta_r^{real} = k(v_x)\cdot\delta_f^{sensor} + \Delta_r
$$

---

## 2. 运动学分析

### 2.1 航向角变化率

$$
\dot\psi = \frac{v_x}{L}\left[(1-k)\delta_f^{sensor} + \Delta_f - \Delta_r\right]
$$

### 2.2 侧向速度（稳态）

$$
v_y = \frac{v_x}{L}\left[(l_r + l_f k)\delta_f^{sensor} + l_r\Delta_f + l_f\Delta_r\right]
$$

### 2.3 可观测性

| 观测通道 | 偏置组合 |
| -------- | -------- |
| $\dot\psi$ | $\Delta_f - \Delta_r$ |
| $v_y$ | $l_r\Delta_f + l_f\Delta_r$ |

**随动控制不改变可观测性结构。**

---

## 3. 核心结论

在传感器闭环 + 后轮随动的架构下：

1. 偏置对运动学的影响与转角工况无关（线性叠加）
2. 随动控制不引入额外的偏置耦合
3. 07a（差值辨识）和 07b（独立辨识）的 EKF 模型均可直接使用，无需修改

**验证（直线行驶，$\delta_f^{cmd}=0$）：**
- 传感器闭环保证 $\delta_f^{sensor} = 0$
- 前轮真实转角 = $\Delta_f$，后轮真实转角 = $\Delta_r$
- 横摆残差 = $\frac{v_x}{L}(\Delta_f - \Delta_r)$，侧向速度残差 = $\frac{v_x}{L}(l_r\Delta_f + l_f\Delta_r)$

**验证（非零转角，$\delta_f^{cmd} \neq 0$）：**
- 传感器读数 $\delta_f^{sensor} = \delta_f^{cmd}$，后轮指令 $\delta_r^{cmd} = k\cdot\delta_f^{cmd}$
- 横摆残差仍为 $\frac{v_x}{L}(\Delta_f - \Delta_r)$，与转角大小无关

---

## 4. 补偿策略

### 4.1 补偿点选择

```
方案 1（推荐）：补偿传感器读数
  δ_f^sensor → [+Δ_f] → δ_f^corrected → 调度表 / 控制器
  δ_r^cmd → [-Δ_r] → 后轮执行

方案 2：分别补偿指令
  δ_f^cmd → [-Δ_f] → 前轮执行
  δ_r^cmd → [-Δ_r] → 后轮执行

方案 3（仅差值）：补偿前轮指令
  δ_f^cmd → [-Δ_diff] → 前轮执行
```

### 4.2 方案对比

| 方案 | 前提 | 效果 | 调度表是否受益 |
| ---- | ---- | ---- | -------------- |
| 1. 传感器端 | 独立辨识 $\Delta_f$, $\Delta_r$ | 完全消除 | 是 |
| 2. 指令端 | 独立辨识 $\Delta_f$, $\Delta_r$ | 消除执行偏差，调度表仍有误差 | 否 |
| 3. 仅差值 | 仅需 $\Delta_{diff}$ | 消除横摆偏差，侧偏角有残差 | 否 |

### 4.3 饱和区域

后轮触及限位（±6°）时线性关系不成立。07b 的伪观测使用实际 $\delta_r^{cmd}$ 值（非 $k\cdot\delta_f$），无需特殊处理。

---

## 5. 推荐分层策略

| 层级 | 方案 | 条件 | 辨识目标 | 补偿方式 |
| ---- | ---- | ---- | -------- | -------- |
| 基础层 | 07a | GPS 航向角 | $\Delta_{diff}$ | 前轮指令 $-\Delta_{diff}$ |
| 增强层 | 07b | GPS 航向角 + 侧向速度 | $\Delta_f$, $\Delta_r$ | 传感器端 $+\Delta_f$，后轮 $-\Delta_r$ |

### EKF 输入处理

EKF 运行时使用**未补偿**的原始传感器值，避免偏置被双重计算。补偿仅施加在 EKF 之外的控制通路上。
