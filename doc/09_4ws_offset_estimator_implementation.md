# 4WS Steering Offset Estimator 实现详解

> 本文档详细描述 `SteerOffsetEstimator4WS` 的算法流程、参数选择依据，以及与原 2WS `SteerOffsetEstimator` 的差异对比。
>
> 代码路径：`control/identification/steer_offset_estimator/steer_offset_estimator_4ws.h/cc`

---

## 1. 算法概览

### 1.1 目标

独立辨识前轮偏置 $\Delta_f$ 和后轮偏置 $\Delta_r$（单位：rad）。

- 2WS 只能辨识 $\Delta_f$（航向角变化率仅依赖前轮偏置）
- 4WS 通过引入 $v_y$ 观测和运动学伪观测，解耦两个偏置

### 1.2 状态向量

$$
x = \begin{bmatrix} \psi \\ v_y \\ \Delta_f \\ \Delta_r \end{bmatrix}
$$

| 状态 | 含义 | 单位 |
|------|------|------|
| $\psi$ | 航向角 | rad |
| $v_y$ | 侧向速度 | m/s |
| $\Delta_f$ | 前轮转角偏置 | rad |
| $\Delta_r$ | 后轮转角偏置 | rad |

---

## 2. 算法流程

### 2.1 整体流程图

```
┌──────────────────────────────────────────────────┐
│                     Run()                         │
├──────────────────────────────────────────────────┤
│  1. CheckStraightCondition()                     │
│  2. FSM 状态转移 (kInvalid→kInit→kRunning→kReset)│
│  3. if kRunning:                                 │
│       a. InitKalmanFilter() (首次)               │
│       b. CheckPseudoObservationCondition()       │
│       c. PredictAndCorrect()                     │
│       d. PostProcess() → 收敛判定                │
│  4. if kReset (from kRunning):                   │
│       UpdateStandardDeviation() (Welford)        │
│  5. return {mean_front_offset, mean_rear_offset} │
└──────────────────────────────────────────────────┘
```

### 2.2 状态机 (FSM)

```
kInvalid ──(valid_count >= limit)──► kInit
kInit ──(invalid < limit)──► kRunning
kRunning ──(invalid >= limit OR low accuracy)──► kReset
kReset ──(always)──► kInvalid
```

状态机与 2WS 完全相同，保证 EKF 仅在直行高精度定位条件下运行。

### 2.3 预测步骤 (Predict)

离散状态转移：

$$
\hat{x}_{k+1|k} = F \hat{x}_{k|k} + B u_k
$$

其中：

$$
F = \begin{bmatrix}
1 & 0 & \frac{v_x}{L}\Delta t & -\frac{v_x}{L}\Delta t \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
B = \begin{bmatrix}
\frac{v_x}{L}\Delta t & -\frac{v_x}{L}\Delta t \\
0 & 0 \\
0 & 0 \\
0 & 0
\end{bmatrix}, \quad
u = \begin{bmatrix} \delta_f^{sensor} \\ \delta_r^{cmd} \end{bmatrix}
$$

**物理含义**：
- 航向角变化 = 前后轮转角差 × (v/L) × dt
- $v_y$、$\Delta_f$、$\Delta_r$ 建模为随机游走（常值 + 过程噪声）

### 2.4 观测步骤 (Correct)

观测向量：

$$
z = \begin{bmatrix} \psi_{GPS} \\ v_{y,GPS} \\ z_3 \end{bmatrix}
$$

观测矩阵：

$$
H = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 1 & -\frac{v_x l_r}{L} & -\frac{v_x l_f}{L}
\end{bmatrix}
$$

**第 3 行（伪观测）的含义**：

运动学稳态约束：$v_y = \frac{v_x}{L}[l_r(\delta_f + \Delta_f) + l_f(\delta_r + \Delta_r)]$

整理为：$v_y - \frac{v_x l_r}{L}\Delta_f - \frac{v_x l_f}{L}\Delta_r = \frac{v_x}{L}(l_r \delta_f + l_f \delta_r) = z_3$

此伪观测将已知输入构造为"测量值"，约束 $\Delta_f$ 和 $\Delta_r$ 的加权和。

### 2.5 伪观测自适应

伪观测仅在**稳态**条件下有效：

```cpp
bool CheckPseudoObservationCondition():
  - |steer_rate| < 0.5 deg/s
  - |yaw_rate_filtered| < 0.005 rad/s
```

非稳态时，将 $R(2,2)$ 设为 $10^6$（等效禁用），避免瞬态误差污染估计。

### 2.6 可观测性保证

| 观测通道 | 提供的偏置信息 |
|---------|--------------|
| $\psi$ 观测 + F 传播 | $\Delta_f - \Delta_r$（差值） |
| 伪观测 $H_3$ | $l_r \Delta_f + l_f \Delta_r$（加权和） |

系数矩阵行列式 = $l_f + l_r = L \neq 0$，两个偏置完全可观。

### 2.7 收敛判定与后处理

1. **MeanFilter 滑窗**：前后轮 offset 各维护一个滑窗
2. **收敛条件**：滑窗满 且 `max - min < 0.002 deg × window_size`（rad 等效）
3. **输出 Clamp**：收敛后 offset 限幅 ±5°（≈±0.087 rad）
4. **Welford 跨 session 统计**：每次 kReset 时（运行 >1s 且已收敛），累积均值和方差

---

## 3. 参数选择

### 3.0 2WS 实际配置参数（基准参照）

2WS `SteerOffsetEstimator` 使用 `steer_offset_estimator_conf` 中的参数，配置位于 `conf/control/control_conf_base.pb.txt`：

**状态向量**：`x = [ψ, -Δf]`（2维），观测 `z = [ψ_GPS]`（1维），控制 `u = [δf_tire]`（1维）

| 矩阵 | 索引 | 配置值 | 对应物理量 | 量级含义 |
|-------|------|--------|-----------|---------|
| P(0,0) | `matrix_p[0]` | 5×10⁻⁵ | ψ 初始方差 | ~0.007 rad (0.4°) 不确定性 |
| P(1,1) | `matrix_p[1]` | 2×10⁻⁵ | Δf 初始方差 | ~0.004 rad (0.25°) 不确定性 |
| Q(0,0) | `matrix_q[0]` | 1×10⁻⁴ | ψ 过程噪声 | 模型误差 ~0.01 rad/step |
| Q(1,1) | `matrix_q[1]` | 2×10⁻⁵ | Δf 过程噪声 | 偏置缓变 ~0.004 rad/step |
| R(0,0) | `matrix_r[0]` | 5×10⁻⁵ | ψ_GPS 观测噪声 | ~0.007 rad 测量精度 |

**其他配置**：

| 参数 | 值 | 含义 |
|------|-----|------|
| `straight_valid_times_limit` | 50 | 需连续 50 帧有效才进入 kInit（50×20ms=1s） |
| `straight_invalid_times_limit` | 15 | 连续 15 帧无效则退出（15×20ms=0.3s） |
| `enable_means` | true | 启用滑窗均值滤波 |
| `means_window_time` | 1.0 s | 收敛判定滑窗时长 |
| `period_time` | 0.02 s | 控制周期（50 Hz） |
| `use_steer_offset_estimated` | true | 启用在线估计值 |

**注意**：2WS 的 Q 值比 4WS 大 8 个数量级（10⁻⁴ vs 10⁻¹²），这是因为 2WS 将偏置存储为取负值 `-Δf`，模型结构不同导致收敛速度/稳定性权衡不同。

### 3.1 4WS 初始协方差 P0

| 状态 | P0 值 | 依据 |
|------|-------|------|
| $\psi$ | 5×10⁻⁵ | GPS 航向角精度 ~0.007 rad (0.4°) |
| $v_y$ | 0.01 | 侧向速度初始不确定性 ~0.1 m/s |
| $\Delta_f$ | 2×10⁻⁵ | 偏置先验不确定性 ~0.005 rad (0.25°) |
| $\Delta_r$ | 2×10⁻⁵ | 同上 |

### 3.2 过程噪声 Q

| 状态 | Q 值 | 2WS 参考 | 依据 |
|------|------|---------|------|
| $\psi$ | 1×10⁻⁴ | 1×10⁻⁴ | 与 2WS 对齐，同一运动学模型 |
| $v_y$ | 2.5×10⁻³ | — | 侧向速度受轮胎力/道路扰动影响，变化快 |
| $\Delta_f$ | 1×10⁻⁶ | 2×10⁻⁵ | 比 2WS 小 20 倍：4WS 有 3 个观测通道提供更多信息 |
| $\Delta_r$ | 1×10⁻⁶ | — | 同 $\Delta_f$ |

**设计思路**：Q(offset) 比 2WS 小但不极端 → 偏置通过观测修正为主，过程模型允许缓慢适应；Q(ψ) 与 2WS 一致保证航向预测精度不退化。

### 3.3 观测噪声 R

| 观测 | R 值 | 2WS 参考 | 依据 |
|------|------|---------|------|
| $\psi_{GPS}$ | 5×10⁻⁵ | 5×10⁻⁵ | 与 2WS 对齐，同一 GPS 传感器 |
| $v_{y,GPS}$ | 0.01 | — | GPS 侧向速度精度 ~0.1 m/s |
| $z_3$ (稳态) | 2.5×10⁻³ | — | 模型误差 + 轮胎侧偏角稳态偏差 |
| $z_3$ (非稳态) | 10⁶ | — | 等效禁用 |

### 3.4 工况条件阈值

| 条件 | 阈值 | 依据 |
|------|------|------|
| 车速 | 30~120 km/h | 低速运动学不准；高速 GPS 可靠 |
| 纵向加速度 | |a_x| ≤ 0.3 m/s² | 排除制动/加速工况 |
| 横摆角速度 | |r| < 0.005 rad/s | 排除转弯工况 |
| 横侧倾角 | |roll| < 0.02 rad | 排除横坡导致的侧向力偏差 |
| 转向速率 | |dδ/dt| < 1.0 deg/s | 排除主动转向工况 |
| GPS 精度 | RTK 固定解 | high_local_accuracy = true |

### 3.5 伪观测启用条件（更严格）

| 条件 | 阈值 |
|------|------|
| 转向速率 | |dδ/dt| < 0.5 deg/s |
| 横摆角速度 | |r| < 0.005 rad/s |

---

## 4. 与 2WS Estimator 的对比

### 4.1 对外接口对比（单位统一为 deg）

两个 estimator 对外接口均以 **deg** 为单位，与下游模块保持一致。内部计算各自用 rad。

#### 2WS `SteerOffsetEstimator`

```cpp
// 入口
double Run(double steer_offset_init_deg,          // 上次估计值或持久化初始值 (deg)
           const ControlDependency& ctrl_input,
           SteerOffsetEstimatorDebug* debug);
// 返回值: mean_steer_offset_estimated_ (deg)
```

| 接口项 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| 输入 `steer_offset_init_deg` | double | deg | 冷启动初始值 / 上一帧输出 |
| 输出（返回值） | double | deg | 跨 session Welford 均值 |
| `identify.steer_offset` | double | deg | 存入 `SystemIdentifyOut`，下游使用 |

**下游消费方式**：
```cpp
// identify_manager.cc — deg + deg，传入 StrAngToDeltaAngle 统一转 rad
ctrl_input.chassis.delta = ControlConfig().StrAngToDeltaAngle(
    ctrl_input.chassis.str_angle + ctrl_input.identify.steer_offset);
```

#### 4WS `SteerOffsetEstimator4WS`

```cpp
// 入口
struct Result { double front_offset_deg; double rear_offset_deg; };

Result Run(double front_offset_init_deg,          // 前轮初始值 (deg)
           double rear_offset_init_deg,           // 后轮初始值 (deg)
           const ControlDependency& ctrl_input,
           SteerOffsetEstimatorDebug* debug);
```

| 接口项 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| 输入 `front_offset_init_deg` | double | deg | 前轮冷启动初始值 |
| 输入 `rear_offset_init_deg` | double | deg | 后轮冷启动初始值 |
| 输出 `result.front_offset_deg` | double | deg | 前轮跨 session 均值 |
| 输出 `result.rear_offset_deg` | double | deg | 后轮跨 session 均值 |
| `identify.steer_offset` | double | deg | 前轮，存入 `SystemIdentifyOut` |
| `identify.rear_steer_offset` | double | deg | 后轮，存入 `SystemIdentifyOut` |

**下游消费方式**：
```cpp
// 前轮：与 2WS 完全一致
ctrl_input.chassis.delta = ControlConfig().StrAngToDeltaAngle(
    ctrl_input.chassis.str_angle + ctrl_input.identify.steer_offset);

// 后轮：deg → rad 后补偿到 delta_r
ctrl_input.chassis.delta_r += ctrl_input.identify.rear_steer_offset * M_PI / 180.0;
```

#### 单位转换边界

```
┌─────────────────────────────────┐
│      对外接口 (deg)              │
│  Run(init_deg) → Result(deg)    │
│  identify.steer_offset (deg)    │
│  identify.rear_steer_offset(deg)│
├─────────────────────────────────┤
│      内部 EKF (rad)             │
│  状态 x = [ψ, vy, Δf, Δr] rad  │
│  F/B/H 矩阵全部 rad             │
│  PostProcess: clamp ±0.087 rad  │
│  Welford: rad                   │
├─────────────────────────────────┤
│      边界转换                    │
│  入口: init_deg * π/180 → rad   │
│  出口: mean_rad * 180/π → deg   │
└─────────────────────────────────┘
```

### 4.2 架构对比

| 维度 | 2WS (`SteerOffsetEstimator`) | 4WS (`SteerOffsetEstimator4WS`) |
|------|-----|-----|
| **KF 模板** | `KalmanFilter<double, 2, 1, 1>` | `KalmanFilter<double, 4, 3, 2>` |
| **状态** | [ψ, -Δf] | [ψ, vy, Δf, Δr] |
| **控制输入** | [δf_tire] | [δf_sensor, δr_cmd] |
| **观测** | [ψ_GPS] | [ψ_GPS, vy_GPS, z3_pseudo] |
| **输出** | Δf (deg) | Δf, Δr (均为 deg) |
| **偏置存储** | 取负值 `-Δf` | 直接存储 Δf, Δr |

### 4.2 模型差异

| 方面 | 2WS | 4WS |
|------|-----|-----|
| **状态转移** | ψ_new = ψ + (v/L)·dt·(-Δf) + (v/L)·dt·δf | ψ_new = ψ + (v/L)·dt·(Δf - Δr) + (v/L)·dt·(δf - δr) |
| **vy 建模** | 无 | 随机游走，受伪观测约束 |
| **后轮** | 不考虑 | δr_cmd 作为输入，Δr 作为状态 |
| **可观测性** | 仅 Δf（或严格说 Δ_diff） | Δf 和 Δr 独立可观 |

### 4.3 信号需求差异

| 信号 | 2WS | 4WS |
|------|-----|-----|
| GPS 航向角 | ✅ 必需 | ✅ 必需 |
| GPS 侧向速度 vy | ❌ 不需要 | ✅ 必需 |
| 后轮转角 δr_cmd | ❌ 不需要 | ✅ 必需（查表值） |
| 前后轴距 lf, lr | ❌ 不需要 | ✅ 必需 |

### 4.4 工况条件差异

| 条件 | 2WS 阈值 | 4WS 阈值 | 说明 |
|------|---------|---------|------|
| 车速下限 | 30 km/h | 30 km/h | 相同 |
| 车速上限 | 105 km/h | 120 km/h | 4WS 放宽，高速 vy 更可靠 |
| 纵向加速度 | ≤ 0.2 m/s² | ≤ 0.3 m/s² | 4WS 略放宽 |
| 横摆角速度 | < 0.00035·speed | < 0.005 rad/s | 4WS 用绝对阈值 |
| 转向速率 | < 1.0 | < 1.0 (straight) / < 0.5 (pseudo) | 4WS 对伪观测更严格 |

### 4.5 收敛特性差异

| 指标 | 2WS | 4WS |
|------|-----|-----|
| 典型收敛时间 | 5~15 s | 10~20 s |
| 收敛判定 | 单窗口 max-min | 双窗口（前后各一个） |
| 跨 session 统计 | Welford 单变量 | Welford 双变量 |
| 输出限幅 | ±5° | ±5°（前后各自限幅） |

### 4.7 补偿逻辑差异

```cpp
// 2WS 补偿：
ctrl_input.chassis.delta = StrAngToDeltaAngle(str_angle + steer_offset);
// steer_offset 在 deg，str_angle 在 deg → deg + deg

// 4WS 补偿（新增后轮）：
ctrl_input.chassis.delta_r += rear_steer_offset * M_PI / 180.0;  // deg → rad
```

---

## 5. 代码结构

### 5.1 文件清单

| 文件 | 作用 |
|------|------|
| `steer_offset_estimator_4ws.h` | 类声明、Result 结构体 |
| `steer_offset_estimator_4ws.cc` | EKF 核心实现 |
| `identify_manager.cc` | 集成：选择 2WS/4WS，执行补偿 |
| `control_dependency.h` | `SystemIdentifyOut` 新增 `rear_steer_offset` |

### 5.2 关键方法

| 方法 | 职责 |
|------|------|
| `Run()` | 入口：FSM + EKF + 后处理 |
| `InitKalmanFilter()` | 初始化状态、P0、Q、R、滑窗 |
| `PredictAndCorrect()` | 构建时变 F/B/H，执行预测和修正 |
| `PostProcess()` | 收敛检测、限幅 |
| `UpdateStandardDeviation()` | Welford 跨 session 均值 |
| `CheckStraightCondition()` | 直行工况门控 |
| `CheckPseudoObservationCondition()` | 伪观测稳态门控 |

### 5.3 接口

```cpp
struct Result {
  double front_offset_deg;
  double rear_offset_deg;
};

Result Run(double front_offset_init_deg, double rear_offset_init_deg,
           const ControlDependency& ctrl_input,
           SteerOffsetEstimatorDebug* debug);
```

---

## 6. 已知限制与后续改进

1. **vy 精度依赖**：GPS 侧向速度在低速/遮挡时精度下降，可考虑引入轮速模型作为备份
2. **伪观测仅限稳态**：瞬态工况下系统退化为仅辨识差值 $\Delta_f - \Delta_r$，收敛更慢
3. **后轮角来自查表**：当前 `delta_r` 非底盘反馈，如果查表本身有误差会引入系统偏差
4. **proto 配置**：当前复用 2WS 的 `steer_offset_estimator_conf`，P/Q/R 矩阵维度映射为 hardcode，后续可扩展专用 proto
5. **rear_steer_offset 持久化**：尚未在 `hnoa_control_component.cc` 中实现关机持久化和冷启动加载
