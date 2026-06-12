# 4WS 中位偏置辨识方案细化

> 前置文档：`07_4ws_steering_offset_identification.md`、`08_steering_scheduler_analysis.md`
>
> 参考实现：`ref_code/steer_offset_estimator.{h,cc}`（2WS 版本）

---

## Part I：现有 2WS 方案分析

---

### 1. 方案概述

现有 `SteerOffsetEstimator` 采用**航向角卡尔曼滤波**方法辨识前轮转向中位偏置。核心思想：

- 利用自行车运动学模型，以前轮转角传感器读数为输入预测航向角变化
- 以 GPS 航向角为观测量进行校正
- 若传感器存在中位偏差，模型预测与观测之间会产生持续性残差
- KF 将该残差归因于偏置状态，从而辨识出 offset

### 2. 数学模型

#### 2.1 状态向量

$$
x = \begin{bmatrix} \psi \\ \Delta_f \end{bmatrix}
$$

- $\psi$：航向角（rad）
- $\Delta_f$：前轮转向中位偏置（rad，轮胎转角域）

**注意：** 代码中初始化为 `X0 << heading_rad, -steer_offset_init`，即状态中存储的是偏置的负值。

#### 2.2 状态转移方程

连续域：

$$
\dot\psi = \frac{v_x}{L}(\delta_f^{sensor} + \Delta_f)
$$

$$
\dot\Delta_f = 0 \quad (\text{随机游走模型})
$$

离散化（前向欧拉）：

$$
\begin{bmatrix} \psi_{k+1} \\ \Delta_{f,k+1} \end{bmatrix}
= \underbrace{\begin{bmatrix} 1 & \frac{v_x}{L}\Delta t \\ 0 & 1 \end{bmatrix}}_{A}
\begin{bmatrix} \psi_k \\ \Delta_{f,k} \end{bmatrix}
+ \underbrace{\begin{bmatrix} \frac{v_x}{L}\Delta t \\ 0 \end{bmatrix}}_{B}
\delta_{f,k}^{sensor}
$$

对应代码 `GetResult`：
```cpp
A << 1.0, v / L * dt,
     0,   1.0;
B << v / L * dt,
     0;
U << tire_steer_angle_rad;
```

#### 2.3 观测方程

$$
z_k = \begin{bmatrix} 1 & 0 \end{bmatrix} x_k = \psi_k
$$

观测量为 GPS 提供的航向角 `ctrl_input.local.head_angle`。

对应代码：
```cpp
H << 1.0, 0;
Z << heading_rad;
```

#### 2.4 噪声矩阵

- **过程噪声 Q**：对角矩阵，`Q(0,0)` 为航向角过程噪声，`Q(1,1)` 为偏置漂移噪声
- **观测噪声 R**：标量，GPS 航向角测量噪声
- **初始协方差 P0**：对角矩阵，反映初始不确定性

参数均由 protobuf 配置文件 `SteerOffsetEstimatorConf` 提供。

#### 2.5 角度归一化

KF 通过 `angle_state_positions_ = {0}` 和 `angle_measure_positions_ = {0}` 标记航向角状态和观测为角度量，在校正步骤中进行角度归一化（`CorrectNormalized`），避免 ±π 跳变问题。

### 3. 状态机设计

```
┌──────────┐   valid_count达标 且    ┌──────┐
│ kInvalid │ ─── invalid_count未超限 ──→ │ kInit │
│          │ ←── low_local_accuracy ──── │      │
└──────────┘                            └──────┘
      ↑                                     │
      │                              invalid_count未超限
      │                                     ↓
┌──────────┐   invalid_count超限     ┌─────────┐
│  kReset  │ ←────────────────────── │ kRunning│
│          │ ←── low_local_accuracy ─│         │
└──────────┘                         └─────────┘
      │
      └──→ kInvalid（下一周期）
```

#### 3.1 状态说明

| 状态 | 含义 | 行为 |
| ---- | ---- | ---- |
| `kInvalid` | 等待条件满足 | 累计 valid/invalid 计数 |
| `kInit` | 初始化 KF | 设置 X0、P0、Q、R、H |
| `kRunning` | KF 运行中 | 每周期执行 Predict + Correct |
| `kReset` | 重置 | 保存结果、清零计数、标记未初始化 |

#### 3.2 转移条件

- **Invalid → Init**：`valid_count` 达到 `straight_valid_times_limit` 且 `invalid_count` 未超限 且定位精度高
- **Init → Running**：`invalid_count` 未超限
- **Running → Reset**：`invalid_count` 超限 或 定位精度低
- **Reset → Invalid**：无条件，下一周期立即转移

### 4. 直线工况判定

`CheckStraightCondition` 函数判定当前是否为适合辨识的稳态直线工况：

| 条件 | 阈值 | 物理意义 |
| ---- | ---- | -------- |
| 车速下限 | 30 kph | 低速运动学误差大 |
| 车速上限 | 105 kph | 高速传感器噪声大 |
| 纵向加速度 | \|a_x_filtered\| ≤ 0.2 m/s² | 排除加减速 |
| 横摆角速度 | \|r_filtered\| < 0.00035·v | 速度自适应，排除转弯 |
| 横滚角 | \|roll\| < 0.02 rad (≈1.15°) | 排除侧倾路面 |
| 方向盘转角变化率 | \|ṡtr_td\| < 1.0 °/s | 排除主动转向 |
| 定位精度 | high_local_accuracy = true | GPS 信号质量 |

**信号预处理：**
- 纵向加速度和横摆角速度使用一阶低通滤波（α=0.9）
- 方向盘转角变化率使用跟踪微分器（二阶，时间常数 T=0.1s）

### 5. 跟踪微分器

用于从方向盘转角信号中提取平滑的变化率，避免直接差分的噪声放大：

$$
\ddot x = -\frac{1}{T^2}(x - v_{input}) - \frac{2}{T}\dot x
$$

离散化：
```cpp
x1_new = x1 + dt * x2;
x2_new = x2 - dt * (1/T/T * (x1 - v) + 2/T * x2);
```

等效为临界阻尼二阶系统，带宽约 $1/(2\pi T) \approx 1.6$ Hz。

### 6. 后处理与收敛判定

#### 6.1 滑动窗口均值滤波

KF 输出的偏置估计经过 `MeanFilter`（滑动窗口均值）平滑：
- 窗口大小 = `means_window_time / period_time`
- 仅在直线条件满足时更新窗口

#### 6.2 收敛判定

当满足以下条件时认为 KF 已收敛：

```cpp
steer_offset_results_window_.size() == means_window_ &&  // 窗口已满
fabs(GetMax() - GetMin()) < 0.002 * means_window_       // 窗口内极差小
```

即窗口内估计值的极差小于阈值（每个样本贡献 0.002°），说明估计已稳定。

#### 6.3 结果钳位

收敛后的偏置估计被钳位在 [-5°, +5°] 范围内（方向盘角度域）。

### 7. 多次估计的统计融合

每次 KF 从 Running 进入 Reset 时（且满足最小运行时长 ≥ 1s 且已收敛），调用 `UpdateStandardDeviation` 进行 Welford 在线统计：

$$
\bar\Delta_n = \frac{(n-1)\bar\Delta_{n-1} + \Delta_n}{n}
$$

$$
S_n = S_{n-1} + (\Delta_n - \bar\Delta_{n-1})(\Delta_n - \bar\Delta_n)
$$

$$
\sigma_n = \sqrt{S_n / n}
$$

最终输出为多次估计的均值 `mean_steer_offset_estimated_`。

### 8. 接口与数据流

```
输入：
  ctrl_input.chassis.str_angle    → 方向盘转角（°）→ 经转向比转换为轮胎转角
  ctrl_input.local.head_angle     → GPS 航向角（rad）
  ctrl_input.local.speed          → 纵向车速（m/s）
  ctrl_input.local.acc            → 纵向加速度（m/s²）
  ctrl_input.local.yaw_rate       → IMU 横摆角速度（rad/s）
  ctrl_input.local.roll_angle     → 横滚角（rad）
  ctrl_input.local.high_local_accuracy → 定位精度标志
  steer_offset_init_deg           → 偏置初始值（°，来自上次标定结果）

输出：
  return mean_steer_offset_estimated_  → 方向盘角度域的偏置估计（°）

调试输出（debug proto）：
  estimator_state                 → 当前状态机状态
  steer_offset_estimated_deg      → 当前单次估计值
  mean_steer_offset_estimated_deg → 多次估计均值
  heading_estimated_rad           → KF 估计的航向角
  raw_steer_offset_estimated_deg  → 滑动窗口原始输出
  std_steer_offset_estimated_deg  → 多次估计标准差
  invalid_count / valid_count     → 工况计数
  steer_rate_td                   → 跟踪微分器输出
```

### 9. 算法流程图

```
每周期 Run() 调用：
  │
  ├─ 获取时间戳
  ├─ CheckStraightCondition → straight_condition
  ├─ 更新 valid_count / invalid_count
  │
  ├─ 状态机转移
  │   ├─ kInvalid: 等待条件
  │   ├─ kInit:    InitKalmanFilter(heading, offset_init)
  │   ├─ kRunning: GetResult(v, heading, steer) → state
  │   │            if straight_condition:
  │   │              SteerOffsetPostProcess(state)
  │   └─ kReset:   if converged: UpdateStandardDeviation()
  │                 清零、重置
  │
  └─ 输出 mean_steer_offset_estimated_
```

---

## Part II：4WS 扩展方案

---

### 10. 核心思想

利用调度表 `scheduler(v, δ_f)` 将后轮转角视为前轮传感器读数的已知函数，从而将 4WS 问题归约为等效的单偏置辨识问题。**KF 结构与 2WS 完全一致，唯一改动是控制输入的计算方式。**

### 11. 信号定义与推导

#### 11.1 4WS 中的转角关系

前轮传感器存在中位偏差：

$$
\delta_f^{real} = \delta_f^{sensor} + \Delta_f
$$

后轮由 ECU 根据调度表执行（输入为前轮传感器读数）：

$$
\delta_r^{cmd} = \text{scheduler}(v_x,\ \delta_f^{sensor})
$$

后轮执行机构存在中位偏差：

$$
\delta_r^{real} = \text{scheduler}(v_x,\ \delta_f^{sensor}) + \Delta_r
$$

#### 11.2 航向角运动学归约

$$
\dot\psi = \frac{v_x}{L}(\delta_f^{real} - \delta_r^{real})
= \frac{v_x}{L}\left[\delta_f^{sensor} + \Delta_f - \text{scheduler}(v_x,\ \delta_f^{sensor}) - \Delta_r\right]
$$

定义**等效转向角**：

$$
\delta_{eff} \triangleq \delta_f^{sensor} - \text{scheduler}(v_x,\ \delta_f^{sensor})
$$

则：

$$
\boxed{\dot\psi = \frac{v_x}{L}\left(\delta_{eff} + \Delta_{diff}\right), \quad \Delta_{diff} \triangleq \Delta_f - \Delta_r}
$$

**与 2WS 模型结构完全一致。**

### 12. KF 设计

状态向量、A/B/H 矩阵、Q/R/P0 均与 2WS 相同，唯一区别：

| | 2WS | 4WS |
|---|---|---|
| 控制输入 $u$ | `tire_steer_angle_rad` | `tire_steer_angle_rad - rear_steer_scheduled_rad` |
| 辨识结果 | $\Delta_f$ | $\Delta_{diff} = \Delta_f - \Delta_r$ |

### 13. 调度表查询

#### 13.1 数据

二维查找表，轴定义：
- 速度轴：25 个点，0~285 kph
- 前轮转角轴：11 个点，0~36°（步长 3.6°）

详见 `output/steering_scheduler_data.csv`。

#### 13.2 查询逻辑

```cpp
double LookupRearSteerScheduler(double speed_kph, double front_steer_deg) {
    double sign = (front_steer_deg >= 0.0) ? 1.0 : -1.0;
    double abs_front = std::fabs(front_steer_deg);
    abs_front = std::clamp(abs_front, 0.0, 36.0);
    speed_kph = std::clamp(speed_kph, 0.0, 285.0);
    double rear_deg = BilinearInterpolate(speed_axis, front_axis, table,
                                          speed_kph, abs_front);
    return sign * rear_deg;
}
```

### 14. 直线工况判定

与 2WS 完全相同，无需修改。

**补充说明：** 4WS 方案不要求转角为零。在高速巡航带有微小转向修正的场景下，只要横摆角速度满足阈值，KF 仍能正常辨识。因为等效转向角 $\delta_{eff}$ 已包含后轮贡献，KF 模型能正确预测非零输入下的航向变化，偏置体现为持续性残差。

### 15. 补偿策略

辨识出 $\Delta_{diff}$ 后，推荐全部补偿在前轮：

$$
\delta_f^{compensated} = \delta_f^{cmd} - \Delta_{diff}
$$

- 消除横摆偏差
- 质心侧偏角残差为 $\Delta_r$（典型 0.1°~0.3°，可接受）
- 实现最简单

### 16. 代码改动

基于现有 `SteerOffsetEstimator`，改动清单：

| 改动项 | 说明 |
| ------ | ---- |
| 新增调度表数据成员 | 速度轴、转角轴、二维表格数组 |
| 新增 `LookupRearSteerScheduler` | 双线性插值查表 |
| 修改 `GetResult` | 控制输入改为 `effective_steer_rad` |
| 修改 `Run` | 传递 `speed_kph` 到 `GetResult` |
| 输出解释变更 | 辨识结果语义从 $\Delta_f$ 变为 $\Delta_{diff}$ |

#### 16.1 GetResult 修改

```cpp
Eigen::VectorXd SteerOffsetEstimator4WS::GetResult(
    const double velocity_forward_mps,
    const double heading_rad,
    const double tire_steer_angle_rad,
    const double speed_kph) {

  double dt = current_timestamp_ - previous_timestamp_;

  // 4WS: 计算等效转向角
  double front_steer_deg = tire_steer_angle_rad * 180.0 / M_PI;
  double rear_scheduled_deg = LookupRearSteerScheduler(speed_kph, front_steer_deg);
  double rear_scheduled_rad = rear_scheduled_deg * M_PI / 180.0;
  double effective_steer_rad = tire_steer_angle_rad - rear_scheduled_rad;

  Eigen::MatrixXd A(2, 2);
  A << 1.0, velocity_forward_mps / wheel_base_ * dt,
       0,   1.0;

  Eigen::MatrixXd B(2, 1);
  B << velocity_forward_mps / wheel_base_ * dt,
       0;

  estimator_.SetTransitionMatrix(A);
  estimator_.SetControlMatrix(B);

  Eigen::MatrixXd U(1, 1);
  U << effective_steer_rad;  // 原为 tire_steer_angle_rad
  estimator_.Predict(U);

  Eigen::MatrixXd Z(1, 1);
  Z << heading_rad;
  estimator_.CorrectNormalized(Z);

  return estimator_.GetStateEstimate();
}
```

### 17. 数值验证

| 场景 | speed | δ_f^sensor | scheduler | δ_eff | 说明 |
| ---- | ----- | ---------- | --------- | ----- | ---- |
| 高速直线 | 80 kph | 0° | 0° | 0° | 与 2WS 等价 |
| 高速微转 | 80 kph | 3.6° | 0.63° | 2.97° | KF 正常预测 |
| 中速巡航 | 50 kph | 7.2° | 0.37° | 6.83° | 可能不满足直线条件 |
| 低速转弯 | 10 kph | 36° | -6.0° | 42° | 不满足直线条件，不更新 |

### 18. 鲁棒性考虑

| 风险 | 应对 |
| ---- | ---- |
| 调度表与 ECU 版本不一致 | 定期同步标定数据 |
| 后轮执行延迟 | 直线条件已排除高动态工况 |
| 后轮限位饱和 | 表中已体现，查询正确 |
| Δ_diff 温度漂移 | Q(1,1) > 0，KF 可跟踪缓变 |

### 19. 与 DOB 的协同

中位补偿应在 DOB 之前施加：

```
δ_f^sensor → [-Δ_diff] → δ_f^corrected → scheduler → δ_r^cmd
                                        → DOB 输入
```

DOB 观测到的干扰不包含已知中位偏差，提高辨识精度。
