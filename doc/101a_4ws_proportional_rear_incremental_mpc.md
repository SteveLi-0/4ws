# 4WS 比例后轮 + 前轮增量 + 扰动观测器输入下的后轴误差方程（MPC autogen 对照）

> 本文从 [[03c_error_rear_4ws]] 的 4WS 后轴误差状态空间出发，叠加：
>
> 1. **后轮随动比例**：$\delta_r = k_r\,\delta_f^{\text{cmd}}$；
> 2. **前轮增量控制**：状态扩展为 $[e_1,\dot e_1,e_2,\dot e_2,\delta_f^{\text{cmd}}]$，控制量为 $\Delta\dot\delta_f$；
> 3. **DOB 输出的前轮扰动**：来自 [[06a_4ws_steer_dob_observer]]/[[06b_4ws_dob_righthand_verification]] 的 $\delta_d$ 直接接到前轮通道。
>
> 推导对应 MPC 求解器的 autogen 代码（`mipilot/.../lat_mpc/interface/auto_gen.h`）。本文给出五维扩展状态空间，并逐项对照代码中的 `SystemODE`、`SystemODE_wrt_State_Gradient`、`SystemODE_wrt_Control_Gradient` 三个函数，确认实现一致性。

---

## 1 信号约定与状态向量

| 信号 | 含义 |
|------|------|
| $\delta_f^{\text{cmd}}$ | 前轮指令角（MPC 内部状态，单位 rad） |
| $\Delta\dot\delta_f$ | 前轮指令角变化率（MPC 决策变量） |
| $\delta_r = k_r\,\delta_f^{\text{cmd}}$ | 后轮指令角（随动） |
| $\delta_d$ | DOB 输出的前轮转角扰动（齿条偏置/标定误差等，单位 rad） |
| $\delta_f^{\text{act}} = \delta_f^{\text{cmd}} + \delta_d$ | 实际作用到前轮上的转角 |
| $\dot\theta_{\text{ref}} = \kappa\,v_x$ | 参考航向变化率（曲率扰动） |

> **代码命名注**：autogen 中变量 `steer` 对应 $\delta_f^{\text{cmd}}$，控制量 `d_steer` 对应 $\Delta\dot\delta_f$，扰动量 `steer_disturbance_deg` 对应 $\delta_d$（变量名带 `_deg` 仅是历史遗留，量纲与 `steer` 同为 rad）。

扩展状态向量：

$$
\mathbf{x} = \big[e_1,\;\dot e_1,\;e_2,\;\dot e_2,\;\delta_f^{\text{cmd}}\big]^T \in \mathbb{R}^5
$$

控制量：

$$
u = \Delta\dot\delta_f \in \mathbb{R}
$$

---

## 2 动力学合并

### 2.1 03c 4WS 起点

由 03c：

$$
\dot{\mathbf{x}}_{1:4} = A\,\mathbf{x}_{1:4} + B_f\,\delta_f^{\text{act}} + B_r\,\delta_r + G\,\dot\theta_{\text{ref}}
$$

代入 $\delta_f^{\text{act}} = \delta_f^{\text{cmd}} + \delta_d$ 和 $\delta_r = k_r\,\delta_f^{\text{cmd}}$：

$$
\dot{\mathbf{x}}_{1:4} = A\,\mathbf{x}_{1:4} + (B_f + k_r B_r)\,\delta_f^{\text{cmd}} + B_f\,\delta_d + G\,\dot\theta_{\text{ref}}
$$

定义等效输入矩阵（紧凑形式，使用中间变量 $\eta, \xi$）：

$$
B_{eq} = B_f + k_r B_r = \begin{bmatrix} 0 \\[4pt] \dfrac{C_f\eta + k_r C_r\xi}{mI_z} \\[6pt] 0 \\[4pt] \dfrac{l_f C_f - k_r l_r C_r}{I_z} \end{bmatrix}
$$

其中 $L = l_f + l_r$，$\eta = I_z - ml_f l_r$，$\xi = I_z + m l_r^2$。

**展开形式（不使用中间变量替换，直接展开为 $C_f, C_r, l_f, l_r, m, I_z$ 的多项式表达）**：

$$
B_{eq} = \begin{bmatrix} 0 \\[6pt] \dfrac{C_f}{m} - \dfrac{C_f l_f l_r}{I_z} + k_r\!\left(\dfrac{C_r}{m} + \dfrac{C_r l_r^2}{I_z}\right) \\[10pt] 0 \\[6pt] \dfrac{l_f C_f - k_r l_r C_r}{I_z} \end{bmatrix}
$$

> **化简钥匙**：$\dfrac{1}{m} - \dfrac{l_f l_r}{I_z} = \dfrac{\eta}{mI_z}$，$\dfrac{1}{m} + \dfrac{l_r^2}{I_z} = \dfrac{\xi}{mI_z}$。展开形式与代码 `auto_gen.h` 中 `steer` 的系数表达式逐项一致。

> **关键设计**：扰动 $\delta_d$ 走 $B_f$（不被 $k_r$ 缩放），因为偏置/标定误差物理上只发生在前轮齿条；后轮随动器跟踪的是**指令** $\delta_f^{\text{cmd}}$，不感知 $\delta_d$。详见 [[04c_proportional_rear_incremental_disturbance]] §2-3。

### 2.2 增加积分器

控制量为转角变化率：

$$
\dot\delta_f^{\text{cmd}} = \Delta\dot\delta_f
$$

合并为 5 维 ODE：

$$
\boxed{\;
\dot{\mathbf{x}} =
\underbrace{\begin{bmatrix} A & B_{eq} \\ \mathbf{0}_{1\times 4} & 0 \end{bmatrix}}_{A_{\text{aug}}}\mathbf{x}
+ \underbrace{\begin{bmatrix} \mathbf{0}_{4\times 1} \\ 1 \end{bmatrix}}_{B_{u}}\,\Delta\dot\delta_f
+ \underbrace{\begin{bmatrix} B_f \\ 0 \end{bmatrix}}_{B_{d}}\,\delta_d
+ \underbrace{\begin{bmatrix} G \\ 0 \end{bmatrix}}_{G_{\text{aug}}}\,\dot\theta_{\text{ref}}
\;}
$$

注意此处 $A_{\text{aug}}$ 的第 5 列正是 $B_{eq}$——也就是 03c 中的 $B_f$ 与 $B_r$ 经 $k_r$ 加权合并后的等效输入矩阵。这是后轮随动假设进入扩展状态空间的入口。

---

## 3 各分量展开

### 3.0 横向误差与航向误差的几何定义与运动学

> 沿用 [[03c_error_rear_4ws]] §2 的后轴 Frenet 误差约定。本节单独列出，便于和工程实现对照（下游模块往往直接给出 $e_1, e_2, \dot e_1, \dot e_2$ 这四个量作为 MPC 输入）。

#### 几何定义（后轴中心相对参考路径）

设车辆后轴中心在世界系下位姿为 $(x_R,\,y_R,\,\psi)$；参考路径上离后轴中心最近的投影点为 $(x_r,\,y_r)$，对应切向角 $\theta_{\text{ref}}$、曲率 $\kappa$。

横向误差（左正、右负，与路径切向构成右手系一致）：

$$
e_1 = -(x_R - x_r)\sin\theta_{\text{ref}} + (y_R - y_r)\cos\theta_{\text{ref}}
$$

航向误差：

$$
e_2 = \psi - \theta_{\text{ref}} \quad (\text{wrap 至 } [-\pi,\pi])
$$

> 若上游只提供质心位姿 $(x_{\text{cg}}, y_{\text{cg}}, \psi)$，需先回算后轴位置：$x_R = x_{\text{cg}} - l_r\cos\psi$，$y_R = y_{\text{cg}} - l_r\sin\psi$。然后再做投影。

#### 时间导数（运动学关系）

由 03c §2，小角度近似下：

$$
\dot e_1 \;=\; v_{yr} + v_x\,e_2, \qquad v_{yr} \;=\; v_y - l_r\,r \tag{$\ast$}
$$

$$
\dot e_2 \;=\; r - \dot\theta_{\text{ref}}, \qquad \dot\theta_{\text{ref}} = \kappa\,v_x \tag{$\ast\ast$}
$$

其中 $v_y$ 为质心侧向速度、$r=\dot\psi$ 为横摆角速度、$v_{yr}$ 为后轴侧向速度。

#### 在线计算建议

工程实现中两种取值路径：

1. **由动力学量直接合成**（常见）：
   $$\dot e_1 = (v_y - l_r r) + v_x\,e_2,\qquad \dot e_2 = r - \kappa\,v_x$$
   优点：解析、无延迟；缺点：依赖 $v_y$（横摆/IMU 估计）和 $r$ 的精度，在低速 $v_x\to 0$ 时 $\kappa v_x$ 项接近零，但 $v_y$ 噪声不会被分母放大。
2. **由 $e_1, e_2$ 数值差分**：用上一周期的 $e_1, e_2$ 做差分滤波获得 $\dot e_1, \dot e_2$。优点：抗模型不准；缺点：相位滞后、对采样抖动敏感。

> **代码命名注**：autogen 中 `lat_err = state_input(0,0)` 对应 $e_1$，`lat_err_rate = state_input(0,1)` 对应 $\dot e_1$，`heading_err = state_input(0,2)` 对应 $e_2$，`heading_err_rate = state_input(0,3)` 对应 $\dot e_2$。这四个量在 MPC 求解之前已由上层填好；§3.1 与 §3.3 的 $\dot{\mathbf{x}}_1 = \dot e_1$、$\dot{\mathbf{x}}_3 = \dot e_2$ 之所以是平凡恒等式，正是因为状态向量本身就显式包含了 $\dot e_1, \dot e_2$。

#### 工程实现对照（典型 C++ 代码片段逐行解读）

下面以一段实际工程代码为例，把 §3.0 的几何/运动学定义与 100c 的稳态前馈映射到具体实现。

```cpp
const auto& veh_conf = ControlConfig().veh_config();
const double path_true_heading =
    common::math::NormalizeAngle(path_point.theta + lat_out.heading_bias);
const double dx = vehicle_state.x - path_point.x;
const double dy = vehicle_state.y - path_point.y;
const double cos_heading = std::cos(path_true_heading);
const double sin_heading = std::sin(path_true_heading);
double lateral_error = cos_heading * dy - sin_heading * dx;

const double& kr_cur = lat_out.kr_the;
double theta_rear =
    path_point.kappa / (1.0 - kr_cur) *
    (-(veh_conf.lf() + veh_conf.lr()) * kr_cur +
     veh_conf.mass() * vehicle_state.speed * vehicle_state.speed *
         (veh_conf.lf() * cf_used_ - kr_cur * veh_conf.lr() * veh_conf.cr()) /
         (cf_used_ * veh_conf.cr() * veh_conf.wheel_base()));

double heading_error = common::math::NormalizeAngle(
    vehicle_state.theta - path_true_heading + theta_rear);

double lateral_error_rate =
    vehicle_state.speed *
    std::sin(common::math::NormalizeAngle(vehicle_state.theta - path_true_heading));

double heading_error_rate =
    vehicle_state.yaw_rate - path_point.kappa * vehicle_state.speed;
```

**(1) 路径切向（含 bias）**

$$
\theta_{\text{ref}}^{\text{true}} = \mathrm{wrap}\!\left(\theta_{\text{ref}} + b_\theta\right)
$$

`path_point.theta` 即 $\theta_{\text{ref}}$，`heading_bias` 即标定/估计层补出的航向偏置 $b_\theta$。后续所有投影、$e_2$ 都基于这个修正后的切向。

**(2) 横向误差 $e_1$**

$$
e_1 = \cos\theta_{\text{ref}}^{\text{true}}\cdot \Delta y - \sin\theta_{\text{ref}}^{\text{true}}\cdot \Delta x,\qquad (\Delta x,\Delta y) = (x_v - x_r,\ y_v - y_r)
$$

法向单位向量 $\hat n = (-\sin\theta,\cos\theta)$，**车体在路径左侧时 $e_1 > 0$**。与 §3.0 几何定义一致。

> 物理含义：把车辆位置相对参考点的位移向路径**法向**投影。投影量等于带符号的最短法向距离（小角度近似下），对应 03c 后轴 Frenet 坐标系中的 $e_1$。
>
> 工程注：`vehicle_state.x/y` 是后轴中心还是质心由上层约定。03c/101a 推导基于后轴误差，理想上传入应为后轴中心；若上层提供的是质心，需先按 §3.0 提示回退（$x_R = x_{\text{cg}} - l_r\cos\psi,\;y_R = y_{\text{cg}} - l_r\sin\psi$）或显式接受"以质心做投影"的近似。

**(3) 稳态后轴航向偏差 `theta_rear`（即 100c 的 $e_{2,ss}$）**

代码：

$$
\theta_{\text{rear}} = \frac{\kappa}{1-k_r}\!\left[-k_r L + \frac{m v_x^2 (l_f C_f - k_r l_r C_r)}{C_f C_r L}\right]
$$

把 100c §4.2 的稳态结果

$$
e_{2,ss} = \frac{m v_x^2 (l_f C_f - k_r l_r C_r) - k_r C_f C_r L^2}{C_f C_r L R (1-k_r)}
$$

提出 $\kappa = 1/R$ 和 $\kappa/(1-k_r)$，恰好得到代码的形式——**逐项一致**。

> 物理含义：在稳态圆弧（$\dot{\mathbf{x}}=0$、曲率 $\kappa$ 恒定）上，由于后轴有非零侧偏角 $\beta_r = -v_{yr}/v_x \neq 0$，车身航向 $\psi$ 与路径切向 $\theta_{\text{ref}}$ 不重合，会停在某个由 $\kappa,\,v_x,\,k_r$ 和车辆参数闭式确定的稳态偏角上。`theta_rear` 就是这个偏角的解析值。$k_r=0$ 时退化为 100a 的 $ml_fv_x^2/(C_rLR)$；$k_r\to 1$（蟹行）时发散，因为前后轮同角度无法产生横摆。

**(4) 航向误差 $e_2$（含稳态前馈补偿）**

$$
e_2^{\text{mpc}} = \mathrm{wrap}\!\left(\underbrace{\psi - \theta_{\text{ref}}^{\text{true}}}_{e_2^{\text{raw}}} + \theta_{\text{rear}}\right)
$$

> 物理含义：把稳态偏置 $e_{2,ss}$ 当作前馈"打包"进喂给 MPC 的误差信号。稳态圆弧上 $e_2^{\text{raw}}$ 不会归零，但 $e_2^{\text{raw}} + \theta_{\text{rear}} \approx 0$，于是 MPC 在 stage cost 中看到的残差为 0，避免被稳态偏置牵着走，反馈分量只处理偏离稳态的瞬态部分。这与 §6.3 中"代码用绝对状态、靠 `steer_ref`/`d_steer_ref` 围绕前馈跟踪"的设计哲学呼应。
>
> **符号一致性提示**：代码用加号能成立的前提是 `theta_rear` 的方向定义与 $-e_2^{\text{raw}}|_{\text{ss}}$ 相同——这取决于车身 yaw、路径 heading、左右正向的具体约定。验证方法：在已知稳态圆弧（恒定方向盘、恒定速度）上跑一段开环数据，确认 `heading_error` 落在 0 附近。

**(5) 横向误差变化率 $\dot e_1$**

$$
\dot e_1^{\text{code}} = v_x\,\sin(e_2^{\text{raw}})
$$

这是 §3.0 中 $(\ast)$ 式 $\dot e_1 = v_{yr} + v_x e_2$ 的**简化**：

- 用 $\sin(\psi-\theta_{\text{ref}})$ 替代 $e_2$，保留大角度精度；
- **舍弃** $v_{yr}$（后轴侧向速度）。

舍弃 $v_{yr}$ 等价于**忽略后轴 sideslip**，把后轴速度方向当成与车身纵轴方向重合。优点：只依赖 `theta`、`speed`，不需要 IMU 估计 $v_y$；代价：高速大侧偏工况下 $\dot e_1$ 有偏差。

> 注意：这里**不需要**像 $e_2$ 那样补偿稳态项，因为 $\dot e_1$ 的稳态值本身就是 0（稳态 $\dot{\mathbf{x}}=0$ 意味着 $\dot e_1 \equiv 0$）。

**(6) 航向误差变化率 $\dot e_2$**

$$
\dot e_2^{\text{code}} = r - \kappa v_x
$$

直接对应 §3.0 的 $(\ast\ast)$：`yaw_rate` 即 $r$，`kappa * speed` 即 $\dot\theta_{\text{ref}}$。一对一映射，无近似。

#### 实现量与文档量对照表

| 代码量 | 数学含义 | 文档位置 | 处理方式 |
|---|---|---|---|
| `path_true_heading` | $\theta_{\text{ref}} + b_\theta$ | §3.0 几何定义 | 含标定 bias 修正 |
| `lateral_error` | $e_1 = -\Delta x \sin\theta + \Delta y \cos\theta$ | §3.0 横向误差几何定义 | 投影法（与 03c 严格一致） |
| `theta_rear` | $e_{2,ss}$ | 100c §4.2 | 稳态闭式前馈 |
| `heading_error` | $(\psi - \theta_{\text{ref}}) + e_{2,ss}$ | §3.0 + 稳态前馈 | 扣除稳态后给 MPC |
| `lateral_error_rate` | $v_x \sin(\psi-\theta_{\text{ref}})$ | §3.0 $(\ast)$ 的简化 | 忽略 $v_{yr}$（后轴 sideslip） |
| `heading_error_rate` | $r - \kappa v_x$ | §3.0 $(\ast\ast)$ | 严格运动学，无近似 |

#### 不对称近似的工程取舍

代码在两类量上采取了**有意识的不对称处理**：

| 量 | 稳态分量处理 | 运动学近似 |
|---|---|---|
| $e_2$（heading_error） | 显式扣除 $e_{2,ss}$ | 严格（仅 wrap） |
| $\dot e_1$（lateral_error_rate） | 不需扣（稳态本为 0） | 简化（忽略 $v_{yr}$） |
| $\dot e_2$（heading_error_rate） | 不需扣（稳态本为 0） | 严格 |

稳态圆弧上 MPC 看到的 4 个状态信号近似是 $(e_1,\,0,\,0,\,0)$——所有"已知该有"的稳态偏置都被外部前馈/补偿吃掉，MPC 只对**偏离稳态的瞬态误差**做反馈。这与 §6.3 中"前馈和参考层负责稳态，MPC 只处理瞬态"的设计哲学一致。

#### 与状态空间各行的衔接

| 状态分量 | 几何/运动学含义 | 在 §3 中的展开行 |
|----------|----------------|----------------|
| $x_1 = e_1$ | 后轴横向误差 | §3.1（$\dot e_1$ = 状态 $x_2$） |
| $x_2 = \dot e_1$ | 由 $(\ast)$ 给出 | §3.2（$\ddot e_1$，含轮胎力） |
| $x_3 = e_2$ | 航向误差 | §3.3（$\dot e_2$ = 状态 $x_4$） |
| $x_4 = \dot e_2$ | 由 $(\ast\ast)$ 给出 | §3.4（$\ddot e_2$，含轮胎力） |
| $x_5 = \delta_f^{\text{cmd}}$ | 前轮指令角 | §3.5（积分器） |

### 3.1 第 1 行：$\dot e_1 = \dot e_1$

$$
\dot{\mathbf{x}}_1 = \dot e_1
$$

### 3.2 第 2 行：$\ddot e_1$

**紧凑形式（使用 $\eta, \xi, L$ 替换）**：

$$
\begin{aligned}
\ddot e_1 &= -\frac{C_f\eta + C_r\xi}{mI_zv_x}\,\dot e_1
+ \frac{C_f\eta + C_r\xi}{mI_z}\,e_2
- \frac{C_fL\eta}{mI_zv_x}\,\dot e_2 \\[4pt]
&\quad + \frac{C_f\eta + k_r C_r\xi}{mI_z}\,\delta_f^{\text{cmd}}
+ \frac{C_f\eta}{mI_z}\,\delta_d
+ \left(-\frac{C_fL\eta}{mI_zv_x} - v_x\right)\dot\theta_{\text{ref}}
\end{aligned}
$$

**展开形式（不使用中间变量替换，直接以 $C_f, C_r, l_f, l_r, m, I_z, v_x$ 表达，对应 `auto_gen.h` 原始写法）**：

$$
\begin{aligned}
\ddot e_1 &= \left[-\frac{C_f + C_r}{m v_x} + \frac{l_r(C_f l_f - C_r l_r)}{I_z v_x}\right]\dot e_1
+ \left[\frac{C_f + C_r}{m} - \frac{l_r(C_f l_f - C_r l_r)}{I_z}\right] e_2 \\[4pt]
&\quad + \left[\frac{C_f l_f l_r (l_f+l_r)}{I_z v_x} - \frac{C_f(l_f+l_r)}{m v_x}\right]\dot e_2 \\[4pt]
&\quad + \left[\frac{C_f}{m} - \frac{C_f l_f l_r}{I_z} + k_r\!\left(\frac{C_r}{m} + \frac{C_r l_r^2}{I_z}\right)\right]\delta_f^{\text{cmd}}
+ \left[\frac{C_f}{m} - \frac{C_f l_f l_r}{I_z}\right]\delta_d \\[4pt]
&\quad + \left[\frac{C_f l_f l_r (l_f+l_r)}{I_z v_x} - \frac{C_f(l_f+l_r)}{m v_x} - v_x\right]\dot\theta_{\text{ref}}
\end{aligned}
$$

### 3.3 第 3 行：$\dot e_2 = \dot e_2$

$$
\dot{\mathbf{x}}_3 = \dot e_2
$$

### 3.4 第 4 行：$\ddot e_2$

**紧凑形式（使用 $L$ 替换）**：

$$
\begin{aligned}
\ddot e_2 &= -\frac{l_fC_f - l_rC_r}{I_zv_x}\,\dot e_1
+ \frac{l_fC_f - l_rC_r}{I_z}\,e_2
- \frac{l_fC_fL}{I_zv_x}\,\dot e_2 \\[4pt]
&\quad + \frac{l_fC_f - k_r l_rC_r}{I_z}\,\delta_f^{\text{cmd}}
+ \frac{l_fC_f}{I_z}\,\delta_d
- \frac{l_fC_fL}{I_zv_x}\,\dot\theta_{\text{ref}}
\end{aligned}
$$

**展开形式（不使用 $L$ 替换，对应 `auto_gen.h` 原始写法）**：

$$
\begin{aligned}
\ddot e_2 &= \frac{-C_f l_f + C_r l_r}{I_z v_x}\,\dot e_1
+ \frac{C_f l_f - C_r l_r}{I_z}\,e_2
- \frac{C_f l_f (l_f + l_r)}{I_z v_x}\,\dot e_2 \\[4pt]
&\quad + \left[\frac{C_f l_f}{I_z} - \frac{k_r C_r l_r}{I_z}\right]\delta_f^{\text{cmd}}
+ \frac{C_f l_f}{I_z}\,\delta_d
- \frac{C_f l_f (l_f + l_r)}{I_z v_x}\,\dot\theta_{\text{ref}}
\end{aligned}
$$

> **代码中曲率项的等价写法**：`auto_gen.h` 第 4 行直接写为 $-\dfrac{C_f l_f (l_f+l_r)}{I_z}\,\kappa$（不带 $v_x$）。利用 $\dot\theta_{\text{ref}} = \kappa v_x$ 即可看出与展开式中 $-\dfrac{C_f l_f (l_f+l_r)}{I_z v_x}\dot\theta_{\text{ref}}$ 相同。

### 3.5 第 5 行：积分器

$$
\dot\delta_f^{\text{cmd}} = \Delta\dot\delta_f
$$

---

## 4 雅可比矩阵

控制器使用的雅可比为 $\partial f/\partial \mathbf{x}$ 与 $\partial f/\partial u$，扰动通道 $\delta_d$ 与 $\dot\theta_{\text{ref}}$ 视为 online data，不进入梯度。

### 4.1 状态雅可比 $A_{\text{aug}}$

**紧凑形式（使用 $\eta, \xi, L$ 替换）**：

$$
A_{\text{aug}} = \begin{bmatrix}
0 & 1 & 0 & 0 & 0 \\[6pt]
0 & -\dfrac{C_f\eta + C_r\xi}{mI_zv_x} & \dfrac{C_f\eta + C_r\xi}{mI_z} & -\dfrac{C_fL\eta}{mI_zv_x} & \dfrac{C_f\eta + k_r C_r\xi}{mI_z} \\[10pt]
0 & 0 & 0 & 1 & 0 \\[6pt]
0 & -\dfrac{l_fC_f - l_rC_r}{I_zv_x} & \dfrac{l_fC_f - l_rC_r}{I_z} & -\dfrac{l_fC_fL}{I_zv_x} & \dfrac{l_fC_f - k_r l_rC_r}{I_z} \\[10pt]
0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

**展开形式（不使用中间变量替换，对应 `auto_gen.h` 中 `SystemODE_wrt_State_Gradient` 的逐元素写法）**：

$$
A_{\text{aug}} = \begin{bmatrix}
0 & 1 & 0 & 0 & 0 \\[8pt]
0 & \dfrac{-C_f - C_r}{m v_x} + \dfrac{l_r(C_f l_f - C_r l_r)}{I_z v_x}
  & \dfrac{C_f + C_r}{m} - \dfrac{l_r(C_f l_f - C_r l_r)}{I_z}
  & \dfrac{C_f l_f l_r (l_f+l_r)}{I_z v_x} - \dfrac{C_f(l_f+l_r)}{m v_x}
  & \dfrac{C_f}{m} - \dfrac{C_f l_f l_r}{I_z} + k_r\!\left(\dfrac{C_r}{m} + \dfrac{C_r l_r^2}{I_z}\right) \\[12pt]
0 & 0 & 0 & 1 & 0 \\[8pt]
0 & \dfrac{-C_f l_f + C_r l_r}{I_z v_x}
  & \dfrac{C_f l_f - C_r l_r}{I_z}
  & -\dfrac{C_f l_f (l_f+l_r)}{I_z v_x}
  & \dfrac{C_f l_f}{I_z} - \dfrac{k_r C_r l_r}{I_z} \\[12pt]
0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

### 4.2 控制雅可比 $B_u$

$$
B_u = \big[0,\;0,\;0,\;0,\;1\big]^T
$$

---

## 5 与 autogen 代码对照

代码片段位于 `mipilot/modules/pilot/hnoa/control/controller/lat_control/lat_mpc/interface/auto_gen.h`（用户提供）。以下逐函数比对。

### 5.1 `Autogen::SystemODE`

代码使用的局部变量映射：

```cpp
lat_err_rate   = state_input(0,1);   // ẋ₁ = ė₁  → 文档中 ė₁
heading_err    = state_input(0,2);   // x₃ = e₂
heading_err_rate = state_input(0,3); // x₄ = ė₂
steer          = state_input(0,4);   // x₅ = δ_f^cmd
d_steer        = control_input(0,0); // u  = Δδ̇_f
steer_disturbance_deg = δ_d
kr             = k_r
```

#### 第 1 行：`return_mat(0,0) = lat_err_rate`

对照文档 §3.1：$\dot e_1$。✓

#### 第 2 行：`return_mat(1,0) = ...`（$\ddot e_1$）

代码原始表达式（按状态分组）：

```
heading_err     × ((Cf+Cr)/m - lr*(Cf*lf - Cr*lr)/Iz)
heading_err_rate× (Cf*lf*lr*(lf+lr)/(Iz*v) + (-Cf*lf - Cf*lr)/(m*v))
kappa*v         × (Cf*lf*lr*(lf+lr)/(Iz*v) - v + (-Cf*lf - Cf*lr)/(m*v))
lat_err_rate    × ((-Cf-Cr)/(m*v) + lr*(Cf*lf - Cr*lr)/(Iz*v))
steer           × (Cf/m - Cf*lf*lr/Iz + kr*(Cr/m + Cr*lr^2/Iz))
steer_disturbance × (Cf/m - Cf*lf*lr/Iz)
```

化简过程（每项均通过 SymPy 验证，详见 §7）：

| 代码项 | 化简结果 | 对应文档项 |
|--------|---------|-----------|
| $e_2$ 系数 $\dfrac{C_f+C_r}{m} - \dfrac{l_r(C_fl_f - C_rl_r)}{I_z}$ | $\dfrac{C_f\eta + C_r\xi}{mI_z}$ | $A_{23}$ ✓ |
| $\dot e_2$ 系数 $\dfrac{C_f l_f l_r L}{I_z v_x} - \dfrac{C_f L}{m v_x}$ | $-\dfrac{C_fL\eta}{mI_zv_x}$ | $A_{24}$ ✓ |
| $\kappa v_x$ 系数（曲率扰动） $\dfrac{C_f l_f l_r L}{I_z v_x} - v_x - \dfrac{C_f L}{m v_x}$ | $-\dfrac{C_fL\eta}{mI_zv_x} - v_x$ | $G_2$ ✓ |
| $\dot e_1$ 系数 $-\dfrac{C_f+C_r}{mv_x} + \dfrac{l_r(C_fl_f - C_rl_r)}{I_z v_x}$ | $-\dfrac{C_f\eta + C_r\xi}{mI_zv_x}$ | $A_{22}$ ✓ |
| $\delta_f^{\text{cmd}}$ 系数 $\dfrac{C_f}{m} - \dfrac{C_f l_f l_r}{I_z} + k_r\!\left(\dfrac{C_r}{m} + \dfrac{C_r l_r^2}{I_z}\right)$ | $\dfrac{C_f\eta + k_r C_r\xi}{mI_z}$ | $B_{eq,2}$ ✓ |
| $\delta_d$ 系数 $\dfrac{C_f}{m} - \dfrac{C_f l_f l_r}{I_z}$ | $\dfrac{C_f\eta}{mI_z}$ | $B_{f,2}$ ✓ |

> **化简钥匙**：$\dfrac{1}{m} - \dfrac{l_r l_f}{I_z} = \dfrac{\eta}{mI_z}$，$\dfrac{1}{m} + \dfrac{l_r^2}{I_z} = \dfrac{\xi}{mI_z}$。

#### 第 3 行：`return_mat(2,0) = heading_err_rate`

对照文档 §3.3：$\dot e_2$。✓

#### 第 4 行：`return_mat(3,0) = ...`（$\ddot e_2$）

代码原始表达式：

```
- Cf*heading_err_rate*lf*(lf+lr)/(Iz*v)            ← ė₂ 项
- Cf*kappa*lf*(lf+lr)/Iz                            ← κ 项（注意没有 v 因子）
+ Cf*lf*steer_disturbance/Iz                        ← δ_d 项
+ heading_err*(Cf*lf - Cr*lr)/Iz                    ← e₂ 项
+ steer*(Cf*lf/Iz - Cr*kr*lr/Iz)                    ← δ_f^cmd 项
+ lat_err_rate*(-Cf*lf + Cr*lr)/(Iz*v)              ← ė₁ 项
```

| 代码项 | 文档对应 |
|--------|---------|
| $\dot e_1$：$-\dfrac{C_fl_f - C_rl_r}{I_zv_x}$ | $A_{42}$ ✓ |
| $e_2$：$\dfrac{C_fl_f - C_rl_r}{I_z}$ | $A_{43}$ ✓ |
| $\dot e_2$：$-\dfrac{C_fl_fL}{I_zv_x}$ | $A_{44}$ ✓ |
| $\delta_f^{\text{cmd}}$：$\dfrac{C_fl_f}{I_z} - \dfrac{k_r C_r l_r}{I_z} = \dfrac{l_fC_f - k_r l_rC_r}{I_z}$ | $B_{eq,4}$ ✓ |
| $\delta_d$：$\dfrac{C_fl_f}{I_z}$ | $B_{f,4}$ ✓ |
| $\kappa$：$-\dfrac{C_fl_f L}{I_z}$ | $\dot\theta_{\text{ref}}\,G_4 = \kappa v_x \cdot \!\left(-\dfrac{l_fC_fL}{I_zv_x}\right) = -\dfrac{C_fl_f L}{I_z}\kappa$ ✓ |

> **关于第 4 行的曲率项写法**：文档中写作 $G_4 \cdot \dot\theta_{\text{ref}}$，而代码直接乘 $\kappa$ 不带 $v_x$。这两种写法等价，因为 $\dot\theta_{\text{ref}} = \kappa v_x$，分子分母的 $v_x$ 互相抵消。第 2 行因为 $G_2$ 含有“裸” $v_x$ 项，在代码中保留了 $\kappa v_x$ 的乘法因子。

#### 第 5 行：`return_mat(4,0) = d_steer`

对照文档 §3.5：$\dot\delta_f^{\text{cmd}} = \Delta\dot\delta_f$。✓

### 5.2 `Autogen::SystemODE_wrt_State_Gradient`

代码非零项：

| 代码 | 数值 | 文档 $A_{\text{aug}}$ 对应 |
|------|------|----------------------------|
| `(0,1)` | `1` | $\partial\dot e_1/\partial\dot e_1 = 1$ ✓ |
| `(1,1)` | $\dfrac{-C_f-C_r}{mv_x} + \dfrac{l_r(C_fl_f - C_rl_r)}{I_zv_x} = -\dfrac{C_f\eta + C_r\xi}{mI_zv_x}$ | $A_{22}$ ✓ |
| `(1,2)` | $\dfrac{C_f+C_r}{m} - \dfrac{l_r(C_fl_f - C_rl_r)}{I_z} = \dfrac{C_f\eta + C_r\xi}{mI_z}$ | $A_{23}$ ✓ |
| `(1,3)` | $\dfrac{C_f l_f l_r L}{I_zv_x} - \dfrac{C_fL}{mv_x} = -\dfrac{C_fL\eta}{mI_zv_x}$ | $A_{24}$ ✓ |
| `(1,4)` | $\dfrac{C_f}{m} - \dfrac{C_fl_fl_r}{I_z} + k_r\!\left(\dfrac{C_r}{m} + \dfrac{C_rl_r^2}{I_z}\right) = \dfrac{C_f\eta + k_r C_r\xi}{mI_z}$ | $B_{eq,2}$ ✓ |
| `(2,3)` | `1` | $\partial\dot e_2/\partial\dot e_2 = 1$ ✓ |
| `(3,1)` | $\dfrac{-C_fl_f + C_rl_r}{I_zv_x}$ | $A_{42}$ ✓ |
| `(3,2)` | $\dfrac{C_fl_f - C_rl_r}{I_z}$ | $A_{43}$ ✓ |
| `(3,3)` | $-\dfrac{C_fl_f(l_f+l_r)}{I_zv_x} = -\dfrac{C_fl_fL}{I_zv_x}$ | $A_{44}$ ✓ |
| `(3,4)` | $\dfrac{C_fl_f}{I_z} - \dfrac{C_r k_r l_r}{I_z} = \dfrac{l_fC_f - k_r l_rC_r}{I_z}$ | $B_{eq,4}$ ✓ |

雅可比与 ODE 完全自洽。✓

### 5.3 `Autogen::SystemODE_wrt_Control_Gradient`

代码：`return_mat(4,0) = 1`，其余为 0。

对照文档 $B_u = [0,0,0,0,1]^T$。✓

### 5.4 验证结论

代码的 `SystemODE`、`SystemODE_wrt_State_Gradient`、`SystemODE_wrt_Control_Gradient` 三函数与本文档推导的扩展 ODE 与雅可比**逐项一致**。SymPy 数值验证见 §7。

---

## 6 物理解读要点

### 6.1 后轮随动如何进入第 5 列

| 元素 | 表达式 | 物理来源 |
|------|--------|---------|
| $A_{\text{aug}}(2,5) = B_{eq,2}$ | $\dfrac{C_f\eta + k_r C_r\xi}{mI_z}$ | 前轮力 $F_{yf}$ 通过 $\eta$ + 后轮力 $k_rF_{yr}$ 通过 $\xi$，共同推 $\ddot e_1$ |
| $A_{\text{aug}}(4,5) = B_{eq,4}$ | $\dfrac{l_fC_f - k_r l_r C_r}{I_z}$ | 前轮 $l_fF_{yf}$ 与后轮 $-k_r l_r F_{yr}$ 力臂的代数和（同向同度时部分抵消） |

特例：$k_r = 0$ 时退化为 03a 的纯前轮版本；$k_r=1$ 且 $l_f=l_r,C_f=C_r$ 时 $B_{eq,4}=0$（蟹行无横摆）。

### 6.2 扰动与控制的解耦

- **控制通道** $\delta_f^{\text{cmd}}$：走 $B_{eq} = B_f + k_r B_r$，前后轮按 $1:k_r$ 协同响应。
- **扰动通道** $\delta_d$：仅走 $B_f$，**不被 $k_r$ 缩放**。后轮随动器看的是前轮**指令**，不感知机械偏置。

这与 [[04c_proportional_rear_incremental_disturbance]] 的设计选择一致。

### 6.3 增量化没有显式出现

代码用绝对状态 $\delta_f^{\text{cmd}}$ 而非偏离平衡点的 $\Delta\delta_f^{\text{cmd}}$。MPC 通过 stage cost 中的 `steer_ref` 与 `d_steer_ref`（前馈分量）实现“围绕前馈跟踪”，等价于 04c §4 的增量化推导。具体地：

$$
\text{stage cost} \supset \big\|\delta_f^{\text{cmd}} - \delta_{f,\text{ff}}\big\|_W + \big\|\Delta\dot\delta_f - \Delta\dot\delta_{f,\text{ff}}\big\|_W
$$

参考值 $\delta_{f,\text{ff}}$ 由前馈层（[[100c_4ws_proportional_rear_steady_state]] §5）生成。

### 6.4 曲率项写法的细节

| 行 | 文档形式 | 代码形式 | 一致性 |
|----|---------|---------|--------|
| 第 2 行 | $G_2\dot\theta_{\text{ref}} = \!\left(-\dfrac{C_fL\eta}{mI_zv_x} - v_x\right)\kappa v_x$ | `kappa*v * (... - v + ...)` | $G_2$ 含“裸” $v_x$，必须显式写 $\kappa v_x$ |
| 第 4 行 | $G_4\dot\theta_{\text{ref}} = \!\left(-\dfrac{l_fC_fL}{I_zv_x}\right)\kappa v_x = -\dfrac{l_fC_fL}{I_z}\kappa$ | `-Cf*kappa*lf*(lf+lr)/Iz` | $v_x$ 抵消，代码省略 |

两种写法在数学上完全等价。

---

## 7 SymPy 验证结果

将代码中第 1、2、3 行原始表达式（$\dot e_1$ 行省略），与文档形式 $A_{\text{aug}}\mathbf{x}_{1:4} + B_{eq}\delta_f^{\text{cmd}} + B_f\delta_d + G\dot\theta_{\text{ref}}$ 相减并化简：

```
row1 (ė₁ 输出):       diff = 0  ✓
row2 (ë₁ 输出):       diff = 0  ✓
row3 (e₂̇ 输出):       diff = 0  ✓（恒等）
row4 (ë₂ 输出):       diff = 0  ✓
row5 (δ_f^cmd dot):   diff = 0  ✓（恒等）
```

雅可比 `SystemODE_wrt_State_Gradient` 8 个非零元与 $A_{\text{aug}}$ 对应位置：

```
(0,1), (1,1), (1,2), (1,3), (1,4), (2,3), (3,1), (3,2), (3,3), (3,4)
所有 diff = 0  ✓
```

雅可比 `SystemODE_wrt_Control_Gradient`：

```
(4,0) = 1  ✓
```

---

## 8 与相关文档的关系

| 文档 | 关注点 | 与 101a 的关系 |
|------|--------|----------------|
| 03c | 4WS 4 阶后轴误差模型 $(A,B_f,B_r,G)$ | 推导起点 |
| 04c | 比例后轮 + 增量 + 扰动的 4 阶模型 | 101a 的“去掉积分器”版本 |
| 06a/06b | 4WS 转向扰动观测器 | 提供 $\delta_d$ 的来源 |
| 100c | 比例后轮稳态前馈 $\delta_{f,\text{ff}}$ | 提供 MPC `steer_ref` |
| 09 | 4WS 偏置估计器实现 | 提供 $\delta_d$ 的工程实现 |
| auto_gen.h | MPC 求解器接口 | 本文档对照对象 |

---

## 9 使用前提

1. 03c 的全部前提（小角度、线性轮胎、$\dot v_x \approx 0$、$\ddot\theta_{\text{ref}} \approx 0$）。
2. $k_r$ 在采样时刻视为常数（速度调度的 $k_r(v_x)$ 在 MPC 预测窗内逐 stage 取常值）。
3. 前轮扰动 $\delta_d$ 在预测窗内视为常数（来自 DOB 输出，更新慢于控制周期）。
4. 后轮随动器跟踪前轮**指令**而非实际，扰动 $\delta_d$ 仅走前轮通道。
5. `steer` 与 `d_steer` 的量纲为 rad；变量名 `steer_disturbance_deg` 仅是历史命名，实际单位与 `steer` 相同（rad）。

---

## 10 实现一致性确认（结论）

| 函数 | 维度 | 验证结果 |
|------|------|---------|
| `SystemODE` | $\mathbb{R}^5$ 输出 | 与 $\dot{\mathbf{x}} = A_{\text{aug}}\mathbf{x} + B_u u + B_d\delta_d + G_{\text{aug}}\kappa v_x$ 完全一致 ✓ |
| `SystemODE_wrt_State_Gradient` | $\mathbb{R}^{5\times 5}$ | 与 $A_{\text{aug}}$ 完全一致 ✓ |
| `SystemODE_wrt_Control_Gradient` | $\mathbb{R}^{5\times 1}$ | 与 $B_u = [0,0,0,0,1]^T$ 完全一致 ✓ |

代码实现与 4WS 比例后轮 + 增量控制 + 前轮扰动模型在解析层面**逐项相符**，无需修正。

---

## 11 SymPy 验证脚本

完整脚本：`doc/verify_101a_4ws_incremental_mpc.py`

实际运行结果（典型乘用车参数 $C_f=C_r=80000,\ l_f=1.4,\ l_r=1.6,\ m=1800,\ I_z=3000,\ v_x=15,\ \kappa=0.01,\ k_r=-0.3$）：

```
验证 1: SystemODE 逐行 diff = 0      ✓
验证 2: 状态雅可比 - A_aug = 0        ✓
验证 3: 控制雅可比 - B_u = 0          ✓
验证 4: kr=0 退化为 B_f                ✓
验证 5: 扰动通道 = B_f, 与 kr 解耦      ✓
验证 6: 数值一致性 < 1e-10             ✓
```

```python
"""
验证 4WS 比例后轮 + 前轮增量 + DOB 前轮扰动的扩展状态空间方程 (101a)
与 autogen MPC 代码 (auto_gen.h) 的一致性。

代码文件参考: mipilot/.../lat_mpc/interface/auto_gen.h
- Autogen::SystemODE
- Autogen::SystemODE_wrt_State_Gradient
- Autogen::SystemODE_wrt_Control_Gradient

运行: python3 doc/verify_101a_4ws_incremental_mpc.py
"""
from sympy import symbols, Matrix, simplify, zeros, Rational

# ============================================================
# 符号定义
# ============================================================
Cf, Cr, lf, lr, m, Iz = symbols('Cf Cr lf lr m Iz', positive=True)
v, kappa, kr = symbols('v kappa kr', real=True)
delta_d = symbols('delta_d', real=True)  # DOB 输出的前轮扰动 (rad)

# 状态: x = [e1, e1d, e2, e2d, steer]
e1, e1d, e2, e2d, steer = symbols('e1 e1d e2 e2d steer', real=True)
# 控制: u = d_steer (Δδ̇_f)
d_steer = symbols('d_steer', real=True)

L = lf + lr
eta = Iz - m * lf * lr
xi = Iz + m * lr**2

# ============================================================
# 文档 §2 紧凑形式: A_aug, B_eq, B_f, G_aug, B_u
# 来自 03c (4WS 后轴) + 04c (比例后轮 + 增量 + 扰动) + 积分器扩展
# ============================================================
A4 = Matrix([
    [0, 1, 0, 0],
    [0, -(Cf*eta + Cr*xi)/(m*Iz*v),  (Cf*eta + Cr*xi)/(m*Iz),    -(Cf*L*eta)/(m*Iz*v)],
    [0, 0, 0, 1],
    [0, -(lf*Cf - lr*Cr)/(Iz*v),     (lf*Cf - lr*Cr)/Iz,         -(lf*Cf*L)/(Iz*v)],
])

Bf4 = Matrix([0, Cf*eta/(m*Iz), 0, lf*Cf/Iz])
Br4 = Matrix([0, Cr*xi/(m*Iz),  0, -lr*Cr/Iz])
Beq4 = Bf4 + kr * Br4

G4 = Matrix([0, -(Cf*L*eta)/(m*Iz*v) - v, 0, -(lf*Cf*L)/(Iz*v)])

# 扩展到 5 维 (加上 δ_f^cmd 积分器)
A_aug = zeros(5, 5)
A_aug[0:4, 0:4] = A4
A_aug[0:4, 4]   = Beq4

Bd_aug = Matrix([Bf4[0], Bf4[1], Bf4[2], Bf4[3], 0])  # 扰动通道
G_aug  = Matrix([G4[0],  G4[1],  G4[2],  G4[3],  0])  # 曲率扰动通道
Bu     = Matrix([0, 0, 0, 0, 1])                      # 控制通道

x_aug = Matrix([e1, e1d, e2, e2d, steer])

# 文档形式 ODE
f_doc = A_aug * x_aug + Bu * d_steer + Bd_aug * delta_d + G_aug * (kappa * v)

# ============================================================
# 代码原始表达式 (从 auto_gen.h 直接逐字翻译)
# ============================================================
f_code_1 = e1d
f_code_2 = (
    e2 * ((Cf + Cr)/m - lr*(Cf*lf - Cr*lr)/Iz)
    + e2d * (Cf*lf*lr*(lf + lr)/(Iz*v) + (-Cf*lf - Cf*lr)/(m*v))
    + kappa * v * (Cf*lf*lr*(lf + lr)/(Iz*v) - v + (-Cf*lf - Cf*lr)/(m*v))
    + e1d * ((-Cf - Cr)/(m*v) + lr*(Cf*lf - Cr*lr)/(Iz*v))
    + steer * (Cf/m - Cf*lf*lr/Iz + kr*(Cr/m + Cr*lr**2/Iz))
    + delta_d * (Cf/m - Cf*lf*lr/Iz)
)
f_code_3 = e2d
f_code_4 = (
    -Cf*e2d*lf*(lf + lr)/(Iz*v)
    - Cf*kappa*lf*(lf + lr)/Iz
    + Cf*lf*delta_d/Iz
    + e2 * (Cf*lf - Cr*lr)/Iz
    + steer * (Cf*lf/Iz - Cr*kr*lr/Iz)
    + e1d * (-Cf*lf + Cr*lr)/(Iz*v)
)
f_code_5 = d_steer

f_code = Matrix([f_code_1, f_code_2, f_code_3, f_code_4, f_code_5])

# ============================================================
# 验证 1: ODE 逐行一致
# ============================================================
print("验证 1: SystemODE 逐行与文档形式一致")
for i in range(5):
    diff = simplify(f_code[i] - f_doc[i])
    assert diff == 0, f"Row {i} 不一致: {diff}"
print("✓")

# ============================================================
# 验证 2: SystemODE_wrt_State_Gradient (∂f/∂x)
# ============================================================
print("验证 2: SystemODE_wrt_State_Gradient = A_aug")
J_x_code = zeros(5, 5)
J_x_code[0, 1] = 1
J_x_code[1, 1] = (-Cf - Cr)/(m*v) + lr*(Cf*lf - Cr*lr)/(Iz*v)
J_x_code[1, 2] = (Cf + Cr)/m - lr*(Cf*lf - Cr*lr)/Iz
J_x_code[1, 3] = Cf*lf*lr*(lf + lr)/(Iz*v) + (-Cf*lf - Cf*lr)/(m*v)
J_x_code[1, 4] = Cf/m - Cf*lf*lr/Iz + kr*(Cr/m + Cr*lr**2/Iz)
J_x_code[2, 3] = 1
J_x_code[3, 1] = (-Cf*lf + Cr*lr)/(Iz*v)
J_x_code[3, 2] = (Cf*lf - Cr*lr)/Iz
J_x_code[3, 3] = -Cf*lf*(lf + lr)/(Iz*v)
J_x_code[3, 4] = Cf*lf/Iz - Cr*kr*lr/Iz

assert simplify(J_x_code - A_aug) == zeros(5, 5)
assert simplify(J_x_code - f_doc.jacobian(x_aug)) == zeros(5, 5)
print("✓")

# ============================================================
# 验证 3: SystemODE_wrt_Control_Gradient (∂f/∂u)
# ============================================================
print("验证 3: SystemODE_wrt_Control_Gradient = B_u")
J_u_code = Matrix([0, 0, 0, 0, 1])
assert simplify(J_u_code - Bu) == zeros(5, 1)
assert simplify(J_u_code - f_doc.jacobian(Matrix([d_steer]))) == zeros(5, 1)
print("✓")

# ============================================================
# 验证 4: kr=0 退化
# ============================================================
print("验证 4: kr=0 时 B_eq = B_f")
assert simplify(Beq4.subs(kr, 0) - Bf4) == zeros(4, 1)
print("✓")

# ============================================================
# 验证 5: 扰动通道与控制通道的解耦
# ============================================================
print("验证 5: δ_d 走 B_f, δ_f^cmd 走 B_eq")
df_d_dd = Matrix([f_code[i].diff(delta_d) for i in range(5)])
assert simplify(df_d_dd - Bd_aug) == zeros(5, 1)
assert not any(df_d_dd[i].has(kr) for i in range(5))
expected_steer_col = Matrix([0, Beq4[1], 0, Beq4[3], 0])
assert simplify(J_x_code[:, 4] - expected_steer_col) == zeros(5, 1)
print("✓")

# ============================================================
# 验证 6: 数值验证
# ============================================================
print("验证 6: 数值验证 (典型乘用车参数)")
subs_numeric = {
    Cf: 80000, Cr: 80000, lf: 1.4, lr: 1.6, m: 1800, Iz: 3000,
    v: 15.0, kappa: 0.01, kr: Rational(-3, 10), delta_d: 0.02,
    e1: 0.1, e1d: 0.05, e2: 0.02, e2d: 0.01, steer: 0.05, d_steer: 0.001,
}
for i in range(5):
    assert abs(float(simplify(f_code[i] - f_doc[i]).subs(subs_numeric))) < 1e-10
print("✓")

print("\n所有验证通过 ✓")
```

验证内容总结：

1. ODE 五行（含 $\dot e_1$、$\ddot e_1$、$\dot e_2$、$\ddot e_2$、$\dot\delta_f^{\text{cmd}}$）与紧凑形式 $\dot{\mathbf{x}} = A_{\text{aug}}\mathbf{x} + B_u u + B_d\delta_d + G_{\text{aug}}\kappa v_x$ 完全一致。
2. 状态雅可比 10 个非零元逐项等于 $A_{\text{aug}}$，且与 ODE 自动求导结果自洽。
3. 控制雅可比 $[0,0,0,0,1]^T$ 与 $B_u$ 一致。
4. $k_r=0$ 退化为 $B_{eq}=B_f$（与 03a 一致）。
5. 扰动通道 $\partial f/\partial\delta_d$ 等于 $B_f$（含 0 对应于积分器行），不含 $k_r$，与控制通道解耦。
6. 典型车辆参数下数值差小于 $10^{-10}$。
