# 4WS 比例后轮随动横向 MPC 的代码对照公式推导

> 本文以工程代码为唯一锚点，按代码中**实际定义的状态、控制、扰动、在线数据**整理 4WS 横向 MPC 的完整公式推导。方案为**后轮按比例随动** $\delta_r = k_r\,\delta_f$。
>
> 对照代码：
> - `mipilot/.../lat_mpc/interface/auto_gen.cc`（`SystemODE` / 雅可比 / cost / 约束）
> - `mipilot/.../lat_mpc/interface/config_manager.h`（状态/控制/参数结构体定义）
> - `mipilot/.../lat_mpc/lat_mpc_controller.cc`（误差构造、参考生成、$k_r$ 调度、命令输出）
>
> 状态空间推导起点见 [[03c_error_rear_4ws]]；稳态前馈闭式解见 [[100c_4ws_proportional_rear_steady_state]]；纯前轮特例见 [[100a_rear_axle_steady_state_feedforward]]；ODE 与 autogen 的逐项对照另见 [[101a_4ws_proportional_rear_incremental_mpc]]。本文在 101a 基础上把**参考生成层、代价函数层、$k_r$ 调度层、命令输出层**一并纳入，形成端到端的代码对照文档。

---

## 1 代码中的变量定义（唯一事实来源）

### 1.1 状态向量（`config_manager.h::OdeState`，`state_num_ = 5`）

| 索引 | 代码字段 | 数学符号 | 含义 | 单位 |
|------|----------|----------|------|------|
| `state_input(0,0)` | `lat_err` | $e_1$ | 后轴中心横向误差 | m |
| `state_input(0,1)` | `lat_err_rate` | $\dot e_1$ | 横向误差变化率 | m/s |
| `state_input(0,2)` | `heading_err` | $e_2$ | 航向误差（含稳态前馈补偿） | rad |
| `state_input(0,3)` | `heading_err_rate` | $\dot e_2$ | 航向误差变化率 | rad/s |
| `state_input(0,4)` | `steer` | $\delta_f^{\text{cmd}}$ | 前轮指令角（积分器状态） | rad |

$$
\mathbf{x} = \big[e_1,\;\dot e_1,\;e_2,\;\dot e_2,\;\delta_f^{\text{cmd}}\big]^T \in \mathbb{R}^5
$$

### 1.2 控制量（`config_manager.h::OdeControl`，`control_num_ = 1`）

| 代码字段 | 数学符号 | 含义 | 单位 |
|----------|----------|------|------|
| `d_steer` = `control_input(0,0)` | $\Delta\dot\delta_f$ | 前轮指令角变化率 | rad/s |

$$
u = \Delta\dot\delta_f \in \mathbb{R}
$$

### 1.3 在线数据：动态参数（`DynamicParam`，逐 stage 取值）

| 代码字段 | 数学符号 | 含义 |
|----------|----------|------|
| `v` | $v_x$ | 纵向速度（参考速度，逐 stage） |
| `kappa` | $\kappa$ | 参考路径曲率 |
| `steer_disturbance_deg` | $\delta_d$ | 前轮转角扰动（DOB 偏置，单位实为 rad，名称带 `_deg` 为历史遗留） |
| `kr` | $k_r$ | 后轮随动比例系数 |
| `steer_ref` | $\delta_{f,\text{ff}}$ | 前轮指令角参考（= 前馈 `delta_ref`） |
| `d_steer_ref` | $\Delta\dot\delta_{f,\text{ff}}$ | 控制量参考（代码置 0） |
| `heading_err_ref` | $e_{2,ss}$ | 航向误差参考（= `beta_ref`） |
| `lat_err_ref`, `lat_err_rate_ref`, `heading_err_rate_ref` | — | 误差参考（代码置 0） |

### 1.4 在线数据：静态参数（`StaticParam`）

| 代码字段 | 数学符号 | 含义 |
|----------|----------|------|
| `cf`, `cr` | $C_f,\;C_r$ | 前/后轴侧偏刚度 |
| `mass` | $m$ | 整车质量 |
| `lf`, `lr` | $l_f,\;l_r$ | 质心到前/后轴距离（$L = l_f + l_r$ = `wheel_base`） |
| `iz` | $I_z$ | 横摆转动惯量 |
| `*_init` | — | 初始状态（`InitPointEqlConstraint`） |

> **关键设计**：扰动 $\delta_d$ 在 `SetSolverDynamicParam` 中被置 0（`steer_disturbance_deg(0,i) = 0.0`），即当前版本未把 DOB 偏置接入 MPC 预测；但 `SystemODE` 中保留了 $\delta_d$ 通道，结构上随时可启用。本文推导保留 $\delta_d$ 以反映代码的完整结构。

### 1.5 中间记号

$$
L = l_f + l_r, \qquad \eta = I_z - ml_fl_r, \qquad \xi = I_z + ml_r^2, \qquad K_{us} = \frac{m}{L}\!\left(\frac{l_r}{C_f} - \frac{l_f}{C_r}\right)
$$

---

## 2 后轮比例随动方案

### 2.1 随动律

后轮转角与前轮指令角成比例：

$$
\delta_r = k_r\,\delta_f^{\text{cmd}}
$$

$k_r$ 由速度与前轮转角调度（见第 7 节 `CalcKr` / `UpdateReference`），在单个采样时刻、单个 stage 内视为常数。

### 2.2 实际作用前轮转角

$$
\delta_f^{\text{act}} = \delta_f^{\text{cmd}} + \delta_d
$$

后轮随动器跟踪的是**指令** $\delta_f^{\text{cmd}}$，不感知前轮机械偏置 $\delta_d$，故 $\delta_d$ 仅走前轮通道，不被 $k_r$ 缩放。

### 2.3 等效输入矩阵

将 $\delta_f^{\text{act}}$、$\delta_r$ 代入 03c 的 $\dot{\mathbf{x}}_{1:4} = A\mathbf{x}_{1:4} + B_f\delta_f^{\text{act}} + B_r\delta_r + G\dot\theta_{\text{ref}}$：

$$
\dot{\mathbf{x}}_{1:4} = A\,\mathbf{x}_{1:4} + \underbrace{(B_f + k_r B_r)}_{B_{eq}}\,\delta_f^{\text{cmd}} + B_f\,\delta_d + G\,\dot\theta_{\text{ref}}
$$

$$
B_{eq} = B_f + k_r B_r = \begin{bmatrix} 0 \\[4pt] \dfrac{C_f\eta + k_r C_r\xi}{mI_z} \\[6pt] 0 \\[4pt] \dfrac{l_fC_f - k_r l_rC_r}{I_z} \end{bmatrix}
\;\;\xleftrightarrow{\text{化简钥匙}}\;\;
\begin{bmatrix} 0 \\[4pt] \dfrac{C_f}{m} - \dfrac{C_f l_f l_r}{I_z} + k_r\!\left(\dfrac{C_r}{m} + \dfrac{C_r l_r^2}{I_z}\right) \\[8pt] 0 \\[4pt] \dfrac{l_f C_f - k_r l_r C_r}{I_z} \end{bmatrix}
$$

右侧展开形式与代码 `auto_gen.cc` 中 `steer` 的系数逐项一致。化简钥匙：$\dfrac{1}{m} - \dfrac{l_fl_r}{I_z} = \dfrac{\eta}{mI_z}$，$\dfrac{1}{m} + \dfrac{l_r^2}{I_z} = \dfrac{\xi}{mI_z}$。

---

## 3 扩展状态空间（与 `SystemODE` 一一对应）

状态、控制、扰动、曲率扰动向量明确写为：

$$
\mathbf{x} = \begin{bmatrix} x_1 \\ x_2 \\ x_3 \\ x_4 \\ x_5 \end{bmatrix}
= \begin{bmatrix} e_1 \\ \dot e_1 \\ e_2 \\ \dot e_2 \\ \delta_f^{\text{cmd}} \end{bmatrix}
= \begin{bmatrix} \texttt{lat\_err} \\ \texttt{lat\_err\_rate} \\ \texttt{heading\_err} \\ \texttt{heading\_err\_rate} \\ \texttt{steer} \end{bmatrix},
\qquad
u = \Delta\dot\delta_f = \texttt{d\_steer}
$$

$$
\delta_d = \texttt{steer\_disturbance\_deg}\ (\text{前轮扰动}),
\qquad
\dot\theta_{\text{ref}} = \kappa\,v_x\ (\text{曲率扰动})
$$

### 3.1 前轮指令积分器

控制量是转角变化率，故第 5 个状态满足：

$$
\dot\delta_f^{\text{cmd}} = \Delta\dot\delta_f
$$

### 3.2 5 维 ODE

$$
\boxed{\;
\dot{\mathbf{x}} =
\underbrace{\begin{bmatrix} A & B_{eq} \\ \mathbf{0}_{1\times4} & 0 \end{bmatrix}}_{A_{\text{aug}}}\mathbf{x}
+ \underbrace{\begin{bmatrix} \mathbf{0}_{4\times1} \\ 1 \end{bmatrix}}_{B_u}\Delta\dot\delta_f
+ \underbrace{\begin{bmatrix} B_f \\ 0 \end{bmatrix}}_{B_d}\delta_d
+ \underbrace{\begin{bmatrix} G \\ 0 \end{bmatrix}}_{G_{\text{aug}}}\dot\theta_{\text{ref}}
\;}
$$

其中 $\dot\theta_{\text{ref}} = \kappa v_x$。$A_{\text{aug}}$ 第 5 列即 $B_{eq}$——这是后轮随动假设进入扩展状态空间的入口。

### 3.3 逐行展开（对照 `auto_gen.cc::SystemODE` 的 `return_mat`）

**第 1 行** `return_mat(0,0) = lat_err_rate`：

$$
\dot{\mathbf{x}}_1 = \dot e_1
$$

**第 2 行** `return_mat(1,0)`（$\ddot e_1$，紧凑形式）：

$$
\ddot e_1 = -\frac{C_f\eta + C_r\xi}{mI_zv_x}\dot e_1 + \frac{C_f\eta + C_r\xi}{mI_z}e_2 - \frac{C_fL\eta}{mI_zv_x}\dot e_2 + \frac{C_f\eta + k_rC_r\xi}{mI_z}\delta_f^{\text{cmd}} + \frac{C_f\eta}{mI_z}\delta_d + \left(-\frac{C_fL\eta}{mI_zv_x} - v_x\right)\dot\theta_{\text{ref}}
$$

**第 3 行** `return_mat(2,0) = heading_err_rate`：

$$
\dot{\mathbf{x}}_3 = \dot e_2
$$

**第 4 行** `return_mat(3,0)`（$\ddot e_2$，紧凑形式）：

$$
\ddot e_2 = -\frac{l_fC_f - l_rC_r}{I_zv_x}\dot e_1 + \frac{l_fC_f - l_rC_r}{I_z}e_2 - \frac{l_fC_fL}{I_zv_x}\dot e_2 + \frac{l_fC_f - k_r l_rC_r}{I_z}\delta_f^{\text{cmd}} + \frac{l_fC_f}{I_z}\delta_d - \frac{l_fC_fL}{I_z}\kappa
$$

> 代码第 4 行曲率项写作 `-cf*kappa*lf*(lf+lr)/iz`（不带 $v_x$）。利用 $\dot\theta_{\text{ref}} = \kappa v_x$，与 $G_4\dot\theta_{\text{ref}} = -\dfrac{l_fC_fL}{I_zv_x}\kappa v_x = -\dfrac{l_fC_fL}{I_z}\kappa$ 等价。第 2 行因 $G_2$ 含“裸” $v_x$，代码保留 `kappa*v` 乘法因子。

**第 5 行** `return_mat(4,0) = d_steer`：

$$
\dot\delta_f^{\text{cmd}} = \Delta\dot\delta_f
$$

### 3.4 代码原始（展开）形式

代码 `auto_gen.cc::SystemODE` 不使用 $\eta,\xi,L$ 中间变量，而是直接以 $C_f, C_r, l_f, l_r, m, I_z, v_x$ 展开。下面写出与代码**逐字一致**的完整表达式。

**第 2 行**（$\ddot e_1$，`return_mat(1,0)`）：

$$
\begin{aligned}
\ddot e_1 &= \left[-\frac{C_f + C_r}{m v_x} + \frac{l_r(C_f l_f - C_r l_r)}{I_z v_x}\right]\dot e_1
+ \left[\frac{C_f + C_r}{m} - \frac{l_r(C_f l_f - C_r l_r)}{I_z}\right] e_2 \\[6pt]
&\quad + \left[\frac{C_f l_f l_r (l_f+l_r)}{I_z v_x} - \frac{C_f(l_f+l_r)}{m v_x}\right]\dot e_2 \\[6pt]
&\quad + \left[\frac{C_f}{m} - \frac{C_f l_f l_r}{I_z} + k_r\!\left(\frac{C_r}{m} + \frac{C_r l_r^2}{I_z}\right)\right]\delta_f^{\text{cmd}}
+ \left[\frac{C_f}{m} - \frac{C_f l_f l_r}{I_z}\right]\delta_d \\[6pt]
&\quad + \left[\frac{C_f l_f l_r (l_f+l_r)}{I_z v_x} - v_x - \frac{C_f(l_f+l_r)}{m v_x}\right]\kappa v_x
\end{aligned}
$$

**第 4 行**（$\ddot e_2$，`return_mat(3,0)`）：

$$
\begin{aligned}
\ddot e_2 &= \frac{-C_f l_f + C_r l_r}{I_z v_x}\,\dot e_1
+ \frac{C_f l_f - C_r l_r}{I_z}\,e_2
- \frac{C_f l_f (l_f + l_r)}{I_z v_x}\,\dot e_2 \\[6pt]
&\quad + \left[\frac{C_f l_f}{I_z} - \frac{k_r C_r l_r}{I_z}\right]\delta_f^{\text{cmd}}
+ \frac{C_f l_f}{I_z}\,\delta_d
- \frac{C_f l_f (l_f + l_r)}{I_z}\,\kappa
\end{aligned}
$$

> 第 4 行曲率项代码写作 `-cf*kappa*lf*(lf+lr)/iz`（乘 $\kappa$，不带 $v_x$）；第 2 行曲率项代码写作 `kappa*v*(...)`（乘 $\kappa v_x$）。差异源于 $G_2$ 含“裸” $v_x$ 项，而 $G_4$ 整体含 $1/v_x$ 与 $\kappa v_x$ 相约。

**展开 ↔ 紧凑系数对照**（化简钥匙：$\dfrac{1}{m} - \dfrac{l_fl_r}{I_z} = \dfrac{\eta}{mI_z}$，$\dfrac{1}{m} + \dfrac{l_r^2}{I_z} = \dfrac{\xi}{mI_z}$；每项由 SymPy 验证，§9）：

| 行 | 代码项 | 展开系数 | 紧凑形式 |
|----|--------|---------|---------|
| 2 | $\dot e_1$ (`lat_err_rate`) | $-\dfrac{C_f+C_r}{mv_x} + \dfrac{l_r(C_fl_f - C_rl_r)}{I_zv_x}$ | $A_{22} = -\dfrac{C_f\eta+C_r\xi}{mI_zv_x}$ |
| 2 | $e_2$ (`heading_err`) | $\dfrac{C_f+C_r}{m} - \dfrac{l_r(C_fl_f - C_rl_r)}{I_z}$ | $A_{23} = \dfrac{C_f\eta+C_r\xi}{mI_z}$ |
| 2 | $\dot e_2$ (`heading_err_rate`) | $\dfrac{C_fl_fl_rL}{I_zv_x} - \dfrac{C_fL}{mv_x}$ | $A_{24} = -\dfrac{C_fL\eta}{mI_zv_x}$ |
| 2 | $\kappa v_x$ (`kappa*v`) | $\dfrac{C_fl_fl_rL}{I_zv_x} - v_x - \dfrac{C_fL}{mv_x}$ | $G_2 = -\dfrac{C_fL\eta}{mI_zv_x} - v_x$ |
| 2 | $\delta_f^{\text{cmd}}$ (`steer`) | $\dfrac{C_f}{m} - \dfrac{C_fl_fl_r}{I_z} + k_r\!\left(\dfrac{C_r}{m} + \dfrac{C_rl_r^2}{I_z}\right)$ | $B_{eq,2} = \dfrac{C_f\eta + k_rC_r\xi}{mI_z}$ |
| 2 | $\delta_d$ (`steer_disturbance`) | $\dfrac{C_f}{m} - \dfrac{C_fl_fl_r}{I_z}$ | $B_{f,2} = \dfrac{C_f\eta}{mI_z}$ |
| 4 | $\dot e_1$ (`lat_err_rate`) | $\dfrac{-C_fl_f + C_rl_r}{I_zv_x}$ | $A_{42} = -\dfrac{l_fC_f - l_rC_r}{I_zv_x}$ |
| 4 | $e_2$ (`heading_err`) | $\dfrac{C_fl_f - C_rl_r}{I_z}$ | $A_{43} = \dfrac{l_fC_f - l_rC_r}{I_z}$ |
| 4 | $\dot e_2$ (`heading_err_rate`) | $-\dfrac{C_fl_f(l_f+l_r)}{I_zv_x}$ | $A_{44} = -\dfrac{l_fC_fL}{I_zv_x}$ |
| 4 | $\delta_f^{\text{cmd}}$ (`steer`) | $\dfrac{C_fl_f}{I_z} - \dfrac{k_rC_rl_r}{I_z}$ | $B_{eq,4} = \dfrac{l_fC_f - k_rl_rC_r}{I_z}$ |
| 4 | $\delta_d$ (`steer_disturbance`) | $\dfrac{C_fl_f}{I_z}$ | $B_{f,4} = \dfrac{l_fC_f}{I_z}$ |
| 4 | $\kappa$ (`kappa`) | $-\dfrac{C_fl_f(l_f+l_r)}{I_z}$ | $G_4\cdot\kappa v_x = -\dfrac{l_fC_fL}{I_z}\kappa$（$v_x$ 抵消） |

---

## 4 雅可比矩阵（与 `SystemODE_wrt_*_Gradient` 对应）

### 4.1 状态雅可比 $A_{\text{aug}}$（`SystemODE_wrt_State_Gradient`）

$$
A_{\text{aug}} = \begin{bmatrix}
0 & 1 & 0 & 0 & 0 \\[6pt]
0 & -\dfrac{C_f\eta + C_r\xi}{mI_zv_x} & \dfrac{C_f\eta + C_r\xi}{mI_z} & -\dfrac{C_fL\eta}{mI_zv_x} & \dfrac{C_f\eta + k_rC_r\xi}{mI_z} \\[10pt]
0 & 0 & 0 & 1 & 0 \\[6pt]
0 & -\dfrac{l_fC_f - l_rC_r}{I_zv_x} & \dfrac{l_fC_f - l_rC_r}{I_z} & -\dfrac{l_fC_fL}{I_zv_x} & \dfrac{l_fC_f - k_rl_rC_r}{I_z} \\[10pt]
0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

代码非零元 `(0,1)=1`、`(1,1)`、`(1,2)`、`(1,3)`、`(1,4)`、`(2,3)=1`、`(3,1)`、`(3,2)`、`(3,3)`、`(3,4)` 与上式逐项一致。注意扰动通道 $B_d$ 与曲率通道 $G_{\text{aug}}$ 作为 online data，**不进入**雅可比。

### 4.2 控制雅可比（`SystemODE_wrt_Control_Gradient`）

$$
B_u = [0,\;0,\;0,\;0,\;1]^T \quad\Leftrightarrow\quad \texttt{return\_mat(4,0) = 1}
$$

---

## 5 误差状态构造（`GetErrorState` 逐行解读）

代码在喂给 MPC 之前，先把当前/预测点的四个误差量算好。这一步把 100c 的稳态前馈"打包"进误差信号。

```cpp
path_true_heading = wrap(path_point.theta + heading_bias);
lateral_error     = cos(path_true_heading)*dy - sin(path_true_heading)*dx;
theta_rear = kappa/(1-kr) * ( -(lf+lr)*kr
              + mass*v*v*(lf*cf_used - kr*lr*cr)/(cf_used*cr*wheel_base) );
heading_error      = wrap(theta - path_true_heading + theta_rear);
lateral_error_rate = v * sin(wrap(theta - path_true_heading));
heading_error_rate = yaw_rate - kappa*v;
```

### 5.1 路径切向（含 bias）

$$
\theta_{\text{ref}}^{\text{true}} = \mathrm{wrap}(\theta_{\text{ref}} + b_\theta)
$$

`heading_bias` 是标定/估计层补出的航向偏置 $b_\theta$，后续投影与 $e_2$ 都基于修正后的切向。

### 5.2 横向误差 $e_1$

$$
e_1 = \cos\theta_{\text{ref}}^{\text{true}}\,\Delta y - \sin\theta_{\text{ref}}^{\text{true}}\,\Delta x, \qquad (\Delta x,\Delta y) = (x_v - x_r,\;y_v - y_r)
$$

即把位置偏差投影到路径法向 $\hat n = (-\sin\theta,\cos\theta)$，车体在路径左侧时 $e_1 > 0$，与 03c 后轴 Frenet 约定一致。

### 5.3 稳态后轴航向偏差 `theta_rear`（= 100c 的 $e_{2,ss}$）

$$
\theta_{\text{rear}} = \frac{\kappa}{1-k_r}\!\left[-k_r L + \frac{mv_x^2(l_fC_f - k_r l_rC_r)}{C_fC_rL}\right]
$$

把 100c §4.2 的稳态结果

$$
e_{2,ss} = \frac{mv_x^2(l_fC_f - k_r l_rC_r) - k_rC_fC_rL^2}{C_fC_rLR(1-k_r)}
$$

提出 $\kappa = 1/R$，恰好得到代码形式——**逐项一致**（§9 验证 3）。物理上这是稳态圆弧上后轴非零侧偏角导致的车身航向与路径切向的固定夹角。

### 5.4 航向误差 $e_2$（扣除稳态前馈）

$$
e_2^{\text{mpc}} = \mathrm{wrap}\big(\underbrace{\psi - \theta_{\text{ref}}^{\text{true}}}_{e_2^{\text{raw}}} + \theta_{\text{rear}}\big)
$$

稳态圆弧上 $e_2^{\text{raw}} \approx -\theta_{\text{rear}}$，故 $e_2^{\text{mpc}} \approx 0$，MPC 只对偏离稳态的瞬态做反馈。

### 5.5 横向误差变化率 $\dot e_1$

$$
\dot e_1^{\text{code}} = v_x\sin(e_2^{\text{raw}})
$$

这是 03c 运动学 $\dot e_1 = v_{yr} + v_x e_2$ 的简化：保留大角度 $\sin$，但**舍弃后轴侧向速度** $v_{yr}$（忽略后轴 sideslip）。优点是只依赖 `theta`、`speed`，不需 IMU 估计 $v_y$；代价是高速大侧偏下 $\dot e_1$ 有偏差。稳态 $\dot e_1 \equiv 0$，故无需补偿稳态项。

### 5.6 航向误差变化率 $\dot e_2$

$$
\dot e_2^{\text{code}} = r - \kappa v_x
$$

严格对应 03c 的 $\dot e_2 = r - \dot\theta_{\text{ref}}$，无近似。

### 5.7 实现量对照表

| 代码量 | 数学含义 | 处理方式 |
|--------|----------|---------|
| `path_true_heading` | $\theta_{\text{ref}} + b_\theta$ | 含 bias 修正 |
| `lateral_error` | $e_1 = -\Delta x\sin\theta + \Delta y\cos\theta$ | 投影法（与 03c 严格一致） |
| `theta_rear` | $e_{2,ss}$（100c §4.2） | 稳态闭式前馈 |
| `heading_error` | $(\psi - \theta_{\text{ref}}) + e_{2,ss}$ | 扣稳态后给 MPC |
| `lateral_error_rate` | $v_x\sin(\psi - \theta_{\text{ref}})$ | 忽略 $v_{yr}$ |
| `heading_error_rate` | $r - \kappa v_x$ | 严格运动学 |

---

## 6 参考生成（`UpdateReference` 逐项解读）

每个 stage $i$ 计算速度、曲率、$k_r$、前馈转角 `delta_ref` 和航向参考 `beta_ref`。

### 6.1 $k_r$ 的两步迭代调度

```cpp
delta_2ws       = (1 + kv*v*v) * wheel_base * kappa;       // 2WS 等效前轮角
kr_i            = clamp(interp(v,|delta_2ws|)/|delta_2ws|, -0.99, 0.99);
delta_ref_est   = |delta_2ws/(1-kr_i)|;                    // 用 kr 修正后的幅值
kr_i            = clamp(interp(v,delta_ref_est)/delta_ref_est, -0.99, 0.99);  // 再查一次
delta_ref       = delta_2ws/(1-kr_i);
```

- `delta_2ws` $= (1 + k_v v_x^2)L\kappa$ 是不含后轮的 2WS 经典前轮前馈（$k_v$ = `lat_out.kv`，即 2WS 不足转向系数）。
- 第一次查表用 2WS 角度估 $k_r$，再用 $\delta_f = \delta_{2ws}/(1-k_r)$ 的幅值二次查表，使 $k_r$ 与实际前轮幅值自洽（一步不动点迭代）。
- `fr_str_interp_` 是 `front_rear_str_scheduler`（按速度、前轮角插值出后轮角）的二维查表，$k_r = \delta_r/|\delta_f|$，clamp 到 $[-0.99,\,0.99]$。

### 6.2 前馈前轮角 `delta_ref`（= 100c 的 $\delta_f^{ss}$）

$$
\texttt{delta\_ref} = \frac{\delta_{2ws}}{1-k_r} = \frac{(1 + k_v v_x^2)L\kappa}{1-k_r}
$$

当 $k_v = K_{us}/L$ 时，与 100c §6 的稳态前轮角

$$
\delta_f^{ss} = \frac{L + K_{us}v_x^2}{(1-k_r)R} = \frac{(L + K_{us}v_x^2)\kappa}{1-k_r}
$$

**逐项一致**（§9 验证 4）。该值经 `steer_ref` 进入 stage cost，是 MPC 跟踪的前轮角参考。

### 6.3 等效转向不变量

$$
(1-k_r)\,\texttt{delta\_ref} = (L + K_{us}v_x^2)\kappa = \frac{L + K_{us}v_x^2}{R}
$$

即前轮减后轮的等效转向角与后轮分配方式 $k_r$ 无关——稳态圆弧跟踪所需的等效转向角是物理不变量（§9 验证 4）。

### 6.4 航向参考 `beta_ref`

代码 `beta_ref[i]` 与 §5.3 的 `theta_rear` 同式，即 100c 的 $e_{2,ss}$，经 `heading_err_ref` 进入 stage/terminal cost。

> **注意**：§5.4 已经把 $\theta_{\text{rear}}$ 加进了状态 `heading_err`，而 cost 又用 `heading_err_ref = beta_ref` 做参考。由于 `GetErrorState` 用当前点 $k_r$、`UpdateReference` 用各预测点 $k_r$，二者在曲率/速度变化的预测窗内不完全抵消，残差正是 MPC 需要压低的瞬态航向偏差。`SetDebugInfo` 中的 `e2_cost` 用 `(e2_state - beta_ref)^2` 计量，印证了这一点。

---

## 7 $k_r$ 调度与命令输出

### 7.1 实时 `CalcKr`（命令侧）

```cpp
delta_f  = StrAngToDeltaAngle(str_ang);          // 方向盘角 -> 前轮角
delta_r  = fr_str_interp_.Interpolate({speed, |delta_f|});
kr       = clamp(delta_r/|delta_f|, -0.99, 0.99);   // |delta_f|<1e-6 时取 0
```

非 4WS 车型直接返回 $k_r = 0$，全文退化为 100a/03a 的纯前轮模型。

### 7.2 后轮命令输出（`LatCommandProcess`）

求解得到前轮请求 `str_request` 后，后轮请求由查表生成并带上前轮符号：

$$
\delta_r^{\text{req}} = \mathrm{sign}(\texttt{str\_request})\cdot \texttt{interp}(v_x,\,|\delta_f^{\text{req}}|)
$$

非 4WS 时 `rear_str_request = 0`。这里后轮角直接由前后轮调度表生成，与预测层用的 $k_r\delta_f$ 在调度表自洽时等价。

---

## 8 代价函数与约束（`StageCostItem` / `TerminalCost` / 约束）

### 8.1 Stage cost（`StageCostItem_Expression`，4 项）

$$
J_{\text{stage}} = \big\|e_1 - 0\big\|_{w_{e_1}}^2 + \big\|e_2 - e_{2,ss}\big\|_{w_{e_2}}^2 + \big\|\delta_f^{\text{cmd}} - \delta_{f,\text{ff}}\big\|_{w_\delta}^2 + \big\|\Delta\dot\delta_f - 0\big\|_{w_{\Delta\delta}}^2
$$

对应代码（`lat_err_ref = 0`，`heading_err_ref = beta_ref`，`steer_ref = delta_ref`，`d_steer_ref = 0`）：

| cost 分量 | 残差 | 参考来源 |
|-----------|------|---------|
| `lat_err - lat_err_ref` | $e_1$ | 0 |
| `heading_err - heading_err_ref` | $e_2 - e_{2,ss}$ | `beta_ref`（100c） |
| `steer - steer_ref` | $\delta_f^{\text{cmd}} - \delta_{f,\text{ff}}$ | `delta_ref`（100c §6） |
| `d_steer - d_steer_ref` | $\Delta\dot\delta_f$ | 0 |

权重 $w$ 由 `UpdateWeights` 按速度、横向误差、曲率调度（`weight_stage_cost` 对角阵）。

### 8.2 Terminal cost（`TerminalCost_Expression`，5 项）

$$
J_{\text{term}} = \big\|e_1\big\|^2 + \big\|\dot e_1\big\|^2 + \big\|e_2 - e_{2,ss}\big\|^2 + \big\|\dot e_2\big\|^2 + \big\|\delta_f^{\text{cmd}} - \delta_{f,\text{ff}}\big\|^2
$$

终端权重由 `UpdateTerminalMatrices`/`BuildTerminalWeight` 解离散 Riccati 反向递推得到（可为稠密阵），用末段 stage 的 $k_r$ 构造 $A_{\text{aug}}$、$B_u$。`lat_err_rate_ref`、`heading_err_rate_ref` 代码置 0。

> **设计哲学**：前馈/参考层（`delta_ref`、`beta_ref`，来自 100c）吸收所有稳态分量，MPC 只对偏离稳态的**瞬态**做反馈。这与 101a §6.3"代码用绝对状态、靠 `steer_ref`/`d_steer_ref` 围绕前馈跟踪"等价于增量化推导一致。

### 8.3 约束（`InitMpcSolver` / `SetSolverStaticParam`）

- 初始状态等式约束 `InitPointEqlConstraint`：$\mathbf{x}_0 = [e_1,\dot e_1,e_2,\dot e_2,\delta_f]_{\text{init}}$。
- 状态盒约束：仅 $\delta_f^{\text{cmd}}$ 有界 $|\delta_f^{\text{cmd}}| \le \delta_{\max}$（由 `max_str` 映射），其余四维放开。
- 控制盒约束：$|\Delta\dot\delta_f| \le \dot\delta_{\max}$（由 `max_str_rate` 映射）。

---

## 9 SymPy 验证

验证脚本：`doc/verify_102_4ws_proportional_rear_mpc.py`

验证内容与结果（典型乘用车参数 $C_f=C_r=80000,\ l_f=1.4,\ l_r=1.6,\ m=1800,\ I_z=3000,\ v_x=15,\ \kappa=0.01,\ k_r=-0.3$）：

```
验证 1: SystemODE 五行与 5 维扩展状态空间一致          ✓ (逐行 diff = 0)
验证 2: 状态/控制雅可比与 A_aug / B_u 一致              ✓
验证 3: 代码 beta_ref(theta_rear) == 100c 的 e2_ss      ✓ (diff = 0)
验证 4: 代码 delta_ref == 100c 的 delta_f^ss (kv=Kus/L) ✓ (diff = 0)
        等效转向 (1-kr)*delta_ref = (L+Kus v^2)kappa     ✓
验证 5: kr=0 退化为纯前轮 (100a)                        ✓
验证 6: 数值一致性 < 1e-10                              ✓
```

要点：

1. `SystemODE` 五行（$\dot e_1,\ \ddot e_1,\ \dot e_2,\ \ddot e_2,\ \dot\delta_f^{\text{cmd}}$）与 $\dot{\mathbf{x}} = A_{\text{aug}}\mathbf{x} + B_u u + B_d\delta_d + G_{\text{aug}}\kappa v_x$ 逐项相符。
2. 雅可比与 $A_{\text{aug}}$、$B_u$ 一致，且与 ODE 自动求导自洽。
3. 误差构造层的 `theta_rear`/`beta_ref` 等于 100c 稳态航向误差 $e_{2,ss}$。
4. 参考生成层的 `delta_ref` 在 $k_v = K_{us}/L$ 时等于 100c 稳态前轮角 $\delta_f^{ss}$；等效转向角 $(1-k_r)\delta_f^{ss} = (L+K_{us}v_x^2)\kappa$ 与 $k_r$ 无关。
5. $k_r=0$ 时整体退化为纯前轮转向（100a / 03a）。

---

## 10 与相关文档的关系

| 文档 | 关注点 | 与 102 的关系 |
|------|--------|---------------|
| 03c | 4WS 4 阶后轴误差模型 $(A,B_f,B_r,G)$ | 推导起点 |
| 100a | 纯前轮稳态前馈 | $k_r=0$ 特例 |
| 100c | 比例后轮稳态前馈 $e_{2,ss}$、$\delta_f^{ss}$ | 提供 `beta_ref`、`delta_ref` 闭式解 |
| 101a | 5 维 ODE 与 autogen 逐项对照 | 102 的 ODE 层来源 |
| 06a/09 | 4WS 转向扰动观测器/偏置估计 | $\delta_d$ 通道的来源（当前代码置 0） |
| `auto_gen.cc` / `lat_mpc_controller.cc` | MPC 求解器与控制器实现 | 本文对照对象 |

---

## 11 使用前提

1. 03c 全部前提（小角度、线性轮胎、$\dot v_x \approx 0$、$\ddot\theta_{\text{ref}} \approx 0$）。
2. $k_r$ 在采样时刻与单个 stage 内视为常数；预测窗内逐 stage 由 `UpdateReference` 取 $k_r(v_x,\delta_f)$。
3. 前轮扰动 $\delta_d$ 在预测窗内视为常数；当前 `SetSolverDynamicParam` 将其置 0（结构保留）。
4. 后轮随动器跟踪前轮**指令** $\delta_f^{\text{cmd}}$，$\delta_d$ 仅走前轮通道、不被 $k_r$ 缩放。
5. `steer`、`d_steer`、`steer_disturbance_deg` 量纲均为 rad（`_deg` 为历史命名）。
6. 误差构造默认以 `vehicle_state.x/y` 做投影；若上游为质心位姿，需先回退到后轴中心，或接受"以质心做投影"的近似。
