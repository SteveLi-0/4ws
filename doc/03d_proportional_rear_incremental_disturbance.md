# 比例后轮 + 前轮增量形式 + 前轮扰动的后轴误差状态方程

> 本文在 [[03c_error_rear_4ws]] 的 4WS 后轴误差模型基础上，引入两条简化与一项扰动：
>
> 1. **后轮随动假设**：$\delta_r = k_r\,\delta_f$（$k_r$ 为速度调度比例，稳态推导中视为常数）。
> 2. **前轮转角增量化**：$\delta_f = \delta_{f,\text{ff}} + \Delta\delta_f$，把前馈分量与反馈增量分离。
> 3. **前轮转角扰动**：在前轮指令上叠加一个未知扰动 $\delta_d$（齿条偏置、传感器零位、标定误差等），其物理入口与 $\delta_f$ 一致。
>
> 与 [[100c_4ws_proportional_rear_steady_state]] 关注稳态前馈与跟踪误差不同，本文直接给出可用于反馈控制器设计的**增量形式动力学**。$k_r=0$ 即退化为前轮转向情况。

---

## 1 起点：03c 的状态方程

状态向量 $\mathbf{x} = [e_1,\;\dot e_1,\;e_2,\;\dot e_2]^T$，参数 $L = l_f + l_r$，$\eta = I_z - ml_fl_r$，$\xi = I_z + ml_r^2$。

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B_f\,\delta_f^{\text{act}} + B_r\,\delta_r + G\,\dot\theta_{\text{ref}}
$$

其中 $\delta_f^{\text{act}}$ 表示**作用到前轮上的实际转角**（指令加扰动），矩阵定义见 03c：

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\[6pt]
0 & -\dfrac{C_f\eta + C_r\xi}{mI_zv_x} & \dfrac{C_f\eta + C_r\xi}{mI_z} & -\dfrac{C_fL\eta}{mI_zv_x} \\[6pt]
0 & 0 & 0 & 1 \\[6pt]
0 & -\dfrac{l_fC_f - l_rC_r}{I_zv_x} & \dfrac{l_fC_f - l_rC_r}{I_z} & -\dfrac{l_fC_fL}{I_zv_x}
\end{bmatrix}
$$

$$
B_f = \begin{bmatrix} 0 \\ \dfrac{C_f\eta}{mI_z} \\ 0 \\ \dfrac{l_fC_f}{I_z} \end{bmatrix},\quad
B_r = \begin{bmatrix} 0 \\ \dfrac{C_r\xi}{mI_z} \\ 0 \\ -\dfrac{l_rC_r}{I_z} \end{bmatrix},\quad
G = \begin{bmatrix} 0 \\ -\dfrac{C_fL\eta}{mI_zv_x} - v_x \\ 0 \\ -\dfrac{l_fC_fL}{I_zv_x} \end{bmatrix}
$$

---

## 2 三层信号约定

| 信号 | 含义 |
|------|------|
| $\delta_{f,\text{ff}}$ | 前馈前轮转角（基于参考曲率，前向通道生成） |
| $\Delta\delta_f$ | 反馈增量（控制器输出，待设计） |
| $\delta_f^{\text{cmd}} = \delta_{f,\text{ff}} + \Delta\delta_f$ | 前轮指令角 |
| $\delta_d$ | 前轮转角扰动（机械偏置、零位漂移等，作用在前轮上） |
| $\delta_f^{\text{act}} = \delta_f^{\text{cmd}} + \delta_d$ | 前轮实际转角（轮胎力实际响应的角度） |
| $\delta_r = k_r\,\delta_f^{\text{cmd}}$ | 后轮随动（跟随**指令**而非实际） |

> **设计选择**：后轮随动器跟踪的是前轮指令 $\delta_f^{\text{cmd}}$ 而非实际值 $\delta_f^{\text{act}}$——前轮扰动 $\delta_d$ 仅出现在前轮通道。这与典型的车辆控制结构一致：随动控制器从前轮指令获取调度信号，物理偏置只发生在前轮齿条/转向柱处。

---

## 3 等效输入合并

将 $\delta_f^{\text{act}}$ 与 $\delta_r$ 代入 03c：

$$
B_f\,\delta_f^{\text{act}} + B_r\,\delta_r
= B_f\,(\delta_f^{\text{cmd}} + \delta_d) + B_r\,(k_r\,\delta_f^{\text{cmd}})
= \underbrace{(B_f + k_r B_r)}_{B_{eq}}\,\delta_f^{\text{cmd}} + B_f\,\delta_d
$$

定义**等效前轮输入矩阵**：

$$
\boxed{B_{eq} = B_f + k_r B_r = \begin{bmatrix} 0 \\[4pt] \dfrac{C_f\eta + k_rC_r\xi}{mI_z} \\[8pt] 0 \\[4pt] \dfrac{l_fC_f - k_r l_rC_r}{I_z} \end{bmatrix}}
$$

代入指令分解 $\delta_f^{\text{cmd}} = \delta_{f,\text{ff}} + \Delta\delta_f$，得到单输入形式：

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B_{eq}\,(\delta_{f,\text{ff}} + \Delta\delta_f) + B_f\,\delta_d + G\,\dot\theta_{\text{ref}} \tag{*}
$$

---

## 4 增量化（围绕前馈平衡点）

### 4.1 平衡点定义

在定圆稳态条件 $\dot\theta_{\text{ref}} = v_x/R = \text{const}$ 下，假设由前馈 $\delta_{f,\text{ff}}$ 维持的稳态向量 $\mathbf{x}_{ss}$ 满足：

$$
0 = A\,\mathbf{x}_{ss} + B_{eq}\,\delta_{f,\text{ff}} + G\,\frac{v_x}{R} \tag{1}
$$

> $\mathbf{x}_{ss}$ 的具体表达式见 [[100c_4ws_proportional_rear_steady_state]] §4。无前轮扰动假设隐含在 (1) 中：扰动是“误差”信号，不进入前馈通道。

### 4.2 误差状态

定义增量状态 $\tilde{\mathbf{x}} = \mathbf{x} - \mathbf{x}_{ss}$。将 (*) 减去 (1)：

$$
\boxed{\dot{\tilde{\mathbf{x}}} = A\,\tilde{\mathbf{x}} + B_{eq}\,\Delta\delta_f + B_f\,\delta_d}
$$

**四个关键结论**：

1. **系统矩阵 $A$ 不变**：动力学结构由车辆决定，与平衡点无关。
2. **反馈通道用 $B_{eq}$**：反馈控制器的输入增量驱动等效输入矩阵。
3. **扰动通道用 $B_f$**：前轮扰动只走前轮物理入口，**不被 $k_r$ 缩放**。
4. **曲率扰动 $G\dot\theta_{\text{ref}}$ 被前馈消去**：增量方程中没有 $\dot\theta_{\text{ref}}$ 项。

### 4.3 物理理解

| 项 | 来源 | 系数 |
|----|------|------|
| $A\tilde{\mathbf{x}}$ | 线化车辆侧向/横摆动力学 | 与 03c 完全一致 |
| $B_{eq}\Delta\delta_f$ | 前后轮按 $1:k_r$ 同步增量动作 | $B_f$ 与 $B_r$ 的线性叠加 |
| $B_f\delta_d$ | 前轮独立的“假指令” | 仅前轮入口，无后轮跟随 |

---

## 5 状态空间矩阵展开

### 5.1 增量动力学矩阵

$$
\dot{\tilde{\mathbf{x}}} = A\,\tilde{\mathbf{x}} + B_{eq}\,\Delta\delta_f + B_d\,\delta_d
$$

其中 $B_d = B_f$（保留独立符号，便于增广扰动观测器使用）：

$$
B_{eq} = \begin{bmatrix} 0 \\[4pt] \dfrac{C_f\eta + k_rC_r\xi}{mI_z} \\[8pt] 0 \\[4pt] \dfrac{l_fC_f - k_r l_rC_r}{I_z} \end{bmatrix},\qquad
B_d = \begin{bmatrix} 0 \\[4pt] \dfrac{C_f\eta}{mI_z} \\[8pt] 0 \\[4pt] \dfrac{l_fC_f}{I_z} \end{bmatrix}
$$

### 5.2 与 03c 各列的对应关系

| 列 | 03c 原始 | 04c 增量 | 备注 |
|----|----------|----------|------|
| 第 1 列：状态耦合 | $A$ | $A$ | 不变 |
| 第 2 列：控制输入 | $B_f$ | $B_{eq} = B_f + k_rB_r$ | 后轮随动后的等效前轮 |
| 第 3 列：后轮独立 | $B_r$ | （已合并入 $B_{eq}$） | 不再独立出现 |
| 第 4 列：曲率扰动 | $G$ | （由前馈吸收） | 不出现在增量方程 |
| 第 5 列：前轮扰动 | （无） | $B_d = B_f$ | 新增 |

---

## 6 特例校核

### 6.1 $k_r = 0$（纯前轮转向）

$B_{eq}\big|_{k_r=0} = B_f$，于是：

$$
\dot{\tilde{\mathbf{x}}} = A\,\tilde{\mathbf{x}} + B_f\,(\Delta\delta_f + \delta_d)
$$

反馈与扰动共用同一通道，与 [[03a_error_rear_front_steer]] 增量化结果一致。✓

### 6.2 $k_r = 1$（蟹行）

$B_{eq,4} = (l_f - l_r)C_f / I_z$（当 $C_f = C_r$ 时退化），$B_{eq,2} = (\eta + \xi)C_f / (mI_z) = (2I_z + m(l_r^2 - l_fl_r))C_f / (mI_z)$。

横摆通道增益按比例减小（$l_f$ 与 $l_r$ 的差），与“前后轮同向无横摆”的物理直觉对齐：当 $l_f = l_r$ 且 $C_f = C_r$ 时 $B_{eq,4} = 0$，前轮指令完全无法激发横摆。

### 6.3 $\eta = 0$（惯量临界）

$B_{eq,2} = k_r C_r\xi / (mI_z)$，前轮通道完全失去对 $\ddot e_1$ 的直接增益，仅靠后轮等效贡献——印证 03c §7 关于 $\eta$ 的物理解读。

---

## 7 应用：反馈控制器设计

围绕平衡点的增量动力学是 LQR/极点配置/$H_\infty$ 等线性方法的标准对象：

$$
\Delta\delta_f = -K\,\tilde{\mathbf{x}},\qquad K \in \mathbb{R}^{1\times 4}
$$

闭环：

$$
\dot{\tilde{\mathbf{x}}} = (A - B_{eq}K)\,\tilde{\mathbf{x}} + B_d\,\delta_d
$$

| 设计目标 | 配套方法 |
|----------|----------|
| 极点配置 | 给定阻尼/带宽，选取 $K$ 使 $A - B_{eq}K$ 特征值就位 |
| LQR | $\min\int(\tilde x^TQ\tilde x + \rho\Delta\delta_f^2)$，得 $K = \rho^{-1}B_{eq}^T P$ |
| 扰动抑制 | 增广 $\delta_d$ 为状态（随机游走 $\dot\delta_d = 0$），构造 5 阶扰动观测器 |

> **扰动观测器结构**：$\dot\delta_d = 0$ 时，增广状态方程为
> $\frac{d}{dt}\begin{bmatrix}\tilde{\mathbf{x}}\\\delta_d\end{bmatrix} = \begin{bmatrix} A & B_d \\ \mathbf{0} & 0\end{bmatrix}\begin{bmatrix}\tilde{\mathbf{x}}\\\delta_d\end{bmatrix} + \begin{bmatrix}B_{eq}\\0\end{bmatrix}\Delta\delta_f$
> 与 [[06_steer_dob_observer]] 系列保持同一架构，但状态从 3 阶（$v_y, r, \delta_d$）扩展为 5 阶（误差四态 + $\delta_d$）。

---

## 8 与相关文档的关系

| 文档 | 关注点 | 与 04c 的关系 |
|------|--------|---------------|
| 03c | 4WS 一般形式 $(A, B_f, B_r, G)$ | 04c 的起点 |
| 04 | 质心误差 ↔ 后轴误差变换 | 提供等价模型路径 |
| 100c | 比例后轮的稳态前馈 $\delta_{f,\text{ff}}$ 与 $\mathbf{x}_{ss}$ | 提供 04c §4.1 的平衡点 |
| 06/06a | 转向扰动观测器 | 04c §7 直接复用其结构 |
| 03a | $\delta_r=0$ 增量化退化版本 | 04c §6.1 的特例 |

---

## 9 使用前提

1. 03c 的全部前提（小角度、线性轮胎、$\dot v_x \approx 0$、$\ddot\theta_{\text{ref}} \approx 0$）。
2. $k_r$ 在工作点附近视为常数；速度调度的 $k_r(v_x)$ 仅在准稳态下成立，快速变化时需补偿 $\dot k_r$ 项。
3. 前馈 $\delta_{f,\text{ff}}$ 与平衡点 $\mathbf{x}_{ss}$ 由参考路径决定，前馈通道不感知扰动 $\delta_d$。
4. 扰动 $\delta_d$ 在控制周期内变化缓慢（适合用积分型观测器估计）。
5. 后轮随动在前轮指令上、前轮扰动不传递到后轮（机械分离假设）。

---

## 10 SymPy 验证

验证脚本：`doc/verify_04c_4ws_proportional_incremental.py`

验证内容：
1. $B_{eq} = B_f + k_rB_r$ 解析展开 ✓
2. 平衡点 (1) 满足完整稳态方程 ✓
3. 增量方程 $\dot{\tilde{\mathbf{x}}} = A\tilde{\mathbf{x}} + B_{eq}\Delta\delta_f + B_f\delta_d$ 与原方程相减一致 ✓
4. $k_r = 0$ 退化为 $B_{eq} = B_f$ ✓
5. 数值验证 $(A, B_{eq})$ 可控（典型参数 $k_r = -0.3$）✓
