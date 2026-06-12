# 4WS 比例后轮转向稳态跟踪误差与前馈转角（总结）

> 本文是 [[100c_4ws_proportional_rear_steady_state]] 的精简总结，保留问题设定、关键中间步骤与最终结论，省略完整代数推导与 SymPy 验证细节。设后轮随动 $\delta_r = k_r\,\delta_f$，$k_r=0$ 时退化为 [[100a_rear_axle_steady_state_feedforward]] 的纯前轮结果。

---

## 1 问题设定

后轴误差状态空间（来自 03c，状态 $\mathbf{x} = [e_1,\dot e_1,e_2,\dot e_2]^T$）：

$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B_f\,\delta_f + B_r\,\delta_r + G\,\dot\theta_{\text{ref}}
$$

**后轮比例随动** $\delta_r = k_r\,\delta_f$ 使系统变为单输入，定义等效输入矩阵：

$$
B_{eq} = B_f + k_r B_r = \begin{bmatrix} 0 \\ \dfrac{C_f\eta + k_r C_r\xi}{mI_z} \\ 0 \\ \dfrac{l_fC_f - k_r l_rC_r}{I_z} \end{bmatrix}
$$

**状态反馈 + 前馈**控制律：$\delta_f = -K\mathbf{x} + \delta_{ff}$，$K = [k_1,k_2,k_3,k_4]$。闭环：

$$
\dot{\mathbf{x}} = (A - B_{eq}K)\,\mathbf{x} + B_{eq}\,\delta_{ff} + G\,\dot\theta_{\text{ref}}
$$

记号：$L = l_f+l_r$，$\eta = I_z - ml_fl_r$，$\xi = I_z + ml_r^2$，$K_{us} = \dfrac{m}{L}\!\left(\dfrac{l_r}{C_f} - \dfrac{l_f}{C_r}\right)$。

---

## 2 定圆稳态条件

- $\dot\theta_{\text{ref}} = v_x/R = \kappa v_x = \text{const}$
- $\dot{\mathbf{x}} = 0 \Rightarrow \mathbf{x}_{ss} = [e_{1,ss},\;0,\;e_{2,ss},\;0]^T$

有效约束来自状态空间的第 2、4 行（$\ddot e_1 = \ddot e_2 = 0$）。两式各除以 $B_{eq}$ 的对应分量后相减，可消去 $-k_1 e_{1,ss} + \delta_{ff}$，得到只含 $e_{2,ss}$ 的方程。

---

## 3 关键结论

### 3.1 稳态航向误差 $e_{2,ss}$（与反馈增益 $K$、前馈 $\delta_{ff}$ 无关）

$$
\boxed{e_{2,ss} = \frac{mv_x^2(C_fl_f - k_rC_rl_r)}{C_fC_rLR(1-k_r)} - \frac{k_rL}{(1-k_r)R}
= \frac{mv_x^2(C_fl_f - k_rC_rl_r) - k_rC_fC_rL^2}{C_fC_rLR(1-k_r)}}
$$

相减时 $e_{2,ss}$ 的系数与常数项均不含 $k_1,k_2,k_3,k_4,\delta_{ff}$——这是单输入系统的固有性质，$e_{2,ss}$ 完全由车辆参数、$v_x$、$\kappa$、$k_r$ 决定。

### 3.2 前馈转角 $\delta_{ff}$（令 $e_{1,ss} = 0$）

$$
\boxed{\delta_{ff} = \frac{L + K_{us}v_x^2}{(1 - k_r)R} + k_3\,e_{2,ss}}
$$

= 等效前馈 $\dfrac{L + K_{us}v_x^2}{(1-k_r)R}$（经典 Ackermann + 不足转向，再除以 $1-k_r$）+ 航向误差反馈补偿 $k_3 e_{2,ss}$。

### 3.3 一般情况 $e_{1,ss}$

$$
e_{1,ss} = \frac{1}{k_1}\left(\delta_{ff} - \delta_{ff}^*\right), \qquad \delta_{ff}^* = \frac{L + K_{us}v_x^2}{(1-k_r)R} + k_3\,e_{2,ss}
$$

即前馈偏离理想值 $\delta_{ff}^*$ 时，横向误差按 $1/k_1$ 比例线性出现。

---

## 4 稳态实际转角与物理不变量

$$
\delta_f^{ss} = \delta_{ff} - k_3\,e_{2,ss} = \frac{L + K_{us}v_x^2}{(1-k_r)R}, \qquad
\delta_r^{ss} = k_r\,\delta_f^{ss} = \frac{k_r(L + K_{us}v_x^2)}{(1-k_r)R}
$$

**等效转向角（前轮减后轮）与 $k_r$ 无关**：

$$
\delta_f^{ss} - \delta_r^{ss} = (1-k_r)\,\delta_f^{ss} = \frac{L + K_{us}v_x^2}{R}
$$

物理上必然如此：稳态圆弧跟踪所需的等效转向角是不变量，不依赖前后轮的分配方式。$1/(1-k_r)$ 因子的含义是——后轮同向转 $k_r\delta_f$ 削弱了等效转向能力，前轮需放大转角补偿。

---

## 5 特殊情况与一致性

| 情况 | $e_{2,ss}$ | $\delta_{ff}$ |
|------|-----------|---------------|
| $k_r = 0$（纯前轮，100a） | $\dfrac{ml_fv_x^2}{C_rLR}$ | $\dfrac{L + K_{us}v_x^2}{R} + k_3 e_{2,ss}$ |
| $k_3 = 0$（无航向反馈） | 同上式 | $\dfrac{L + K_{us}v_x^2}{(1-k_r)R}$ |
| $k_r \to 1$（蟹行） | $\to \infty$ | 发散（前后轮同角度无法产生横摆） |
| $v_x \to 0$（低速运动学） | $\to 0$ | $\dfrac{L}{(1-k_r)R}$ |

低速极限 $\delta_{ff}\big|_{v_x\to0,k_3=0} = \dfrac{L}{(1-k_r)R}$ 与 07a 的纯运动学模型 $\dot\psi = \dfrac{v_x(1-k_r)}{L}\delta_f$ 完全一致。

---

## 6 总结表

| 量 | 表达式 | 是否依赖 $K$ |
|---|---|---|
| $e_{2,ss}$ | $\dfrac{mv_x^2(C_fl_f - k_rC_rl_r) - k_rC_fC_rL^2}{C_fC_rLR(1-k_r)}$ | **否** |
| $\delta_{ff}$（使 $e_1=0$） | $\dfrac{L + K_{us}v_x^2}{(1-k_r)R} + k_3\,e_{2,ss}$ | 仅依赖 $k_3$ |
| $e_{1,ss}$（一般） | $\dfrac{1}{k_1}\left(\delta_{ff} - \delta_{ff}^*\right)$ | 依赖 $k_1, k_3$ |
| $\delta_f^{ss} - \delta_r^{ss}$ | $\dfrac{L + K_{us}v_x^2}{R}$ | 否（物理不变量） |

> 完整代数推导、矩阵逆求解与 SymPy 数值验证见 [[100c_4ws_proportional_rear_steady_state]] §3–§5、§11。本结论被工程代码采用：MPC 的 `beta_ref` = $e_{2,ss}$、`delta_ref` = $\delta_f^{ss}$，对照见 [[102_4ws_proportional_rear_mpc_code]] §5–§6。
