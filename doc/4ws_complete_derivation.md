# 4WS 建模与控制完整推导

> 本文从 4WS 自行车模型出发，依次推导：等效前轮转角、Frenet 误差动力学（质心 / 后轴）、稳态前馈（纯前轮 / 独立 4WS / 比例后轮）、转向扰动观测器、前轮增量化与 MPC 接口。所有推导基于同一组符号约定与同一组动力学方程，每章末尾给出 2WS 退化形式以便对比。
>
> 散布在 doc/ 下的 01–101a 各文档被本文逐节吸收并统一术语；旧文档保留为补充资料。

---

## 第 0 章 引言与约定

### 0.1 文档目标与适用边界

本文目标是从一组**统一**的物理假设出发，给出 4WS 车辆侧向控制全链条的解析推导：

| 链条环节 | 章节 |
|----------|------|
| 自行车模型（运动学 + 动力学） | 第 1 章 |
| 4WS → 等效前轮 | 第 2 章 |
| Frenet 误差动力学（质心 / 后轴） | 第 3、4 章 |
| 稳态前馈（基础理论 + 纯前轮 + 4WS） | 第 5–7 章 |
| 转向扰动观测器（DOB） | 第 8 章 |
| 前轮增量化、扰动建模与 MPC 接口 | 第 9 章 |

**适用前提**（贯穿全文，仅在个别章节单独强化）：

1. 小角度近似：$\sin\delta\approx\delta$，$\cos\delta\approx 1$，$\delta_f,\delta_r,e_2,\beta$ 均为小量。
2. 纵向速度近似恒定：$\dot v_x\approx 0$。
3. 线性轮胎模型：$F_{yf}=C_f\alpha_f$，$F_{yr}=C_r\alpha_r$，$C_f,C_r>0$。
4. 参考航向变化率缓慢：$\ddot\theta_{\text{ref}}\approx 0$。
5. 忽略纵向力对侧向 / 横摆的影响。
6. 后轴投影点与质心投影点处的曲率近似相等。

### 0.2 坐标系

| 坐标系 | 用途 | 章节 |
|--------|------|------|
| 全局 $OXY$ | 描述车辆位姿 $(X_c,Y_c,\psi)$ | 第 1 章 |
| 车体 $Cxy$ | $x$ 前、$y$ 左、$z$ 上，描述 $v_y,r,\delta$ 等 | 第 1、8、9 章 |
| Frenet 路径系 | 沿参考路径切向 $\theta_{\text{ref}}$ 与法向 | 第 3、4 章 |
| 定位坐标系 | $\tilde v_y$ 向右为正，工程实现常用约定 | 第 8 章 §8.4 |

### 0.3 正方向约定（右手系）

| 物理量 | 正方向 | 右手定则 |
|--------|--------|----------|
| $v_y$ | 向左 | $y$ 轴正方向 |
| $r=\dot\psi$ | 逆时针（俯视） | 右手绕 $z$ 轴 |
| $\delta_f,\delta_r$ | 向左转 | 右手绕 $z$ 轴 |
| $\phi$（道路横滚角） | 左高右低 | 右手绕 $x$ 轴 |
| $a_y,F_{yf},F_{yr}$ | 向左 | $y$ 轴正方向 |

> $\phi>0$（左高右低）→ 重力分量指向**右**（$-y$ 方向）。

侧偏角采用**正值定义**：

$$
\alpha=\delta-\zeta,\qquad C_\alpha>0,\qquad F_y=C_\alpha\alpha
$$

即"车轮指向角减去车轮速度方向角"。该写法与部分文献中 $\alpha=\zeta-\delta$、$C_\alpha<0$ 的写法等价，仅符号约定不同。

### 0.4 符号速查表

#### 0.4.1 基本变量

| 符号 | 含义 | 单位 |
|------|------|------|
| $X_c,Y_c$ | 质心在全局坐标系下的位置 | m |
| $\psi$ | 车辆航向角 | rad |
| $v_x$ | 质心纵向速度 | m/s |
| $v_y$ | 质心侧向速度 | m/s |
| $V$ | 质心速度大小 | m/s |
| $\beta$ | 质心侧偏角 | rad |
| $r$ | 横摆角速度，$r=\dot\psi$ | rad/s |
| $l_f$ | 质心到前轴距离 | m |
| $l_r$ | 质心到后轴距离 | m |
| $\delta_f$ | 前轮等效转角 | rad |
| $\delta_r$ | 后轮等效转角 | rad |
| $m$ | 整车质量 | kg |
| $I_z$ | 绕质心的横摆转动惯量 | kg·m² |
| $C_f$ | 前轮等效侧偏刚度（$>0$） | N/rad |
| $C_r$ | 后轮等效侧偏刚度（$>0$） | N/rad |
| $\alpha_f,\alpha_r$ | 前 / 后轮侧偏角 | rad |
| $\zeta_f,\zeta_r$ | 前 / 后轮速度方向角 | rad |
| $\kappa$ | 参考路径曲率 | 1/m |
| $R$ | 参考路径半径，$R=1/\kappa$ | m |
| $\dot\theta_{\text{ref}}$ | 参考航向变化率，$\dot\theta_{\text{ref}}=\kappa v_x$ | rad/s |
| $e_1$ | 横向位置误差（取决于参考点） | m |
| $e_2$ | 航向误差，$e_2=\psi-\theta_{\text{ref}}$ | rad |

#### 0.4.2 派生紧凑变量

后轴误差动力学中反复出现以下三个组合，引入紧凑符号：

| 紧凑符号 | 展开形式 | 物理意义 |
|----------|----------|----------|
| $L$ | $l_f+l_r$ | 轴距 |
| $\eta$ | $I_z-ml_fl_r$ | 惯量耦合参数 |
| $\xi$ | $I_z+ml_r^2$ | 后轴等效惯量参数（关于后轴的转动惯量） |

**化简钥匙**——后续推导中频繁用到的两个恒等式：

$$
\boxed{\;\frac{1}{m}-\frac{l_fl_r}{I_z}=\frac{\eta}{mI_z},\qquad \frac{1}{m}+\frac{l_r^2}{I_z}=\frac{\xi}{mI_z}\;}
$$

第一个由 $\ddot e_1$ 中前轮力的力臂分解给出（$\dfrac{F_{yf}}{m}-\dfrac{l_rl_fF_{yf}}{I_z}$），第二个由后轮力的力臂分解给出（$\dfrac{F_{yr}}{m}+\dfrac{l_r^2F_{yr}}{I_z}$）。详见第 4.2.2 节。

紧凑形式的优势是矩阵元素简洁、物理含义清晰；展开形式的优势是无需附加定义、便于代码逐项校核。本文采用**先紧凑、后展开**的写法：第 4、8、10 章主推导用紧凑形式，章末给出展开形式速查；附录 B 汇总两套写法并附 2WS 退化版本。

#### 0.4.3 扰动 / 控制信号

| 符号 | 含义 | 引入章节 |
|------|------|---------|
| $\delta_d$ | 前轮转角扰动（齿条偏置、标定误差等） | 第 8、9 章 |
| $\phi$ | 道路横滚角 | 第 8 章 |
| $\delta_{ff}$ | 前馈前轮转角 | 第 5–7 章 |
| $\Delta\delta_f$ | 反馈增量（控制器输出） | 第 9 章 |
| $\delta_f^{\text{cmd}}$ | 前轮指令角，$\delta_f^{\text{cmd}}=\delta_{ff}+\Delta\delta_f$ | 第 9 章 |
| $\delta_f^{\text{act}}$ | 实际作用到前轮的转角，$\delta_f^{\text{act}}=\delta_f^{\text{cmd}}+\delta_d$ | 第 9 章 |
| $k_r$ | 后轮随动比例，$\delta_r=k_r\delta_f^{\text{cmd}}$ | 第 7、9 章 |
| $K_{us}$ | 不足转向梯度，$K_{us}=\dfrac{m}{L}\!\left(\dfrac{l_r}{C_f}-\dfrac{l_f}{C_r}\right)$ | 第 5 章 |



---

## 第 1 章 4WS 自行车模型

将左右前轮等效为一个前轮、左右后轮等效为一个后轮，质心位于轴距上某一点。本章给出全部基础关系，后续章节均从这里出发。

### 1.1 质心速度、$\beta$ 与全局位置导数

由速度合成：

$$
V=\sqrt{v_x^2+v_y^2},\qquad \beta=\arctan\!\frac{v_y}{v_x}
$$

因此：

$$
v_x=V\cos\beta,\qquad v_y=V\sin\beta
$$

车辆速度方向相对全局 $X$ 轴的角度为 $\chi=\psi+\beta$，质心在全局坐标系下的速度：

$$
\dot X_c=v_x\cos\psi-v_y\sin\psi=V\cos(\psi+\beta)
$$

$$
\dot Y_c=v_x\sin\psi+v_y\cos\psi=V\sin(\psi+\beta)
$$

### 1.2 4WS 运动学约束

前轴中心速度方向沿前轮指向（小滑移假设下），故：

$$
\tan\delta_f=\frac{v_y+l_fr}{v_x}
$$

后轴同理：

$$
\tan\delta_r=\frac{v_y-l_rr}{v_x}
$$

两式相减消去 $v_y$ 得横摆角速度：

$$
\boxed{\;r=\frac{v_x}{L}\left(\tan\delta_f-\tan\delta_r\right)\;}
$$

两式按权重 $l_r:l_f$ 加权相加得质心侧偏角：

$$
\tan\beta=\frac{v_y}{v_x}=\frac{l_r\tan\delta_f+l_f\tan\delta_r}{L}
$$

即：

$$
\boxed{\;\beta=\arctan\!\frac{l_r\tan\delta_f+l_f\tan\delta_r}{L}\;}
$$

特例：$l_f=l_r=L/2$ 时 $\beta=\arctan\!\dfrac{\tan\delta_f+\tan\delta_r}{2}$。

### 1.3 小角度近似与典型转向模式

小角度下 $\tan\delta\approx\delta$：

$$
r\approx\frac{v_x}{L}(\delta_f-\delta_r),\qquad \beta\approx\frac{l_r\delta_f+l_f\delta_r}{L}
$$

可见 $\delta_f-\delta_r$ 主要影响横摆，$\delta_f+\delta_r$ 主要影响侧偏（蟹行）。三种典型模式：

| 模式 | 条件 | 横摆 | 侧偏 | 应用 |
|------|------|------|------|------|
| 反相位 | $\delta_r=-\delta_f$ | $r\approx\dfrac{2v_x\delta_f}{L}$ | $\beta\approx 0$（$l_f=l_r$） | 低速大曲率（泊车） |
| 同相位（蟹行） | $\delta_r=\delta_f$ | $r\approx 0$ | $\beta\approx\delta_f$ | 平移变道 |
| $\delta_r=k_r\delta_f$ | 一般工程方案 | $r\approx\dfrac{v_x(1-k_r)\delta_f}{L}$ | $\beta\approx\dfrac{(l_r+k_rl_f)\delta_f}{L}$ | 速度调度 4WS |

### 1.4 车体坐标系下加速度耦合

车体坐标系随车辆旋转，质心**真实**加速度不是简单的 $\dot v_x,\dot v_y$，存在旋转耦合：

$$
a_x=\dot v_x-rv_y,\qquad a_y=\dot v_y+rv_x
$$

反过来：

$$
\dot v_x=a_x+rv_y,\qquad \dot v_y=a_y-rv_x
$$

### 1.5 4WS 动力学三方程

平面三自由度动力学：

$$
\boxed{\;m(\dot v_x-rv_y)=\sum F_x\;}
$$

$$
\boxed{\;m(\dot v_y+rv_x)=\sum F_y\;}
$$

$$
\boxed{\;I_z\dot r=\sum M_z\;}
$$

### 1.6 轮胎力投影到车体坐标

设前 / 后轮在轮胎坐标系中的纵向力为 $F_{xf},F_{xr}$，侧向力为 $F_{yf},F_{yr}$。投影到车体系：

$$
\sum F_x=F_{xf}\cos\delta_f-F_{yf}\sin\delta_f+F_{xr}\cos\delta_r-F_{yr}\sin\delta_r-F_w-F_{\text{roll}}
$$

$$
\sum F_y=F_{xf}\sin\delta_f+F_{yf}\cos\delta_f+F_{xr}\sin\delta_r+F_{yr}\cos\delta_r
$$

横摆力矩（小角度并忽略 $F_x$ 的力矩贡献）：

$$
\sum M_z\approx l_fF_{yf}-l_rF_{yr}
$$

完整形式见 [01_bicycle_model.md] §10。

### 1.7 轮胎速度方向角与侧偏角

前 / 后轮中心速度方向角：

$$
\zeta_f=\arctan\!\frac{v_y+l_fr}{v_x}\approx\frac{v_y+l_fr}{v_x},\qquad \zeta_r=\arctan\!\frac{v_y-l_rr}{v_x}\approx\frac{v_y-l_rr}{v_x}
$$

侧偏角（正值定义）：

$$
\alpha_f=\delta_f-\zeta_f\approx\delta_f-\frac{v_y+l_fr}{v_x}
$$

$$
\alpha_r=\delta_r-\zeta_r\approx\delta_r-\frac{v_y-l_rr}{v_x}
$$

### 1.8 线性化侧向 – 横摆 ODE

线性轮胎力 $F_{yf}=C_f\alpha_f$，$F_{yr}=C_r\alpha_r$，代入侧向方程和横摆方程（忽略 $F_x$ 对侧向 / 横摆的耦合）：

$$
m(\dot v_y+v_xr)=C_f\!\left(\delta_f-\frac{v_y+l_fr}{v_x}\right)+C_r\!\left(\delta_r-\frac{v_y-l_rr}{v_x}\right)
$$

$$
I_z\dot r=l_fC_f\!\left(\delta_f-\frac{v_y+l_fr}{v_x}\right)-l_rC_r\!\left(\delta_r-\frac{v_y-l_rr}{v_x}\right)
$$

整理为标准 ODE：

$$
\boxed{\;
\dot v_y=-\frac{C_f+C_r}{mv_x}v_y+\!\left(\frac{-C_fl_f+C_rl_r}{mv_x}-v_x\right)r+\frac{C_f}{m}\delta_f+\frac{C_r}{m}\delta_r
\;}\tag{1.1}
$$

$$
\boxed{\;
\dot r=\frac{-C_fl_f+C_rl_r}{I_zv_x}v_y-\frac{C_fl_f^2+C_rl_r^2}{I_zv_x}r+\frac{C_fl_f}{I_z}\delta_f-\frac{C_rl_r}{I_z}\delta_r
\;}\tag{1.2}
$$

(1.1)、(1.2) 是后续所有章节的核心起点。

### 1.9 退化为 2WS（$\delta_r=0$）

令 $\delta_r=0$：

**运动学**：

$$
r\approx\frac{v_x\delta_f}{L},\qquad \beta\approx\frac{l_r\delta_f}{L}
$$

**动力学 ODE**——直接将 (1.1)、(1.2) 中的 $\delta_r$ 项删去：

$$
\dot v_y=-\frac{C_f+C_r}{mv_x}v_y+\!\left(\frac{-C_fl_f+C_rl_r}{mv_x}-v_x\right)r+\frac{C_f}{m}\delta_f
$$

$$
\dot r=\frac{-C_fl_f+C_rl_r}{I_zv_x}v_y-\frac{C_fl_f^2+C_rl_r^2}{I_zv_x}r+\frac{C_fl_f}{I_z}\delta_f
$$

**A、B 矩阵的对应变化**（$x=[v_y,r]^T$，$u$ 为标量 $\delta_f$ 或向量 $[\delta_f,\delta_r]^T$）：

| 项 | 4WS | 2WS（$\delta_r=0$） |
|----|-----|---------------------|
| $A$ 矩阵 | 见 (1.1)、(1.2) | **完全相同**（A 不含输入项） |
| $B$ 矩阵 | $\begin{bmatrix}C_f/m & C_r/m\\ l_fC_f/I_z & -l_rC_r/I_z\end{bmatrix}$ | $\begin{bmatrix}C_f/m\\ l_fC_f/I_z\end{bmatrix}$（仅前轮列） |

> **重要观察**：4WS → 2WS 的退化只影响 $B$，不影响 $A$。这是因为 A 矩阵描述自由响应，只取决于车辆参数；B 矩阵描述输入通道，4WS 多一列后轮通道。

### 1.10 关键关系总表

| 关系 | 公式 |
|------|------|
| 质心速度大小 | $V=\sqrt{v_x^2+v_y^2}$ |
| 质心侧偏角 | $\beta=\arctan(v_y/v_x)$ |
| 全局速度 | $\dot X_c=v_x\cos\psi-v_y\sin\psi$，$\dot Y_c=v_x\sin\psi+v_y\cos\psi$ |
| 运动学横摆 | $r=\dfrac{v_x}{L}(\tan\delta_f-\tan\delta_r)$ |
| 运动学侧偏 | $\beta=\arctan\!\dfrac{l_r\tan\delta_f+l_f\tan\delta_r}{L}$ |
| 加速度耦合 | $a_x=\dot v_x-rv_y$，$a_y=\dot v_y+rv_x$ |
| 动力学三方程 | $m(\dot v_x-rv_y)=\sum F_x$ 等 |
| 前轮速度方向角 | $\zeta_f=(v_y+l_fr)/v_x$ |
| 后轮速度方向角 | $\zeta_r=(v_y-l_rr)/v_x$ |
| 前轮侧偏角 | $\alpha_f=\delta_f-\zeta_f$ |
| 后轮侧偏角 | $\alpha_r=\delta_r-\zeta_r$ |
| 线性侧向 ODE | (1.1) |
| 线性横摆 ODE | (1.2) |


---

## 第 2 章 等效前轮转角

工程中常希望用一个"等效前轮转角"$\delta_{eq}$ 把 4WS 模型当成 2WS 处理，以便复用前轮转向的控制器和分析工具。本章给出几种等效定义并指出其局限。

### 2.1 运动学等效（瞬时横摆增益）

由 §1.2 的 4WS 横摆关系：

$$
r=\frac{v_x}{L}(\tan\delta_f-\tan\delta_r)
$$

等效 2WS 模型（仅前轮转向）：

$$
r=\frac{v_x}{L}\tan\delta_{eq}
$$

令两者相等：

$$
\boxed{\;\tan\delta_{eq}=\tan\delta_f-\tan\delta_r\;}
$$

小角度近似：

$$
\delta_{eq}\approx\delta_f-\delta_r
$$

**含义**：$\delta_{eq}$ 等效后保证瞬时**横摆角速度**和**转弯半径**与 4WS 相同。

### 2.2 侧向力等效与横摆力矩等效

将 4WS 的合侧向力与 2WS 的合侧向力相等（小角度线性化下）：

$$
C_f\delta_{eq}+C_r\cdot 0=C_f\delta_f+C_r\delta_r
$$

得：

$$
\boxed{\;\delta_{eq}^{(F)}=\delta_f+\frac{C_r}{C_f}\delta_r\;}
$$

将 4WS 的合横摆力矩与 2WS 的相等：

$$
l_fC_f\delta_{eq}-l_rC_r\cdot 0=l_fC_f\delta_f-l_rC_r\delta_r
$$

得：

$$
\boxed{\;\delta_{eq}^{(M)}=\delta_f-\frac{l_rC_r}{l_fC_f}\delta_r\;}
$$

**关键观察**：$\delta_{eq}^{(F)}\neq\delta_{eq}^{(M)}$（除非 $\delta_r=0$）。**单一等效转角无法同时**精确等效侧向力与横摆力矩——4WS 有两个独立输入，2WS 只有一个，自由度不匹配。

### 2.3 稳态横摆增益等效

稳态圆周运动 $\dot v_y=0$、$\dot r=0$ 下联立侧向方程与横摆方程，可得 4WS 稳态横摆增益：

$$
\frac{r_{ss}}{\delta_f-\delta_r}=\frac{v_x/L}{1+K_{us}v_x^2}
$$

而 2WS 稳态横摆增益为：

$$
\frac{r_{ss}}{\delta_{eq}}=\frac{v_x/L}{1+K_{us}v_x^2}
$$

两者形式完全相同（$K_{us}$ 仅与车辆参数有关，与输入无关），因此稳态等效仍为：

$$
\boxed{\;\delta_{eq}^{(ss)}=\delta_f-\delta_r\;}
$$

与运动学等效一致。

### 2.4 等效模型的局限性与适用场景

| 等效方式 | $\delta_{eq}$ | 保证相同的量 |
|----------|---------------|--------------|
| 运动学 / 稳态横摆增益 | $\delta_f-\delta_r$ | $r$、转弯半径、稳态横摆 |
| 侧向力 | $\delta_f+(C_r/C_f)\delta_r$ | 合侧向力 |
| 横摆力矩 | $\delta_f-(l_rC_r/l_fC_f)\delta_r$ | 合横摆力矩 |

工程中绝大多数情况推荐**运动学等效** $\delta_{eq}=\delta_f-\delta_r$：

- 物理含义清晰，等效了横摆与转弯半径
- 与稳态结论一致，分析与设计简洁
- 形式简单，便于实时计算

**等效会丢失的信息**：

1. **质心侧偏角**：4WS 的 $\beta$ 与等效 2WS 的 $\beta$ 相差约 $\delta_r$，蟹行运动特征丢失
2. **瞬态侧向力 / 横摆力矩**：动力学层面无法同时匹配
3. **轮胎力分配**：前后轮各自的侧偏角与侧向力信息丢失

适用：路径规划简化、状态估计降阶、控制器初步设计。
不适用：精确侧偏角控制、轮胎力分配、极限工况仿真。

> **本文后续策略**：第 3、4 章直接用 4WS 双输入误差动力学，不依赖等效；第 7 章在 $\delta_r=k_r\delta_f$ 假设下回到单输入形式（合并到 $B_{eq}$）；第 9 章 MPC 在状态层面也使用单输入 + 后轮随动结构。


---

## 第 3 章 Frenet 误差动力学（质心参考点）

将车辆动力学投影到 Frenet 坐标系下的横向 / 航向误差，得到控制器设计常用的状态空间形式。本章用**质心**作为误差参考点。

### 3.1 误差定义与误差运动学

| 符号 | 含义 |
|------|------|
| $e_1$ | 质心到参考路径的横向距离（左正） |
| $e_2$ | 航向误差，$e_2=\psi-\theta_{\text{ref}}$ |

小角度近似下的误差运动学：

$$
\boxed{\;\dot e_1=v_y+v_xe_2\;}\tag{3.1}
$$

$$
\boxed{\;\dot e_2=r-\dot\theta_{\text{ref}}\;}\tag{3.2}
$$

由此得到状态替换关系：

$$
v_y=\dot e_1-v_xe_2,\qquad r=\dot e_2+\dot\theta_{\text{ref}}\tag{3.3}
$$

### 3.2 4WS 二阶误差方程推导

#### 3.2.1 $\ddot e_1$ 的推导

对 (3.1) 求导：

$$
\ddot e_1=\dot v_y+v_x\dot e_2
$$

将 §1.8 的侧向 ODE (1.1) 中 $\dot v_y+v_xr$ 这一组合（即 $(F_{yf}+F_{yr})/m$）凑出来：

$$
\dot v_y=\underbrace{\frac{F_{yf}+F_{yr}}{m}}_{\text{由(1.1)}}-v_xr
$$

代入：

$$
\ddot e_1=\frac{F_{yf}+F_{yr}}{m}-v_xr+v_x(r-\dot\theta_{\text{ref}})=\frac{F_{yf}+F_{yr}}{m}-v_x\dot\theta_{\text{ref}}
$$

> **关键消去**：$-v_xr+v_xr=0$，离心耦合项被吸入扰动 $-v_x\dot\theta_{\text{ref}}$。

按状态替换 (3.3) 把 $v_y,r$ 写成误差量：

$$
\ddot e_1=\frac{1}{m}\!\left[C_f\delta_f+C_r\delta_r-\frac{C_f+C_r}{v_x}(\dot e_1-v_xe_2)-\frac{l_fC_f-l_rC_r}{v_x}(\dot e_2+\dot\theta_{\text{ref}})\right]-v_x\dot\theta_{\text{ref}}
$$

整理为按状态分量的标准形式：

$$
\boxed{\;\ddot e_1=-\frac{C_f+C_r}{mv_x}\dot e_1+\frac{C_f+C_r}{m}e_2-\frac{l_fC_f-l_rC_r}{mv_x}\dot e_2+\frac{C_f}{m}\delta_f+\frac{C_r}{m}\delta_r+\!\left(-v_x-\frac{l_fC_f-l_rC_r}{mv_x}\right)\dot\theta_{\text{ref}}\;}
$$

#### 3.2.2 $\ddot e_2$ 的推导

对 (3.2) 求导，利用 $\ddot\theta_{\text{ref}}\approx 0$：

$$
\ddot e_2=\dot r
$$

代入 §1.8 的横摆 ODE (1.2)，并用 (3.3) 替换 $v_y,r$：

$$
\boxed{\;\ddot e_2=-\frac{l_fC_f-l_rC_r}{I_zv_x}\dot e_1+\frac{l_fC_f-l_rC_r}{I_z}e_2-\frac{l_f^2C_f+l_r^2C_r}{I_zv_x}\dot e_2+\frac{l_fC_f}{I_z}\delta_f-\frac{l_rC_r}{I_z}\delta_r-\frac{l_f^2C_f+l_r^2C_r}{I_zv_x}\dot\theta_{\text{ref}}\;}
$$

### 3.3 状态空间形式

定义 $\mathbf x=[e_1,\dot e_1,e_2,\dot e_2]^T$，控制 $\mathbf u=[\delta_f,\delta_r]^T$，扰动 $w=\dot\theta_{\text{ref}}$：

$$
\dot{\mathbf x}=A\,\mathbf x+B_f\,\delta_f+B_r\,\delta_r+G\,\dot\theta_{\text{ref}}
$$

#### 系统矩阵 $A$

$$
A=\begin{bmatrix}
0 & 1 & 0 & 0 \\[6pt]
0 & -\dfrac{C_f+C_r}{mv_x} & \dfrac{C_f+C_r}{m} & -\dfrac{l_fC_f-l_rC_r}{mv_x} \\[6pt]
0 & 0 & 0 & 1 \\[6pt]
0 & -\dfrac{l_fC_f-l_rC_r}{I_zv_x} & \dfrac{l_fC_f-l_rC_r}{I_z} & -\dfrac{l_f^2C_f+l_r^2C_r}{I_zv_x}
\end{bmatrix}
$$

#### 输入矩阵

$$
B_f=\begin{bmatrix}0\\ \dfrac{C_f}{m}\\ 0\\ \dfrac{l_fC_f}{I_z}\end{bmatrix},\qquad B_r=\begin{bmatrix}0\\ \dfrac{C_r}{m}\\ 0\\ -\dfrac{l_rC_r}{I_z}\end{bmatrix}
$$

#### 扰动矩阵

$$
G=\begin{bmatrix}0\\ -v_x-\dfrac{l_fC_f-l_rC_r}{mv_x}\\ 0\\ -\dfrac{l_f^2C_f+l_r^2C_r}{I_zv_x}\end{bmatrix}
$$

### 3.4 各元素物理解读

| 元素 | 表达式 | 物理意义 |
|------|--------|---------|
| $A_{22}$ | $-\dfrac{C_f+C_r}{mv_x}$ | 前后轮侧偏力对横向速度的阻尼 |
| $A_{23}$ | $\dfrac{C_f+C_r}{m}$ | 航向偏差通过侧偏力产生的横向加速度 |
| $A_{24}$ | $-\dfrac{l_fC_f-l_rC_r}{mv_x}$ | 前后力臂不对称对横向运动的耦合 |
| $A_{42}$ | $-\dfrac{l_fC_f-l_rC_r}{I_zv_x}$ | 横向速度通过力矩差对横摆的耦合 |
| $A_{43}$ | $\dfrac{l_fC_f-l_rC_r}{I_z}$ | 航向偏差产生的横摆力矩 |
| $A_{44}$ | $-\dfrac{l_f^2C_f+l_r^2C_r}{I_zv_x}$ | 轮胎侧偏力对横摆的阻尼 |
| $B_{f,2}$ | $C_f/m$ | 前轮转角对横向加速度增益 |
| $B_{f,4}$ | $l_fC_f/I_z$ | 前轮转角对横摆加速度增益 |
| $B_{r,2}$ | $C_r/m$ | 后轮转角对横向加速度增益 |
| $B_{r,4}$ | $-l_rC_r/I_z$ | 后轮转角对横摆加速度增益（**负号**：后轮力臂方向相反） |
| $G_2$ | $-v_x-\dfrac{l_fC_f-l_rC_r}{mv_x}$ | 路径曲率扰动（含离心项 $v_x$） |
| $G_4$ | $-\dfrac{l_f^2C_f+l_r^2C_r}{I_zv_x}$ | 路径曲率对横摆加速度的扰动 |

> **物理一致性检验**：$A_{23}=-v_x\cdot A_{22}$，$A_{43}=-v_x\cdot A_{42}$。这是因为 $\dot e_1$ 和 $e_2$ 的系数来自同一个力（$v_y=\dot e_1-v_xe_2$ 的结构），比例关系恒为 $-v_x$。

### 3.5 退化为 2WS（$\delta_r=0$）

直接将 $B_r$ 项删去即得：

$$
\dot{\mathbf x}=A\,\mathbf x+B_f\,\delta_f+G\,\dot\theta_{\text{ref}}
$$

其中 $A,B_f,G$ 与 4WS 完全相同。**A、$B_f$、$G$ 不变，仅 $B_r$ 被移除**——这与 §1.9 的结论一致：4WS 的额外自由度只体现在多出来的输入通道上。

| 量 | 4WS | 2WS（$\delta_r=0$） |
|----|-----|---------------------|
| 控制量维数 | 2 | 1 |
| 控制自由度 | 可同时控制 $e_1,e_2$ | 单输入，无法同时使 $e_{1,ss}=e_{2,ss}=0$ |
| 零侧偏条件 | 选 $\delta_r/\delta_f=l_r/l_f$ 可零 $\beta$ | 一般 $\beta\neq 0$ |
| $A$ 矩阵 | 见上 | **完全相同** |
| 输入矩阵 | $[B_f \ \ B_r]$ | $B_f$ |

> 质心模型不含紧凑变量 $\eta,\xi$，A、B、G 矩阵元素天然以 $C_f,C_r,l_f,l_r,m,I_z,v_x$ 表达，无需额外的"展开形式"小节。第 4 章的后轴模型才会引入紧凑变量。


---

## 第 4 章 Frenet 误差动力学（后轴参考点）

工程实现中往往采用**后轴中心**作为误差参考点：路径跟踪传感器（GPS / 后轴中心点定位）天然对准后轴；低速时后轴几乎无侧偏（见 §1.3），$e_2$ 在低速极限下趋于零，控制器更鲁棒。本章直接由后轴 Frenet 关系正向推导 4WS 误差状态空间，不依赖第 3 章的质心结果。

### 4.1 后轴运动学与误差定义

后轴中心在车体坐标系下的速度分量：

$$
v_{x,r}=v_x,\qquad v_{yr}=v_y-l_rr\tag{4.1}
$$

后轴速度方向相对车身纵向的偏角：

$$
\beta_r=\arctan\!\frac{v_{yr}}{v_x}\approx\frac{v_y-l_rr}{v_x}
$$

后轴中心到参考路径的横向距离 $e_1$ 与航向误差 $e_2=\psi-\theta_{\text{ref}}$ 的运动学：

$$
\boxed{\;\dot e_1=v_{yr}+v_xe_2=(v_y-l_rr)+v_xe_2\;}\tag{4.2}
$$

$$
\boxed{\;\dot e_2=r-\dot\theta_{\text{ref}}\;}\tag{4.3}
$$

> **与质心定义的核心区别**：(4.2) 中 $v_y$ 被替换为后轴侧向速度 $v_{yr}=v_y-l_rr$。

由 (4.2)、(4.3) 反解车辆状态：

$$
v_{yr}=\dot e_1-v_xe_2,\qquad r=\dot e_2+\dot\theta_{\text{ref}}\tag{4.4}
$$

进而恢复质心侧向速度：

$$
v_y=v_{yr}+l_rr=(\dot e_1-v_xe_2)+l_r(\dot e_2+\dot\theta_{\text{ref}})\tag{4.5}
$$

### 4.2 4WS 二阶误差方程的正向推导

#### 4.2.1 $\ddot e_2$ 推导

由 (4.3)：$\ddot e_2=\dot r$（$\ddot\theta_{\text{ref}}\approx 0$）。代入 §1.8 的横摆 ODE (1.2)。

为使前轮速度方向角用 $v_{yr}$ 表达，注意：

$$
v_y+l_fr=v_{yr}+l_rr+l_fr=v_{yr}+Lr
$$

因此前 / 后轮侧偏角变为：

$$
\alpha_f=\delta_f-\frac{v_{yr}+Lr}{v_x},\qquad \alpha_r=\delta_r-\frac{v_{yr}}{v_x}\tag{4.6}
$$

代入 (1.2) 并用 (4.4) 替换 $v_{yr},r$，整理：

$$
\boxed{\;\ddot e_2=\frac{l_fC_f}{I_z}\delta_f-\frac{l_rC_r}{I_z}\delta_r-\frac{l_fC_f-l_rC_r}{I_zv_x}\dot e_1+\frac{l_fC_f-l_rC_r}{I_z}e_2-\frac{l_fC_fL}{I_zv_x}\dot e_2-\frac{l_fC_fL}{I_zv_x}\dot\theta_{\text{ref}}\;}\tag{4.7}
$$

> **与质心结果对比**：(4.7) 的 $\dot e_1,e_2,\delta_f,\delta_r$ 系数与第 3 章完全相同；$\dot e_2$ 阻尼系数从 $-\dfrac{l_f^2C_f+l_r^2C_r}{I_zv_x}$ 变为 $-\dfrac{l_fC_fL}{I_zv_x}$。差异来源：前轮侧偏角中 $r$ 的系数从 $l_f$ 变为 $L=l_f+l_r$。

#### 4.2.2 $\ddot e_1$ 的力臂分解（$\eta,\xi$ 的出现）

对 (4.2) 求导：

$$
\ddot e_1=\dot v_{yr}+v_x\dot e_2=(\dot v_y-l_r\dot r)+v_x(r-\dot\theta_{\text{ref}})
$$

由 §1.8 的侧向 ODE：$\dot v_y=(F_{yf}+F_{yr})/m-v_xr$，代入：

$$
\ddot e_1=\frac{F_{yf}+F_{yr}}{m}-v_xr-l_r\dot r+v_xr-v_x\dot\theta_{\text{ref}}
$$

$$
=\frac{F_{yf}+F_{yr}}{m}-l_r\dot r-v_x\dot\theta_{\text{ref}}
$$

$\dot r=(l_fF_{yf}-l_rF_{yr})/I_z$，因此：

$$
\ddot e_1=F_{yf}\!\left(\frac{1}{m}-\frac{l_rl_f}{I_z}\right)+F_{yr}\!\left(\frac{1}{m}+\frac{l_r^2}{I_z}\right)-v_x\dot\theta_{\text{ref}}
$$

利用 §0.4 的化简钥匙：

$$
\frac{1}{m}-\frac{l_fl_r}{I_z}=\frac{\eta}{mI_z},\qquad \frac{1}{m}+\frac{l_r^2}{I_z}=\frac{\xi}{mI_z}
$$

得到紧凑形式：

$$
\boxed{\;\ddot e_1=\frac{\eta}{mI_z}F_{yf}+\frac{\xi}{mI_z}F_{yr}-v_x\dot\theta_{\text{ref}}\;}\tag{4.8}
$$

> **物理解读**：
> - $\eta=I_z-ml_fl_r$ 是惯量耦合参数。$\eta>0$（实际车辆通常如此）意味着前轮力对后轴横向加速度有正增益；$\eta=0$ 时前轮力对 $\ddot e_1$ 无直接影响；$\eta<0$ 极少见。
> - $\xi=I_z+ml_r^2$ 是后轴等效转动惯量（平行轴定理），**恒大于零**。后轮力对后轴横向加速度始终有正增益。

代入 (4.6) 的轮胎力，并用 (4.4) 替换 $v_{yr},r$，按状态分量整理：

$$
\boxed{\;\ddot e_1=\frac{C_f\eta}{mI_z}\delta_f+\frac{C_r\xi}{mI_z}\delta_r-\frac{C_f\eta+C_r\xi}{mI_zv_x}\dot e_1+\frac{C_f\eta+C_r\xi}{mI_z}e_2-\frac{C_fL\eta}{mI_zv_x}\dot e_2+\!\left(-\frac{C_fL\eta}{mI_zv_x}-v_x\right)\dot\theta_{\text{ref}}\;}\tag{4.9}
$$

### 4.3 状态空间形式（紧凑形式）

$$
\dot{\mathbf x}=A\,\mathbf x+B_f\,\delta_f+B_r\,\delta_r+G\,\dot\theta_{\text{ref}}
$$

#### 系统矩阵 $A$

$$
A=\begin{bmatrix}
0 & 1 & 0 & 0 \\[8pt]
0 & -\dfrac{C_f\eta+C_r\xi}{mI_zv_x} & \dfrac{C_f\eta+C_r\xi}{mI_z} & -\dfrac{C_fL\eta}{mI_zv_x} \\[8pt]
0 & 0 & 0 & 1 \\[8pt]
0 & -\dfrac{l_fC_f-l_rC_r}{I_zv_x} & \dfrac{l_fC_f-l_rC_r}{I_z} & -\dfrac{l_fC_fL}{I_zv_x}
\end{bmatrix}\tag{4.10}
$$

#### 输入矩阵

$$
B_f=\begin{bmatrix}0\\ \dfrac{C_f\eta}{mI_z}\\ 0\\ \dfrac{l_fC_f}{I_z}\end{bmatrix},\qquad B_r=\begin{bmatrix}0\\ \dfrac{C_r\xi}{mI_z}\\ 0\\ -\dfrac{l_rC_r}{I_z}\end{bmatrix}\tag{4.11}
$$

#### 扰动矩阵

$$
G=\begin{bmatrix}0\\ -\dfrac{C_fL\eta}{mI_zv_x}-v_x\\ 0\\ -\dfrac{l_fC_fL}{I_zv_x}\end{bmatrix}\tag{4.12}
$$

### 4.4 物理一致性检验

1. $A_{23}=-v_x\cdot A_{22}$ ✓（来自 $v_{yr}=\dot e_1-v_xe_2$ 的结构）
2. $A_{43}=-v_x\cdot A_{42}$ ✓（同理）
3. $B_{r,2}=\dfrac{C_r\xi}{mI_z}>0$ 恒成立——后轮转角对后轴横向加速度始终有正增益，符合"后轮直接控制后轴侧向力"的直觉。
4. 当 $\eta=0$（惯量临界）时 $B_{f,2}=0$：前轮转角对 $\ddot e_1$ 无直接增益，仅通过横摆通道间接影响——印证 §4.2.2 关于 $\eta$ 的物理解读。

### 4.5 与质心定义的关键关系（一致性校验）

由 (3.1) 与 (4.2) 直接相减：

$$
\dot e_{1c}-\dot e_{1r}=v_y-(v_y-l_rr)=l_rr=l_r(\dot e_2+\dot\theta_{\text{ref}})
$$

进而：

$$
\boxed{\;\ddot e_{1r}=\ddot e_{1c}-l_r\ddot e_2\;}\tag{4.13}
$$

**用途**：将第 3 章的 $\ddot e_{1c}$ 表达式减去 $l_r$ 倍的 $\ddot e_2$ 表达式，再用状态替换 $\dot e_{1c}=\dot e_{1r}+l_r\dot e_2+l_r\dot\theta_{\text{ref}}$ 替换，可独立得到 (4.9)，与本章正向推导一致——这是两条独立路径的交叉印证。完整的逐项化简可由 SymPy 验证（详见 doc/verify_cg_to_rear_transform.py）。

> **小结**：质心 → 后轴 = "横向加速度减去横摆加速度的力臂"，这是后轴模型出现 $\eta,\xi$ 的根本原因。

### 4.6 紧凑形式 → 展开形式

将 (4.10)–(4.12) 中的 $\eta,\xi,L$ 全部回代为 $C_f,C_r,l_f,l_r,m,I_z,v_x$，得到无紧凑符号的等价表达，便于代码逐项校核。化简钥匙：

$$
\eta=I_z-ml_fl_r,\quad \xi=I_z+ml_r^2,\quad L=l_f+l_r
$$

#### $A$ 矩阵展开

$$
A_{22}=-\frac{C_f(I_z-ml_fl_r)+C_r(I_z+ml_r^2)}{mI_zv_x}=-\frac{C_f+C_r}{mv_x}+\frac{l_r(C_fl_f-C_rl_r)}{I_zv_x}
$$

$$
A_{23}=\frac{C_f(I_z-ml_fl_r)+C_r(I_z+ml_r^2)}{mI_z}=\frac{C_f+C_r}{m}-\frac{l_r(C_fl_f-C_rl_r)}{I_z}
$$

$$
A_{24}=-\frac{C_fL(I_z-ml_fl_r)}{mI_zv_x}=\frac{C_fl_fl_r(l_f+l_r)}{I_zv_x}-\frac{C_f(l_f+l_r)}{mv_x}
$$

$$
A_{42}=-\frac{l_fC_f-l_rC_r}{I_zv_x},\qquad A_{43}=\frac{l_fC_f-l_rC_r}{I_z},\qquad A_{44}=-\frac{C_fl_f(l_f+l_r)}{I_zv_x}
$$

#### $B_f,B_r$ 展开

$$
B_{f,2}=\frac{C_f(I_z-ml_fl_r)}{mI_z}=\frac{C_f}{m}-\frac{C_fl_fl_r}{I_z},\qquad B_{f,4}=\frac{l_fC_f}{I_z}
$$

$$
B_{r,2}=\frac{C_r(I_z+ml_r^2)}{mI_z}=\frac{C_r}{m}+\frac{C_rl_r^2}{I_z},\qquad B_{r,4}=-\frac{l_rC_r}{I_z}
$$

#### $G$ 展开

$$
G_2=-\frac{C_fL(I_z-ml_fl_r)}{mI_zv_x}-v_x=\frac{C_fl_fl_r(l_f+l_r)}{I_zv_x}-\frac{C_f(l_f+l_r)}{mv_x}-v_x
$$

$$
G_4=-\frac{l_fC_f(l_f+l_r)}{I_zv_x}
$$

#### 4WS 后轴模型展开形式速查表

| 元素 | 紧凑形式 | 展开形式 |
|------|---------|---------|
| $A_{22}$ | $-\dfrac{C_f\eta+C_r\xi}{mI_zv_x}$ | $-\dfrac{C_f+C_r}{mv_x}+\dfrac{l_r(C_fl_f-C_rl_r)}{I_zv_x}$ |
| $A_{23}$ | $\dfrac{C_f\eta+C_r\xi}{mI_z}$ | $\dfrac{C_f+C_r}{m}-\dfrac{l_r(C_fl_f-C_rl_r)}{I_z}$ |
| $A_{24}$ | $-\dfrac{C_fL\eta}{mI_zv_x}$ | $\dfrac{C_fl_fl_r(l_f+l_r)}{I_zv_x}-\dfrac{C_f(l_f+l_r)}{mv_x}$ |
| $A_{42}$ | $-\dfrac{l_fC_f-l_rC_r}{I_zv_x}$ | 同左 |
| $A_{43}$ | $\dfrac{l_fC_f-l_rC_r}{I_z}$ | 同左 |
| $A_{44}$ | $-\dfrac{l_fC_fL}{I_zv_x}$ | $-\dfrac{C_fl_f(l_f+l_r)}{I_zv_x}$ |
| $B_{f,2}$ | $\dfrac{C_f\eta}{mI_z}$ | $\dfrac{C_f}{m}-\dfrac{C_fl_fl_r}{I_z}$ |
| $B_{f,4}$ | $\dfrac{l_fC_f}{I_z}$ | 同左 |
| $B_{r,2}$ | $\dfrac{C_r\xi}{mI_z}$ | $\dfrac{C_r}{m}+\dfrac{C_rl_r^2}{I_z}$ |
| $B_{r,4}$ | $-\dfrac{l_rC_r}{I_z}$ | 同左 |
| $G_2$ | $-\dfrac{C_fL\eta}{mI_zv_x}-v_x$ | $\dfrac{C_fl_fl_r(l_f+l_r)}{I_zv_x}-\dfrac{C_f(l_f+l_r)}{mv_x}-v_x$ |
| $G_4$ | $-\dfrac{l_fC_fL}{I_zv_x}$ | $-\dfrac{C_fl_f(l_f+l_r)}{I_zv_x}$ |

### 4.7 退化为 2WS（$\delta_r=0$）

只保留前轮通道，$B_r$ 项被移除，**$A,B_f,G$ 完全保留**：

$$
\dot{\mathbf x}=A\,\mathbf x+B_f\,\delta_f+G\,\dot\theta_{\text{ref}}
$$

紧凑与展开形式见 §4.6。与第 3 章质心 2WS 模型相比，关键差异表：

| 元素 | 质心 2WS | 后轴 2WS（紧凑） | 后轴 2WS（展开） |
|------|---------|------------------|------------------|
| $A_{22}$ | $-\dfrac{C_f+C_r}{mv_x}$ | $-\dfrac{C_f\eta+C_r\xi}{mI_zv_x}$ | $-\dfrac{C_f+C_r}{mv_x}+\dfrac{l_r(C_fl_f-C_rl_r)}{I_zv_x}$ |
| $A_{23}$ | $\dfrac{C_f+C_r}{m}$ | $\dfrac{C_f\eta+C_r\xi}{mI_z}$ | $\dfrac{C_f+C_r}{m}-\dfrac{l_r(C_fl_f-C_rl_r)}{I_z}$ |
| $A_{24}$ | $-\dfrac{l_fC_f-l_rC_r}{mv_x}$ | $-\dfrac{C_fL\eta}{mI_zv_x}$ | 见上 |
| $A_{44}$ | $-\dfrac{l_f^2C_f+l_r^2C_r}{I_zv_x}$ | $-\dfrac{l_fC_fL}{I_zv_x}$ | $-\dfrac{C_fl_f(l_f+l_r)}{I_zv_x}$ |
| $B_{f,2}$ | $\dfrac{C_f}{m}$ | $\dfrac{C_f\eta}{mI_z}$ | $\dfrac{C_f}{m}-\dfrac{C_fl_fl_r}{I_z}$ |
| $B_{f,4}$ | $\dfrac{l_fC_f}{I_z}$ | 同左 | 同左 |

> 第 4 行（横摆方程）的 $\delta_f$ 增益 $B_{f,4}$ 在两种参考点下相同，因为横摆动力学 $\dot r=(l_fF_{yf}-l_rF_{yr})/I_z$ 不依赖参考点选择。


---

## 第 5 章 稳态前馈基础

第 6、7 章会基于不同的转向架构（纯前轮、4WS 独立 $\delta_r$、比例后轮）求解定圆稳态跟踪误差与前馈转角。这些推导共享同一套稳态分析工具，本章集中讲清楚，后续章节直接套用。

### 5.1 什么是稳态前馈，为什么需要它

考虑车辆以恒定速度 $v_x$ 沿半径 $R$（曲率 $\kappa=1/R$）的圆弧行驶。即便误差状态全部归零，车辆要"维持"在这条圆弧上仍然需要一个非零的前轮转角——这就是**前馈转角** $\delta_{ff}$。

反馈控制器（$-K\mathbf x$）的作用是镇定误差动力学、抑制扰动，但**单纯反馈无法高效维持稳态转向**：

- 若没有前馈，反馈必须靠"先产生误差、再修正误差"来凑出维持圆弧所需的转角，导致稳态残差。
- 前馈直接提供维持圆弧所需的转角，反馈只需处理偏离稳态的瞬态，二者分工明确。

因此控制律统一写作：

$$
\delta_f=\underbrace{-K\mathbf x}_{\text{反馈}}+\underbrace{\delta_{ff}}_{\text{前馈}},\qquad K=[k_1,k_2,k_3,k_4]
$$

本章及第 6、7 章的核心问题是：**给定曲率 $\kappa$，$\delta_{ff}$ 应取多少才能使稳态横向误差 $e_{1,ss}=0$？此时稳态航向误差 $e_{2,ss}$ 是多少？**

### 5.2 定圆稳态条件

定圆稳态意味着所有状态量不再随时间变化：

- 曲率恒定：$\kappa=1/R=\text{const}$
- 参考航向变化率恒定：$\dot\theta_{\text{ref}}=\kappa v_x=v_x/R=\text{const}$
- 稳态条件：$\dot{\mathbf x}=0$

由于状态向量为 $\mathbf x=[e_1,\dot e_1,e_2,\dot e_2]^T$，$\dot{\mathbf x}=0$ 中第 1、3 个分量给出 $\dot e_1=0$、$\dot e_2=0$，因此稳态状态向量简化为：

$$
\mathbf x_{ss}=[e_{1,ss},\,0,\,e_{2,ss},\,0]^T
$$

闭环系统在稳态下的方程：

$$
0=(A-B_{(\cdot)}K)\,\mathbf x_{ss}+B_{(\cdot)}\,\delta_{ff}+(\text{后轮项})+G\,\frac{v_x}{R}
$$

其中 $B_{(\cdot)}$ 视章节而定（纯前轮用 $B_f$；4WS 独立 $\delta_r$ 用 $B_f$ 且后轮项为 $B_r\delta_r$；比例后轮用 $B_{eq}=B_f+k_rB_r$）。

**有效约束只来自第 2、4 行**：第 1 行给出 $\dot e_1=\mathbf x_{ss}[2]=0$（恒等成立），第 3 行给出 $\dot e_2=\mathbf x_{ss}[4]=0$（恒等成立），都不含未知量。真正的约束是第 2 行（$\ddot e_1=0$）和第 4 行（$\ddot e_2=0$），共两个方程，恰好解两个未知量 $e_{1,ss},e_{2,ss}$（给定 $\delta_{ff}$），或解 $e_{2,ss},\delta_{ff}$（给定 $e_{1,ss}=0$）。

### 5.3 稳态侧偏角（车辆动力学层面）

在分析误差之前，先从车辆动力学求出稳态时前后轮的侧偏角，这是后续所有结果的物理根基。

稳态圆周运动 $\dot v_y=0$、$\dot r=0$，横摆角速度 $r=v_x\kappa$。由 §1.8 的侧向与横摆方程：

**侧向力平衡**（向心力由轮胎侧向力提供）：

$$
mv_xr=C_f\alpha_f+C_r\alpha_r\quad\Rightarrow\quad mv_x^2\kappa=C_f\alpha_{f,ss}+C_r\alpha_{r,ss}\tag{5.1}
$$

**横摆力矩平衡**（稳态无横摆角加速度，前后力矩相等）：

$$
0=l_fC_f\alpha_f-l_rC_r\alpha_r\quad\Rightarrow\quad l_fC_f\alpha_{f,ss}=l_rC_r\alpha_{r,ss}\tag{5.2}
$$

由 (5.2) 解出 $\alpha_{f,ss}=\dfrac{l_rC_r}{l_fC_f}\alpha_{r,ss}$，代入 (5.1)：

$$
mv_x^2\kappa=C_f\cdot\frac{l_rC_r}{l_fC_f}\alpha_{r,ss}+C_r\alpha_{r,ss}=C_r\alpha_{r,ss}\!\left(\frac{l_r}{l_f}+1\right)=\frac{C_rL}{l_f}\alpha_{r,ss}
$$

解得：

$$
\boxed{\;\alpha_{r,ss}=\frac{ml_fv_x^2\kappa}{C_rL},\qquad \alpha_{f,ss}=\frac{ml_rv_x^2\kappa}{C_fL}\;}\tag{5.3}
$$

> **重要**：(5.3) 只用到了侧向平衡 (5.1) 与横摆平衡 (5.2)，**与前后轮如何转向（2WS 还是 4WS）无关**——稳态侧偏角完全由车辆参数与运动状态唯一确定。后续无论纯前轮、独立 $\delta_r$ 还是比例后轮，$\alpha_{f,ss},\alpha_{r,ss}$ 都取这两个值。

### 5.4 经典 Ackermann + 不足转向梯度

纯前轮转向（$\delta_r=0$，$\alpha_r=-\zeta_r$）下，由侧偏角定义 $\alpha_f=\delta_f-\zeta_f$ 恢复稳态前轮转角。

稳态后轮速度方向角 $\zeta_{r,ss}=-\alpha_{r,ss}$（因 $\delta_r=0$）给出稳态后轴侧向速度 $v_{yr,ss}=v_x\zeta_{r,ss}=-\alpha_{r,ss}v_x$。前轮速度方向角：

$$
\zeta_{f,ss}=\frac{v_{y,ss}+l_fr}{v_x}=\frac{v_{yr,ss}+Lr}{v_x}=-\alpha_{r,ss}+L\kappa
$$

因此：

$$
\delta_{f,ff}^{\text{2WS}}=\alpha_{f,ss}+\zeta_{f,ss}=\alpha_{f,ss}-\alpha_{r,ss}+L\kappa=\frac{ml_rv_x^2\kappa}{C_fL}-\frac{ml_fv_x^2\kappa}{C_rL}+L\kappa
$$

整理：

$$
\delta_{f,ff}^{\text{2WS}}=L\kappa+\frac{mv_x^2}{L}\!\left(\frac{l_r}{C_f}-\frac{l_f}{C_r}\right)\kappa
$$

定义**不足转向梯度**：

$$
\boxed{\;K_{us}=\frac{m}{L}\!\left(\frac{l_r}{C_f}-\frac{l_f}{C_r}\right)\;}\tag{5.4}
$$

则经典稳态前轮转角：

$$
\boxed{\;\delta_{f,ff}^{\text{2WS}}=(L+K_{us}v_x^2)\kappa=\frac{L+K_{us}v_x^2}{R}\;}\tag{5.5}
$$

物理分解：

$$
\delta_{f,ff}^{\text{2WS}}=\underbrace{\frac{L}{R}}_{\text{纯运动学 Ackermann}}+\underbrace{K_{us}\frac{v_x^2}{R}}_{\text{动力学修正}}
$$

| 条件 | 含义 |
|------|------|
| $K_{us}>0$（$l_r/C_f>l_f/C_r$） | 不足转向，高速需增大转角 |
| $K_{us}=0$ | 中性转向 |
| $K_{us}<0$（$l_r/C_f<l_f/C_r$） | 过度转向，高速需减小转角 |

(5.5) 是第 6、7 章前馈公式中"经典前馈"部分的来源。

### 5.5 稳态侧偏角与稳态航向误差的几何关系

稳态时 $\dot e_1=0$ 要求横向误差率为零，即所选参考点的侧向速度为零。这把车辆侧偏角与误差状态的 $e_{2,ss}$ 直接联系起来。

**质心参考点**：由 (3.1) $\dot e_{1c}=v_y+v_xe_2$，令其为零：

$$
e_{2,ss}^{c}=-\frac{v_{y,ss}}{v_x}=-\beta_{ss}
$$

**后轴参考点**：由 (4.2) $\dot e_{1r}=v_{yr}+v_xe_2$，令其为零：

$$
e_{2,ss}^{r}=-\frac{v_{yr,ss}}{v_x}=-\beta_{r,ss}
$$

两个参考点的侧向速度相差 $l_rr$（$v_{yr}=v_y-l_rr$），故两个侧偏角相差：

$$
\beta_{ss}-\beta_{r,ss}=\frac{l_rr_{ss}}{v_x}=l_r\kappa
$$

**质心侧偏角的稳态值**。质心侧向速度由后轴侧偏角决定：纯前轮转向下后轮速度方向角 $\zeta_{r,ss}=-\alpha_{r,ss}$，故 $v_{yr,ss}=-\alpha_{r,ss}v_x$，$v_{y,ss}=v_{yr,ss}+l_rr=-\alpha_{r,ss}v_x+l_rv_x\kappa$：

$$
\beta_{ss}=\frac{v_{y,ss}}{v_x}=l_r\kappa-\alpha_{r,ss}=l_r\kappa-\frac{ml_fv_x^2\kappa}{C_rL}
$$

**后轴侧偏角的稳态值**（纯前轮）：$\beta_{r,ss}=v_{yr,ss}/v_x=-\alpha_{r,ss}$。

由此得到两种参考点下的稳态航向误差：

| 参考点 | $e_{2,ss}^{\text{2WS}}=-\beta_{(\cdot),ss}$ | 低速极限 | 物理含义 |
|--------|---------------------------------------------|---------|---------|
| 质心 | $\dfrac{ml_fv_x^2}{C_rLR}-\dfrac{l_r}{R}$ | $-l_r\kappa$（纯几何项） | $-\beta_{ss}$ |
| 后轴 | $\dfrac{ml_fv_x^2}{C_rLR}=\alpha_{r,ss}$ | $0$ | $\alpha_{r,ss}$（后轮侧偏角） |

> **后轴定义在低速时稳态航向误差自然趋于零**（因为低速 $\alpha_{r,ss}\propto v_x^2\to 0$），而质心定义保留几何项 $-l_r\kappa$ 不消失。这是工程中倾向选择后轴参考点的重要原因——低速泊车工况下控制器看到的稳态偏置更小。

> 第 7 章会看到，4WS 下后轴 $\beta_{r,ss}=\delta_r-\alpha_{r,ss}$（后轮主动转向改变了后轴速度方向），从而 $e_{2,ss}$ 多出 $-\delta_r$ 项。

### 5.6 求解 $e_{2,ss}$ 与 $\delta_{ff}$ 的标准套路

第 6、7 章统一使用以下五步套路（在第 6 章会完整演示一次）：

1. **写出闭环稳态方程的第 2、4 行**：分别对应 $\ddot e_1=0$ 和 $\ddot e_2=0$。每行形如
   $$
   0=A_{i3}e_{2,ss}-B_{(\cdot),i}(k_1e_{1,ss}+k_3e_{2,ss})+B_{(\cdot),i}\delta_{ff}+(\text{后轮项})+G_i\frac{v_x}{R}
   $$
   （用到 $\mathbf x_{ss}=[e_{1,ss},0,e_{2,ss},0]^T$，故 $A$、$BK$ 只剩 $e_{1,ss}$、$e_{2,ss}$ 项）。
2. **两侧除以输入系数** $B_{(\cdot),i}$，把方程归一化为
   $$
   0=(\tfrac{A_{i3}}{B_{(\cdot),i}}-k_3)e_{2,ss}-k_1e_{1,ss}+\delta_{ff}+(\text{归一化后轮项})+\tfrac{G_i}{B_{(\cdot),i}}\tfrac{v_x}{R}
   $$
   归一化后两行的 $-k_1e_{1,ss}+\delta_{ff}$ 项**结构完全相同**。
3. **第 2 行减第 4 行**：消去 $-k_1e_{1,ss}+\delta_{ff}$，得到只含 $e_{2,ss}$（与已知后轮项、扰动项）的方程，解出 $e_{2,ss}$。由于 $-k_1e_{1,ss}+\delta_{ff}$ 被消去，**$e_{2,ss}$ 与反馈增益 $K$、前馈 $\delta_{ff}$ 均无关**。
4. **令 $e_{1,ss}=0$ 代回第 4 行**（或第 2 行），求解使横向误差归零的理想前馈 $\delta_{ff}^*$。
5. **一般情形**：不指定 $e_{1,ss}=0$，从第 4 行直接得到 $e_{1,ss}=\dfrac{1}{k_1}(\delta_{ff}-\delta_{ff}^*)$——即前馈偏离理想值时残留的稳态横向误差。

> **关键洞察**：第 3 步表明 $e_{2,ss}$ 是车辆的"固有"稳态航向偏差，反馈无法消除（单前轮反馈通道）；第 4 步表明前馈可以把 $e_{1,ss}$ 调零，但代价是接受一个非零的 $e_{2,ss}$。这是单输入系统的根本限制，也是第 7 章 4WS 通过后轮自由度突破该限制的出发点。


---

## 第 6 章 单输入闭环稳态（纯前轮转向）

本章把第 5 章的标准套路完整演示一遍：纯前轮转向（$\delta_r=0$）下，求后轴参考点的稳态航向误差与前馈转角。质心参考点结果并列给出。

### 6.1 控制律与闭环

状态反馈 + 前馈：

$$
\delta_f=-K\mathbf x+\delta_{ff},\qquad K=[k_1,k_2,k_3,k_4]
$$

闭环（后轴参考点，$\delta_r=0$，$A,B_f,G$ 取第 4 章 (4.10)–(4.12)）：

$$
\dot{\mathbf x}=(A-B_fK)\,\mathbf x+B_f\,\delta_{ff}+G\,\dot\theta_{\text{ref}}
$$

### 6.2 后轴参考点的稳态求解（完整推导）

#### 步骤 1–2：写出并归一化第 2、4 行

稳态 $\mathbf x_{ss}=[e_{1,ss},0,e_{2,ss},0]^T$。$A\mathbf x_{ss}$ 的第 $i$ 行为 $A_{i3}e_{2,ss}$（因 $A_{i1}=0$，第 2、4 列乘以零），$B_fK\mathbf x_{ss}=B_{f,i}(k_1e_{1,ss}+k_3e_{2,ss})$。

**第 4 行**（$\ddot e_2=0$），代入 $A_{43}=\dfrac{l_fC_f-l_rC_r}{I_z}$，$B_{f,4}=\dfrac{l_fC_f}{I_z}$，$G_4=-\dfrac{l_fC_fL}{I_zv_x}$：

$$
0=\frac{l_fC_f-l_rC_r}{I_z}e_{2,ss}-\frac{l_fC_f}{I_z}(k_1e_{1,ss}+k_3e_{2,ss})+\frac{l_fC_f}{I_z}\delta_{ff}-\frac{l_fC_fL}{I_zv_x}\cdot\frac{v_x}{R}
$$

除以 $B_{f,4}=l_fC_f/I_z$。其中 $\dfrac{A_{43}}{B_{f,4}}=\dfrac{l_fC_f-l_rC_r}{l_fC_f}=1-\dfrac{l_rC_r}{l_fC_f}$，$\dfrac{G_4}{B_{f,4}}\cdot\dfrac{v_x}{R}=-\dfrac{L}{R}$：

$$
0=\!\left(1-\frac{l_rC_r}{l_fC_f}-k_3\right)e_{2,ss}-k_1e_{1,ss}+\delta_{ff}-\frac{L}{R}\tag{I}
$$

**第 2 行**（$\ddot e_1=0$），代入 $A_{23}=\dfrac{C_f\eta+C_r\xi}{mI_z}$，$B_{f,2}=\dfrac{C_f\eta}{mI_z}$，$G_2=-\dfrac{C_fL\eta}{mI_zv_x}-v_x$：

$$
0=\frac{C_f\eta+C_r\xi}{mI_z}e_{2,ss}-\frac{C_f\eta}{mI_z}(k_1e_{1,ss}+k_3e_{2,ss})+\frac{C_f\eta}{mI_z}\delta_{ff}+\!\left(-\frac{C_fL\eta}{mI_zv_x}-v_x\right)\frac{v_x}{R}
$$

除以 $B_{f,2}=C_f\eta/(mI_z)$（设 $\eta\neq 0$）。其中：

$$
\frac{A_{23}}{B_{f,2}}=\frac{C_f\eta+C_r\xi}{C_f\eta}=1+\frac{C_r\xi}{C_f\eta},\qquad \frac{G_2}{B_{f,2}}\cdot\frac{v_x}{R}=-\frac{L}{R}-\frac{mI_zv_x^2}{C_f\eta R}
$$

（$G_2/B_{f,2}$ 的化简：$\dfrac{mI_z}{C_f\eta}\!\left(-\dfrac{C_fL\eta}{mI_zv_x}-v_x\right)\dfrac{v_x}{R}=-\dfrac{L}{R}-\dfrac{mI_zv_x^2}{C_f\eta R}$。）

$$
0=\!\left(1+\frac{C_r\xi}{C_f\eta}-k_3\right)e_{2,ss}-k_1e_{1,ss}+\delta_{ff}-\frac{L}{R}-\frac{mI_zv_x^2}{C_f\eta R}\tag{II}
$$

#### 步骤 3：相减求 $e_{2,ss}$

(II) − (I)，$-k_1e_{1,ss}+\delta_{ff}$ 与 $-L/R$ 抵消：

$$
0=\!\left[\frac{C_r\xi}{C_f\eta}+\frac{l_rC_r}{l_fC_f}\right]e_{2,ss}-\frac{mI_zv_x^2}{C_f\eta R}
$$

化简方括号系数，通分 $l_fC_f\eta$：

$$
\frac{C_r\xi}{C_f\eta}+\frac{l_rC_r}{l_fC_f}=\frac{C_r(l_f\xi+l_r\eta)}{l_fC_f\eta}
$$

利用恒等式 $l_f\xi+l_r\eta=l_f(I_z+ml_r^2)+l_r(I_z-ml_fl_r)=LI_z$：

$$
\frac{C_r\xi}{C_f\eta}+\frac{l_rC_r}{l_fC_f}=\frac{C_rLI_z}{l_fC_f\eta}
$$

代入：

$$
\frac{C_rLI_z}{l_fC_f\eta}\,e_{2,ss}=\frac{mI_zv_x^2}{C_f\eta R}
$$

消去 $I_z/(C_f\eta)$：

$$
\frac{C_rL}{l_f}\,e_{2,ss}=\frac{mv_x^2}{R}
$$

$$
\boxed{\;e_{2,ss}^{\text{后轴}}=\frac{ml_fv_x^2}{C_rLR}\;}\tag{6.1}
$$

这正是 §5.3 求出的后轮稳态侧偏角 $\alpha_{r,ss}$，与反馈增益 $K$ 无关。

### 6.3 质心参考点的稳态求解

对第 3 章的质心模型应用同一套路（$A,B_f,G$ 取质心版本），相减后得：

$$
\frac{C_rL}{l_fC_f}\,e_{2,ss}=\frac{mv_x^2}{C_fR}-\frac{l_rC_rL}{l_fC_fR}
$$

解得：

$$
\boxed{\;e_{2,ss}^{\text{质心}}=\frac{ml_fv_x^2}{C_rLR}-\frac{l_r}{R}\;}\tag{6.2}
$$

与后轴结果相差几何项 $-l_r/R=-l_r\kappa$，与 §5.5 的几何关系一致。完整推导见 [99b_steady_state_feedforward.md]。

### 6.4 前馈转角（步骤 4）

将 $e_{1,ss}=0$ 代入式 (I)：

$$
0=\!\left(1-\frac{l_rC_r}{l_fC_f}-k_3\right)e_{2,ss}+\delta_{ff}-\frac{L}{R}
$$

$$
\delta_{ff}=\frac{L}{R}-\!\left(1-\frac{l_rC_r}{l_fC_f}\right)e_{2,ss}+k_3e_{2,ss}
$$

代入 $e_{2,ss}=\dfrac{ml_fv_x^2}{C_rLR}$，展开第二项：

$$
\left(1-\frac{l_rC_r}{l_fC_f}\right)e_{2,ss}=\frac{l_fC_f-l_rC_r}{l_fC_f}\cdot\frac{ml_fv_x^2}{C_rLR}=\frac{(l_fC_f-l_rC_r)mv_x^2}{C_fC_rLR}
$$

利用 $-\dfrac{l_fC_f-l_rC_r}{C_fC_rL}=\dfrac{l_r}{C_fL}-\dfrac{l_f}{C_rL}=\dfrac{K_{us}}{m}$（由 (5.4) $K_{us}=\dfrac{m}{L}(\frac{l_r}{C_f}-\frac{l_f}{C_r})$）：

$$
-\!\left(1-\frac{l_rC_r}{l_fC_f}\right)e_{2,ss}=\frac{mv_x^2}{R}\!\left(\frac{l_r}{C_fL}-\frac{l_f}{C_rL}\right)=K_{us}\frac{v_x^2}{R}
$$

因此：

$$
\boxed{\;\delta_{ff}=\frac{L+K_{us}v_x^2}{R}+k_3\,e_{2,ss}\;}\tag{6.3}
$$

物理分解：

$$
\delta_{ff}=\underbrace{\frac{L}{R}}_{\text{Ackermann}}+\underbrace{K_{us}\frac{v_x^2}{R}}_{\text{不足转向补偿}}+\underbrace{k_3\,e_{2,ss}}_{\text{航向误差反馈耦合}}
$$

> 第三项的物理意义：稳态时 $e_{2,ss}\neq 0$，反馈律 $-k_3e_2$ 会产生额外修正转角 $-k_3e_{2,ss}$，前馈需要预补偿这部分以维持 $e_{1,ss}=0$。**$k_3=0$ 时退化为经典前馈** $\delta_{ff}=(L+K_{us}v_x^2)/R$，即 (5.5)。

### 6.5 一般情形（步骤 5）

不指定 $e_{1,ss}=0$，由式 (I) 直接解：

$$
\boxed{\;e_{1,ss}=\frac{1}{k_1}\!\left(\delta_{ff}-\delta_{ff}^*\right)\;}\tag{6.4}
$$

其中 $\delta_{ff}^*=\dfrac{L+K_{us}v_x^2}{R}+k_3\,e_{2,ss}$ 是使 $e_{1,ss}=0$ 的理想前馈。前馈每偏离理想值 $\Delta\delta_{ff}$，稳态横向误差就线性增加 $\Delta\delta_{ff}/k_1$——增大 $k_1$ 可减小前馈误差对横向位置的影响。

### 6.6 总结

| 量 | 后轴参考点 | 质心参考点 |
|----|-----------|-----------|
| $e_{2,ss}$ | $\dfrac{ml_fv_x^2}{C_rLR}$ | $\dfrac{ml_fv_x^2}{C_rLR}-\dfrac{l_r}{R}$ |
| $\delta_{ff}^*$ | $\dfrac{L+K_{us}v_x^2}{R}+k_3\,e_{2,ss}$ | 同左（代入各自 $e_{2,ss}$） |
| $k_3=0$ 时 $\delta_{ff}^*$ | $(L+K_{us}v_x^2)/R$（两种参考点相同） | 同左 |
| 是否依赖 $K$ | $e_{2,ss}$ 不依赖；$\delta_{ff}^*$ 仅依赖 $k_3$；$e_{1,ss}$ 依赖 $k_1,k_3$ | 同左 |

> **物理事实不依赖参考点**：$k_3=0$ 时两种参考点的前馈相同，因为车辆在曲率 $\kappa$ 圆弧上行驶所需的前轮转角是物理量，与误差如何定义无关。差异仅出现在 $e_{2,ss}\neq 0$ 时与反馈律的耦合项 $k_3e_{2,ss}$ 上。


---

## 第 7 章 双输入 4WS 闭环稳态

本章在后轴误差状态空间上求解定圆稳态跟踪误差与前馈转角。先处理**后轮转角 $\delta_r$ 独立给定**的一般情形（§7.1–7.6），再把工程常用的**比例随动** $\delta_r=k_r\delta_f$ 作为特化代入，化简出比例后轮的稳态公式（§7.7）。

控制律：前轮采用状态反馈 + 前馈，后轮转角作为已知量。

$$
\delta_f=-K\mathbf x+\delta_{ff},\qquad K=[k_1,k_2,k_3,k_4]
$$

### 7.1 闭环稳态方程（完整展开）

闭环系统（后轴参考点，$B_f,B_r,G$ 取第 4 章 (4.10)–(4.12)）：

$$
\dot{\mathbf x}=(A-B_fK)\,\mathbf x+B_f\,\delta_{ff}+B_r\,\delta_r+G\,\dot\theta_{\text{ref}}
$$

稳态条件 $\dot{\mathbf x}=0$，$\mathbf x_{ss}=[e_{1,ss},0,e_{2,ss},0]^T$，$\dot\theta_{\text{ref}}=v_x/R$。代入稳态方程：

$$
0=(A-B_fK)\,\mathbf x_{ss}+B_f\,\delta_{ff}+B_r\,\delta_r+G\,\frac{v_x}{R}
$$

第 1、3 行在 $\dot e_1=\dot e_2=0$ 下自动满足，有效约束来自第 2、4 行。先逐项展开。

**矩阵 – 向量乘积**。由于 $\mathbf x_{ss}=[e_{1,ss},0,e_{2,ss},0]^T$，$A$ 的第 $i$ 行作用结果为 $A_{i2}\cdot 0+A_{i3}e_{2,ss}=A_{i3}e_{2,ss}$（$A_{i1}=0$，$A_{i4}\cdot 0=0$）。而 $B_fK\mathbf x_{ss}=B_{f,i}\,(k_1e_{1,ss}+k_3e_{2,ss})$。

**第 4 行**（$\ddot e_2=0$），代入 $A_{43}=\dfrac{l_fC_f-l_rC_r}{I_z}$，$B_{f,4}=\dfrac{l_fC_f}{I_z}$，$B_{r,4}=-\dfrac{l_rC_r}{I_z}$，$G_4=-\dfrac{l_fC_fL}{I_zv_x}$：

$$
0=\frac{l_fC_f-l_rC_r}{I_z}e_{2,ss}-\frac{l_fC_f}{I_z}(k_1e_{1,ss}+k_3e_{2,ss})+\frac{l_fC_f}{I_z}\delta_{ff}-\frac{l_rC_r}{I_z}\delta_r-\frac{l_fC_fL}{I_zv_x}\cdot\frac{v_x}{R}
$$

两侧乘以 $I_z/(l_fC_f)$，并用 $\dfrac{l_fC_f-l_rC_r}{l_fC_f}=1-\dfrac{l_rC_r}{l_fC_f}$、$\dfrac{G_4}{B_{f,4}}\cdot\dfrac{v_x}{R}=-\dfrac{L}{R}$：

$$
0=\!\left(1-\frac{l_rC_r}{l_fC_f}-k_3\right)e_{2,ss}-k_1e_{1,ss}+\delta_{ff}-\frac{l_rC_r}{l_fC_f}\delta_r-\frac{L}{R}\tag{I}
$$

**第 2 行**（$\ddot e_1=0$），代入 $A_{23}=\dfrac{C_f\eta+C_r\xi}{mI_z}$，$B_{f,2}=\dfrac{C_f\eta}{mI_z}$，$B_{r,2}=\dfrac{C_r\xi}{mI_z}$，$G_2=-\dfrac{C_fL\eta}{mI_zv_x}-v_x$：

$$
0=\frac{C_f\eta+C_r\xi}{mI_z}e_{2,ss}-\frac{C_f\eta}{mI_z}(k_1e_{1,ss}+k_3e_{2,ss})+\frac{C_f\eta}{mI_z}\delta_{ff}+\frac{C_r\xi}{mI_z}\delta_r+\!\left(-\frac{C_fL\eta}{mI_zv_x}-v_x\right)\frac{v_x}{R}
$$

两侧乘以 $mI_z/(C_f\eta)$（设 $\eta\neq 0$）。其中：

$$
\frac{A_{23}}{B_{f,2}}=\frac{C_f\eta+C_r\xi}{C_f\eta}=1+\frac{C_r\xi}{C_f\eta},\qquad \frac{G_2}{B_{f,2}}\cdot\frac{v_x}{R}=\left(-\frac{L}{v_x}-\frac{mI_zv_x}{C_f\eta}\right)\frac{v_x}{R}=-\frac{L}{R}-\frac{mI_zv_x^2}{C_f\eta R}
$$

得到：

$$
0=\!\left(1+\frac{C_r\xi}{C_f\eta}-k_3\right)e_{2,ss}-k_1e_{1,ss}+\delta_{ff}+\frac{C_r\xi}{C_f\eta}\delta_r-\frac{L}{R}-\frac{mI_zv_x^2}{C_f\eta R}\tag{II}
$$

### 7.2 稳态航向误差 $e_{2,ss}$

(II) − (I)，两式中的 $-k_1e_{1,ss}+\delta_{ff}$ 与 $-L/R$ 相互抵消：

$$
0=\underbrace{\left(\frac{C_r\xi}{C_f\eta}+\frac{l_rC_r}{l_fC_f}\right)}_{\text{$e_{2,ss}$ 与 $\delta_r$ 共同系数}}e_{2,ss}+\left(\frac{C_r\xi}{C_f\eta}+\frac{l_rC_r}{l_fC_f}\right)\delta_r-\frac{mI_zv_x^2}{C_f\eta R}
$$

> **观察**：$e_{2,ss}$ 和 $\delta_r$ 的系数完全相同，因此二者以 $(e_{2,ss}+\delta_r)$ 的组合出现。这预示着 $\delta_r$ 将以 $-\delta_r$ 的形式直接进入 $e_{2,ss}$。

化简公共系数，通分 $l_fC_f\eta$：

$$
\frac{C_r\xi}{C_f\eta}+\frac{l_rC_r}{l_fC_f}=\frac{C_r\xi l_f+l_rC_r\eta}{l_fC_f\eta}=\frac{C_r(l_f\xi+l_r\eta)}{l_fC_f\eta}
$$

利用恒等式（见附录 A.2）：

$$
l_f\xi+l_r\eta=l_f(I_z+ml_r^2)+l_r(I_z-ml_fl_r)=(l_f+l_r)I_z=LI_z
$$

因此公共系数为 $\dfrac{C_rLI_z}{l_fC_f\eta}$，代入：

$$
\frac{C_rLI_z}{l_fC_f\eta}(e_{2,ss}+\delta_r)=\frac{mI_zv_x^2}{C_f\eta R}
$$

消去 $I_z/(C_f\eta)$：

$$
\frac{C_rL}{l_f}(e_{2,ss}+\delta_r)=\frac{mv_x^2}{R}
$$

解得：

$$
\boxed{\;e_{2,ss}=\frac{ml_fv_x^2}{C_rLR}-\delta_r\;}\tag{7.1}
$$

**物理解读**：

| 项 | 含义 |
|----|------|
| $\dfrac{ml_fv_x^2}{C_rLR}$ | 后轮稳态侧偏角 $\alpha_{r,ss}$（与第 6 章 2WS 后轴情形相同，见 §5.3 (5.3)） |
| $-\delta_r$ | 后轮主动转向直接削减稳态航向误差 |

**关键性质**：

- $e_{2,ss}$ **与反馈增益 $K$ 无关**（单前轮反馈环路的固有限制）
- 选择 $\delta_r=ml_fv_x^2\kappa/(C_rL)$ 时 $e_{2,ss}=0$ —— **4WS 可以完全消除稳态航向误差**，纯前轮转向做不到

### 7.3 完全消除稳态航向误差的条件

由 (7.1) 令 $e_{2,ss}=0$：

$$
\boxed{\;\delta_r^*=\frac{ml_fv_x^2}{C_rL}\,\kappa\;}\tag{7.2}
$$

这正是后轮稳态侧偏角 $\alpha_{r,ss}$。物理含义：让后轮转角"主动补偿"后轮稳态侧偏，使后轴速度方向沿车身纵轴，从而 $\beta_{r,ss}=0$、$e_{2,ss}=0$。

### 7.4 前馈转角 $\delta_{ff}$

将 $e_{1,ss}=0$ 代入式 (I)：

$$
\delta_{ff}=\frac{L}{R}+\frac{l_rC_r}{l_fC_f}\delta_r-\!\left(\frac{l_fC_f-l_rC_r}{l_fC_f}-k_3\right)e_{2,ss}
$$

代入 (7.1) 展开 $\left(\dfrac{l_fC_f-l_rC_r}{l_fC_f}\right)e_{2,ss}$：

$$
\frac{l_fC_f-l_rC_r}{l_fC_f}\!\left(\frac{ml_fv_x^2}{C_rLR}-\delta_r\right)=\underbrace{\frac{(l_fC_f-l_rC_r)mv_x^2}{C_fC_rLR}}_{=-K_{us}v_x^2/R}-\frac{l_fC_f-l_rC_r}{l_fC_f}\delta_r
$$

其中用到 $\dfrac{l_fC_f-l_rC_r}{C_fC_rL}=\dfrac{l_f}{C_rL}-\dfrac{l_r}{C_fL}=-\dfrac{1}{m}K_{us}$。代回并合并 $\delta_r$ 项：

$$
\frac{l_rC_r}{l_fC_f}\delta_r+\frac{l_fC_f-l_rC_r}{l_fC_f}\delta_r=\frac{l_rC_r+l_fC_f-l_rC_r}{l_fC_f}\delta_r=\delta_r
$$

得到：

$$
\boxed{\;\delta_{ff}=\frac{L+K_{us}v_x^2}{R}+\delta_r+k_3\,e_{2,ss}\;}\tag{7.3}
$$

物理分解：

$$
\delta_{ff}=\underbrace{\frac{L+K_{us}v_x^2}{R}}_{\text{经典前馈（同 6.3）}}+\underbrace{\delta_r}_{\text{后轮转角补偿}}+\underbrace{k_3\,e_{2,ss}}_{\text{航向误差反馈耦合}}
$$

> **$\delta_r$ 项的来源**：后轮主动转向产生侧向力改变了稳态力平衡，前轮需要额外转角维持圆弧跟踪。从运动学等效角度看，$\delta_f-\delta_r$ 决定瞬时横摆，故 $\delta_f$ 中需多加 $\delta_r$ 才能保持等效转向角不变（见 §7.7.2 不变量）。

### 7.5 一般情形 $e_{1,ss}$

不指定 $e_{1,ss}=0$，由式 (I) 直接解：

$$
\boxed{\;e_{1,ss}=\frac{1}{k_1}\!\left(\delta_{ff}-\delta_{ff}^*\right)\;}\tag{7.4}
$$

其中 $\delta_{ff}^*=\dfrac{L+K_{us}v_x^2}{R}+\delta_r+k_3\,e_{2,ss}$ 是使 $e_{1,ss}=0$ 的理想前馈（即 (7.3)）。

### 7.6 退化为 2WS（$\delta_r=0$）

直接令 $\delta_r=0$：

$$
e_{2,ss}\big|_{\delta_r=0}=\frac{ml_fv_x^2}{C_rLR},\qquad \delta_{ff}\big|_{\delta_r=0}=\frac{L+K_{us}v_x^2}{R}+k_3\,e_{2,ss}
$$

与第 6 章 (6.1)、(6.3) 完全一致 ✓

---

### 7.7 比例后轮随动特化 $\delta_r=k_r\delta_f$

工程上最常用的方案不是独立给定 $\delta_r$，而是按速度调度的比例随动：

$$
\delta_r=k_r(v_x)\,\delta_f
$$

稳态分析中将 $k_r$ 视为常数。本节**不重新推导**，而是把 §7.1–7.4 的独立 $\delta_r$ 结果在 $\delta_r=k_r\delta_f$ 约束下化简——因为 (7.1) 的 $e_{2,ss}$ 与反馈增益 $K$ 无关，只要确定稳态时实际施加的 $\delta_r^{ss}$ 即可代入。

#### 7.7.1 等效输入矩阵 $B_{eq}$

代入 $\delta_r=k_r\delta_f$，系统从双输入退化为单输入：

$$
\dot{\mathbf x}=A\,\mathbf x+(B_f+k_rB_r)\,\delta_f+G\,\dot\theta_{\text{ref}}
$$

定义**等效输入矩阵**（紧凑形式）：

$$
\boxed{\;B_{eq}=B_f+k_rB_r=\begin{bmatrix}0\\[4pt] \dfrac{C_f\eta+k_rC_r\xi}{mI_z}\\[8pt] 0\\[4pt] \dfrac{l_fC_f-k_rl_rC_r}{I_z}\end{bmatrix}\;}\tag{7.5}
$$

该矩阵在第 9 章（5 阶 MPC 模型）会作为 $A_{\text{aug}}$ 的第 5 列复用。

#### 7.7.2 稳态等效转向角不变量 → $\delta_f^{ss},\delta_r^{ss}$

稳态圆弧跟踪所需的**等效转向角** $\delta_f-\delta_r$ 是物理不变量：稳态侧偏角 $\alpha_{f,ss},\alpha_{r,ss}$ 由 §5.3 (5.3) 唯一确定（只取决于 $v_x,\kappa$），而 $\delta_f-\delta_r=(\alpha_f-\alpha_r)+(\zeta_f-\zeta_r)$，其中 $\zeta_f-\zeta_r=\dfrac{(v_y+l_fr)-(v_y-l_rr)}{v_x}=\dfrac{Lr}{v_x}=L\kappa$。因此无论前后轮如何分配：

$$
\boxed{\;\delta_f^{ss}-\delta_r^{ss}=\frac{L+K_{us}v_x^2}{R}\;}\tag{7.6}
$$

即第 5 章 (5.5) 的经典稳态转向公式。代入 $\delta_r^{ss}=k_r\delta_f^{ss}$：

$$
(1-k_r)\delta_f^{ss}=\frac{L+K_{us}v_x^2}{R}
$$

$$
\boxed{\;\delta_f^{ss}=\frac{L+K_{us}v_x^2}{(1-k_r)R},\qquad \delta_r^{ss}=k_r\,\delta_f^{ss}=\frac{k_r(L+K_{us}v_x^2)}{(1-k_r)R}\;}\tag{7.7}
$$

> $1/(1-k_r)$ 因子的含义：后轮同向转 $k_r\delta_f$（$k_r>0$）削弱了等效转向能力 $(1-k_r)\delta_f$，前轮需增大转角补偿；$k_r<0$（反相位）时 $1-k_r>1$，前轮转角反而减小。

#### 7.7.3 代入 (7.1) 得比例后轮 $e_{2,ss}$

把 $\delta_r^{ss}$（7.7）代入独立结果 (7.1)：

$$
e_{2,ss}=\frac{ml_fv_x^2}{C_rLR}-\delta_r^{ss}=\frac{ml_fv_x^2}{C_rLR}-\frac{k_r(L+K_{us}v_x^2)}{(1-k_r)R}
$$

通分到 $C_fC_rL(1-k_r)R$，利用 $L+K_{us}v_x^2=\dfrac{L^2C_fC_r+mv_x^2(l_rC_r-l_fC_f)}{LC_fC_r}$（即 $K_{us}=\dfrac{m}{L}(\frac{l_r}{C_f}-\frac{l_f}{C_r})$ 的整理），分子合并后：

$$
\boxed{\;e_{2,ss}=\frac{mv_x^2(C_fl_f-k_rC_rl_r)-k_rC_fC_rL^2}{C_fC_rL(1-k_r)R}\;}\tag{7.8}
$$

等价形式：

$$
e_{2,ss}=\frac{mv_x^2(C_fl_f-k_rC_rl_r)}{C_fC_rL(1-k_r)R}-\frac{k_rL}{(1-k_r)R}
$$

> 此结果亦可由 $B_{eq}$ 直接做 (II)−(I) 的闭式求解得到（SymPy 验证见 doc/verify_100c_4ws_proportional_rear.py），两条路径一致。

#### 7.7.4 代入 (7.3) 得比例后轮 $\delta_{ff}$

把 $\delta_r^{ss}$（7.7）代入独立前馈 (7.3)：

$$
\delta_{ff}=\frac{L+K_{us}v_x^2}{R}+\delta_r^{ss}+k_3e_{2,ss}=\frac{L+K_{us}v_x^2}{R}\!\left(1+\frac{k_r}{1-k_r}\right)+k_3e_{2,ss}
$$

利用 $1+\dfrac{k_r}{1-k_r}=\dfrac{1}{1-k_r}$：

$$
\boxed{\;\delta_{ff}=\frac{L+K_{us}v_x^2}{(1-k_r)R}+k_3\,e_{2,ss}\;}\tag{7.9}
$$

物理分解：

$$
\delta_{ff}=\underbrace{\frac{L+K_{us}v_x^2}{(1-k_r)R}}_{\text{等效前馈，被 }(1-k_r)\text{ 缩放}}+\underbrace{k_3\,e_{2,ss}}_{\text{航向误差反馈耦合}}
$$

注意 $\delta_f^{ss}=\delta_{ff}-k_3e_{2,ss}=\dfrac{L+K_{us}v_x^2}{(1-k_r)R}$，与 (7.7) 自洽。

#### 7.7.5 与运动学模型的一致性

[07a 文档] §11.2 给出运动学约束（无侧偏）下的稳态前轮转角：

$$
\dot\psi=\frac{v_x(1-k_r)\delta_f}{L}\quad\text{令}\;\dot\psi=\frac{v_x}{R}\Rightarrow\delta_f^{ss}=\frac{L}{(1-k_r)R}
$$

(7.9) 在低速极限（$v_x\to 0$，$k_3=0$）下 $\delta_{ff}\to\dfrac{L}{(1-k_r)R}$，与运动学一致 ✓

#### 7.7.6 工程化 $k_r(v_x)$ 调度

实测调度表（详见 [08_steering_scheduler_analysis.md]）：

| 速度区间 | $k_r$ | 转向模式 | 物理目的 |
|----------|-------|---------|---------|
| $\le 15$ kph | $-1/6\approx-0.167$ | 反相位 | 缩短转弯半径，提高低速灵活性 |
| 15–80 kph | 线性过渡，$k_r(v_x)\approx\dfrac{v_x-30}{300}$ | 平滑切换 | 30 kph 为零交叉点 |
| $\ge 80$ kph | $+1/6\approx+0.167$ | 同相位 | 减小 $\beta$，提高高速变道稳定性 |

后轮转角硬限位 $\pm 6°$。**等效轴距变化**：

$$
R=\frac{L}{(1-k_r)\delta_f}=\frac{L_{\text{eq}}}{\delta_f},\qquad L_{\text{eq}}=\frac{L}{1-k_r}
$$

| $k_r$ | $L_{\text{eq}}/L$ | 表现 |
|-------|------------------|------|
| $-1/6$ | $6/7\approx 0.86$ | 等效轴距缩短 14%，半径减小 14% |
| $0$ | $1$ | 等效 2WS |
| $+1/6$ | $6/5=1.20$ | 等效轴距增加 20%，半径增大 20% |

#### 7.7.7 稳态侧偏角抑制

由 §1.3 小角度近似：

$$
\beta_{ss}\approx\frac{l_r\delta_f^{ss}+l_f\delta_r^{ss}}{L}=\frac{l_r+k_rl_f}{L}\delta_f^{ss}
$$

当 $k_r=-l_r/l_f$ 时 $\beta_{ss}=0$。实际工程常取 $k_r=+1/6$（同向）权衡稳定性与转向能力，并不追求完全零侧偏。

#### 7.7.8 $B_{eq}$ 紧凑形式 → 展开形式

将 (7.5) 中的 $\eta,\xi$ 回代：

| 元素 | 紧凑形式 | 展开形式 |
|------|---------|---------|
| $B_{eq,2}$ | $\dfrac{C_f\eta+k_rC_r\xi}{mI_z}$ | $\dfrac{C_f}{m}-\dfrac{C_fl_fl_r}{I_z}+k_r\!\left(\dfrac{C_r}{m}+\dfrac{C_rl_r^2}{I_z}\right)$ |
| $B_{eq,4}$ | $\dfrac{l_fC_f-k_rl_rC_r}{I_z}$ | 同左（无 $\eta,\xi$） |

#### 7.7.9 退化为 2WS（$k_r=0$）

(7.5) 退化：$B_{eq}\big|_{k_r=0}=B_f$，与第 4 章后轴 2WS 一致。
(7.8)、(7.9) 退化：

$$
e_{2,ss}\big|_{k_r=0}=\frac{ml_fv_x^2}{C_rLR},\qquad \delta_{ff}\big|_{k_r=0}=\frac{L+K_{us}v_x^2}{R}+k_3\,e_{2,ss}
$$

与第 6 章 (6.1)、(6.3) 完全一致 ✓

### 7.8 三种工况对比总结

| 工况 | $\delta_r$ 来源 | $e_{2,ss}$ | $\delta_{ff}^*$（使 $e_{1,ss}=0$） |
|------|----------------|-----------|------------------------------------|
| 第 6 章：纯前轮 | $\delta_r=0$ | $\dfrac{ml_fv_x^2}{C_rLR}$ | $\dfrac{L+K_{us}v_x^2}{R}+k_3e_{2,ss}$ |
| §7.1–7.5：独立 $\delta_r$ | 任意 | $\dfrac{ml_fv_x^2}{C_rLR}-\delta_r$ | $\dfrac{L+K_{us}v_x^2}{R}+\delta_r+k_3e_{2,ss}$ |
| §7.7：比例后轮 | $\delta_r=k_r\delta_f$ | $\dfrac{mv_x^2(C_fl_f-k_rC_rl_r)-k_rC_fC_rL^2}{C_fC_rL(1-k_r)R}$ | $\dfrac{L+K_{us}v_x^2}{(1-k_r)R}+k_3e_{2,ss}$ |

> 三者是同一闭环稳态分析在不同 $\delta_r$ 假设下的特化：比例后轮结果由独立结果代入 $\delta_r^{ss}=k_r\delta_f^{ss}$ 得到，$k_r=0$ 时全部重合于纯前轮情形。


---

## 第 8 章 转向扰动观测器（DOB）

车辆实际作用到前轮的转角与控制器指令之间存在偏差（齿条机械偏置、温度漂移、标定误差、横风等价转向干扰），统称**转向扰动** $\delta_d$。本章基于 §1.8 的二自由度动力学增广扰动状态，构造 3 阶 Luenberger 观测器，估计 $\delta_d$ 用于前馈补偿。

### 8.1 2-DOF 基础模型（前轮转向）

由 §1.8 (1.1)、(1.2) 在 $\delta_r=0$、外加道路横滚 $\phi$ 的影响下：

$$
\dot v_y=-\frac{C_f+C_r}{mv_x}v_y+\!\left(\frac{-C_fl_f+C_rl_r}{mv_x}-v_x\right)r+\frac{C_f}{m}\delta_f-g\phi\tag{8.1}
$$

$$
\dot r=\frac{-C_fl_f+C_rl_r}{I_zv_x}v_y-\frac{C_fl_f^2+C_rl_r^2}{I_zv_x}r+\frac{C_fl_f}{I_z}\delta_f\tag{8.2}
$$

其中 $\phi>0$（左高右低）→ 重力分量向**右**（$-y$ 方向）→ 进入侧向方程为 $-g\phi$。

### 8.2 增广扰动状态 → 3 阶 Luenberger 观测器

将扰动 $\delta_d$ 建模为**等效附加前轮转角**，作为第 3 个状态增广。由于 $\delta_d$ 物理上与 $\delta_f$ 走同一通道，其耦合系数与 $\delta_f$ 完全相同：

$$
\mathbf x=\begin{bmatrix}v_y\\ r\\ \delta_d\end{bmatrix},\qquad \mathbf u=\begin{bmatrix}\delta_f\\ \phi\end{bmatrix}
$$

扰动假设为随机游走 $\dot\delta_d=0$（第 3 行全零）。状态方程：

$$
A=\begin{bmatrix}
-\dfrac{C_f+C_r}{mv_x} & \dfrac{-C_fl_f+C_rl_r}{mv_x}-v_x & \dfrac{C_f}{m}\\[8pt]
\dfrac{-C_fl_f+C_rl_r}{I_zv_x} & -\dfrac{C_fl_f^2+C_rl_r^2}{I_zv_x} & \dfrac{C_fl_f}{I_z}\\[8pt]
0 & 0 & 0
\end{bmatrix}
$$

$$
B=\begin{bmatrix}
\dfrac{C_f}{m} & -g\\[8pt]
\dfrac{C_fl_f}{I_z} & 0\\[8pt]
0 & 0
\end{bmatrix},\qquad C=[0,\;1,\;0]
$$

仅量测 $y=r_{\text{meas}}$（由 IMU 提供）。可观测性：扰动通过链路 $\delta_d\to v_y\to r\to y$ 间接可观，只要 $C_f\neq 0$ 且 $l_f\neq 0$，观测矩阵 $\mathcal{O}=[C;CA;CA^2]$ 满秩。

### 8.3 4WS 扩展

输入扩展为 $\mathbf u=[\delta_f,\delta_r,\phi]^T$。由 §1.8 (1.1)、(1.2) 的 4WS 完整形式：

$$
A=\begin{bmatrix}
-\dfrac{C_f+C_r}{mv_x} & \dfrac{-C_fl_f+C_rl_r}{mv_x}-v_x & \dfrac{C_f}{m}\\[8pt]
\dfrac{-C_fl_f+C_rl_r}{I_zv_x} & -\dfrac{C_fl_f^2+C_rl_r^2}{I_zv_x} & \dfrac{C_fl_f}{I_z}\\[8pt]
0 & 0 & 0
\end{bmatrix}\tag{8.3}
$$

$$
B=\begin{bmatrix}
\dfrac{C_f}{m} & \dfrac{C_r}{m} & -g\\[8pt]
\dfrac{C_fl_f}{I_z} & -\dfrac{C_rl_r}{I_z} & 0\\[8pt]
0 & 0 & 0
\end{bmatrix}\tag{8.4}
$$

> $A$ 矩阵与 §8.2 完全相同——再次印证 §1.9 的结论：4WS → 2WS 退化只影响 $B$，不影响 $A$。

**物理解读**（$B$ 矩阵第 1、2 列）：

| 输入 | $\dot v_y$ 系数 | $\dot r$ 系数 | 含义 |
|------|----------------|----------------|------|
| $\delta_f>0$（左转） | $+C_f/m$（向左） | $+C_fl_f/I_z$（逆时针） | 前轮力 + 逆时针力矩 |
| $\delta_r>0$（左转） | $+C_r/m$（向左） | $-C_rl_r/I_z$（顺时针） | 后轮力 + **反向**力矩 |
| $\phi>0$（左高右低） | $-g$（向右） | $0$ | 重力沿坡面向右 |

### 8.4 定位坐标系变换

工程实现常用约定 $\tilde v_y$ 向右为正（与某些 GPS / IMU 输出方向一致）。变换 $\tilde v_y=-v_y$，仅翻转第 1 个状态：

$$
T=\text{diag}(-1,1,1),\qquad \tilde A=TAT^{-1},\qquad \tilde B=TB
$$

变换后矩阵：

$$
\tilde A=\begin{bmatrix}
-\dfrac{C_f+C_r}{mv_x} & \dfrac{C_fl_f-C_rl_r}{mv_x}+v_x & -\dfrac{C_f}{m}\\[8pt]
\dfrac{C_fl_f-C_rl_r}{I_zv_x} & -\dfrac{C_fl_f^2+C_rl_r^2}{I_zv_x} & \dfrac{C_fl_f}{I_z}\\[8pt]
0 & 0 & 0
\end{bmatrix}\tag{8.5}
$$

$$
\tilde B=\begin{bmatrix}
-\dfrac{C_f}{m} & -\dfrac{C_r}{m} & g\\[8pt]
\dfrac{C_fl_f}{I_z} & -\dfrac{C_rl_r}{I_z} & 0\\[8pt]
0 & 0 & 0
\end{bmatrix}\tag{8.6}
$$

> **变换规律**：$T$ 取反 $A$ 的第 1 行非对角 + 第 1 列非对角元 + $B$ 的第 1 行。横摆行（第 2 行）不变，因为横摆方程不依赖 $v_y$ 的方向约定。$\tilde B_{13}=(-1)\times(-g)=+g$：定位系下 $\phi>0$ 仍意味着重力使车右移（$\tilde v_y$ 向右增大），符号正确。

### 8.5 离散化、增益调度、快慢扰动分离

#### 8.5.1 前向欧拉离散化

$$
A_d=I+AT_s,\qquad B_d=BT_s
$$

#### 8.5.2 预测—校正

$$
\hat{\mathbf x}^-=A_d\hat{\mathbf x}+B_d\mathbf u,\quad e=r_{\text{meas}}-\hat r^-,\quad \hat{\mathbf x}=\hat{\mathbf x}^-+L\cdot e
$$

$L\in\mathbb R^{3\times 1}$ 由极点配置或 LQR 设计。

#### 8.5.3 增益调度

扰动通道 $L_3$ 动态缩放：

$$
L_3^{\text{actual}}=L_3^{\text{base}}\cdot f(v_x)\cdot\max(k_{\text{lc}},k_{\text{mhe}})
$$

- $f(v_x)$：速度插值表（低速增益小、高速增益大）
- $k_{\text{lc}}$：变道附加增益
- $k_{\text{mhe}}$：MHE 辨识工作时附加增益

#### 8.5.4 快慢扰动分离

将新息分为高频（路面冲击瞬态）与低频（缓变中位偏置），分别处理：

$$
\hat r'=\hat r^-+d_{r,\text{last}},\qquad e_{\text{total}}=r_{\text{meas}}-\hat r'
$$

一阶高通：

$$
e_{\text{HP}}=\frac{\tau}{\tau+T_s}(e_{\text{HP,last}}+e_{\text{total}}-e_{\text{total,last}}),\qquad e_{\text{LP}}=e_{\text{total}}-e_{\text{HP}}
$$

$e_{\text{LP}}$ 驱动主观测器，$e_{\text{HP}}$ 驱动独立的横摆扰动项 $d_r$：

$$
d_r=e^{-T_s/\tau}d_{r,\text{last}}+k_{\text{rapid}}e_{\text{HP}},\qquad |d_r|\le d_{\max}
$$

### 8.6 输出限幅与方向盘角度换算

$$
\delta_{\text{disturb}}=\text{Clamp}\!\left(\text{DeltaAngleToStrAng}(\hat\delta_d),\;\pm\delta_{\max}\right)
$$

将估计的等效前轮扰动角换算回方向盘域，便于下游补偿（与现有控制链路接口一致）。

### 8.7 退化为 2WS（$\delta_r=0$）

直接令 $\delta_r$ 列消失，$B$ 退化为：

$$
B^{\text{2WS}}=\begin{bmatrix}
C_f/m & -g\\
C_fl_f/I_z & 0\\
0 & 0
\end{bmatrix}\quad(\text{右手系})
$$

定位系下：

$$
\tilde B^{\text{2WS}}=\begin{bmatrix}
-C_f/m & g\\
C_fl_f/I_z & 0\\
0 & 0
\end{bmatrix}
$$

$A,\tilde A,C$ 与 4WS 完全相同。


---

## 第 9 章 前轮增量化、扰动模型与 MPC 接口

本章把第 4 章的 4WS 后轴误差状态空间、第 7 章的比例后轮假设、第 8 章的扰动观测器输出整合到一起，得到 MPC 控制器实际使用的扩展状态空间。流程：

$$
\text{4 阶后轴 4WS 模型}\xrightarrow{\text{比例后轮+扰动}}\text{4 阶增量模型}\xrightarrow{\text{加积分器}}\text{5 阶 MPC 模型}
$$

### 9.1 信号分层

| 信号 | 含义 | 来源 |
|------|------|------|
| $\delta_{ff}$ | 前馈前轮转角 | 第 7 章 (7.9) |
| $\Delta\delta_f$ | 反馈增量 | MPC 决策 |
| $\delta_f^{\text{cmd}}$ | 前轮指令角，$\delta_f^{\text{cmd}}=\delta_{ff}+\Delta\delta_f$ | 控制器输出 |
| $\delta_d$ | 前轮转角扰动 | 第 8 章 DOB 估计 |
| $\delta_f^{\text{act}}$ | 实际作用到前轮的转角，$\delta_f^{\text{act}}=\delta_f^{\text{cmd}}+\delta_d$ | 物理量 |
| $\delta_r$ | 后轮转角，$\delta_r=k_r\delta_f^{\text{cmd}}$ | 比例随动 |

### 9.2 后轮随动假设的物理细节

$$
\delta_r=k_r\,\delta_f^{\text{cmd}}\quad(\text{跟踪指令而非实际})
$$

**关键设计**：后轮随动器从前轮**指令**取调度信号，**不感知**前轮扰动 $\delta_d$。物理依据：

- 偏置 $\delta_d$（齿条偏置、标定误差）只发生在前轮齿条 / 转向柱处
- 后轮转向系统是独立机构，从 ECU 接收前轮指令值作为输入

这意味着 $\delta_d$ 仅走前轮通道 $B_f$，不被 $k_r$ 缩放。

### 9.3 4 阶增量动力学

由第 4 章 (4.10)–(4.12) 的 4WS 后轴误差模型，代入 $\delta_f^{\text{act}}=\delta_f^{\text{cmd}}+\delta_d$ 和 $\delta_r=k_r\delta_f^{\text{cmd}}$：

$$
\dot{\mathbf x}=A\mathbf x+B_f(\delta_f^{\text{cmd}}+\delta_d)+B_r(k_r\delta_f^{\text{cmd}})+G\dot\theta_{\text{ref}}
$$

$$
=A\mathbf x+(B_f+k_rB_r)\delta_f^{\text{cmd}}+B_f\delta_d+G\dot\theta_{\text{ref}}\tag{9.1}
$$

$$
=A\mathbf x+B_{eq}\,\delta_f^{\text{cmd}}+B_f\,\delta_d+G\,\dot\theta_{\text{ref}}
$$

其中 $B_{eq}$ 见 (7.5)。

#### 平衡点定义

设前馈 $\delta_{ff}$（由第 7 章稳态分析给出）与稳态 $\mathbf x_{ss}$ 满足：

$$
0=A\mathbf x_{ss}+B_{eq}\delta_{ff}+G\,\frac{v_x}{R}\tag{9.2}
$$

**前馈通道不感知 $\delta_d$**：扰动是"误差信号"，只进入反馈环路。

#### 增量动力学

定义增量状态 $\tilde{\mathbf x}=\mathbf x-\mathbf x_{ss}$，将 (9.1) 减去 (9.2)：

$$
\boxed{\;\dot{\tilde{\mathbf x}}=A\,\tilde{\mathbf x}+B_{eq}\,\Delta\delta_f+B_f\,\delta_d\;}\tag{9.3}
$$

**四个关键性质**：

1. **系统矩阵 $A$ 不变**：动力学结构由车辆决定，与平衡点无关。
2. **反馈通道用 $B_{eq}$**：反馈控制器的增量驱动等效输入矩阵。
3. **扰动通道用 $B_f$**：前轮扰动只走前轮物理入口，不被 $k_r$ 缩放。
4. **曲率扰动 $G\dot\theta_{\text{ref}}$ 被前馈消去**：增量方程中没有 $\dot\theta_{\text{ref}}$ 项。

| 项 | 来源 |
|----|------|
| $A\tilde{\mathbf x}$ | 4 阶后轴误差动力学 |
| $B_{eq}\Delta\delta_f$ | 前后轮按 $1:k_r$ 同步增量响应 |
| $B_f\delta_d$ | 前轮独立扰动（无后轮跟随） |

### 9.4 控制通道与扰动通道的解耦

| 通道 | 入口矩阵 | 是否被 $k_r$ 缩放 |
|------|---------|------------------|
| 反馈控制 $\Delta\delta_f$ | $B_{eq}=B_f+k_rB_r$ | 是 |
| 前轮扰动 $\delta_d$ | $B_f$ | **否** |

退化校核：

| 条件 | 结果 |
|------|------|
| $k_r=0$（纯前轮） | $B_{eq}=B_f$，反馈与扰动共用同一通道 |
| $k_r=1$（蟹行） | $B_{eq,4}=(l_fC_f-l_rC_r)/I_z$，$l_f=l_r,C_f=C_r$ 时 $B_{eq,4}=0$，前轮指令完全无法激发横摆 |
| $\eta=0$（惯量临界） | $B_{eq,2}=k_rC_r\xi/(mI_z)$，前轮通道对 $\ddot e_1$ 无直接增益 |

### 9.5 状态扩展为 5 阶

工程上 MPC 通常以**转角变化率** $\Delta\dot\delta_f$ 作为决策变量（而非转角本身），便于约束转角速率、惩罚抖动。这要求把 $\delta_f^{\text{cmd}}$ 加入状态：

$$
\mathbf x=[e_1,\;\dot e_1,\;e_2,\;\dot e_2,\;\delta_f^{\text{cmd}}]^T\in\mathbb R^5,\qquad u=\Delta\dot\delta_f\in\mathbb R
$$

新增积分器：

$$
\dot\delta_f^{\text{cmd}}=\Delta\dot\delta_f
$$

### 9.6 完整 5 阶 ODE（紧凑形式）

合并 (9.1) 与积分器，写成 5 阶状态空间。注意此处使用**绝对状态**（非增量状态）以匹配 MPC 通常的实现（围绕前馈跟踪通过 stage cost 实现，见 §9.8）：

$$
\boxed{\;\dot{\mathbf x}=A_{\text{aug}}\mathbf x+B_u\,\Delta\dot\delta_f+B_d\,\delta_d+G_{\text{aug}}\,\dot\theta_{\text{ref}}\;}\tag{9.4}
$$

各矩阵：

$$
A_{\text{aug}}=\begin{bmatrix}A & B_{eq}\\ \mathbf 0_{1\times 4} & 0\end{bmatrix}\in\mathbb R^{5\times 5}
$$

$$
B_u=\begin{bmatrix}\mathbf 0_{4\times 1}\\ 1\end{bmatrix},\quad B_d=\begin{bmatrix}B_f\\ 0\end{bmatrix},\quad G_{\text{aug}}=\begin{bmatrix}G\\ 0\end{bmatrix}
$$

逐行展开：

| 行 | 方程 |
|----|------|
| 1 | $\dot e_1=\dot e_1$（恒等，状态向量已含 $\dot e_1$） |
| 2 | $\ddot e_1=A_{22}\dot e_1+A_{23}e_2+A_{24}\dot e_2+B_{eq,2}\,\delta_f^{\text{cmd}}+B_{f,2}\,\delta_d+G_2\dot\theta_{\text{ref}}$ |
| 3 | $\dot e_2=\dot e_2$（恒等） |
| 4 | $\ddot e_2=A_{42}\dot e_1+A_{43}e_2+A_{44}\dot e_2+B_{eq,4}\,\delta_f^{\text{cmd}}+B_{f,4}\,\delta_d+G_4\dot\theta_{\text{ref}}$ |
| 5 | $\dot\delta_f^{\text{cmd}}=\Delta\dot\delta_f$ |

第 2、4 行的紧凑展开（直接代入第 4 章 (4.10)–(4.12) 与第 7 章 (7.5) 的元素）：

$$
\ddot e_1=-\frac{C_f\eta+C_r\xi}{mI_zv_x}\dot e_1+\frac{C_f\eta+C_r\xi}{mI_z}e_2-\frac{C_fL\eta}{mI_zv_x}\dot e_2
$$

$$
\quad+\frac{C_f\eta+k_rC_r\xi}{mI_z}\delta_f^{\text{cmd}}+\frac{C_f\eta}{mI_z}\delta_d+\!\left(-\frac{C_fL\eta}{mI_zv_x}-v_x\right)\dot\theta_{\text{ref}}
$$

$$
\ddot e_2=-\frac{l_fC_f-l_rC_r}{I_zv_x}\dot e_1+\frac{l_fC_f-l_rC_r}{I_z}e_2-\frac{l_fC_fL}{I_zv_x}\dot e_2
$$

$$
\quad+\frac{l_fC_f-k_rl_rC_r}{I_z}\delta_f^{\text{cmd}}+\frac{l_fC_f}{I_z}\delta_d-\frac{l_fC_fL}{I_zv_x}\dot\theta_{\text{ref}}
$$

### 9.7 状态雅可比与控制雅可比

$A_{\text{aug}}$ 即状态雅可比 $\partial f/\partial \mathbf x$。其结构为：

$$
A_{\text{aug}}=\begin{bmatrix}
0 & 1 & 0 & 0 & 0\\
0 & A_{22} & A_{23} & A_{24} & B_{eq,2}\\
0 & 0 & 0 & 1 & 0\\
0 & A_{42} & A_{43} & A_{44} & B_{eq,4}\\
0 & 0 & 0 & 0 & 0
\end{bmatrix}\tag{9.5}
$$

**第 5 列**正是 $B_{eq}$——后轮随动假设进入扩展状态空间的入口。

控制雅可比：

$$
B_u=[0,\;0,\;0,\;0,\;1]^T\tag{9.6}
$$

扰动通道（不进入控制器决策梯度，但在仿真和扰动观测器集成时使用）：

$$
B_d=[0,\;B_{f,2},\;0,\;B_{f,4},\;0]^T,\qquad G_{\text{aug}}=[0,\;G_2,\;0,\;G_4,\;0]^T
$$

### 9.8 stage cost 中的 "围绕前馈跟踪"

(9.4) 用绝对状态 $\delta_f^{\text{cmd}}$ 而非偏离平衡点的 $\Delta\delta_f^{\text{cmd}}$。MPC 通过 stage cost 中的 `steer_ref` 与 `d_steer_ref`（前馈分量）实现"围绕前馈跟踪"，等价于 (9.3) 的增量化推导：

$$
\text{stage cost}\supset\big\|\delta_f^{\text{cmd}}-\delta_{ff}\big\|_W^2+\big\|\Delta\dot\delta_f-\Delta\dot\delta_{ff}\big\|_W^2+\sum_i\big\|x_i\big\|_{Q_i}^2
$$

参考值 $\delta_{ff}$ 由第 7 章 (7.9) 生成，$\Delta\dot\delta_{ff}$ 一般取 0（前馈缓变）。

### 9.9 MPC 输入信号的工程组装

工程实现中，MPC 求解器接收的 4 个误差状态由上层模块计算填好。下面给出每一项的标准做法。

#### 9.9.1 $e_1$（后轴投影法）

设车辆后轴中心在世界系下位姿为 $(x_R,y_R,\psi)$，参考路径上离后轴最近的投影点为 $(x_r,y_r)$，对应切向角 $\theta_{\text{ref}}$（含路径航向偏置 $b_\theta$ 修正后的 "true heading"）。

$$
e_1=-\Delta x\sin\theta_{\text{ref}}^{\text{true}}+\Delta y\cos\theta_{\text{ref}}^{\text{true}},\quad (\Delta x,\Delta y)=(x_v-x_r,y_v-y_r)
$$

物理含义：把车辆位置相对参考点的位移向路径**法向**（左正）投影。

> 若上层只提供质心位姿 $(x_{cg},y_{cg},\psi)$，需先回算后轴位置 $x_R=x_{cg}-l_r\cos\psi$，$y_R=y_{cg}-l_r\sin\psi$。

#### 9.9.2 $e_2$（含稳态前馈嵌入）

$$
e_2=\text{wrap}\!\left(\psi-\theta_{\text{ref}}^{\text{true}}+e_{2,ss}\right)
$$

其中 $e_{2,ss}$ 由第 7 章 (7.8) 给出：

$$
e_{2,ss}=\frac{\kappa}{1-k_r}\!\left[-k_rL+\frac{mv_x^2(l_fC_f-k_rl_rC_r)}{C_fC_rL}\right]
$$

**目的**：把稳态偏置当作前馈打包进 MPC 的误差信号——稳态圆弧上 $\psi-\theta_{\text{ref}}^{\text{true}}$ 不归零，但加上 $e_{2,ss}$ 后归零，反馈分量只处理偏离稳态的瞬态。这与 §9.8 "围绕前馈跟踪" 的设计哲学一致。

> **符号约定校验**：$+e_{2,ss}$ 加号成立的前提是 `theta_rear` 的方向定义与 $-(\psi-\theta_{\text{ref}})|_{ss}$ 相同。验证方法：在已知稳态圆弧上跑一段开环数据，确认 `heading_error` 落在 0 附近。

#### 9.9.3 $\dot e_1$（忽略 $v_{yr}$ 的工程取舍）

严格运动学（第 4 章 (4.2)）：

$$
\dot e_1=v_{yr}+v_xe_2=(v_y-l_rr)+v_xe_2
$$

工程实现常用近似：

$$
\boxed{\;\dot e_1\approx v_x\sin(\psi-\theta_{\text{ref}})\;}\tag{9.7}
$$

**取舍**：

- 用 $\sin(\psi-\theta_{\text{ref}})$ 替代 $e_2$，保留大角度精度
- **舍弃 $v_{yr}$**：等价于忽略后轴 sideslip，把后轴速度方向当成与车身纵轴重合
- 优点：只依赖 `theta`、`speed`，不需要 IMU 估计 $v_y$
- 代价：高速大侧偏工况下 $\dot e_1$ 有偏差

> **不需补偿稳态**：$\dot e_1$ 的稳态值本身就是 0（稳态 $\dot{\mathbf x}=0$ 意味着 $\dot e_1\equiv 0$），不像 $e_2$ 需要扣稳态项。

#### 9.9.4 $\dot e_2$（严格运动学）

$$
\boxed{\;\dot e_2=r-\kappa v_x\;}\tag{9.8}
$$

直接对应 (4.3)：`yaw_rate` 即 $r$，`kappa * speed` 即 $\dot\theta_{\text{ref}}$。一对一映射，无近似。

#### 不对称近似一览

工程实现对四个状态采取**有意的不对称处理**：

| 量 | 稳态分量处理 | 运动学近似 |
|----|-------------|-----------|
| $e_2$ | 显式扣除 $e_{2,ss}$ | 严格（仅 wrap） |
| $\dot e_1$ | 不需扣（稳态为 0） | 简化（忽略 $v_{yr}$） |
| $\dot e_2$ | 不需扣（稳态为 0） | 严格 |

稳态圆弧上 MPC 看到的 4 个状态信号近似是 $(e_1,0,0,0)$——所有"已知该有"的稳态偏置都被外部前馈吃掉，MPC 只对**偏离稳态的瞬态误差**做反馈。

### 9.10 紧凑形式 → 展开形式

把 (9.5) 中各元素的 $\eta,\xi,L$ 全部回代。基础元素已在 §4.6、§7.7.8 给出，这里集中列出 $A_{\text{aug}}$ 的非零元：

$$
A_{\text{aug}}=\begin{bmatrix}
0 & 1 & 0 & 0 & 0\\[6pt]
0 & A_{22}^{\text{exp}} & A_{23}^{\text{exp}} & A_{24}^{\text{exp}} & B_{eq,2}^{\text{exp}}\\[6pt]
0 & 0 & 0 & 1 & 0\\[6pt]
0 & A_{42} & A_{43} & A_{44}^{\text{exp}} & B_{eq,4}\\[6pt]
0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

| 元素 | 紧凑形式 | 展开形式 |
|------|---------|---------|
| $A_{22}$ | $-\dfrac{C_f\eta+C_r\xi}{mI_zv_x}$ | $-\dfrac{C_f+C_r}{mv_x}+\dfrac{l_r(C_fl_f-C_rl_r)}{I_zv_x}$ |
| $A_{23}$ | $\dfrac{C_f\eta+C_r\xi}{mI_z}$ | $\dfrac{C_f+C_r}{m}-\dfrac{l_r(C_fl_f-C_rl_r)}{I_z}$ |
| $A_{24}$ | $-\dfrac{C_fL\eta}{mI_zv_x}$ | $\dfrac{C_fl_fl_r(l_f+l_r)}{I_zv_x}-\dfrac{C_f(l_f+l_r)}{mv_x}$ |
| $A_{42}$ | $-\dfrac{l_fC_f-l_rC_r}{I_zv_x}$ | 同左 |
| $A_{43}$ | $\dfrac{l_fC_f-l_rC_r}{I_z}$ | 同左 |
| $A_{44}$ | $-\dfrac{l_fC_fL}{I_zv_x}$ | $-\dfrac{C_fl_f(l_f+l_r)}{I_zv_x}$ |
| $B_{eq,2}$ | $\dfrac{C_f\eta+k_rC_r\xi}{mI_z}$ | $\dfrac{C_f}{m}-\dfrac{C_fl_fl_r}{I_z}+k_r\!\left(\dfrac{C_r}{m}+\dfrac{C_rl_r^2}{I_z}\right)$ |
| $B_{eq,4}$ | $\dfrac{l_fC_f-k_rl_rC_r}{I_z}$ | 同左 |

扰动通道 $B_d$ 的元素（前轮通道，与第 4 章 $B_f$ 相同）：

| 元素 | 紧凑形式 | 展开形式 |
|------|---------|---------|
| $B_{d,2}=B_{f,2}$ | $\dfrac{C_f\eta}{mI_z}$ | $\dfrac{C_f}{m}-\dfrac{C_fl_fl_r}{I_z}$ |
| $B_{d,4}=B_{f,4}$ | $\dfrac{l_fC_f}{I_z}$ | 同左 |

曲率扰动 $G_{\text{aug}}$ 的元素：

| 元素 | 紧凑形式 | 展开形式 |
|------|---------|---------|
| $G_2$ | $-\dfrac{C_fL\eta}{mI_zv_x}-v_x$ | $\dfrac{C_fl_fl_r(l_f+l_r)}{I_zv_x}-\dfrac{C_f(l_f+l_r)}{mv_x}-v_x$ |
| $G_4$ | $-\dfrac{l_fC_fL}{I_zv_x}$ | $-\dfrac{C_fl_f(l_f+l_r)}{I_zv_x}$ |

> **$G_4$ 在代码中的等价写法**：第 4 行的曲率项可写作 $G_4\cdot\dot\theta_{\text{ref}}=G_4\cdot\kappa v_x=-\dfrac{l_fC_fL}{I_z}\kappa$（$v_x$ 抵消）。第 2 行因为 $G_2$ 含"裸" $v_x$ 项，必须保留 $\kappa v_x$ 因子。

### 9.11 退化为 2WS（$k_r=0$）

#### 紧凑形式退化

$B_{eq}\big|_{k_r=0}=B_f$：

$$
B_{eq,2}^{\text{2WS}}=\frac{C_f\eta}{mI_z},\qquad B_{eq,4}^{\text{2WS}}=\frac{l_fC_f}{I_z}
$$

5 阶 ODE 中 $\delta_f^{\text{cmd}}$ 的系数变为前轮单通道：第 2 行 $C_f\eta/(mI_z)$、第 4 行 $l_fC_f/I_z$。

#### 展开形式退化

| 元素 | 4WS（紧凑） | 2WS 紧凑 | 2WS 展开 |
|------|-----------|----------|---------|
| $B_{eq,2}$ | $\dfrac{C_f\eta+k_rC_r\xi}{mI_z}$ | $\dfrac{C_f\eta}{mI_z}$ | $\dfrac{C_f}{m}-\dfrac{C_fl_fl_r}{I_z}$ |
| $B_{eq,4}$ | $\dfrac{l_fC_f-k_rl_rC_r}{I_z}$ | $\dfrac{l_fC_f}{I_z}$ | 同左 |
| $A_{\text{aug}}$ 其它元素 | 见 §9.10 | 同左 | 同左 |
| $B_u$ | $[0,0,0,0,1]^T$ | 同左 | 同左 |
| $B_d$ | $[0,\dfrac{C_f\eta}{mI_z},0,\dfrac{l_fC_f}{I_z},0]^T$ | 同左 | $[0,\dfrac{C_f}{m}-\dfrac{C_fl_fl_r}{I_z},0,\dfrac{l_fC_f}{I_z},0]^T$ |

> **关键观察**：在 5 阶 MPC 模型中，4WS → 2WS 退化只改变 $A_{\text{aug}}$ 的**第 5 列**（即 $B_{eq}$）。$A,A_{\text{aug}}$ 其余元素、$B_u,B_d,G_{\text{aug}}$ 全部保持不变。这是因为：
> - $A$ 描述车辆自由响应，与输入分配无关（贯穿全文的结论）
> - $B_d$ 是前轮扰动通道，与后轮无关
> - $B_u$ 仅作用于积分器
> - $G_{\text{aug}}$ 是曲率扰动，与控制输入无关

### 9.12 与前面章节的衔接

- **稳态前馈**（第 7 章 (7.8)、(7.9)）→ MPC 的 `steer_ref` / `d_steer_ref` 与 $e_2$ 中的稳态项 $e_{2,ss}$
- **扰动观测器**（第 8 章）→ MPC 的 $\delta_d$ 信号
- **后轴误差状态空间**（第 4 章 (4.10)–(4.12)）→ $A_{\text{aug}}$ 的左上 $4\times 4$ 子块
- **比例后轮等效输入**（第 7 章 (7.5)）→ $A_{\text{aug}}$ 的第 5 列
- **积分器** → 第 5 行
- **MPC 输入信号** $e_1,e_2,\dot e_1,\dot e_2$ → §9.9 给出工程组装方法


---

## 附录 A 符号、单位与坐标系总表

### A.1 基本变量

| 符号 | 含义 | 单位 |
|------|------|------|
| $X_c,Y_c$ | 全局坐标 | m |
| $\psi$ | 航向角 | rad |
| $v_x,v_y$ | 质心纵 / 侧向速度 | m/s |
| $V$ | 质心速度大小 | m/s |
| $\beta$ | 质心侧偏角 | rad |
| $r=\dot\psi$ | 横摆角速度 | rad/s |
| $l_f,l_r,L$ | 前 / 后轴距、轴距 $L=l_f+l_r$ | m |
| $\delta_f,\delta_r$ | 前 / 后轮等效转角 | rad |
| $m,I_z$ | 整车质量、横摆转动惯量 | kg, kg·m² |
| $C_f,C_r$ | 前 / 后轴侧偏刚度（$>0$） | N/rad |
| $\alpha_f,\alpha_r$ | 前 / 后轮侧偏角 | rad |
| $\zeta_f,\zeta_r$ | 前 / 后轮速度方向角 | rad |
| $\kappa,R$ | 路径曲率、半径 | 1/m, m |
| $\dot\theta_{\text{ref}}=\kappa v_x$ | 参考航向变化率 | rad/s |
| $e_1,e_2$ | 横向、航向误差 | m, rad |
| $K_{us}=\dfrac{m}{L}\!\left(\dfrac{l_r}{C_f}-\dfrac{l_f}{C_r}\right)$ | 不足转向梯度 | s²/m |
| $\delta_d$ | 前轮转角扰动 | rad |
| $\phi$ | 道路横滚角 | rad |
| $k_r$ | 后轮随动比例 | 无量纲 |

### A.2 紧凑变量与展开

| 紧凑符号 | 展开形式 | 物理意义 |
|----------|---------|---------|
| $\eta$ | $I_z-ml_fl_r$ | 惯量耦合参数 |
| $\xi$ | $I_z+ml_r^2$ | 后轴等效转动惯量 |

化简钥匙：

$$
\frac{1}{m}-\frac{l_fl_r}{I_z}=\frac{\eta}{mI_z},\qquad \frac{1}{m}+\frac{l_r^2}{I_z}=\frac{\xi}{mI_z}
$$

恒等式（在第 6、7 章稳态推导中使用）：

$$
l_f\xi+l_r\eta=l_f(I_z+ml_r^2)+l_r(I_z-ml_fl_r)=LI_z
$$

### A.3 坐标系约定

| 坐标系 | 用途 | 主要章节 |
|--------|------|---------|
| 全局 $OXY$ | 车辆位姿 | 第 1 章 |
| 车体 $Cxy$（右手系，$y$ 向左） | $v_y,r,\delta$ 等 | 全文 |
| Frenet 路径系 | $e_1,e_2$ | 第 3、4 章 |
| 定位坐标系（$\tilde v_y$ 向右为正） | DOB 工程实现 | 第 9.4 节 |

正方向：$v_y$ 左、$r$ 逆时针、$\delta$ 向左转、$\phi$ 左高右低、$F_y$ 向左。

---

## 附录 B 矩阵形式集锦

### B.1 第 4 章 4WS 后轴误差状态空间

#### B.1.1 紧凑形式

$$
A=\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\dfrac{C_f\eta+C_r\xi}{mI_zv_x} & \dfrac{C_f\eta+C_r\xi}{mI_z} & -\dfrac{C_fL\eta}{mI_zv_x} \\
0 & 0 & 0 & 1 \\
0 & -\dfrac{l_fC_f-l_rC_r}{I_zv_x} & \dfrac{l_fC_f-l_rC_r}{I_z} & -\dfrac{l_fC_fL}{I_zv_x}
\end{bmatrix}
$$

$$
B_f=\begin{bmatrix}0\\ \dfrac{C_f\eta}{mI_z}\\ 0\\ \dfrac{l_fC_f}{I_z}\end{bmatrix},\;
B_r=\begin{bmatrix}0\\ \dfrac{C_r\xi}{mI_z}\\ 0\\ -\dfrac{l_rC_r}{I_z}\end{bmatrix},\;
G=\begin{bmatrix}0\\ -\dfrac{C_fL\eta}{mI_zv_x}-v_x\\ 0\\ -\dfrac{l_fC_fL}{I_zv_x}\end{bmatrix}
$$

#### B.1.2 展开形式

$A$ 矩阵第 2 行：

$$
A_{22}=-\frac{C_f+C_r}{mv_x}+\frac{l_r(C_fl_f-C_rl_r)}{I_zv_x},\quad A_{23}=\frac{C_f+C_r}{m}-\frac{l_r(C_fl_f-C_rl_r)}{I_z}
$$

$$
A_{24}=\frac{C_fl_fl_r(l_f+l_r)}{I_zv_x}-\frac{C_f(l_f+l_r)}{mv_x}
$$

$A$ 矩阵第 4 行：

$$
A_{42}=-\frac{l_fC_f-l_rC_r}{I_zv_x},\quad A_{43}=\frac{l_fC_f-l_rC_r}{I_z},\quad A_{44}=-\frac{C_fl_f(l_f+l_r)}{I_zv_x}
$$

输入与扰动：

$$
B_{f,2}=\frac{C_f}{m}-\frac{C_fl_fl_r}{I_z},\quad B_{f,4}=\frac{l_fC_f}{I_z}
$$

$$
B_{r,2}=\frac{C_r}{m}+\frac{C_rl_r^2}{I_z},\quad B_{r,4}=-\frac{l_rC_r}{I_z}
$$

$$
G_2=\frac{C_fl_fl_r(l_f+l_r)}{I_zv_x}-\frac{C_f(l_f+l_r)}{mv_x}-v_x,\quad G_4=-\frac{C_fl_f(l_f+l_r)}{I_zv_x}
$$

#### B.1.3 2WS 退化形式（$\delta_r=0$）

$A$、$B_f$、$G$ 与 4WS 完全相同；$B_r$ 项移除。

### B.2 第 7 章比例后轮等效输入矩阵

| 元素 | 紧凑形式 | 展开形式 | 2WS 退化（$k_r=0$） |
|------|---------|---------|---------------------|
| $B_{eq,2}$ | $\dfrac{C_f\eta+k_rC_r\xi}{mI_z}$ | $\dfrac{C_f}{m}-\dfrac{C_fl_fl_r}{I_z}+k_r\!\left(\dfrac{C_r}{m}+\dfrac{C_rl_r^2}{I_z}\right)$ | $\dfrac{C_f}{m}-\dfrac{C_fl_fl_r}{I_z}=\dfrac{C_f\eta}{mI_z}=B_{f,2}$ |
| $B_{eq,4}$ | $\dfrac{l_fC_f-k_rl_rC_r}{I_z}$ | 同左 | $\dfrac{l_fC_f}{I_z}=B_{f,4}$ |

### B.3 第 8 章 DOB 状态空间

#### B.3.1 4WS 右手系

$$
A=\begin{bmatrix}
-\dfrac{C_f+C_r}{mv_x} & \dfrac{-C_fl_f+C_rl_r}{mv_x}-v_x & \dfrac{C_f}{m}\\
\dfrac{-C_fl_f+C_rl_r}{I_zv_x} & -\dfrac{C_fl_f^2+C_rl_r^2}{I_zv_x} & \dfrac{C_fl_f}{I_z}\\
0 & 0 & 0
\end{bmatrix}
$$

$$
B=\begin{bmatrix}
\dfrac{C_f}{m} & \dfrac{C_r}{m} & -g\\
\dfrac{C_fl_f}{I_z} & -\dfrac{C_rl_r}{I_z} & 0\\
0 & 0 & 0
\end{bmatrix}
$$

#### B.3.2 4WS 定位系

$$
\tilde A=\begin{bmatrix}
-\dfrac{C_f+C_r}{mv_x} & \dfrac{C_fl_f-C_rl_r}{mv_x}+v_x & -\dfrac{C_f}{m}\\
\dfrac{C_fl_f-C_rl_r}{I_zv_x} & -\dfrac{C_fl_f^2+C_rl_r^2}{I_zv_x} & \dfrac{C_fl_f}{I_z}\\
0 & 0 & 0
\end{bmatrix}
$$

$$
\tilde B=\begin{bmatrix}
-\dfrac{C_f}{m} & -\dfrac{C_r}{m} & g\\
\dfrac{C_fl_f}{I_z} & -\dfrac{C_rl_r}{I_z} & 0\\
0 & 0 & 0
\end{bmatrix}
$$

#### B.3.3 2WS 退化（$\delta_r=0$）

$A,\tilde A$ 不变；$B,\tilde B$ 仅保留 $\delta_f$ 列与 $\phi$ 列。

### B.4 第 9 章 5 阶 MPC 状态空间

$$
A_{\text{aug}}=\begin{bmatrix}A & B_{eq}\\ \mathbf 0 & 0\end{bmatrix},\;
B_u=\begin{bmatrix}\mathbf 0\\ 1\end{bmatrix},\;
B_d=\begin{bmatrix}B_f\\ 0\end{bmatrix},\;
G_{\text{aug}}=\begin{bmatrix}G\\ 0\end{bmatrix}
$$

| 矩阵 | 4WS | 2WS（$k_r=0$） |
|------|-----|----------------|
| $A_{\text{aug}}$ 左上 $4\times 4$ | 第 4 章 $A$ | **完全相同** |
| $A_{\text{aug}}$ 第 5 列 | $B_{eq}$（含 $k_r$） | $B_f$ |
| $B_u$ | $[0,0,0,0,1]^T$ | 完全相同 |
| $B_d$ | $[0,B_{f,2},0,B_{f,4},0]^T$ | 完全相同 |
| $G_{\text{aug}}$ | $[0,G_2,0,G_4,0]^T$ | 完全相同 |

---

## 附录 C 现有文档到本文档章节的映射

| 旧文档 | 主题 | 本文章节 |
|--------|------|---------|
| 01_bicycle_model.md | 4WS 自行车模型 | 第 1 章 |
| 02a_error_cg_front_steer.md | 质心 2WS 误差状态空间 + 前馈 | 第 3.5、6.3 节 |
| 02b_error_cg_4ws.md | 质心 4WS 误差状态空间 | 第 3 章 |
| 03a_error_rear_front_steer.md | 后轴 2WS 误差状态空间 + 前馈 | 第 4.7、6.2 节 |
| 03b_error_rear_compact.md | 后轴 2WS 误差状态空间紧凑形式 | 第 4 章（合并） |
| 03c_error_rear_4ws.md | 后轴 4WS 误差状态空间正向推导 | 第 4 章主体 |
| 03d_proportional_rear_incremental_disturbance.md | 比例后轮 + 增量化 | 第 9.3 节 |
| 04_transform_cg_to_rear.md | 质心 ↔ 后轴变换 | 第 4.5 节（仅一致性校验） |
| 05_equivalent_front_steer.md | 等效前轮转角 | 第 2 章 |
| 06_steer_dob_observer.md | 2WS DOB | 第 8.1、8.2 节 |
| 06a/06b/06c_4ws_dob_*.md | 4WS DOB（多路径推导） | 第 8.3、8.4 节 |
| 07_4ws_steering_offset_identification.md 等 | 偏置辨识（独立主题） | 不纳入本文 |
| 08_steer_offset_downstream_impact.md | 下游影响（独立主题） | 不纳入本文 |
| 08_steering_scheduler_analysis.md | $k_r(v_x)$ 调度表 | 第 7.7.6 节 |
| 09_4ws_offset_estimator_implementation.md | 偏置估计实现（独立主题） | 不纳入本文 |
| 99_tmp.md | 公式整理 | 第 6、7 章（旧的整理结果） |
| 99b_steady_state_feedforward.md | 质心 2WS 稳态前馈 | 第 6.3 节 |
| 99c_4ws_steady_state.md | 质心 4WS（$\delta_r=k_r\delta_f$）稳态前馈 | 第 7 章（替换为后轴版本） |
| 100a_rear_axle_steady_state_feedforward.md | 后轴 2WS 稳态前馈 | 第 6.2 节 |
| 100b_4ws_rear_axle_steady_state_feedforward.md | 后轴 4WS（独立 $\delta_r$）稳态前馈 | 第 7.1–7.6 节 |
| 100c_4ws_proportional_rear_steady_state.md | 后轴比例后轮稳态前馈 | 第 7.7 节 |
| 101a_4ws_proportional_rear_incremental_mpc.md | 5 阶 MPC 模型与 autogen 对照 | 第 9 章主体（去除 autogen 比对） |
