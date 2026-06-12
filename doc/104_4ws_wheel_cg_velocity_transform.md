# 4WS 轮速 ↔ 质心速度相互转换公式推导

本文从刚体平面运动学出发，推导四轮转向（4WS）车辆的轮速与质心速度之间的
正向（质心 → 轮）与逆向（轮 → 质心）转换公式。推导不依赖任何具体代码实现，
仅基于运动学第一性原理。

---

## 1. 坐标系与方向定义

### 1.1 车体坐标系

采用固连于车身的右手坐标系 $O\text{–}xyz$：

- 原点 $O$ 取在质心（CG）。
- $x$ 轴：指向车辆**前方**（纵向）。
- $y$ 轴：指向车辆**左侧**（横向）。
- $z$ 轴：竖直**向上**，满足右手定则 $\hat{z} = \hat{x} \times \hat{y}$。

整个推导都在地面投影的二维平面 $x\text{–}y$ 内进行（忽略俯仰、侧倾）。

### 1.2 方向（符号）约定

| 量 | 符号 | 正方向定义 |
|----|------|-----------|
| 纵向速度 | $v_x$ | 沿 $+x$（前进为正，倒车为负） |
| 侧向速度 | $v_y$ | 沿 $+y$（指向左侧为正） |
| 横摆角速度 | $\omega_z$ | 绕 $+z$，**逆时针（左转）为正** |
| 车轮转角 | $\delta_i$ | 车轮纵向（滚动）方向相对 $+x$ 轴的夹角，**逆时针（左偏）为正** |
| 轮速 | $w_i$ | 车轮滚动线速度，沿轮平面纵向，前进为正 |

### 1.3 几何参数与轮位置

| 参数 | 含义 |
|------|------|
| $l_f$ | 质心到前轴的纵向距离（$>0$） |
| $l_r$ | 质心到后轴的纵向距离（$>0$） |
| $L = l_f + l_r$ | 轴距 |
| $l_w$ | 轮距（同轴左右轮中心间距） |

四个车轮中心在车体系下的位置 $(x_i, y_i)$（轮序 FL, FR, RL, RR）：

| 轮 | $x_i$ | $y_i$ |
|----|-------|-------|
| FL（左前） | $+l_f$ | $+l_w/2$ |
| FR（右前） | $+l_f$ | $-l_w/2$ |
| RL（左后） | $-l_r$ | $+l_w/2$ |
| RR（右后） | $-l_r$ | $-l_w/2$ |

4WS 中，前轴等效转角记为 $\delta_f$，后轴等效转角记为 $\delta_r$（单轨/自行车模型
意义下的轴中心转角）。各车轮的实际转角 $\delta_i$ 由 Ackermann 几何修正得到
（见第 4 节）。

---

## 2. 刚体上任一点的速度

车身视为刚体。设质心速度矢量为 $\mathbf{v}_{cg} = (v_x, v_y)$，角速度为
$\boldsymbol{\omega} = \omega_z \hat{z}$。刚体上任一点 $P$（相对质心位置
$\mathbf{r} = (x, y, 0)$）的速度为：

$$
\mathbf{v}_P = \mathbf{v}_{cg} + \boldsymbol{\omega} \times \mathbf{r}
$$

其中

$$
\boldsymbol{\omega} \times \mathbf{r} = \omega_z \hat{z} \times (x, y, 0) = \omega_z\,(-y,\; x,\; 0)
$$

因此位于 $(x_i, y_i)$ 的轮心 $i$ 的速度分量为：

$$
v_{ix} = v_x - \omega_z y_i \quad (\text{纵向分量，沿 } +x)
$$

$$
v_{iy} = v_y + \omega_z x_i \quad (\text{侧向分量，沿 } +y)
$$

代入四个轮位置：

| 轮 | $v_{ix}$ | $v_{iy}$ |
|----|----------|----------|
| FL | $v_x - \omega_z l_w/2$ | $v_y + \omega_z l_f$ |
| FR | $v_x + \omega_z l_w/2$ | $v_y + \omega_z l_f$ |
| RL | $v_x - \omega_z l_w/2$ | $v_y - \omega_z l_r$ |
| RR | $v_x + \omega_z l_w/2$ | $v_y - \omega_z l_r$ |

> 注意：前轴两轮共享侧向分量 $v_y + \omega_z l_f$，后轴两轮共享 $v_y - \omega_z l_r$；
> 左侧两轮共享纵向分量 $v_x - \omega_z l_w/2$，右侧两轮共享 $v_x + \omega_z l_w/2$。

---

## 3. 轮速：轮心速度在轮平面上的投影

车轮转角为 $\delta_i$，则轮平面的**纵向（滚动）**单位方向矢量为
$\mathbf{e}_\parallel = (\cos\delta_i, \sin\delta_i)$，**横向（侧滑）**单位方向矢量为
$\mathbf{e}_\perp = (-\sin\delta_i, \cos\delta_i)$。

车轮在纯滚动假设下，轮速传感器测得的滚动线速度等于轮心速度沿 $\mathbf{e}_\parallel$
的投影：

$$
w_i = \mathbf{v}_i \cdot \mathbf{e}_\parallel = v_{ix}\cos\delta_i + v_{iy}\sin\delta_i
$$

垂直分量给出轮胎侧滑速度（用于侧偏角）：

$$
v_{i,\text{slip}} = \mathbf{v}_i \cdot \mathbf{e}_\perp = -v_{ix}\sin\delta_i + v_{iy}\cos\delta_i
$$

$$
\alpha_i = \operatorname{atan2}\!\left(v_{i,\text{slip}},\; v_{ix}\cos\delta_i + v_{iy}\sin\delta_i\right)
$$

---

## 4. 4WS 各轮转角的 Ackermann 修正

等效单轨转角 $\delta_f, \delta_r$ 给出的是前/后轴**中心**车轮的转角。左右两轮为
保证纯滚动（所有车轮速度方向都垂直于到瞬时转动中心 ICR 的连线），转角需各自修正。

### 4.1 瞬时转动中心（ICR）

设 ICR 在车体系下坐标为 $(X_c, Y_c)$。刚体上任一点 $i$ 的速度方向垂直于该点到
ICR 的连线 $(x_i - X_c,\; y_i - Y_c)$，即速度方向 $\propto (-(y_i - Y_c),\; x_i - X_c)$。
于是该轮转角：

$$
\delta_i = \operatorname{atan2}(x_i - X_c,\; Y_c - y_i) \tag{$*$}
$$

### 4.2 由 $\delta_f$、$\delta_r$ 求 ICR

将 $(*)$ 应用于前、后轴中心（$y = 0$）：

$$
\tan\delta_f = \frac{l_f - X_c}{Y_c}, \qquad \tan\delta_r = \frac{-l_r - X_c}{Y_c}
$$

两式相减消去 $X_c$：

$$
L = l_f + l_r = Y_c\,(\tan\delta_f - \tan\delta_r)
$$

$$
\Rightarrow\quad Y_c = \frac{L}{\tan\delta_f - \tan\delta_r}, \qquad X_c = l_f - Y_c\tan\delta_f \;\; (= -l_r - Y_c\tan\delta_r)
$$

> 前轮转向（$\delta_r = 0$）时 $Y_c = L/\tan\delta_f$，即经典转弯半径关系。
> 反相 4WS（$\delta_r = -\delta_f$）时 $Y_c = L/(2\tan\delta_f)$，转弯半径约为
> 前轮转向的一半。

### 4.3 各轮转角

把 ICR 代回 $(*)$，并利用 $l_f - X_c = Y_c\tan\delta_f$、$-l_r - X_c = Y_c\tan\delta_r$：

$$
\delta_{FL} = \operatorname{atan2}\!\big(Y_c\sin\delta_f,\; (Y_c - l_w/2)\cos\delta_f\big)
$$

$$
\delta_{FR} = \operatorname{atan2}\!\big(Y_c\sin\delta_f,\; (Y_c + l_w/2)\cos\delta_f\big)
$$

$$
\delta_{RL} = \operatorname{atan2}\!\big(Y_c\sin\delta_r,\; (Y_c - l_w/2)\cos\delta_r\big)
$$

$$
\delta_{RR} = \operatorname{atan2}\!\big(Y_c\sin\delta_r,\; (Y_c + l_w/2)\cos\delta_r\big)
$$

**显式表达式**：把 $Y_c = L/(\tan\delta_f - \tan\delta_r)$ 代入。以 $\delta_{FL}$ 为例，
先将 `atan2` 两参数同除以 $\cos\delta_f$ 得 $\operatorname{atan2}(Y_c\tan\delta_f,\; Y_c - l_w/2)$，
再同乘 $(\tan\delta_f - \tan\delta_r)/L$，并记 $h = l_w/(2L)$，得：

$$
\delta_{FL} = \operatorname{atan2}\!\big(\tan\delta_f,\; 1 - h(\tan\delta_f - \tan\delta_r)\big)
$$

$$
\delta_{FR} = \operatorname{atan2}\!\big(\tan\delta_f,\; 1 + h(\tan\delta_f - \tan\delta_r)\big)
$$

$$
\delta_{RL} = \operatorname{atan2}\!\big(\tan\delta_r,\; 1 - h(\tan\delta_f - \tan\delta_r)\big)
$$

$$
\delta_{RR} = \operatorname{atan2}\!\big(\tan\delta_r,\; 1 + h(\tan\delta_f - \tan\delta_r)\big)
$$

四式仅由前/后轴等效转角 $\delta_f, \delta_r$ 与几何常数 $h$ 决定：前轮分子含 $\tan\delta_f$，
后轮分子含 $\tan\delta_r$；左轮分母取 $1 - h(\cdots)$，右轮分母取 $1 + h(\cdots)$。

**验证（前轮转向退化）**：令 $\delta_r = 0$，则 $Y_c = L\cos\delta_f/\sin\delta_f$，
代入 $\delta_{FL}$（记 $h = l_w/(2L)$）：

$$
\delta_{FL} = \operatorname{atan2}(\sin\delta_f,\; \cos\delta_f - h\sin\delta_f), \qquad
\delta_{FR} = \operatorname{atan2}(\sin\delta_f,\; \cos\delta_f + h\sin\delta_f)
$$

与经典前轮 Ackermann 公式一致。

---

## 5. 正向转换：质心速度 → 轮速

把第 2 节的轮心速度分量与第 4 节的各轮转角代入第 3 节投影式：

$$
w_{FL} = (v_x - \omega_z l_w/2)\cos\delta_{FL} + (v_y + \omega_z l_f)\sin\delta_{FL}
$$

$$
w_{FR} = (v_x + \omega_z l_w/2)\cos\delta_{FR} + (v_y + \omega_z l_f)\sin\delta_{FR}
$$

$$
w_{RL} = (v_x - \omega_z l_w/2)\cos\delta_{RL} + (v_y - \omega_z l_r)\sin\delta_{RL}
$$

$$
w_{RR} = (v_x + \omega_z l_w/2)\cos\delta_{RR} + (v_y - \omega_z l_r)\sin\delta_{RR}
$$

即对任意轮 $i$：

$$
\boxed{\,w_i = (v_x - \omega_z y_i)\cos\delta_i + (v_y + \omega_z x_i)\sin\delta_i\,}
$$

---

## 6. 逆向转换：轮速 → 质心速度

实际中横摆角速度 $\omega_z$ 由陀螺仪直接测量，侧向速度 $v_y$ 由车辆动力学/自行车
模型估计（或运动学关系，见 6.3），因此每个轮速可单独反解出一个纵向速度 $v_x$
的估计值。

由 $w_i = (v_x - \omega_z y_i)\cos\delta_i + (v_y + \omega_z x_i)\sin\delta_i$ 解出 $v_x$：

$$
(v_x - \omega_z y_i)\cos\delta_i = w_i - (v_y + \omega_z x_i)\sin\delta_i
$$

$$
\boxed{\,v_x = \frac{w_i - (v_y + \omega_z x_i)\sin\delta_i}{\cos\delta_i} + \omega_z y_i\,}
$$

逐轮展开：

$$
v_x^{FL} = \frac{w_{FL} - (v_y + \omega_z l_f)\sin\delta_{FL}}{\cos\delta_{FL}} + \omega_z l_w/2
$$

$$
v_x^{FR} = \frac{w_{FR} - (v_y + \omega_z l_f)\sin\delta_{FR}}{\cos\delta_{FR}} - \omega_z l_w/2
$$

$$
v_x^{RL} = \frac{w_{RL} - (v_y - \omega_z l_r)\sin\delta_{RL}}{\cos\delta_{RL}} + \omega_z l_w/2
$$

$$
v_x^{RR} = \frac{w_{RR} - (v_y - \omega_z l_r)\sin\delta_{RR}}{\cos\delta_{RR}} - \omega_z l_w/2
$$

> 工程实现中 $\cos\delta_i$ 需做除零保护（转角接近 $\pm 90^\circ$ 时）。
> 四个 $v_x^{(i)}$ 通常再经鲁棒融合（如 Huber 加权 / 取低滑移轮）得到最终质心纵向速度。

### 6.1 与正向的互逆性

逆向式由正向式直接代数求解得到，二者严格互逆。例如左前轮：

正向 $w_{FL} = (v_x - \omega_z l_w/2)\cos\delta_{FL} + (v_y + \omega_z l_f)\sin\delta_{FL}$，
反解即得

$$
v_x^{FL} = \frac{w_{FL} - (v_y + \omega_z l_f)\sin\delta_{FL}}{\cos\delta_{FL}} + \omega_z l_w/2
$$

### 6.2 倒车方向

引入方向符号 $s = +1$（前进）/ $-1$（倒车）。轮速传感器恒输出非负的轮转速幅值，
质心纵向速度的符号由行驶方向给出。此时把上式中与方向相关的项乘以 $s$：

$$
v_x = \frac{w_i - s\,(v_y + \omega_z x_i)\sin\delta_i}{\cos\delta_i} + s\,\omega_z y_i
$$

并在最终对 $v_x$ 取符号 $s$。前进时退化为第 6 节标准式。

### 6.3 运动学侧向速度（可选）

若采用纯运动学（低速、无侧滑）模型，质心速度也可直接由 ICR 给出。质心位于
$(0,0)$，故：

$$
\mathbf{v}_{cg} = \omega_z \hat{z} \times (0 - X_c,\; 0 - Y_c) = \omega_z\,(Y_c,\; -X_c)
$$

$$
\Rightarrow\quad v_x = \omega_z Y_c, \qquad v_y = -\omega_z X_c
$$

即此时 $v_y = -\omega_z X_c$，其中 $X_c = l_f - Y_c\tan\delta_f$、
$Y_c = L/(\tan\delta_f - \tan\delta_r)$。动力学场景下 $v_y$ 应改用考虑轮胎侧偏刚度的
自行车稳态模型。

---

## 7. 小结

- **正向（质心 → 轮）**：$w_i = (v_x - \omega_z y_i)\cos\delta_i + (v_y + \omega_z x_i)\sin\delta_i$
- **逆向（轮 → 质心）**：$v_x = \dfrac{w_i - (v_y + \omega_z x_i)\sin\delta_i}{\cos\delta_i} + \omega_z y_i$
- 4WS 相比前轮转向，额外引入后轴转角 $\delta_r$，使后轮也有非零 $\delta_{RL}, \delta_{RR}$，
  ICR 横坐标 $X_c$ 不再固定在后轴；前轮转向是 $\delta_r = 0$ 的特例。
- 左右轮通过 $\pm\omega_z l_w/2$（纵向）区分，前后轴通过 $+\omega_z l_f$、$-\omega_z l_r$
  （侧向）区分；各轮转角由 ICR 经 Ackermann 几何统一确定。
