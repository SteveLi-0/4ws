# 4WS 等效前轮转角建模

> 目标：将四轮转向（4WS）车辆的前轮转角 $\delta_f$ 和后轮转角 $\delta_r$ 等效为一个单一的等效前轮转角 $\delta_{eq}$，使得等效后的模型在特定意义下与原 4WS 模型行为一致。

---

## 1. 问题定义

4WS 自行车模型中，前后轮均可转向，输入为 $(\delta_f, \delta_r)$。

标准前轮转向（2WS）自行车模型中，仅前轮可转向，后轮转角为零，输入为单一转角 $\delta_{eq}$。

我们希望找到一个等效前轮转角 $\delta_{eq}$，使得 2WS 模型在某种意义上等价于原始 4WS 模型。

---

## 2. 运动学等效

### 2.1 4WS 运动学横摆角速度

由 4WS 自行车模型运动学关系：

$$
r = \frac{v_x}{L}(\tan\delta_f - \tan\delta_r)
$$

### 2.2 标准 2WS 运动学横摆角速度

对于仅前轮转向的标准自行车模型（后轮转角为零）：

$$
r = \frac{v_x}{L}\tan\delta_{eq}
$$

### 2.3 运动学等效条件

令两者横摆角速度相等：

$$
\frac{v_x}{L}\tan\delta_{eq} = \frac{v_x}{L}(\tan\delta_f - \tan\delta_r)
$$

得到：

$$
\boxed{\tan\delta_{eq} = \tan\delta_f - \tan\delta_r}
$$

即：

$$
\delta_{eq} = \arctan(\tan\delta_f - \tan\delta_r)
$$

小角度近似下：

$$
\boxed{\delta_{eq} \approx \delta_f - \delta_r}
$$

### 2.4 运动学等效的含义

该等效保证了：
- 等效模型与原 4WS 模型具有**相同的瞬时转弯半径**
- 等效模型与原 4WS 模型具有**相同的横摆角速度**

但需注意：运动学等效**不保证质心侧偏角相同**。

原 4WS 模型的质心侧偏角为：

$$
\beta_{4WS} = \arctan\left(\frac{l_r\tan\delta_f + l_f\tan\delta_r}{L}\right)
$$

等效 2WS 模型的质心侧偏角为：

$$
\beta_{2WS} = \arctan\left(\frac{l_r\tan\delta_{eq}}{L}\right)
= \arctan\left(\frac{l_r(\tan\delta_f - \tan\delta_r)}{L}\right)
$$

两者之差为：

$$
\beta_{4WS} - \beta_{2WS} = \arctan\left(\frac{l_r\tan\delta_f + l_f\tan\delta_r}{L}\right) - \arctan\left(\frac{l_r(\tan\delta_f - \tan\delta_r)}{L}\right)
$$

小角度近似下：

$$
\beta_{4WS} - \beta_{2WS} \approx \frac{l_r\delta_f + l_f\delta_r}{L} - \frac{l_r(\delta_f - \delta_r)}{L} = \frac{(l_f + l_r)\delta_r}{L} = \delta_r
$$

即质心侧偏角之差近似等于后轮转角 $\delta_r$。

---

## 3. 动力学等效

### 3.1 4WS 线性动力学模型

4WS 小角度线性模型的侧向力与横摆力矩为：

$$
\sum F_y = C_f\left(\delta_f - \frac{v_y + l_f r}{v_x}\right) + C_r\left(\delta_r - \frac{v_y - l_r r}{v_x}\right)
$$

$$
\sum M_z = l_f C_f\left(\delta_f - \frac{v_y + l_f r}{v_x}\right) - l_r C_r\left(\delta_r - \frac{v_y - l_r r}{v_x}\right)
$$

### 3.2 标准 2WS 线性动力学模型

对于仅前轮转向的标准 2WS 模型（后轮转角为零）：

$$
\sum F_y = C_f\left(\delta_{eq} - \frac{v_y + l_f r}{v_x}\right) + C_r\left(0 - \frac{v_y - l_r r}{v_x}\right)
$$

$$
\sum M_z = l_f C_f\left(\delta_{eq} - \frac{v_y + l_f r}{v_x}\right) - l_r C_r\left(0 - \frac{v_y - l_r r}{v_x}\right)
$$

### 3.3 侧向力等效条件

令 4WS 与 2WS 的合侧向力相等：

$$
C_f\left(\delta_{eq} - \frac{v_y + l_f r}{v_x}\right) - C_r\frac{v_y - l_r r}{v_x}
= C_f\left(\delta_f - \frac{v_y + l_f r}{v_x}\right) + C_r\left(\delta_r - \frac{v_y - l_r r}{v_x}\right)
$$

化简得：

$$
C_f\delta_{eq} = C_f\delta_f + C_r\delta_r
$$

因此基于侧向力等效的等效前轮转角为：

$$
\boxed{\delta_{eq}^{(F)} = \delta_f + \frac{C_r}{C_f}\delta_r}
$$

### 3.4 横摆力矩等效条件

令 4WS 与 2WS 的横摆力矩相等：

$$
l_f C_f\left(\delta_{eq} - \frac{v_y + l_f r}{v_x}\right) + l_r C_r\frac{v_y - l_r r}{v_x}
= l_f C_f\left(\delta_f - \frac{v_y + l_f r}{v_x}\right) - l_r C_r\left(\delta_r - \frac{v_y - l_r r}{v_x}\right)
$$

化简得：

$$
l_f C_f\delta_{eq} = l_f C_f\delta_f - l_r C_r\delta_r
$$

因此基于横摆力矩等效的等效前轮转角为：

$$
\boxed{\delta_{eq}^{(M)} = \delta_f - \frac{l_r C_r}{l_f C_f}\delta_r}
$$

### 3.5 不可能同时满足两个等效条件

注意到 $\delta_{eq}^{(F)} \neq \delta_{eq}^{(M)}$（除非 $\delta_r = 0$），这说明：

> 用单一等效前轮转角**无法同时**精确等效 4WS 模型的侧向力和横摆力矩。

这是因为 4WS 模型有两个独立输入 $(\delta_f, \delta_r)$，而 2WS 模型只有一个输入 $\delta_{eq}$，自由度不匹配。

---

## 4. 稳态等效

### 4.1 稳态条件

在稳态圆周运动中 $\dot v_y = 0$，$\dot r = 0$。此时可以联立侧向和横摆两个方程求解稳态横摆角速度与转角的关系。

### 4.2 4WS 稳态横摆角速度

由稳态侧向动力学方程：

$$
m v_x r = C_f\left(\delta_f - \frac{v_y + l_f r}{v_x}\right) + C_r\left(\delta_r - \frac{v_y - l_r r}{v_x}\right)
$$

由稳态横摆动力学方程：

$$
0 = l_f C_f\left(\delta_f - \frac{v_y + l_f r}{v_x}\right) - l_r C_r\left(\delta_r - \frac{v_y - l_r r}{v_x}\right)
$$

由横摆方程可得：

$$
l_f C_f\alpha_f = l_r C_r\alpha_r
$$

其中 $\alpha_f = \delta_f - \frac{v_y + l_f r}{v_x}$，$\alpha_r = \delta_r - \frac{v_y - l_r r}{v_x}$。

利用 $\alpha_f - \alpha_r = (\delta_f - \delta_r) - \frac{Lr}{v_x}$，以及 $l_f C_f\alpha_f = l_r C_r\alpha_r$ 联立求解：

$$
\alpha_f = \frac{l_r C_r}{l_f C_f + l_r C_r}\left[(\delta_f - \delta_r) - \frac{Lr}{v_x}\right]
$$

$$
\alpha_r = \frac{l_f C_f}{l_f C_f + l_r C_r}\left[(\delta_f - \delta_r) - \frac{Lr}{v_x}\right]
$$

代入侧向方程：

$$
m v_x r = (C_f + C_r)\cdot\frac{l_f C_f + l_r C_r \cdot \frac{C_f}{C_f}}{...}
$$

更直接地，将轮胎力代入侧向方程：

$$
m v_x r = C_f\alpha_f + C_r\alpha_r
$$

利用 $l_f C_f\alpha_f = l_r C_r\alpha_r$，设 $\alpha_f = k$，则 $\alpha_r = \frac{l_f C_f}{l_r C_r}k$：

$$
m v_x r = C_f k + C_r\cdot\frac{l_f C_f}{l_r C_r}k = C_f k\left(1 + \frac{l_f}{l_r}\right) = C_f k\cdot\frac{L}{l_r}
$$

同时：

$$
\alpha_f - \alpha_r = k\left(1 - \frac{l_f C_f}{l_r C_r}\right) = (\delta_f - \delta_r) - \frac{Lr}{v_x}
$$

由此可求得稳态横摆增益。为简化表达，定义：

$$
K_u = \frac{m}{L}\left(\frac{l_r}{C_f \cdot L} - \frac{l_f}{C_r \cdot L}\right) = \frac{m(l_r C_r - l_f C_f)}{C_f C_r L^2}
$$

（注意：此处 $K_u$ 为不足转向梯度的一种表达形式）

则 4WS 稳态横摆增益为：

$$
\frac{r}{\delta_f - \delta_r} = \frac{v_x / L}{1 + K_u v_x^2}
$$

### 4.3 稳态等效前轮转角

由于标准 2WS 模型的稳态横摆增益为：

$$
\frac{r}{\delta_{eq}} = \frac{v_x / L}{1 + K_u v_x^2}
$$

（$K_u$ 相同，因为它只取决于车辆参数而不取决于转角输入）

因此，在稳态意义下：

$$
\boxed{\delta_{eq}^{(ss)} = \delta_f - \delta_r}
$$

这与运动学等效的小角度结果一致。

---

## 5. 各等效定义总结

| 等效方式 | 等效前轮转角 $\delta_{eq}$ | 保证相同的量 |
| -------- | -------------------------- | ------------ |
| 运动学等效（瞬时转弯半径） | $\delta_f - \delta_r$ | 横摆角速度 $r$ |
| 侧向力等效 | $\delta_f + \frac{C_r}{C_f}\delta_r$ | 合侧向力 $\sum F_y$ |
| 横摆力矩等效 | $\delta_f - \frac{l_r C_r}{l_f C_f}\delta_r$ | 合横摆力矩 $\sum M_z$ |
| 稳态横摆增益等效 | $\delta_f - \delta_r$ | 稳态横摆角速度 |

---

## 6. 实用建议

### 6.1 推荐使用运动学等效

在大多数工程应用中，推荐使用：

$$
\delta_{eq} = \delta_f - \delta_r
$$

理由如下：
1. 物理含义清晰：等效了瞬时转弯半径和横摆角速度
2. 与稳态分析结论一致
3. 形式简单，便于实时计算
4. 在小角度范围内精度足够

### 6.2 等效后的运动学模型

使用等效前轮转角后，4WS 运动学模型退化为标准前轮转向自行车模型：

$$
\dot X_c = V\cos(\psi + \beta_{eq})
$$

$$
\dot Y_c = V\sin(\psi + \beta_{eq})
$$

$$
\dot\psi = r = \frac{V\cos\beta_{eq}}{L}\tan\delta_{eq}
$$

其中：

$$
\beta_{eq} = \arctan\left(\frac{l_r\tan\delta_{eq}}{L}\right)
$$

### 6.3 等效的局限性

使用等效前轮转角会丢失以下信息：

1. **质心侧偏角差异**：等效模型的 $\beta$ 与原 4WS 模型不同，差异约为 $\delta_r$
2. **瞬态响应差异**：在动力学层面无法同时匹配侧向力和横摆力矩
3. **轮胎力分配信息**：前后轮各自的侧偏角和侧向力信息丢失

因此，等效前轮转角适用于：
- 路径规划与轨迹跟踪的简化模型
- 车辆状态估计中的降阶观测器
- 控制器初步设计阶段

不适用于：
- 需要精确描述车辆姿态（侧偏角）的场景
- 轮胎力分配与稳定性分析
- 极限工况下的动力学仿真
