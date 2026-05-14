# 4WS 自行车模型下的运动学与动力学关系总结

> 符号约定：本文采用**侧偏角取正值定义**：轮胎侧偏角为“车轮指向角减去车轮速度方向角”，即 $\alpha=\delta-\zeta$；侧偏刚度 $C_\alpha>0$，轮胎侧向力写为 $F_y=C_\alpha\alpha$。该写法与部分论文中 $\alpha=\zeta-\delta$、$C_\alpha<0$ 的写法等价，只是符号约定不同。相关模型来源于 4WS 运动学论文和 4WS4WD 动力学论文。 

---

## 1. 坐标系与基本变量

建立两个坐标系：

* 全局坐标系：$OXY$
* 车体坐标系：$Cxy$

其中，$C$ 为车辆质心，车体坐标系 $x$ 轴沿车辆纵向向前，$y$ 轴沿车辆横向向左。

主要变量如下：

| 符号       | 含义                     |
| ---------- | ------------------------ |
| $X_c,Y_c$  | 质心在全局坐标系下的位置 |
| $\psi$     | 车辆航向角               |
| $v_x$      | 质心纵向速度             |
| $v_y$      | 质心侧向速度             |
| $V$        | 质心速度大小             |
| $\beta$    | 质心侧偏角               |
| $r$        | 横摆角速度，$r=\dot\psi$ |
| $l_f$      | 质心到前轴距离           |
| $l_r$      | 质心到后轴距离           |
| $L$        | 轴距，$L=l_f+l_r$        |
| $\delta_f$ | 前轮等效转角             |
| $\delta_r$ | 后轮等效转角             |
| $m$        | 整车质量                 |
| $I_z$      | 车辆绕质心的横摆转动惯量 |

---

## 2. 质心速度、侧偏角与坐标关系

质心速度大小为：

$$
V=\sqrt{v_x^2+v_y^2}
$$

质心侧偏角定义为：

$$
\beta=\arctan\left(\frac{v_y}{v_x}\right)
$$

因此：

$$
v_x=V\cos\beta
$$

$$
v_y=V\sin\beta
$$

车辆速度方向角为：

$$
\chi=\psi+\beta
$$

其中，$\psi$ 表示车身朝向，$\beta$ 表示质心速度方向相对车身纵向的偏转角。

质心在全局坐标系下的速度为：

$$
\dot X_c=v_x\cos\psi-v_y\sin\psi
$$

$$
\dot Y_c=v_x\sin\psi+v_y\cos\psi
$$

也可以写成：

$$
\dot X_c=V\cos(\psi+\beta)
$$

$$
\dot Y_c=V\sin(\psi+\beta)
$$

---

## 3. 前后轮等效转角与 4WS 运动学约束

4WS 自行车模型将左右前轮等效为一个前轮，将左右后轮等效为一个后轮。

前轮等效转角为：

$$
\delta_f
$$

后轮等效转角为：

$$
\delta_r
$$

前轴中心点速度方向满足：

$$
\tan\delta_f=\frac{v_y+l_f r}{v_x}
$$

后轴中心点速度方向满足：

$$
\tan\delta_r=\frac{v_y-l_r r}{v_x}
$$

由上面两式可得横摆角速度：

$$
r=
\frac{v_x}{L}
\left(
\tan\delta_f-\tan\delta_r
\right)
$$

质心侧偏角满足：

$$
\tan\beta=
\frac{v_y}{v_x}
=\frac{
l_r\tan\delta_f+l_f\tan\delta_r
}{L}
$$

因此：

$$
\beta=
\arctan
\left(
\frac{
l_r\tan\delta_f+l_f\tan\delta_r
}{L}
\right)
$$

当质心位于轴距中点，即 $l_f=l_r=\frac{L}{2}$ 时：

$$
\beta=
\arctan
\left(
\frac{\tan\delta_f+\tan\delta_r}{2}
\right)
$$

---

## 4. 4WS 自行车模型运动学方程

完整运动学模型可写为：

$$
\dot X_c=V\cos(\psi+\beta)
$$

$$
\dot Y_c=V\sin(\psi+\beta)
$$

$$
\dot\psi=r
$$

$$
r=
\frac{V\cos\beta}{L}
\left(
\tan\delta_f-\tan\delta_r
\right)
$$

其中：

$$
\beta=
\arctan
\left(
\frac{
l_r\tan\delta_f+l_f\tan\delta_r
}{L}
\right)
$$

---

## 5. 小角度近似关系

当 $\delta_f,\delta_r$ 较小时，有：

$$
\tan\delta_f\approx\delta_f
$$

$$
\tan\delta_r\approx\delta_r
$$

于是：

$$
r\approx
\frac{v_x}{L}
(\delta_f-\delta_r)
$$

$$
\beta\approx
\frac{
l_r\delta_f+l_f\delta_r
}{L}
$$

若 $v_x\approx V$，则：

$$
r\approx
\frac{V}{L}
(\delta_f-\delta_r)
$$

对于质心位于轴距中点的情况：

$$
\beta\approx
\frac{\delta_f+\delta_r}{2}
$$

因此可以总结为：

$$
\delta_f-\delta_r
\quad
\text{主要影响横摆角速度}
$$

$$
\delta_f+\delta_r
\quad
\text{主要影响质心侧偏角和蟹行运动}
$$

---

## 6. 典型 4WS 转向模式

### 6.1 前后轮反相转向

当前后轮反相转向时：

$$
\delta_r=-\delta_f
$$

横摆角速度近似为：

$$
r\approx
\frac{2v_x}{L}\delta_f
$$

若 $l_f=l_r$，则：

$$
\beta\approx 0
$$

此时车辆转弯能力强，转弯半径小，适合低速大曲率转向。

---

### 6.2 前后轮同相转向

当前后轮同相转向时：

$$
\delta_r=\delta_f
$$

横摆角速度近似为：

$$
r\approx 0
$$

质心侧偏角近似为：

$$
\beta\approx \delta_f
$$

此时车辆接近蟹行运动，即车身姿态变化小，但整体运动方向发生侧向偏移。

---

## 7. 车体坐标系下的加速度关系

因为车体坐标系随车辆一起旋转，所以质心真实加速度不是简单的 $\dot v_x,\dot v_y$。

车体坐标系下的纵向加速度为：

$$
a_x=\dot v_x-rv_y
$$

车体坐标系下的侧向加速度为：

$$
a_y=\dot v_y+rv_x
$$

反过来：

$$
\dot v_x=a_x+rv_y
$$

$$
\dot v_y=a_y-rv_x
$$

其中，$rv_y$ 和 $rv_x$ 是由车体坐标系旋转产生的耦合项。

---

## 8. 4WS 自行车动力学基本方程

车辆平面三自由度动力学模型包括纵向、侧向和横摆三个方向。

纵向动力学：

$$
m(\dot v_x-rv_y)=\sum F_x
$$

侧向动力学：

$$
m(\dot v_y+rv_x)=\sum F_y
$$

横摆动力学：

$$
I_z\dot r=\sum M_z
$$

也可以写成：

$$
m a_x=\sum F_x
$$

$$
m a_y=\sum F_y
$$

$$
I_z\dot r=\sum M_z
$$

其中：

* $\sum F_x$ 为车体坐标系下的合纵向力；
* $\sum F_y$ 为车体坐标系下的合侧向力；
* $\sum M_z$ 为绕质心的合横摆力矩。

---

## 9. 轮胎力向车体坐标系的投影

设前轮轮胎坐标系下的纵向力和侧向力分别为：

$$
F_{xf},\quad F_{yf}
$$

后轮轮胎坐标系下的纵向力和侧向力分别为：

$$
F_{xr},\quad F_{yr}
$$

则车体坐标系下的合纵向力为：

$$
\sum F_x = 
F_{xf}\cos\delta_f -
F_{yf}\sin\delta_f +
F_{xr}\cos\delta_r -
F_{yr}\sin\delta_r -
F_w - F_{roll}
$$ 

车体坐标系下的合侧向力为：

$$
\sum F_y = 
F_{xf}\sin\delta_f
+
F_{yf}\cos\delta_f
+
F_{xr}\sin\delta_r
+
F_{yr}\cos\delta_r
$$

其中：

* $F_w$ 为空气阻力；
* $F_{roll}$ 为滚动阻力。

常见近似形式为：

$$
F_w=C_d A\rho V^2
$$

$$
F_{roll}=C_{roll}mg
$$

---

## 10. 横摆力矩关系

前轴对质心产生的横摆力矩为：

$$
M_f = l_f \left(F_{xf}\sin\delta_f + F_{yf}\cos\delta_f\right)
$$

后轴对质心产生的横摆力矩为：

$$
M_r=-l_r\left(
F_{xr}\sin\delta_r+F_{yr}\cos\delta_r
\right)
$$

所以总横摆力矩为：

$$
\sum M_z = l_f 
\left(
F_{xf}\sin\delta_f+F_{yf}\cos\delta_f
\right)
-l_r
\left(
F_{xr}\sin\delta_r+F_{yr}\cos\delta_r
\right)
$$

横摆角加速度为：

$$
\dot r =
\frac{1}{I_z}
\left[
l_f
\left(
F_{xf}\sin\delta_f
+F_{yf}\cos\delta_f
\right)
-l_r
\left(
F_{xr}\sin\delta_r
+
F_{yr}\cos\delta_r
\right)
\right]
$$




若忽略纵向力对横摆力矩的影响，并采用小角度近似，则：

$$
\sum M_z\approx l_fF_{yf}-l_rF_{yr}
$$

$$
\dot r\approx
\frac{l_fF_{yf}-l_rF_{yr}}{I_z}
$$

---

## 11. 轮胎速度方向角

前轮中心速度方向角为：

$$
\zeta_f=
\arctan
\left(
\frac{v_y+l_f r}{v_x}
\right)
$$

后轮中心速度方向角为：

$$
\zeta_r=
\arctan
\left(
\frac{v_y-l_r r}{v_x}
\right)
$$

其中：

* $\zeta_f$ 为前轮速度方向角；
* $\zeta_r$ 为后轮速度方向角。

---

## 12. 侧偏角正值定义

本文采用侧偏角正值定义：

$$
\alpha_f=\delta_f-\zeta_f
$$

$$
\alpha_r=\delta_r-\zeta_r
$$

即：

$$
\alpha_f=
\delta_f-
\arctan
\left(
\frac{v_y+l_f r}{v_x}
\right)
$$

$$
\alpha_r=
\delta_r-
\arctan
\left(
\frac{v_y-l_r r}{v_x}
\right)
$$

小角度下：

$$
\alpha_f\approx
\delta_f-
\frac{v_y+l_f r}{v_x}
$$

$$
\alpha_r\approx
\delta_r-
\frac{v_y-l_r r}{v_x}
$$

---

## 13. 侧偏刚度正值定义

采用正侧偏刚度定义：

$$
C_f>0
$$

$$
C_r>0
$$

其中：

* $C_f$ 为前轮等效侧偏刚度；
* $C_r$ 为后轮等效侧偏刚度。

线性轮胎模型为：

$$
F_{yf}=C_f\alpha_f
$$

$$
F_{yr}=C_r\alpha_r
$$

代入侧偏角后：

$$
F_{yf}
=C_f
\left[
\delta_f-
\arctan
\left(
\frac{v_y+l_f r}{v_x}
\right)
\right]
$$

$$
F_{yr}
=C_r
\left[
\delta_r-
\arctan
\left(
\frac{v_y-l_r r}{v_x}
\right)
\right]
$$

小角度下：

$$
F_{yf}
\approx
C_f
\left(
\delta_f-
\frac{v_y+l_f r}{v_x}
\right)
$$

$$
F_{yr}
\approx
C_r
\left(
\delta_r-
\frac{v_y-l_r r}{v_x}
\right)
$$

---

## 14. 小角度线性化侧向-横摆动力学模型

忽略纵向力对侧向动力学的影响时：

$$
m(\dot v_y+v_x r)=F_{yf}+F_{yr}
$$

$$
I_z\dot r=l_fF_{yf}-l_rF_{yr}
$$

代入线性轮胎模型：

$$
m(\dot v_y+v_x r)
=C_f
\left(
\delta_f-
\frac{v_y+l_f r}{v_x}
\right)
+
C_r
\left(
\delta_r-
\frac{v_y-l_r r}{v_x}
\right)
$$

$$
I_z\dot r
=l_f C_f
\left(
\delta_f-
\frac{v_y+l_f r}{v_x}
\right)
-l_r C_r
\left(
\delta_r-
\frac{v_y-l_r r}{v_x}
\right)
$$

展开后，侧向速度变化率为：

$$
\dot v_y
=\frac{1}{m}
\left[
C_f
\left(
\delta_f-
\frac{v_y+l_f r}{v_x}
\right)
+
C_r
\left(
\delta_r-
\frac{v_y-l_r r}{v_x}
\right)
\right]
-v_x r
$$

横摆角速度变化率为：

$$
\dot r
=\frac{1}{I_z}
\left[
l_f C_f
\left(
\delta_f-
\frac{v_y+l_f r}{v_x}
\right)
-l_r C_r
\left(
\delta_r-
\frac{v_y-l_r r}{v_x}
\right)
\right]
$$

---

## 15. 纵向动力学关系

纵向动力学为：

$$
m(\dot v_x-rv_y)=\sum F_x
$$

因此：

$$
\dot v_x=
\frac{\sum F_x}{m}
+
rv_y
$$

若忽略侧向速度耦合，即 $v_y\approx0$，则：

$$
\dot v_x\approx
\frac{\sum F_x}{m}
$$

若进一步忽略轮胎侧向力对纵向投影的影响，则：

$$
\sum F_x \approx F_{xf}\cos\delta_f+F_{xr}\cos\delta_r - F_w - F_{roll}
$$

小角度下：

$$
\sum F_x
\approx
F_{xf}+F_{xr}-F_w-F_{roll}
$$

---

## 16. 质心速度大小与侧偏角变化率

由：

$$
V=\sqrt{v_x^2+v_y^2}
$$

$$
\beta=\arctan\left(\frac{v_y}{v_x}\right)
$$

可得：

$$
\dot V=
\dot v_x\cos\beta+
\dot v_y\sin\beta
$$

$$
\dot\beta=
\frac{
v_x\dot v_y-v_y\dot v_x
}{V^2}
$$

车辆速度方向角为：

$$
\chi=\psi+\beta
$$

因此：

$$
\dot\chi=r+\dot\beta
$$

这说明车辆的实际运动方向变化由两部分组成：

$$
r
$$

表示车身姿态变化速度；

$$
\dot\beta
$$

表示质心速度方向相对车身方向的变化速度。

---

## 17. 关键关系总表

| 关系              | 公式                                                                  |
| ----------------- | --------------------------------------------------------------------- |
| 质心速度大小      | $V=\sqrt{v_x^2+v_y^2}$                                                |
| 质心侧偏角        | $\beta=\arctan\left(\frac{v_y}{v_x}\right)$                           |
| 纵向速度          | $v_x=V\cos\beta$                                                      |
| 侧向速度          | $v_y=V\sin\beta$                                                      |
| 全局 $X$ 方向速度 | $\dot X_c=v_x\cos\psi-v_y\sin\psi$                                    |
| 全局 $Y$ 方向速度 | $\dot Y_c=v_x\sin\psi+v_y\cos\psi$                                    |
| 横摆角速度        | $r=\dot\psi$                                                          |
| 运动学横摆关系    | $r=\frac{v_x}{L}(\tan\delta_f-\tan\delta_r)$                          |
| 运动学质心侧偏角  | $\beta=\arctan\left(\frac{l_r\tan\delta_f+l_f\tan\delta_r}{L}\right)$ |
| 纵向加速度        | $a_x=\dot v_x-rv_y$                                                   |
| 侧向加速度        | $a_y=\dot v_y+rv_x$                                                   |
| 纵向动力学        | $m(\dot v_x-rv_y)=\sum F_x$                                           |
| 侧向动力学        | $m(\dot v_y+rv_x)=\sum F_y$                                           |
| 横摆动力学        | $I_z\dot r=\sum M_z$                                                  |
| 前轮速度方向角    | $\zeta_f=\arctan\left(\frac{v_y+l_f r}{v_x}\right)$                   |
| 后轮速度方向角    | $\zeta_r=\arctan\left(\frac{v_y-l_r r}{v_x}\right)$                   |
| 前轮侧偏角        | $\alpha_f=\delta_f-\zeta_f$                                           |
| 后轮侧偏角        | $\alpha_r=\delta_r-\zeta_r$                                           |
| 前轮侧向力        | $F_{yf}=C_f\alpha_f$                                                  |
| 后轮侧向力        | $F_{yr}=C_r\alpha_r$                                                  |

---

## 18. 最简状态方程形式

若选取状态变量为：

$$
x=
\begin{bmatrix}
v_y\\
r
\end{bmatrix}
$$

控制输入为：

$$
u=
\begin{bmatrix}
\delta_f\\
\delta_r
\end{bmatrix}
$$

在纵向速度 $v_x$ 近似恒定时，4WS 线性侧向-横摆模型可写为：

$$
\dot v_y
=\frac{1}{m}
\left[
C_f
\left(
\delta_f-
\frac{v_y+l_f r}{v_x}
\right)+C_r
\left(
\delta_r-
\frac{v_y-l_r r}{v_x}
\right)
\right]
-v_x r
$$

$$
\dot r
=\frac{1}{I_z}
\left[
l_f C_f
\left(
\delta_f-
\frac{v_y+l_f r}{v_x}
\right)
-l_r C_r
\left(
\delta_r-
\frac{v_y-l_r r}{v_x}
\right)
\right]
$$

该形式清晰体现了 4WS 车辆中：$\delta_f$和$\delta_r$

同时影响侧向运动与横摆运动。相比传统前轮转向模型，4WS 模型多了后轮转角输入，因此具有更强的轨迹跟踪能力和姿态控制能力。
