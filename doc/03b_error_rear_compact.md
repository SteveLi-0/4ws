# 基于车身方向的后轴误差状态方程

## 误差方程推导

相较于基于质心的误差方程，基于后轴的方程主要差异点在于：误差 $e_1$ 为后轴中心距离后轴 path 的横向距离，$e_2$ 为车身朝向与后轴 path 切线的夹角，要推导此 $e_1$ 与 $e_2$ 的关系。

首先，由后轴的状态量推导动力学方程：

$$
\dot y_r = \dot y - \dot\theta \cdot l_r
$$

前后轴的横向力分别为：

$$
F_{yf} = C_f\!\left(\delta - \frac{\dot y_r + \dot\theta(l_f + l_r)}{v_x}\right)
$$

$$
F_{yr} = -\frac{C_r \dot y_r}{v_x}
$$

则：

$$
ma_y = m(\ddot y + v_x \dot\theta) = m(\ddot y_r + \ddot\theta l_r + v_x \dot\theta) = F_{yf} + F_{yr}
$$

$$
I_z \ddot\theta = l_f F_{yf} - l_r F_{yr}
$$

联立以上式子，可得：

$$
\ddot\theta = \frac{l_f C_f}{I_z}\delta + \frac{l_r C_r - l_f C_f}{I_z v_x}\dot y_r - \frac{l_f C_f(l_f + l_r)}{I_z v_x}\dot\theta
$$

$$
\ddot y_r = \left(\frac{C_f}{m} - \frac{C_f l_f l_r}{I_z}\right)\delta + \left(\frac{-(C_f + C_r)}{m v_x} + \frac{l_r(l_f C_f - l_r C_r)}{I_z v_x}\right)\dot y_r + \left(-v_x - \frac{C_f(l_f + l_r)}{m v_x} + \frac{C_f l_f l_r(l_f + l_r)}{I_z v_x}\right)\dot\theta
$$

又：

$$
\dot e_1 = \dot y_r + v_x(\theta - \theta_{ref})
$$

$$
e_2 = \theta - \theta_{ref}
$$

则以 $e_1$、$\dot e_1$、$e_2$ 和 $\dot e_2$ 为状态量，$\delta$ 为控制量，$\dot\theta_{ref}$ 为扰动项写成矩阵形式：

$$
\dot x = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & \beta & -\beta v_x & \gamma + v_x \\
0 & 0 & 0 & 1 \\
0 & \nu & -\nu v_x & \omega
\end{bmatrix} x
+\begin{bmatrix} 0 \\ \alpha \\ 0 \\ \mu \end{bmatrix} \delta
+\begin{bmatrix} 0 \\ \gamma \\ 0 \\ \omega \end{bmatrix} \dot\theta_{ref}
$$

其中：

$$
\alpha = \frac{C_f}{m} - \frac{C_f l_f l_r}{I_z}
$$

$$
\beta = \frac{-(C_f + C_r)}{m v_x} + \frac{l_r(l_f C_f - l_r C_r)}{I_z v_x}
$$

$$
\gamma = -v_x - \frac{C_f(l_f + l_r)}{m v_x} + \frac{C_f l_f l_r(l_f + l_r)}{I_z v_x}
$$

$$
\mu = \frac{l_f C_f}{I_z}
$$

$$
\nu = \frac{l_r C_r - l_f C_f}{I_z v_x}
$$

$$
\omega = -\frac{l_f C_f(l_f + l_r)}{I_z v_x}
$$

---

## 求稳态偏差和前馈角

设：

$$
\dot x = Ax + B_1 \delta + B_2 \dot\theta_{ref}
$$

$$
\delta = -Kx + \delta_{ff}
$$

$$
K = [k_1,\; k_2,\; k_3,\; k_4]
$$

$$
\dot\theta_{ref} = v_x / R
$$

则：

$$
\dot x = (A - B_1 K)x + B_1 \delta_{ff} + B_2 \dot\theta_{ref}
$$

求：

$$
x_{ss} = -(A - B_1 K)^{-1}(B_1 \delta_{ff} + B_2 v_x / R)
$$

得：

$$
x_{1ss} = \frac{1}{k_1}\!\left(\delta_{ff} - \frac{l_f + l_r}{R} + \frac{m v_x^2\big(C_f l_f(1 - k_3) - C_r l_r\big)}{C_f C_r R(l_f + l_r)}\right)
$$

$$
x_{3ss} = \frac{l_f m v_x^2}{C_r R(l_f + l_r)}
$$

$$
x_{2ss} = 0, \quad x_{4ss} = 0
$$

$x_{3ss}$ 即为稳态时，后轮速度方向和车身方向夹角的负数。
