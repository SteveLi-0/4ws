# 公式整理

令 $L = l_f + l_r$，$C_{\alpha f} = C_f$，$C_{\alpha r} = C_r$。

---

## 公式 1

### 原始形式

$$
\frac{-(C_f C_r l_f^2 + C_f C_r l_r^2 - C_f C_r k_3 l_r^2 - C_f l_f m v_x^2 + C_r l_r m v_x^2 + 2C_f C_r l_f l_r - C_f C_r R \delta_{ff} l_f - C_f C_r R \delta_{ff} l_r - C_f C_r k_3 l_f l_r + C_f k_3 l_f m v_x^2)}{C_f R k_1 (C_r l_f + C_r l_r)}
$$

### 整理形式

$$
\boxed{
\frac{1}{k_1} \left( \delta_{ff} - \left[ \frac{L}{R} + \frac{v_x^2}{R} \left( \frac{m l_r}{C_{\alpha f} L} - \frac{m l_f}{C_{\alpha r} L} \right) - k_3 \left( \frac{l_r}{R} - \frac{l_f}{C_{\alpha r}} \frac{m v_x^2}{R L} \right) \right] \right)
}
$$

**结构解读**：

$$
\frac{1}{k_1}\Big(\underbrace{\delta_{ff}}_{\text{前馈转角}} - \underbrace{\Big[\frac{L}{R} + \frac{v_x^2}{R}\Big(\frac{ml_r}{C_{\alpha f}L} - \frac{ml_f}{C_{\alpha r}L}\Big) - k_3\Big(\frac{l_r}{R} - \frac{l_f}{C_{\alpha r}}\frac{mv_x^2}{RL}\Big)\Big]}_{\delta_{ss}\text{：稳态所需前轮转角}}\Big)
$$

| 项 | 表达式 | 含义 |
|----|--------|------|
| 几何稳态转角 | $\dfrac{L}{R}$ | Ackermann 纯几何转角 |
| 不足转向梯度 | $\dfrac{v_x^2}{R}\left(\dfrac{ml_r}{C_{\alpha f}L} - \dfrac{ml_f}{C_{\alpha r}L}\right)$ | 前后轮侧偏刚度差异导致的动力学修正 |
| 后轮转向修正 | $-k_3\left(\dfrac{l_r}{R} - \dfrac{l_f}{C_{\alpha r}}\dfrac{mv_x^2}{RL}\right)$ | 后轮主动转向减小所需前轮转角 |

---

## 公式 2

### 原始形式

$$
\frac{-(C_r l_r^2 + C_r l_f l_r - l_f m v_x^2)}{R(C_r l_f + C_r l_r)}
$$

### 整理形式

$$
\boxed{
-\frac{l_r}{R} + \frac{l_f}{C_{\alpha r}(l_f + l_r)} \cdot \frac{m v_x^2}{R}
}
$$

| 项 | 表达式 | 含义 |
|----|--------|------|
| 几何项 | $-\dfrac{l_r}{R}$ | 后轴相对圆心的纯几何角度关系 |
| 动力学修正 | $\dfrac{l_f}{C_{\alpha r} L} \cdot \dfrac{m v_x^2}{R}$ | 后轮侧偏角贡献（离心力 → 后轮侧偏 → 侧向速度） |

---

## 公式 3

### 原始形式

$$
\frac{C_f C_r l_f^2 + 2C_f C_r l_f l_r - C_f m l_f v_x^2 + C_f C_r l_r^2 + C_r m l_r v_x^2}{C_f C_r R \delta_{ff} k_1 (l_f + l_r)(k_r - 1)}
$$

$$
- \frac{C_f C_r k_3 l_r^2 + C_f C_r R l_f + C_f C_r R l_r - C_f C_r R k_r l_f - C_f C_r R k_r l_r + C_f C_r k_3 l_f l_r + C_f C_r k_3 k_r l_f^2 - C_f k_3 l_f m v_x^2 + C_r k_3 k_r l_r m v_x^2 + C_f C_r k_3 k_r l_f l_r}{C_f C_r R k_1 (l_f + l_r)(k_r - 1)}
$$

### 整理形式

$$
\boxed{
\frac{1}{k_1(k_r - 1)} \left( (k_r - 1) + \frac{\delta_{ss}}{\delta_{ff}} - k_3 \left[ \frac{l_r + k_r l_f}{R} + \frac{v_x^2}{R} \left( \frac{k_r m l_r}{C_{\alpha f} L} - \frac{m l_f}{C_{\alpha r} L} \right) \right] \right)
}
$$

其中**稳态转角**（与公式 1 中方括号内前两项相同）：

$$
\delta_{ss} = \frac{L}{R} + \frac{v_x^2}{R} \left( \frac{m l_r}{C_{\alpha f} L} - \frac{m l_f}{C_{\alpha r} L} \right)
$$

### 等价展开形式

$$
= \frac{1}{k_1} + \frac{1}{k_1(k_r - 1)} \left\{ \frac{1}{\delta_{ff}} \left[ \frac{L}{R} + \frac{v_x^2}{R} \left( \frac{m l_r}{C_{\alpha f} L} - \frac{m l_f}{C_{\alpha r} L} \right) \right] - k_3 \left[ \frac{l_r + k_r l_f}{R} + \frac{v_x^2}{R} \left( \frac{k_r m l_r}{C_{\alpha f} L} - \frac{m l_f}{C_{\alpha r} L} \right) \right] \right\}
$$

| 项 | 表达式 | 含义 |
|----|--------|------|
| 常数项 | $k_r - 1$ | 后轮转向比例 $k_r$ 的偏移 |
| 稳态比 | $\dfrac{\delta_{ss}}{\delta_{ff}}$ | 稳态所需转角与前馈转角之比 |
| $k_3$ 修正 | $k_3\left[\dfrac{l_r + k_r l_f}{R} + \dfrac{v_x^2}{R}\left(\dfrac{k_r m l_r}{C_{\alpha f} L} - \dfrac{m l_f}{C_{\alpha r} L}\right)\right]$ | 后轮转向 $k_3$ 与 $k_r$ 耦合的动力学修正 |

---

## 公式间关系

- 公式 2 恰好是公式 1 中 $k_3$ 项括号内容取负：$-\left(\dfrac{l_r}{R} - \dfrac{l_f m v_x^2}{C_{\alpha r} R L}\right)$
- 公式 1 中的 $\delta_{ss}$ 在公式 3 中以 $\delta_{ss}/\delta_{ff}$ 的形式复现
- 公式 3 的 $k_3$ 括号是公式 1 中 $k_3$ 括号的 $k_r$ 加权推广

---

## 验证脚本

```python
"""
用 SymPy 验证三个公式整理前后的代数等价性。
运行: python3 此脚本
期望输出: 三个 0
"""
from sympy import symbols, simplify

Cf, Cr, lf, lr, m, vx, k1, k3, kr, R, delta_ff = symbols(
    'Cf Cr lf lr m vx k1 k3 kr R delta_ff')
L = lf + lr

# ===== 公式 1 =====
orig1 = -(Cf*Cr*lf**2 + Cf*Cr*lr**2 - Cf*Cr*k3*lr**2 - Cf*lf*m*vx**2
          + Cr*lr*m*vx**2 + 2*Cf*Cr*lf*lr - Cf*Cr*R*delta_ff*lf
          - Cf*Cr*R*delta_ff*lr - Cf*Cr*k3*lf*lr
          + Cf*k3*lf*m*vx**2) / (Cf*R*k1*(Cr*lf + Cr*lr))

tidy1 = (1/k1) * (delta_ff - L/R
         - vx**2/R * (m*lr/(Cf*L) - m*lf/(Cr*L))
         + k3*(lr/R - lf*m*vx**2/(Cr*R*L)))

print('公式 1 验证: orig - tidy =', simplify(orig1 - tidy1))

# ===== 公式 2 =====
orig2 = -(Cr*lr**2 + Cr*lf*lr - lf*m*vx**2) / (R*(Cr*lf + Cr*lr))

tidy2 = -lr/R + lf*m*vx**2/(Cr*R*L)

print('公式 2 验证: orig - tidy =', simplify(orig2 - tidy2))

# ===== 公式 3 =====
t1 = (Cf*Cr*lf**2 + 2*Cf*Cr*lf*lr - Cf*m*lf*vx**2
      + Cf*Cr*lr**2 + Cr*m*lr*vx**2) / (
      Cf*Cr*R*delta_ff*k1*(lf + lr)*(kr - 1))

t2 = (Cf*Cr*k3*lr**2 + Cf*Cr*R*lf + Cf*Cr*R*lr
      - Cf*Cr*R*kr*lf - Cf*Cr*R*kr*lr
      + Cf*Cr*k3*lf*lr + Cf*Cr*k3*kr*lf**2
      - Cf*k3*lf*m*vx**2 + Cr*k3*kr*lr*m*vx**2
      + Cf*Cr*k3*kr*lf*lr) / (
      Cf*Cr*R*k1*(lf + lr)*(kr - 1))

orig3 = t1 - t2

delta_ss = L/R + vx**2/R * (m*lr/(Cf*L) - m*lf/(Cr*L))

tidy3 = 1/(k1*(kr - 1)) * (
    (kr - 1)
    + delta_ss / delta_ff
    - k3 * ((lr + kr*lf)/R + vx**2/R*(kr*m*lr/(Cf*L) - m*lf/(Cr*L)))
)

print('公式 3 验证: orig - tidy =', simplify(orig3 - tidy3))
```


---

## 公式 3 与公式 1 在 $k_r = 0$ 时的关系

### 结论

**公式 3 在 $k_r = 0$ 时与公式 1 不等价。**

差异来源：公式 3 的第一项分母中含有 $\delta_{ff}$，导致 $(\delta_{ff} - \delta_{ss})$ 项被 $\delta_{ff}$ 归一化。

具体地：

$$
\text{公式3}|_{k_r=0} = \frac{1}{k_1} \left( 1 - \frac{\delta_{ss}}{\delta_{ff}} + k_3 \left( \frac{l_r}{R} - \frac{l_f m v_x^2}{C_{\alpha r} R L} \right) \right)
$$

$$
\text{公式1} = \frac{1}{k_1} \left( \delta_{ff} - \delta_{ss} + k_3 \left( \frac{l_r}{R} - \frac{l_f m v_x^2}{C_{\alpha r} R L} \right) \right)
$$

两者的差值：

$$
\text{公式3}|_{k_r=0} - \text{公式1} = \frac{(\delta_{ff} - \delta_{ss})(1 - \delta_{ff})}{k_1 \cdot \delta_{ff}}
$$

**特殊情况**：当 $\delta_{ff} = 1$ 时两者相等（差值为零）。

### 验证脚本

```python
from sympy import symbols, simplify

Cf, Cr, lf, lr, m, vx, k1, k3, R, delta_ff = symbols(
    'Cf Cr lf lr m vx k1 k3 R delta_ff')
L = lf + lr

# 公式 1
tidy1 = (1/k1) * (delta_ff - L/R
         - vx**2/R * (m*lr/(Cf*L) - m*lf/(Cr*L))
         + k3*(lr/R - lf*m*vx**2/(Cr*R*L)))

# 公式 3, kr=0
delta_ss = L/R + vx**2/R * (m*lr/(Cf*L) - m*lf/(Cr*L))
tidy3_kr0 = 1/(k1*(0 - 1)) * (
    (0 - 1)
    + delta_ss / delta_ff
    - k3 * (lr/R + vx**2/R*(-m*lf/(Cr*L)))
)

# 验证不等价
print('公式3(kr=0) - 公式1 =', simplify(tidy3_kr0 - tidy1))

# 验证差值公式
diff_expected = (delta_ff - delta_ss)*(1 - delta_ff)/(k1*delta_ff)
print('差值 - 预期 =', simplify((tidy3_kr0 - tidy1) - diff_expected))

# 验证 delta_ff=1 时等价
print('delta_ff=1 时差值 =', simplify(
    tidy3_kr0.subs(delta_ff, 1) - tidy1.subs(delta_ff, 1)))
```
