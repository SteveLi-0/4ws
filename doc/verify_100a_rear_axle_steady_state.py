"""
验证后轴参考点闭环稳态跟踪误差和前馈转角的推导 (100a)。
基于 03a_error_rear_front_steer.md 的后轴误差状态空间。

运行: python3 doc/verify_100a_rear_axle_steady_state.py
"""
from sympy import symbols, Matrix, simplify, solve, factor, cancel, pprint

# 定义符号
Cf, Cr, lf, lr, m, Iz, vx, R = symbols('Cf Cr lf lr m Iz vx R', positive=True)
k1, k2, k3, k4, delta_ff = symbols('k1 k2 k3 k4 delta_ff')
L = lf + lr
eta = Iz - m * lf * lr
xi = Iz + m * lr**2

# ============================================================
# 后轴定义的系统矩阵 (来自 03a)
# ============================================================
A = Matrix([
    [0, 1, 0, 0],
    [0, -(Cf*eta + Cr*xi)/(m*Iz*vx), (Cf*eta + Cr*xi)/(m*Iz), -(Cf*L*eta)/(m*Iz*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf - lr*Cr)/(Iz*vx), (lf*Cf - lr*Cr)/Iz, -(lf*Cf*L)/(Iz*vx)]
])

B = Matrix([0, Cf*eta/(m*Iz), 0, lf*Cf/Iz])

G = Matrix([0, -(Cf*L*eta)/(m*Iz*vx) - vx, 0, -(lf*Cf*L)/(Iz*vx)])

K = Matrix([[k1, k2, k3, k4]])

# 闭环矩阵
A_cl = A - B * K

# 稳态条件
dot_theta_ref = vx / R

# 稳态时 x_ss = [e1_ss, 0, e2_ss, 0]
e1_ss, e2_ss = symbols('e1_ss e2_ss')
x_ss = Matrix([e1_ss, 0, e2_ss, 0])

# 稳态方程
steady_eq = A_cl * x_ss + B * delta_ff + G * dot_theta_ref

# 有效方程: 第2行和第4行
eq2 = steady_eq[1]  # ddot_e1 = 0
eq4 = steady_eq[3]  # ddot_e2 = 0

print("=" * 60)
print("验证 1: 稳态航向误差 e2_ss 与 K 无关")
print("=" * 60)

# 解两个方程求 e1_ss 和 e2_ss
sol = solve([eq2, eq4], [e1_ss, e2_ss])
e2_ss_expr = sol[e2_ss]
e2_ss_simplified = simplify(e2_ss_expr)

print(f"e2_ss (sympy) = {e2_ss_simplified}")

# 验证 e2_ss 不含 k1, k2, k3, k4
has_k = any(e2_ss_expr.has(ki) for ki in [k1, k2, k3, k4])
print(f"e2_ss 含有反馈增益 K: {has_k}")

# 验证解析表达式
e2_ss_expected = m * lf * vx**2 / (Cr * L * R)
diff_e2 = simplify(e2_ss_expr - e2_ss_expected)
print(f"e2_ss - 预期值 (ml_f v_x^2 / (C_r L R)) = {diff_e2}")
assert diff_e2 == 0, "e2_ss 验证失败!"
print("✓ e2_ss 验证通过")

print("\n" + "=" * 60)
print("验证 2: 前馈转角 delta_ff 使 e1_ss = 0")
print("=" * 60)

# 令 e1_ss = 0，解 delta_ff
e1_ss_expr = sol[e1_ss]
delta_ff_sol = solve(e1_ss_expr, delta_ff)[0]
delta_ff_simplified = simplify(delta_ff_sol)

print(f"delta_ff(e1=0) = {delta_ff_simplified}")

# 预期前馈: L/R + K_us * vx^2/R + k3 * e2_ss
K_us = m / L * (lr / Cf - lf / Cr)
delta_ff_expected = L / R + K_us * vx**2 / R + k3 * e2_ss_expected

diff_ff = simplify(delta_ff_sol - delta_ff_expected)
print(f"delta_ff - 预期值 = {diff_ff}")
assert diff_ff == 0, "delta_ff 验证失败!"
print("✓ delta_ff 验证通过")

print("\n" + "=" * 60)
print("验证 3: k3=0 时退化为经典公式")
print("=" * 60)

delta_ff_k3_0 = delta_ff_expected.subs(k3, 0)
delta_ff_classic = L / R + K_us * vx**2 / R
diff_classic = simplify(delta_ff_k3_0 - delta_ff_classic)
print(f"k3=0 时 delta_ff - 经典公式 = {diff_classic}")
assert diff_classic == 0, "经典公式退化验证失败!"
print("✓ k3=0 退化验证通过")

print("\n" + "=" * 60)
print("验证 4: 直接用矩阵逆验证")
print("=" * 60)

# 代入 delta_ff = delta_ff_expected，用矩阵逆求 x_ss
rhs = B * delta_ff_expected + G * dot_theta_ref
x_ss_full = simplify(-A_cl.inv() * rhs)

# 验证 e1 = 0
e1_check = simplify(x_ss_full[0])
print(f"e1_ss (矩阵逆) = {e1_check}")
assert e1_check == 0, "矩阵逆验证 e1=0 失败!"
print("✓ e1_ss = 0 验证通过")

# 验证 e2 = e2_ss_expected
e2_check = simplify(x_ss_full[2] - e2_ss_expected)
print(f"e2_ss (矩阵逆) - 预期 = {e2_check}")
assert e2_check == 0, "矩阵逆验证 e2 失败!"
print("✓ e2_ss 验证通过")

# 验证 dot_e1 = dot_e2 = 0
de1_check = simplify(x_ss_full[1])
de2_check = simplify(x_ss_full[3])
print(f"dot_e1_ss (矩阵逆) = {de1_check}")
print(f"dot_e2_ss (矩阵逆) = {de2_check}")
assert de1_check == 0, "矩阵逆验证 dot_e1=0 失败!"
assert de2_check == 0, "矩阵逆验证 dot_e2=0 失败!"
print("✓ dot_e1 = dot_e2 = 0 验证通过")

print("\n" + "=" * 60)
print("验证 5: e1_ss 一般表达式")
print("=" * 60)

# 一般情况: e1_ss = (1/k1) * (delta_ff - delta_ff_star)
delta_ff_star = L / R + K_us * vx**2 / R + k3 * e2_ss_expected
e1_ss_general_expected = (delta_ff - delta_ff_star) / k1

diff_e1 = simplify(e1_ss_expr - e1_ss_general_expected)
print(f"e1_ss - 预期 (1/k1)(delta_ff - delta_ff*) = {diff_e1}")
assert diff_e1 == 0, "e1_ss 一般表达式验证失败!"
print("✓ e1_ss 一般表达式验证通过")

print("\n" + "=" * 60)
print("验证 6: 与质心定义的对比")
print("=" * 60)

# 质心定义的 e2_ss
e2_ss_cg = m * lf * vx**2 / (Cr * L * R) - lr / R
print(f"e2_ss (质心) = ml_f v_x^2/(C_r L R) - l_r/R")
print(f"e2_ss (后轴) = ml_f v_x^2/(C_r L R)")
print(f"差值 = e2_rear - e2_cg = {simplify(e2_ss_expected - e2_ss_cg)} = l_r/R")
print("✓ 差值为 l_r/R，即后轴侧偏角 β_r 的几何分量")

print("\n" + "=" * 60)
print("验证 7: 数值验证（典型车辆参数）")
print("=" * 60)

# 典型参数
params = {
    Cf: 80000,   # N/rad
    Cr: 80000,   # N/rad
    lf: 1.2,     # m
    lr: 1.5,     # m
    m: 1500,     # kg
    Iz: 3000,    # kg*m^2
    vx: 20,      # m/s
    R: 100,      # m
    k1: 0.05,
    k2: 0.1,
    k3: 1.0,
    k4: 0.5,
}

# 数值计算
e2_num = float(e2_ss_expected.subs(params))
delta_ff_num = float(delta_ff_expected.subs(params))

print(f"车辆参数: Cf={params[Cf]}, Cr={params[Cr]}, lf={params[lf]}, lr={params[lr]}")
print(f"          m={params[m]}, Iz={params[Iz]}, vx={params[vx]}, R={params[R]}")
print(f"反馈增益: k1={params[k1]}, k2={params[k2]}, k3={params[k3]}, k4={params[k4]}")
print(f"")
print(f"e2_ss = {e2_num:.6f} rad = {e2_num*180/3.14159:.4f} deg")
print(f"delta_ff = {delta_ff_num:.6f} rad = {delta_ff_num*180/3.14159:.4f} deg")
print(f"经典前馈 (k3=0) = {float((L/R + K_us*vx**2/R).subs(params)):.6f} rad")

# 数值矩阵逆验证
A_num = A.subs(params)
B_num = B.subs(params)
G_num = G.subs(params)
K_num = K.subs(params)
A_cl_num = A_num - B_num * K_num

rhs_num = B_num * delta_ff_num + G_num * float(dot_theta_ref.subs(params))
x_ss_num = -A_cl_num.inv() * rhs_num

print(f"\n数值矩阵逆验证:")
print(f"  e1_ss  = {float(x_ss_num[0]):.2e} (应为 0)")
print(f"  de1_ss = {float(x_ss_num[1]):.2e} (应为 0)")
print(f"  e2_ss  = {float(x_ss_num[2]):.6f} (预期 {e2_num:.6f})")
print(f"  de2_ss = {float(x_ss_num[3]):.2e} (应为 0)")

print("\n" + "=" * 60)
print("所有验证通过 ✓")
print("=" * 60)
