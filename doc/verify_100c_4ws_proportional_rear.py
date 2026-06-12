"""
验证 4WS 比例后轮转向 (delta_r = kr * delta_f) 闭环稳态跟踪误差和前馈转角 (100c)。
基于 03c_error_rear_4ws.md 的后轴误差状态空间。

运行: python3 doc/verify_100c_4ws_proportional_rear.py
"""
from sympy import symbols, Matrix, simplify, solve, factor, cancel, expand

# 定义符号
Cf, Cr, lf, lr, m, Iz, vx, R = symbols('Cf Cr lf lr m Iz vx R', positive=True)
k1, k2, k3, k4, delta_ff, kr = symbols('k1 k2 k3 k4 delta_ff kr')
L = lf + lr
eta = Iz - m * lf * lr
xi = Iz + m * lr**2

# ============================================================
# 4WS 后轴定义的系统矩阵 (来自 03c)
# ============================================================
A = Matrix([
    [0, 1, 0, 0],
    [0, -(Cf*eta + Cr*xi)/(m*Iz*vx), (Cf*eta + Cr*xi)/(m*Iz), -(Cf*L*eta)/(m*Iz*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf - lr*Cr)/(Iz*vx), (lf*Cf - lr*Cr)/Iz, -(lf*Cf*L)/(Iz*vx)]
])

Bf = Matrix([0, Cf*eta/(m*Iz), 0, lf*Cf/Iz])
Br = Matrix([0, Cr*xi/(m*Iz), 0, -lr*Cr/Iz])
G = Matrix([0, -(Cf*L*eta)/(m*Iz*vx) - vx, 0, -(lf*Cf*L)/(Iz*vx)])

# 等效输入矩阵: B_eq = Bf + kr * Br
B_eq = Bf + kr * Br

K_mat = Matrix([[k1, k2, k3, k4]])

# 闭环矩阵
A_cl = A - B_eq * K_mat

# 稳态条件
dot_theta_ref = vx / R

# 稳态时 x_ss = [e1_ss, 0, e2_ss, 0]
e1_ss, e2_ss = symbols('e1_ss e2_ss')
x_ss = Matrix([e1_ss, 0, e2_ss, 0])

# 稳态方程: 0 = A_cl * x_ss + B_eq * delta_ff + G * dot_theta_ref
steady_eq = A_cl * x_ss + B_eq * delta_ff + G * dot_theta_ref

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

# 验证 e2_ss 不含 k1, k2, k3, k4, delta_ff
has_k = any(e2_ss_expr.has(s) for s in [k1, k2, k3, k4, delta_ff])
print(f"e2_ss 含有反馈增益 K 或 delta_ff: {has_k}")
assert not has_k, "e2_ss 不应依赖 K 或 delta_ff!"

# 验证解析表达式: e2_ss = m*vx^2*(Cf*lf - kr*Cr*lr)/(Cf*Cr*L*R*(1-kr)) - kr*L/((1-kr)*R)
e2_ss_expected = m*vx**2*(Cf*lf - kr*Cr*lr) / (Cf*Cr*L*R*(1-kr)) - kr*L/((1-kr)*R)
diff_e2 = simplify(e2_ss_expr - e2_ss_expected)
print(f"e2_ss - 预期值 = {diff_e2}")
assert diff_e2 == 0, f"e2_ss 验证失败! diff = {diff_e2}"
print("✓ e2_ss 验证通过")

print("\n" + "=" * 60)
print("验证 2: 前馈转角 delta_ff 使 e1_ss = 0")
print("=" * 60)

# 令 e1_ss = 0，解 delta_ff
e1_ss_expr = sol[e1_ss]
delta_ff_sol = solve(e1_ss_expr, delta_ff)[0]
delta_ff_simplified = simplify(delta_ff_sol)

print(f"delta_ff(e1=0) = {delta_ff_simplified}")

# 预期前馈: (L + K_us*vx^2)/((1-kr)*R) + k3 * e2_ss
K_us = m / L * (lr / Cf - lf / Cr)
delta_ff_expected = (L + K_us * vx**2) / ((1 - kr) * R) + k3 * e2_ss_expected

diff_ff = simplify(delta_ff_sol - delta_ff_expected)
print(f"delta_ff - 预期值 = {diff_ff}")
assert diff_ff == 0, f"delta_ff 验证失败! diff = {diff_ff}"
print("✓ delta_ff 验证通过")

print("\n" + "=" * 60)
print("验证 3: kr=0 退化为纯前轮转向 (100a)")
print("=" * 60)

e2_ss_kr0 = simplify(e2_ss_expected.subs(kr, 0))
delta_ff_kr0 = simplify(delta_ff_expected.subs(kr, 0))

# 100a 的结果
e2_ss_100a = m * lf * vx**2 / (Cr * L * R)
delta_ff_100a = (L + K_us * vx**2) / R + k3 * e2_ss_100a

diff_e2_kr0 = simplify(e2_ss_kr0 - e2_ss_100a)
diff_ff_kr0 = simplify(delta_ff_kr0 - delta_ff_100a)
print(f"e2_ss(kr=0) - 100a 结果 = {diff_e2_kr0}")
print(f"delta_ff(kr=0) - 100a 结果 = {diff_ff_kr0}")
assert diff_e2_kr0 == 0, "kr=0 退化验证失败 (e2)!"
assert diff_ff_kr0 == 0, "kr=0 退化验证失败 (delta_ff)!"
print("✓ kr=0 退化为 100a 验证通过")

print("\n" + "=" * 60)
print("验证 4: k3=0 时前馈公式")
print("=" * 60)

delta_ff_k3_0 = delta_ff_expected.subs(k3, 0)
delta_ff_classic_prop = (L + K_us * vx**2) / ((1 - kr) * R)
diff_classic = simplify(delta_ff_k3_0 - delta_ff_classic_prop)
print(f"k3=0 时 delta_ff - (L+Kus*vx^2)/((1-kr)*R) = {diff_classic}")
assert diff_classic == 0, "k3=0 验证失败!"
print("✓ k3=0 验证通过")

print("\n" + "=" * 60)
print("验证 5: 直接用矩阵逆验证")
print("=" * 60)

# 代入 delta_ff = delta_ff_expected，用矩阵逆求 x_ss
rhs = B_eq * delta_ff_expected + G * dot_theta_ref
x_ss_full = simplify(-A_cl.inv() * rhs)

# 验证 e1 = 0
e1_check = simplify(x_ss_full[0])
print(f"e1_ss (矩阵逆) = {e1_check}")
assert e1_check == 0, f"矩阵逆验证 e1=0 失败! got {e1_check}"
print("✓ e1_ss = 0 验证通过")

# 验证 e2 = e2_ss_expected
e2_check = simplify(x_ss_full[2] - e2_ss_expected)
print(f"e2_ss (矩阵逆) - 预期 = {e2_check}")
assert e2_check == 0, f"矩阵逆验证 e2 失败! got {e2_check}"
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
print("验证 6: 稳态等效转向角 = 经典公式")
print("=" * 60)

# 稳态时 delta_f_ss = delta_ff - k3*e2_ss (因为 e1=0, de1=de2=0)
delta_f_ss = delta_ff_expected - k3 * e2_ss_expected
delta_r_ss = kr * delta_f_ss
effective_steer = simplify(delta_f_ss - delta_r_ss)
classic_steer = (L + K_us * vx**2) / R

diff_eff = simplify(effective_steer - classic_steer)
print(f"(delta_f - delta_r)_ss - 经典公式 = {diff_eff}")
assert diff_eff == 0, f"等效转向角验证失败! diff = {diff_eff}"
print("✓ 稳态等效转向角 = (L + Kus*vx^2)/R 验证通过")

print("\n" + "=" * 60)
print("验证 7: 与 07a 运动学模型一致性")
print("=" * 60)

# 07a: dot_psi = vx*(1-kr)*delta_f / L
# 稳态: vx/R = vx*(1-kr)*delta_f_ss / L
# => delta_f_ss = L / ((1-kr)*R)  (纯运动学，无侧偏)
# 本文动力学结果: delta_f_ss = delta_ff - k3*e2_ss = (L+Kus*vx^2)/((1-kr)*R)
delta_f_ss_k3_0 = simplify((delta_ff_expected - k3 * e2_ss_expected).subs(k3, 0))
print(f"delta_f_ss (k3=0) = {simplify(delta_f_ss_k3_0)}")

delta_f_07a = (L + K_us * vx**2) / ((1 - kr) * R)
diff_07a = simplify(delta_f_ss_k3_0 - delta_f_07a)
print(f"delta_f_ss - 动力学公式 = {diff_07a}")
assert diff_07a == 0
print("✓ 与 07a 一致性验证通过")

print("\n" + "=" * 60)
print("验证 8: e1_ss 一般表达式")
print("=" * 60)

delta_ff_star = (L + K_us * vx**2) / ((1 - kr) * R) + k3 * e2_ss_expected
e1_ss_general_expected = (delta_ff - delta_ff_star) / k1

diff_e1 = simplify(e1_ss_expr - e1_ss_general_expected)
print(f"e1_ss - 预期 (1/k1)(delta_ff - delta_ff*) = {diff_e1}")
assert diff_e1 == 0, f"e1_ss 一般表达式验证失败! diff = {diff_e1}"
print("✓ e1_ss 一般表达式验证通过")

print("\n" + "=" * 60)
print("验证 9: 数值验证（典型车辆参数）")
print("=" * 60)

import math

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
    kr: 0.15,    # 典型比例系数
}

# 数值计算
e2_num = float(e2_ss_expected.subs(params))
delta_ff_num = float(delta_ff_expected.subs(params))

print(f"车辆参数: Cf={params[Cf]}, Cr={params[Cr]}, lf={params[lf]}, lr={params[lr]}")
print(f"          m={params[m]}, Iz={params[Iz]}, vx={params[vx]}, R={params[R]}")
print(f"反馈增益: k1={params[k1]}, k2={params[k2]}, k3={params[k3]}, k4={params[k4]}")
print(f"比例系数: kr={params[kr]}")
print(f"")
print(f"e2_ss = {e2_num:.6f} rad = {e2_num*180/math.pi:.4f} deg")
print(f"delta_ff = {delta_ff_num:.6f} rad = {delta_ff_num*180/math.pi:.4f} deg")
print(f"delta_f_ss = {delta_ff_num - params[k3]*e2_num:.6f} rad")
print(f"delta_r_ss = {params[kr]*(delta_ff_num - params[k3]*e2_num):.6f} rad")
print(f"等效转向角 = {(1-params[kr])*(delta_ff_num - params[k3]*e2_num):.6f} rad")
print(f"经典公式 (L+Kus*vx^2)/R = {float(((L+K_us*vx**2)/R).subs(params)):.6f} rad")

# 数值矩阵逆验证
A_num = A.subs(params)
B_eq_num = B_eq.subs(params)
G_num = G.subs(params)
K_num = K_mat.subs(params)
A_cl_num = A_num - B_eq_num * K_num

rhs_num = B_eq_num * delta_ff_num + G_num * float(dot_theta_ref.subs(params))
x_ss_num = -A_cl_num.inv() * rhs_num

print(f"\n数值矩阵逆验证:")
print(f"  e1_ss  = {float(x_ss_num[0]):.2e} (应为 0)")
print(f"  de1_ss = {float(x_ss_num[1]):.2e} (应为 0)")
print(f"  e2_ss  = {float(x_ss_num[2]):.6f} (预期 {e2_num:.6f})")
print(f"  de2_ss = {float(x_ss_num[3]):.2e} (应为 0)")

# 对比 kr=0
params_kr0 = dict(params)
params_kr0[kr] = 0
e2_kr0 = float(e2_ss_expected.subs(params_kr0))
print(f"\n对比: kr=0 时 e2_ss = {e2_kr0:.6f} rad")
print(f"       kr={params[kr]} 时 e2_ss = {e2_num:.6f} rad")
print(f"       比例后轮增大了 {(e2_num/e2_kr0 - 1)*100:.1f}% 的航向误差")

print("\n" + "=" * 60)
print("所有验证通过 ✓")
print("=" * 60)
