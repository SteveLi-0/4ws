"""
验证稳态跟踪误差和前馈转角的推导。
运行: python3 doc/verify_steady_state_feedforward.py
"""
from sympy import symbols, Matrix, simplify, solve, Rational, factor, cancel

# 定义符号
Cf, Cr, lf, lr, m, Iz, vx, R = symbols('Cf Cr lf lr m Iz vx R', positive=True)
k1, k2, k3, k4, delta_ff = symbols('k1 k2 k3 k4 delta_ff')
L = lf + lr

# 定义矩阵
A = Matrix([
    [0, 1, 0, 0],
    [0, -(Cf + Cr)/(m*vx), (Cf + Cr)/m, -(lf*Cf - lr*Cr)/(m*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf - lr*Cr)/(Iz*vx), (lf*Cf - lr*Cr)/Iz, -(lf**2*Cf + lr**2*Cr)/(Iz*vx)]
])

B = Matrix([0, Cf/m, 0, lf*Cf/Iz])

G = Matrix([0, -vx - (lf*Cf - lr*Cr)/(m*vx), 0, -(lf**2*Cf + lr**2*Cr)/(Iz*vx)])

K = Matrix([[k1, k2, k3, k4]])

# 闭环矩阵
A_cl = A - B * K

# 稳态条件: dot_x = 0
# 0 = A_cl * x_ss + B * delta_ff + G * dot_theta_ref
# dot_theta_ref = vx / R
dot_theta_ref = vx / R

# 稳态时 dot_e1 = dot_e2 = 0, 所以 x_ss = [e1_ss, 0, e2_ss, 0]
e1_ss, e2_ss = symbols('e1_ss e2_ss')
x_ss = Matrix([e1_ss, 0, e2_ss, 0])

# 稳态方程
steady_eq = A_cl * x_ss + B * delta_ff + G * dot_theta_ref

# 有效方程是第2行和第4行
eq2 = steady_eq[1]  # ddot_e1 = 0
eq4 = steady_eq[3]  # ddot_e2 = 0

print("=" * 60)
print("验证 1: 稳态航向误差 e2_ss 与 K 无关")
print("=" * 60)

# 解两个方程求 e1_ss 和 e2_ss
sol = solve([eq2, eq4], [e1_ss, e2_ss])
e2_ss_expr = sol[e2_ss]

# 验证 e2_ss 不含 k1, k2, k3, k4
has_k = any(e2_ss_expr.has(ki) for ki in [k1, k2, k3, k4])
print(f"e2_ss 含有反馈增益 K: {has_k}")

# 验证解析表达式
e2_ss_expected = lf*m*vx**2/(Cr*L*R) - lr/R
diff_e2 = simplify(e2_ss_expr - e2_ss_expected)
print(f"e2_ss - 预期 = {diff_e2}")

print("\n" + "=" * 60)
print("验证 2: e1_ss 表达式")
print("=" * 60)

e1_ss_expr = sol[e1_ss]

# 预期: e1_ss = (1/k1) * (delta_ff - delta_ff_star)
delta_ff_star = L/R + m*vx**2/R * (lr/(Cf*L) - lf/(Cr*L)) + k3*e2_ss_expected
e1_ss_expected = (1/k1) * (delta_ff - delta_ff_star)

diff_e1 = simplify(e1_ss_expr - e1_ss_expected)
print(f"e1_ss - 预期 = {diff_e1}")

print("\n" + "=" * 60)
print("验证 3: 前馈转角 delta_ff 使 e1_ss = 0")
print("=" * 60)

# 令 e1_ss = 0，解 delta_ff
delta_ff_sol = solve(e1_ss_expr, delta_ff)[0]

# 预期前馈
delta_ff_expected = L/R + m*vx**2/R * (lr/(Cf*L) - lf/(Cr*L)) + k3*e2_ss_expected

diff_ff = simplify(delta_ff_sol - delta_ff_expected)
print(f"delta_ff(e1=0) - 预期 = {diff_ff}")

print("\n" + "=" * 60)
print("验证 4: 直接用矩阵逆验证")
print("=" * 60)

# 用完整矩阵逆求解 x_ss = -A_cl^{-1} * (B*delta_ff + G*dot_theta_ref)
# 代入 delta_ff = delta_ff_expected
rhs = B * delta_ff_expected + G * dot_theta_ref
x_ss_full = -A_cl.inv() * rhs

# 验证 x_ss[0] = 0 (e1 = 0)
e1_check = simplify(x_ss_full[0])
print(f"e1_ss (矩阵逆) = {e1_check}")

# 验证 x_ss[2] = e2_ss_expected
e2_check = simplify(x_ss_full[2] - e2_ss_expected)
print(f"e2_ss (矩阵逆) - 预期 = {e2_check}")

# 验证 dot_e1 = dot_e2 = 0
print(f"dot_e1_ss (矩阵逆) = {simplify(x_ss_full[1])}")
print(f"dot_e2_ss (矩阵逆) = {simplify(x_ss_full[3])}")

print("\n" + "=" * 60)
print("验证 5: k3=0 时退化为经典公式")
print("=" * 60)

delta_ff_k3_0 = delta_ff_expected.subs(k3, 0)
delta_ff_classic = L/R + m*vx**2/(R*L) * (lr/Cf - lf/Cr)
diff_classic = simplify(delta_ff_k3_0 - delta_ff_classic)
print(f"k3=0 时 delta_ff - 经典公式 = {diff_classic}")

print("\n" + "=" * 60)
print("所有验证完成")
print("=" * 60)
