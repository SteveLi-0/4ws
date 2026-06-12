"""
验证四轮转向（delta_r = kr * delta_f）稳态跟踪误差和前馈转角。
运行: python3 doc/verify_99c_4ws_steady_state.py
"""
from sympy import symbols, Matrix, simplify, solve, factor, cancel, expand

# 定义符号
Cf, Cr, lf, lr, m, Iz, vx, R = symbols('Cf Cr lf lr m Iz vx R', positive=True)
k1, k2, k3, k4, kr, delta_ff = symbols('k1 k2 k3 k4 kr delta_ff')
L = lf + lr

# 4WS 系统矩阵 (02b 第5节)
A = Matrix([
    [0, 1, 0, 0],
    [0, -(Cf + Cr)/(m*vx), (Cf + Cr)/m, -(lf*Cf - lr*Cr)/(m*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf - lr*Cr)/(Iz*vx), (lf*Cf - lr*Cr)/Iz, -(lf**2*Cf + lr**2*Cr)/(Iz*vx)]
])

# 4WS 输入矩阵 (4x2)
B2 = Matrix([
    [0, 0],
    [Cf/m, Cr/m],
    [0, 0],
    [lf*Cf/Iz, -lr*Cr/Iz]
])

# 扰动矩阵
G = Matrix([0, -vx - (lf*Cf - lr*Cr)/(m*vx), 0, -(lf**2*Cf + lr**2*Cr)/(Iz*vx)])

# 等效输入矩阵: B_eq = B2 * [1; kr]
B_eq = B2 * Matrix([1, kr])

# 反馈增益
K = Matrix([[k1, k2, k3, k4]])

# 闭环矩阵
A_cl = A - B_eq * K

# 稳态条件
dot_theta_ref = vx / R
e1_ss, e2_ss = symbols('e1_ss e2_ss')
x_ss = Matrix([e1_ss, 0, e2_ss, 0])

# 稳态方程
steady_eq = A_cl * x_ss + B_eq * delta_ff + G * dot_theta_ref

# 有效方程: 第2行和第4行
eq2 = steady_eq[1]
eq4 = steady_eq[3]

print("=" * 60)
print("验证 1: e2_ss 与反馈增益 K 无关")
print("=" * 60)

sol = solve([eq2, eq4], [e1_ss, e2_ss])
e2_ss_expr = sol[e2_ss]

has_k = any(e2_ss_expr.has(ki) for ki in [k1, k2, k3, k4])
print(f"e2_ss 含有反馈增益 K: {has_k}")

# 预期 e2_ss
e2_ss_expected = (m*vx**2*(lf*Cf - kr*lr*Cr)/(Cf*Cr*L*(1-kr)*R)
                  - (lr + kr*lf)/((1-kr)*R))
diff_e2 = simplify(e2_ss_expr - e2_ss_expected)
print(f"e2_ss - 预期 = {diff_e2}")

print("\n" + "=" * 60)
print("验证 2: kr=0 时 e2_ss 退化为 99b 结果")
print("=" * 60)

e2_ss_kr0 = e2_ss_expected.subs(kr, 0)
e2_ss_99b = lf*m*vx**2/(Cr*L*R) - lr/R
diff_kr0 = simplify(e2_ss_kr0 - e2_ss_99b)
print(f"e2_ss(kr=0) - 99b = {diff_kr0}")

print("\n" + "=" * 60)
print("验证 3: delta_ff 使 e1_ss = 0")
print("=" * 60)

e1_ss_expr = sol[e1_ss]
delta_ff_sol = solve(e1_ss_expr, delta_ff)[0]

# 预期 delta_ff
delta_ff_expected = (L/((1-kr)*R)
                     + m*vx**2/((1-kr)*R) * (lr/(Cf*L) - lf/(Cr*L))
                     + k3*e2_ss_expected)

diff_ff = simplify(delta_ff_sol - delta_ff_expected)
print(f"delta_ff(e1=0) - 预期 = {diff_ff}")

print("\n" + "=" * 60)
print("验证 4: kr=0 时 delta_ff 退化为 99b 结果")
print("=" * 60)

delta_ff_kr0 = delta_ff_expected.subs(kr, 0)
delta_ff_99b = L/R + m*vx**2/R*(lr/(Cf*L) - lf/(Cr*L)) + k3*(lf*m*vx**2/(Cr*L*R) - lr/R)
diff_ff_kr0 = simplify(delta_ff_kr0 - delta_ff_99b)
print(f"delta_ff(kr=0) - 99b = {diff_ff_kr0}")

print("\n" + "=" * 60)
print("验证 5: 矩阵逆直接验证")
print("=" * 60)

# 代入 delta_ff_expected, 求 x_ss = -A_cl^{-1} * (B_eq*delta_ff + G*dot_theta_ref)
rhs = B_eq * delta_ff_expected + G * dot_theta_ref
x_ss_full = -A_cl.inv() * rhs

e1_check = simplify(x_ss_full[0])
print(f"e1_ss (矩阵逆) = {e1_check}")

e2_check = simplify(x_ss_full[2] - e2_ss_expected)
print(f"e2_ss (矩阵逆) - 预期 = {e2_check}")

print(f"dot_e1_ss (矩阵逆) = {simplify(x_ss_full[1])}")
print(f"dot_e2_ss (矩阵逆) = {simplify(x_ss_full[3])}")

print("\n" + "=" * 60)
print("验证 6: e1_ss 一般表达式")
print("=" * 60)

# delta_ff_star 是使 e1=0 的前馈
delta_ff_star = delta_ff_expected
e1_ss_formula = (1/k1) * (delta_ff - delta_ff_star)
diff_e1 = simplify(e1_ss_expr - e1_ss_formula)
print(f"e1_ss - (1/k1)*(delta_ff - delta_ff*) = {diff_e1}")

print("\n" + "=" * 60)
print("所有验证完成")
print("=" * 60)
