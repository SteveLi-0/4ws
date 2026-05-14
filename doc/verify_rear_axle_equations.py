"""
验证 rear_axle_error_compact.md 与 rear_axle_lateral_error.md 中
误差方程、前馈角、稳态误差的一致性。
"""
import sympy as sp

# ── 符号定义 ──
Cf, Cr, m, Iz, lf, lr, vx, R = sp.symbols(
    'C_f C_r m I_z l_f l_r v_x R', positive=True
)
k1, k2, k3, k4 = sp.symbols('k_1 k_2 k_3 k_4')
delta_ff = sp.Symbol('delta_ff')
L = lf + lr
eta = Iz - m * lf * lr
kappa = 1 / R  # 曲率
theta_ref_dot = vx / R  # 参考航向变化率

print("=" * 60)
print("1. 验证 compact 缩写与 detailed 矩阵元素的等价性")
print("=" * 60)

# ── compact 文档的缩写定义 ──
alpha_c = Cf / m - Cf * lf * lr / Iz
beta_c  = -(Cf + Cr) / (m * vx) + lr * (lf * Cf - lr * Cr) / (Iz * vx)
gamma_c = -vx - Cf * (lf + lr) / (m * vx) + Cf * lf * lr * (lf + lr) / (Iz * vx)
mu_c    = lf * Cf / Iz
nu_c    = (lr * Cr - lf * Cf) / (Iz * vx)
omega_c = -lf * Cf * (lf + lr) / (Iz * vx)

# ── compact 文档的 A, B1, B2 矩阵 ──
A_compact = sp.Matrix([
    [0, 1,             0,             0],
    [0, beta_c,        -beta_c * vx,  gamma_c + vx],
    [0, 0,             0,             1],
    [0, nu_c,          -nu_c * vx,    omega_c],
])

B1_compact = sp.Matrix([0, alpha_c, 0, mu_c])
B2_compact = sp.Matrix([0, gamma_c, 0, omega_c])

# ── detailed 文档的 A, B, G 矩阵 ──
A_detailed = sp.Matrix([
    [0, 1, 0, 0],
    [0, -(Cf * eta + Cr * (Iz + m * lr**2)) / (m * Iz * vx),
        (Cf * eta + Cr * (Iz + m * lr**2)) / (m * Iz),
       -Cf * L * eta / (m * Iz * vx)],
    [0, 0, 0, 1],
    [0, -(lf * Cf - lr * Cr) / (Iz * vx),
        (lf * Cf - lr * Cr) / Iz,
       -lf * Cf * L / (Iz * vx)],
])

B_detailed = sp.Matrix([
    0,
    Cf * eta / (m * Iz),
    0,
    lf * Cf / Iz,
])

G_detailed = sp.Matrix([
    0,
    -Cf * L * eta / (m * Iz * vx) - vx,
    0,
    -lf * Cf * L / (Iz * vx),
])

# ── 逐元素比较 ──
diff_A = sp.simplify(A_compact - A_detailed)
diff_B = sp.simplify(B1_compact - B_detailed)
diff_G = sp.simplify(B2_compact - G_detailed)

print(f"A_compact - A_detailed = {diff_A}")
print(f"  → 全零: {diff_A.equals(sp.zeros(4, 4))}")
print()
print(f"B1_compact - B_detailed = {diff_B.T}")
print(f"  → 全零: {diff_B.equals(sp.zeros(4, 1))}")
print()
print(f"B2_compact - G_detailed = {diff_G.T}")
print(f"  → 全零: {diff_G.equals(sp.zeros(4, 1))}")

print()
print("=" * 60)
print("2. 验证 detailed 文档 A23 = -vx * A22 (物理一致性)")
print("=" * 60)

check_consistency = sp.simplify(A_detailed[1, 2] + vx * A_detailed[1, 1])
print(f"A23 + vx * A22 = {check_consistency}")
print(f"  → 为零: {check_consistency == 0}")

print()
print("=" * 60)
print("3. 验证闭环稳态解")
print("=" * 60)

K_mat = sp.Matrix([[k1, k2, k3, k4]])
A_cl = A_compact - B1_compact * K_mat

# 稳态: 0 = A_cl * x_ss + B1 * delta_ff + B2 * (vx/R)
# => x_ss = -A_cl^{-1} (B1 * delta_ff + B2 * vx/R)
rhs = B1_compact * delta_ff + B2_compact * (vx / R)
x_ss = -A_cl.inv() * rhs
x_ss_simplified = sp.simplify(x_ss)

print("x_ss (闭环稳态):")
for i in range(4):
    print(f"  x{i+1}_ss = {x_ss_simplified[i]}")

print()

# ── 验证 x3_ss 与 detailed 文档一致 ──
x3_ss_detailed = lf * m * vx**2 / (Cr * R * L)
diff_x3 = sp.simplify(x_ss_simplified[2] - x3_ss_detailed)
print(f"x3_ss(compact) - x3_ss(detailed) = {diff_x3}")
print(f"  → 为零: {diff_x3 == 0}")

print()

# ── 验证 x2_ss = 0, x4_ss = 0 ──
print(f"x2_ss = {sp.simplify(x_ss_simplified[1])}")
print(f"x4_ss = {sp.simplify(x_ss_simplified[3])}")

print()
print("=" * 60)
print("4. 验证前馈角：令 x1_ss = 0 且 k3=0 时的 delta_ff")
print("=" * 60)

# compact 文档: x1_ss = (1/k1)(delta_ff - L/R + mv_x^2(Cf*lf*(1-k3)-Cr*lr)/(Cf*Cr*R*L))
# 当 k3=0, x1_ss=0 时:
# delta_ff = L/R - mv_x^2(Cf*lf - Cr*lr)/(Cf*Cr*R*L)

x1_ss_k3_0 = x_ss_simplified[0].subs(k3, 0)
delta_ff_sol = sp.solve(x1_ss_k3_0, delta_ff)
print(f"由 x1_ss=0, k3=0 解出 delta_ff = {sp.simplify(delta_ff_sol[0])}")

# detailed 文档的前馈: delta_ff = (L + K_us * vx^2) * kappa
K_us = (m / L) * (lr / Cf - lf / Cr)
delta_ff_detailed = (L + K_us * vx**2) * kappa

diff_ff = sp.simplify(delta_ff_sol[0] - delta_ff_detailed)
print(f"delta_ff(compact) - delta_ff(detailed) = {diff_ff}")
print(f"  → 为零: {diff_ff == 0}")

print()
print("=" * 60)
print("5. 验证 compact x1_ss 公式与符号解的一致性")
print("=" * 60)

# compact 文档给出:
# x1_ss = (1/k1)(delta_ff - L/R + mv_x^2(Cf*lf*(1-k3) - Cr*lr)/(Cf*Cr*R*L))
x1_ss_formula = (1 / k1) * (
    delta_ff - L / R
    + m * vx**2 * (Cf * lf * (1 - k3) - Cr * lr) / (Cf * Cr * R * L)
)

diff_x1 = sp.simplify(x_ss_simplified[0] - x1_ss_formula)
print(f"x1_ss(sympy求解) - x1_ss(compact公式) = {diff_x1}")
print(f"  → 为零: {diff_x1 == 0}")

print()
print("=" * 60)
print("6. 稳态航向误差 x3_ss 的物理意义验证")
print("=" * 60)

# detailed 文档: e2_ss = alpha_r_ss = m*lf*vx^2*kappa / (Cr*L)
alpha_r_ss = m * lf * vx**2 * kappa / (Cr * L)
diff_e2 = sp.simplify(x_ss_simplified[2] - alpha_r_ss)
print(f"x3_ss - alpha_r_ss = {diff_e2}")
print(f"  → x3_ss 等于后轮稳态侧偏角: {diff_e2 == 0}")

print()
print("=" * 60)
print("结论")
print("=" * 60)
all_pass = (
    diff_A.equals(sp.zeros(4, 4))
    and diff_B.equals(sp.zeros(4, 1))
    and diff_G.equals(sp.zeros(4, 1))
    and diff_x3 == 0
    and diff_ff == 0
)
if all_pass:
    print("✓ 两份文档的误差方程、前馈角、稳态误差完全一致。")
else:
    print("✗ 存在不一致，请检查上方输出。")
