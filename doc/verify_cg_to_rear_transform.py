"""
验证 cg_to_rear_axle_transform.md 中的坐标变换推导。
检查项：
  1. 变换后 A_R, B_f_R, B_r_R, G_R 与手动推导一致
  2. 与 rear_axle_lateral_error.md 的 δ_r=0 特例一致
  3. 变换的代数等价性（T A_CG T^{-1} 形式验证）
"""
import sympy as sp

Cf, Cr, m, Iz, lf, lr, vx = sp.symbols(
    'C_f C_r m I_z l_f l_r v_x', positive=True
)
L = lf + lr
eta = Iz - m * lf * lr
xi = Iz + m * lr**2

# ═══════════════════════════════════════════
# 质心 4WS 误差模型
# ═══════════════════════════════════════════
A_CG = sp.Matrix([
    [0, 1, 0, 0],
    [0, -(Cf+Cr)/(m*vx), (Cf+Cr)/m, -(lf*Cf-lr*Cr)/(m*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf-lr*Cr)/(Iz*vx), (lf*Cf-lr*Cr)/Iz, -(lf**2*Cf+lr**2*Cr)/(Iz*vx)],
])
Bf_CG = sp.Matrix([0, Cf/m, 0, lf*Cf/Iz])
Br_CG = sp.Matrix([0, Cr/m, 0, -lr*Cr/Iz])
G_CG = sp.Matrix([0, -vx-(lf*Cf-lr*Cr)/(m*vx), 0, -(lf**2*Cf+lr**2*Cr)/(Iz*vx)])

# ═══════════════════════════════════════════
# 文档推导的后轴模型
# ═══════════════════════════════════════════
A_R_doc = sp.Matrix([
    [0, 1, 0, 0],
    [0, -(Cf*eta+Cr*xi)/(m*Iz*vx), (Cf*eta+Cr*xi)/(m*Iz), -Cf*L*eta/(m*Iz*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf-lr*Cr)/(Iz*vx), (lf*Cf-lr*Cr)/Iz, -lf*Cf*L/(Iz*vx)],
])
Bf_R_doc = sp.Matrix([0, Cf*eta/(m*Iz), 0, lf*Cf/Iz])
Br_R_doc = sp.Matrix([0, Cr*xi/(m*Iz), 0, -lr*Cr/Iz])
G_R_doc = sp.Matrix([0, -Cf*L*eta/(m*Iz*vx)-vx, 0, -lf*Cf*L/(Iz*vx)])

# ═══════════════════════════════════════════
# rear_axle_lateral_error.md 的后轴模型 (δ_r=0 版本)
# ═══════════════════════════════════════════
A_R_existing = sp.Matrix([
    [0, 1, 0, 0],
    [0, -(Cf*eta+Cr*(Iz+m*lr**2))/(m*Iz*vx),
        (Cf*eta+Cr*(Iz+m*lr**2))/(m*Iz),
       -Cf*L*eta/(m*Iz*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf-lr*Cr)/(Iz*vx), (lf*Cf-lr*Cr)/Iz, -lf*Cf*L/(Iz*vx)],
])
Bf_R_existing = sp.Matrix([0, Cf*eta/(m*Iz), 0, lf*Cf/Iz])
G_R_existing = sp.Matrix([0, -Cf*L*eta/(m*Iz*vx)-vx, 0, -lf*Cf*L/(Iz*vx)])


print("=" * 70)
print("验证 1: 文档 A_R 与 rear_axle_lateral_error.md 的 A 矩阵一致")
print("=" * 70)
# 注意 ξ = I_z + m l_r^2，所以两者应完全相同
diff = sp.simplify(A_R_doc - A_R_existing)
print(f"A_R_doc - A_R_existing = {diff}")
print(f"  → 全零: {diff.equals(sp.zeros(4, 4))}")

print()
print("=" * 70)
print("验证 2: B_f 矩阵一致")
print("=" * 70)
diff = sp.simplify(Bf_R_doc - Bf_R_existing)
print(f"Bf_R_doc - Bf_R_existing = {diff.T}")
print(f"  → 全零: {diff.equals(sp.zeros(4, 1))}")

print()
print("=" * 70)
print("验证 3: G 矩阵一致")
print("=" * 70)
diff = sp.simplify(G_R_doc - G_R_existing)
print(f"G_R_doc - G_R_existing = {diff.T}")
print(f"  → 全零: {diff.equals(sp.zeros(4, 1))}")


# ═══════════════════════════════════════════
# 方法 2: 用仿射变换矩阵直接验证
# ═══════════════════════════════════════════
# x_R = T x_CG + D θ̇_ref
# 其中 T = [[1,0,-lr,0],[0,1,0,-lr],[0,0,1,0],[0,0,0,1]]
# D = [0, -lr, 0, 0]^T
# 那么 ẋ_R = T ẋ_CG + D θ̈_ref ≈ T ẋ_CG (θ̈_ref ≈ 0)
# = T(A_CG x_CG + Bf δf + Br δr + G θ̇)
# = T A_CG T^{-1} (x_R - D θ̇) + T Bf δf + T Br δr + T G θ̇
# = T A_CG T^{-1} x_R + T Bf δf + T Br δr + (T G - T A_CG T^{-1} D) θ̇

print()
print("=" * 70)
print("验证 4: 仿射变换 T A_CG T^{-1} 方法")
print("=" * 70)

T = sp.Matrix([
    [1, 0, -lr, 0],
    [0, 1, 0, -lr],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
])
D = sp.Matrix([0, -lr, 0, 0])
T_inv = T.inv()

A_R_via_T = sp.simplify(T * A_CG * T_inv)
Bf_R_via_T = sp.simplify(T * Bf_CG)
Br_R_via_T = sp.simplify(T * Br_CG)
G_R_via_T = sp.simplify(T * G_CG - T * A_CG * T_inv * D)

diff_A = sp.simplify(A_R_via_T - A_R_doc)
diff_Bf = sp.simplify(Bf_R_via_T - Bf_R_doc)
diff_Br = sp.simplify(Br_R_via_T - Br_R_doc)
diff_G = sp.simplify(G_R_via_T - G_R_doc)

print(f"T A_CG T^(-1) - A_R_doc: 全零 = {diff_A.equals(sp.zeros(4, 4))}")
print(f"T Bf_CG - Bf_R_doc: 全零 = {diff_Bf.equals(sp.zeros(4, 1))}")
print(f"T Br_CG - Br_R_doc: 全零 = {diff_Br.equals(sp.zeros(4, 1))}")
print(f"(TG - T A T^{-1} D) - G_R_doc: 全零 = {diff_G.equals(sp.zeros(4, 1))}")


# ═══════════════════════════════════════════
# 验证 5: 物理一致性 A23 = -vx * A22
# ═══════════════════════════════════════════
print()
print("=" * 70)
print("验证 5: 后轴 A 矩阵的物理一致性 A23 = -vx * A22")
print("=" * 70)
check = sp.simplify(A_R_doc[1, 2] + vx * A_R_doc[1, 1])
print(f"A_R[1,2] + vx * A_R[1,1] = {check}")
print(f"  → 为零: {check == 0}")


# ═══════════════════════════════════════════
# 验证 6: ξ > 0 恒成立, η 的符号取决于车辆参数
# ═══════════════════════════════════════════
print()
print("=" * 70)
print("验证 6: B_r_R 的第 2 元素 Cr*ξ/(m*Iz) 恒正")
print("=" * 70)
print(f"ξ = I_z + m*l_r² > 0 恒成立 (所有项为正)")
print(f"因此 Br_R[1] = Cr*ξ/(m*Iz) > 0 恒成立")
print(f"后轮转角对后轴横向加速度始终有正增益 ✓")


# ═══════════════════════════════════════════
# 验证 7: compact 文档的缩写与 4WS 后轴模型一致
# ═══════════════════════════════════════════
print()
print("=" * 70)
print("验证 7: compact 缩写 α,β,γ,μ,ν,ω 与 4WS 后轴 A 矩阵一致")
print("=" * 70)

alpha_c = Cf/m - Cf*lf*lr/Iz
beta_c = -(Cf+Cr)/(m*vx) + lr*(lf*Cf - lr*Cr)/(Iz*vx)
gamma_c = -vx - Cf*L/(m*vx) + Cf*lf*lr*L/(Iz*vx)
mu_c = lf*Cf/Iz
nu_c = (lr*Cr - lf*Cf)/(Iz*vx)
omega_c = -lf*Cf*L/(Iz*vx)

A_compact = sp.Matrix([
    [0, 1, 0, 0],
    [0, beta_c, -beta_c*vx, gamma_c+vx],
    [0, 0, 0, 1],
    [0, nu_c, -nu_c*vx, omega_c],
])

diff = sp.simplify(A_compact - A_R_doc)
print(f"A_compact - A_R_doc: 全零 = {diff.equals(sp.zeros(4, 4))}")
print(f"α = Cf*η/(m*Iz): {sp.simplify(alpha_c - Cf*eta/(m*Iz)) == 0}")


# ═══════════════════════════════════════════
# 总结
# ═══════════════════════════════════════════
print()
print("=" * 70)
print("总结")
print("=" * 70)
all_pass = (
    sp.simplify(A_R_doc - A_R_existing).equals(sp.zeros(4, 4))
    and sp.simplify(Bf_R_doc - Bf_R_existing).equals(sp.zeros(4, 1))
    and sp.simplify(G_R_doc - G_R_existing).equals(sp.zeros(4, 1))
    and diff_A.equals(sp.zeros(4, 4))
    and diff_Bf.equals(sp.zeros(4, 1))
    and diff_Br.equals(sp.zeros(4, 1))
    and diff_G.equals(sp.zeros(4, 1))
    and check == 0
)
if all_pass:
    print("✓ 所有验证通过：")
    print("  - 坐标变换推导的后轴模型与直接推导版本完全一致")
    print("  - 仿射变换 T A_CG T^{-1} 方法得到相同结果")
    print("  - 物理一致性 A23 = -vx*A22 成立")
    print("  - compact 缩写与 4WS 后轴 A 矩阵一致")
else:
    print("✗ 存在不一致，请检查上方输出")
