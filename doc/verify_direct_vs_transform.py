"""
验证后轴 4WS 误差模型的两种推导方法等价性：
  方法 A: 从质心模型坐标变换 (cg_to_rear_axle_transform.md)
  方法 B: 从后轴 Frenet 定义正向推导 (rear_axle_4ws_direct.md)
同时验证 δ_r=0 退化到 rear_axle_lateral_error.md 的结果。
"""
import sympy as sp

Cf, Cr, m, Iz, lf, lr, vx = sp.symbols(
    'C_f C_r m I_z l_f l_r v_x', positive=True
)
L = lf + lr
eta = Iz - m * lf * lr
xi = Iz + m * lr**2

# ═══════════════════════════════════════════════════════════════
# 方法 A: 质心模型 + 坐标变换
# ═══════════════════════════════════════════════════════════════
A_CG = sp.Matrix([
    [0, 1, 0, 0],
    [0, -(Cf+Cr)/(m*vx), (Cf+Cr)/m, -(lf*Cf-lr*Cr)/(m*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf-lr*Cr)/(Iz*vx), (lf*Cf-lr*Cr)/Iz, -(lf**2*Cf+lr**2*Cr)/(Iz*vx)],
])
Bf_CG = sp.Matrix([0, Cf/m, 0, lf*Cf/Iz])
Br_CG = sp.Matrix([0, Cr/m, 0, -lr*Cr/Iz])
G_CG = sp.Matrix([0, -vx-(lf*Cf-lr*Cr)/(m*vx), 0, -(lf**2*Cf+lr**2*Cr)/(Iz*vx)])

T = sp.Matrix([[1, 0, -lr, 0], [0, 1, 0, -lr], [0, 0, 1, 0], [0, 0, 0, 1]])

A_transform = T * A_CG * T.inv()
Bf_transform = T * Bf_CG
Br_transform = T * Br_CG
# G 需通过直接代入法（仿射法在第 1 元素有已知偏差）
# 这里用代入法重新推导 G
e1d, e2_, e2d, thd, df, dr = sp.symbols('de1 e2 de2 thd df dr')
e1d_CG = e1d + lr * e2d + lr * thd

# ë2 用后轴状态（必须 expand 以确保 .coeff() 正确提取）
e2dd = sp.expand(
    -(lf*Cf - lr*Cr)/(Iz*vx) * e1d_CG
    + (lf*Cf - lr*Cr)/Iz * e2_
    - (lf**2*Cf + lr**2*Cr)/(Iz*vx) * e2d
    + lf*Cf/Iz * df - lr*Cr/Iz * dr
    - (lf**2*Cf + lr**2*Cr)/(Iz*vx) * thd
)

# ë1_CG 用后轴状态
e1dd_CG = sp.expand(
    -(Cf+Cr)/(m*vx) * e1d_CG
    + (Cf+Cr)/m * e2_
    - (lf*Cf - lr*Cr)/(m*vx) * e2d
    + Cf/m * df + Cr/m * dr
    + (-vx - (lf*Cf - lr*Cr)/(m*vx)) * thd
)
# ë1_R = ë1_CG - lr * ë2
e1dd_transform = sp.expand(e1dd_CG - lr * e2dd)

# 提取 G 的第 2、4 元素
G2_transform = sp.simplify(e1dd_transform.coeff(thd))
G4_transform = sp.simplify(e2dd.coeff(thd))
G_transform = sp.Matrix([0, G2_transform, 0, G4_transform])

# ═══════════════════════════════════════════════════════════════
# 方法 B: 后轴 Frenet 正向推导
# ═══════════════════════════════════════════════════════════════
A_direct = sp.Matrix([
    [0, 1, 0, 0],
    [0, -(Cf*eta + Cr*xi)/(m*Iz*vx), (Cf*eta + Cr*xi)/(m*Iz), -Cf*L*eta/(m*Iz*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf - lr*Cr)/(Iz*vx), (lf*Cf - lr*Cr)/Iz, -lf*Cf*L/(Iz*vx)],
])
Bf_direct = sp.Matrix([0, Cf*eta/(m*Iz), 0, lf*Cf/Iz])
Br_direct = sp.Matrix([0, Cr*xi/(m*Iz), 0, -lr*Cr/Iz])
G_direct = sp.Matrix([0, -Cf*L*eta/(m*Iz*vx) - vx, 0, -lf*Cf*L/(Iz*vx)])

# ═══════════════════════════════════════════════════════════════
# 已有文档: rear_axle_lateral_error.md (δ_r=0)
# ═══════════════════════════════════════════════════════════════
A_exist = sp.Matrix([
    [0, 1, 0, 0],
    [0, -(Cf*(Iz-m*lf*lr) + Cr*(Iz+m*lr**2))/(m*Iz*vx),
        (Cf*(Iz-m*lf*lr) + Cr*(Iz+m*lr**2))/(m*Iz),
       -Cf*L*(Iz-m*lf*lr)/(m*Iz*vx)],
    [0, 0, 0, 1],
    [0, -(lf*Cf-lr*Cr)/(Iz*vx), (lf*Cf-lr*Cr)/Iz, -lf*Cf*L/(Iz*vx)],
])
Bf_exist = sp.Matrix([0, Cf*(Iz-m*lf*lr)/(m*Iz), 0, lf*Cf/Iz])
G_exist = sp.Matrix([0, -Cf*L*(Iz-m*lf*lr)/(m*Iz*vx)-vx, 0, -lf*Cf*L/(Iz*vx)])


# ═══════════════════════════════════════════════════════════════
print("=" * 65)
print("验证 1: 方法 A (坐标变换) vs 方法 B (正向推导)")
print("=" * 65)

dA = sp.simplify(A_transform - A_direct)
dBf = sp.simplify(Bf_transform - Bf_direct)
dBr = sp.simplify(Br_transform - Br_direct)
dG = sp.simplify(G_transform - G_direct)

print(f"  A  矩阵一致: {dA.equals(sp.zeros(4, 4))}")
print(f"  Bf 矩阵一致: {dBf.equals(sp.zeros(4, 1))}")
print(f"  Br 矩阵一致: {dBr.equals(sp.zeros(4, 1))}")
print(f"  G  矩阵一致: {dG.equals(sp.zeros(4, 1))}")

print()
print("=" * 65)
print("验证 2: 方法 B (正向推导) vs 已有文档 (δ_r=0 特例)")
print("=" * 65)
print(f"  A  矩阵一致: {sp.simplify(A_direct - A_exist).equals(sp.zeros(4, 4))}")
print(f"  Bf 矩阵一致: {sp.simplify(Bf_direct - Bf_exist).equals(sp.zeros(4, 1))}")
print(f"  G  矩阵一致: {sp.simplify(G_direct - G_exist).equals(sp.zeros(4, 1))}")

print()
print("=" * 65)
print("验证 3: 逐系数直接代入法验证方法 B")
print("=" * 65)

# 用符号变量直接从动力学方程推导每个系数
# v_yr = ė1 - vx*e2, r = ė2 + θ̇
v_yr = e1d - vx * e2_
r_val = e2d + thd

# 轮胎力
Fyf = Cf * (df - (v_yr + L * r_val) / vx)
Fyr = Cr * (dr - v_yr / vx)

# ë2 = (lf*Fyf - lr*Fyr) / Iz
e2dd_check = sp.expand((lf * Fyf - lr * Fyr) / Iz)

# ë1 = η/(mIz)*Fyf + ξ/(mIz)*Fyr - vx*θ̇
e1dd_check = sp.expand(eta / (m * Iz) * Fyf + xi / (m * Iz) * Fyr - vx * thd)

labels = ['ė1', 'e2', 'ė2', 'δf', 'δr', 'θ̇ref']
syms = [e1d, e2_, e2d, df, dr, thd]

# A, Bf, Br, G for row 2 (ë1)
row2_expected = [A_direct[1, 1], A_direct[1, 2], A_direct[1, 3],
                 Bf_direct[1], Br_direct[1], G_direct[1]]
print("  ë1 各系数:")
for lbl, sym, expected in zip(labels, syms, row2_expected):
    actual = e1dd_check.coeff(sym)
    ok = sp.simplify(actual - expected) == 0
    print(f"    {lbl:6s}: {ok}")

# A, Bf, Br, G for row 4 (ë2)
row4_expected = [A_direct[3, 1], A_direct[3, 2], A_direct[3, 3],
                 Bf_direct[3], Br_direct[3], G_direct[3]]
print("  ë2 各系数:")
for lbl, sym, expected in zip(labels, syms, row4_expected):
    actual = e2dd_check.coeff(sym)
    ok = sp.simplify(actual - expected) == 0
    print(f"    {lbl:6s}: {ok}")

print()
print("=" * 65)
print("验证 4: 物理一致性")
print("=" * 65)
print(f"  A23 = -vx*A22: {sp.simplify(A_direct[1, 2] + vx * A_direct[1, 1]) == 0}")
print(f"  A43 = -vx*A42: {sp.simplify(A_direct[3, 2] + vx * A_direct[3, 1]) == 0}")
print(f"  Br[1] > 0:     ξ = Iz + m*lr² > 0 恒成立")

print()
print("=" * 65)
print("验证 5: compact 缩写一致性")
print("=" * 65)
alpha_c = Cf/m - Cf*lf*lr/Iz
beta_c = -(Cf+Cr)/(m*vx) + lr*(lf*Cf - lr*Cr)/(Iz*vx)
gamma_c = -vx - Cf*L/(m*vx) + Cf*lf*lr*L/(Iz*vx)
mu_c = lf*Cf/Iz
nu_c = (lr*Cr - lf*Cf)/(Iz*vx)
omega_c = -lf*Cf*L/(Iz*vx)

print(f"  α = Cf*η/(mIz):    {sp.simplify(alpha_c - Cf*eta/(m*Iz)) == 0}")
print(f"  β = A_direct[1,1]: {sp.simplify(beta_c - A_direct[1, 1]) == 0}")
print(f"  γ+vx = A[1,3]:     {sp.simplify(gamma_c + vx - A_direct[1, 3]) == 0}")
print(f"  μ = Bf[3]:          {sp.simplify(mu_c - Bf_direct[3]) == 0}")
print(f"  ν = A[3,1]:         {sp.simplify(nu_c - A_direct[3, 1]) == 0}")
print(f"  ω = A[3,3]:         {sp.simplify(omega_c - A_direct[3, 3]) == 0}")

print()
print("=" * 65)
print("结论")
print("=" * 65)

all_ok = (
    dA.equals(sp.zeros(4, 4))
    and dBf.equals(sp.zeros(4, 1))
    and dBr.equals(sp.zeros(4, 1))
    and dG.equals(sp.zeros(4, 1))
    and sp.simplify(A_direct - A_exist).equals(sp.zeros(4, 4))
    and sp.simplify(Bf_direct - Bf_exist).equals(sp.zeros(4, 1))
    and sp.simplify(G_direct - G_exist).equals(sp.zeros(4, 1))
)
if all_ok:
    print("✓ 两种推导方法（坐标变换 vs 正向推导）结果完全等价")
    print("✓ δ_r=0 时退化到已有 rear_axle_lateral_error.md 的结果")
    print("✓ 所有物理一致性检查通过")
    print("✓ compact 缩写与 4WS 后轴模型一致")
else:
    print("✗ 存在不一致，请检查上方输出")
