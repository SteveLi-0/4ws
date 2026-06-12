"""
验证 102 文档：4WS 比例后轮随动横向 MPC 的代码对照公式推导。

对照代码:
  mipilot/.../lat_mpc/interface/auto_gen.cc  (SystemODE / 雅可比 / cost)
  mipilot/.../lat_mpc/lat_mpc_controller.cc  (GetErrorState / UpdateReference)

核心验证:
  1. SystemODE 五行与 5 维扩展状态空间 (A_aug, B_u, B_d, G_aug) 逐行一致
  2. 状态/控制雅可比与 A_aug / B_u 一致
  3. 代码 UpdateReference 的 beta_ref(theta_rear) == 100c 的 e2_ss
  4. 代码 UpdateReference 的 delta_ref == 100c 的 delta_f^ss = (L+Kus v^2)/((1-kr)R)
     (当 kv = Kus/L 时)
  5. kr=0 退化为纯前轮 (100a) 的 e2_ss / delta_ff
  6. 数值一致性

运行: python3 doc/verify_102_4ws_proportional_rear_mpc.py
"""
from sympy import symbols, Matrix, simplify, zeros, Rational

# ============================================================
# 符号定义 (与代码命名对应)
# ============================================================
Cf, Cr, lf, lr, m, Iz = symbols('Cf Cr lf lr m Iz', positive=True)
v, kappa, kr, R = symbols('v kappa kr R', real=True)
delta_d = symbols('delta_d', real=True)       # steer_disturbance_deg (rad)
kv = symbols('kv', real=True)                  # 2WS 不足转向系数 (lat_out.kv)

# 状态: x = [lat_err, lat_err_rate, heading_err, heading_err_rate, steer]
e1, e1d, e2, e2d, steer = symbols('e1 e1d e2 e2d steer', real=True)
d_steer = symbols('d_steer', real=True)        # 控制量 u

L = lf + lr
eta = Iz - m * lf * lr
xi = Iz + m * lr**2
Kus = m / L * (lr / Cf - lf / Cr)              # 不足转向梯度

# ============================================================
# 5 维扩展状态空间 (03c 4WS 后轴 + 比例后轮 + 积分器 + 前轮扰动)
# ============================================================
A4 = Matrix([
    [0, 1, 0, 0],
    [0, -(Cf*eta + Cr*xi)/(m*Iz*v),  (Cf*eta + Cr*xi)/(m*Iz),    -(Cf*L*eta)/(m*Iz*v)],
    [0, 0, 0, 1],
    [0, -(lf*Cf - lr*Cr)/(Iz*v),     (lf*Cf - lr*Cr)/Iz,         -(lf*Cf*L)/(Iz*v)],
])
Bf4 = Matrix([0, Cf*eta/(m*Iz), 0, lf*Cf/Iz])
Br4 = Matrix([0, Cr*xi/(m*Iz),  0, -lr*Cr/Iz])
Beq4 = Bf4 + kr * Br4
G4 = Matrix([0, -(Cf*L*eta)/(m*Iz*v) - v, 0, -(lf*Cf*L)/(Iz*v)])

A_aug = zeros(5, 5)
A_aug[0:4, 0:4] = A4
A_aug[0:4, 4] = Beq4
Bu = Matrix([0, 0, 0, 0, 1])
Bd_aug = Matrix([Bf4[0], Bf4[1], Bf4[2], Bf4[3], 0])
G_aug = Matrix([G4[0], G4[1], G4[2], G4[3], 0])

x_aug = Matrix([e1, e1d, e2, e2d, steer])
f_doc = A_aug * x_aug + Bu * d_steer + Bd_aug * delta_d + G_aug * (kappa * v)

# ============================================================
# 代码 auto_gen.cc::SystemODE 原始表达式 (逐字翻译)
# ============================================================
f_code = Matrix([
    e1d,
    (e2 * ((Cf + Cr)/m - lr*(Cf*lf - Cr*lr)/Iz)
     + e2d * (Cf*lf*lr*(lf + lr)/(Iz*v) + (-Cf*lf - Cf*lr)/(m*v))
     + kappa * v * (Cf*lf*lr*(lf + lr)/(Iz*v) - v + (-Cf*lf - Cf*lr)/(m*v))
     + e1d * ((-Cf - Cr)/(m*v) + lr*(Cf*lf - Cr*lr)/(Iz*v))
     + steer * (Cf/m - Cf*lf*lr/Iz + kr*(Cr/m + Cr*lr**2/Iz))
     + delta_d * (Cf/m - Cf*lf*lr/Iz)),
    e2d,
    (-Cf*e2d*lf*(lf + lr)/(Iz*v)
     - Cf*kappa*lf*(lf + lr)/Iz
     + Cf*lf*delta_d/Iz
     + e2 * (Cf*lf - Cr*lr)/Iz
     + steer * (Cf*lf/Iz - Cr*kr*lr/Iz)
     + e1d * (-Cf*lf + Cr*lr)/(Iz*v)),
    d_steer,
])

print("=" * 64)
print("验证 1: SystemODE 五行与 5 维扩展状态空间一致")
print("=" * 64)
for i in range(5):
    diff = simplify(f_code[i] - f_doc[i])
    assert diff == 0, f"Row {i} 不一致: {diff}"
    print(f"  row{i}: diff = 0  OK")

print("\n" + "=" * 64)
print("验证 2: 状态/控制雅可比与 A_aug / B_u 一致")
print("=" * 64)
Jx = zeros(5, 5)
Jx[0, 1] = 1
Jx[1, 1] = (-Cf - Cr)/(m*v) + lr*(Cf*lf - Cr*lr)/(Iz*v)
Jx[1, 2] = (Cf + Cr)/m - lr*(Cf*lf - Cr*lr)/Iz
Jx[1, 3] = Cf*lf*lr*(lf + lr)/(Iz*v) + (-Cf*lf - Cf*lr)/(m*v)
Jx[1, 4] = Cf/m - Cf*lf*lr/Iz + kr*(Cr/m + Cr*lr**2/Iz)
Jx[2, 3] = 1
Jx[3, 1] = (-Cf*lf + Cr*lr)/(Iz*v)
Jx[3, 2] = (Cf*lf - Cr*lr)/Iz
Jx[3, 3] = -Cf*lf*(lf + lr)/(Iz*v)
Jx[3, 4] = Cf*lf/Iz - Cr*kr*lr/Iz
assert simplify(Jx - A_aug) == zeros(5, 5)
assert simplify(Jx - f_doc.jacobian(x_aug)) == zeros(5, 5)
Ju = Matrix([0, 0, 0, 0, 1])
assert simplify(Ju - Bu) == zeros(5, 1)
print("  Jx == A_aug,  Ju == B_u  OK")

# ============================================================
# 验证 3/4: 代码参考量 == 100c 闭式解
# ============================================================
print("\n" + "=" * 64)
print("验证 3: 代码 beta_ref (theta_rear) == 100c 的 e2_ss")
print("=" * 64)
# 代码 (UpdateReference / GetErrorState): wheel_base = L
beta_ref_code = (kappa / (1 - kr)
                 * (-(lf + lr) * kr
                    + m * v**2 * (lf*Cf - kr*lr*Cr) / (Cf*Cr*L)))
# 100c §4.2: e2_ss (用 kappa=1/R)
e2_ss_100c = (m*v**2*(Cf*lf - kr*Cr*lr) - kr*Cf*Cr*L**2) / (Cf*Cr*L*(1-kr)) * kappa
diff_beta = simplify(beta_ref_code - e2_ss_100c)
print(f"  beta_ref_code - e2_ss(100c) = {diff_beta}")
assert diff_beta == 0
print("  OK")

print("\n" + "=" * 64)
print("验证 4: 代码 delta_ref == 100c 的 delta_f^ss (kv = Kus/L)")
print("=" * 64)
# 代码: delta_2ws = (1 + kv v^2) L kappa ; delta_ref = delta_2ws/(1-kr)
delta_ref_code = (1 + kv * v**2) * L * kappa / (1 - kr)
# 100c §6: delta_f^ss = (L + Kus v^2)/((1-kr) R) = (L + Kus v^2) kappa /(1-kr)
delta_f_ss_100c = (L + Kus * v**2) * kappa / (1 - kr)
diff_delta = simplify(delta_ref_code.subs(kv, Kus / L) - delta_f_ss_100c)
print(f"  delta_ref_code(kv=Kus/L) - delta_f^ss(100c) = {diff_delta}")
assert diff_delta == 0
print("  OK")

# 等效转向不变量: (1-kr) delta_ref = (L + Kus v^2) kappa
eff = simplify((1 - kr) * delta_ref_code.subs(kv, Kus / L))
assert simplify(eff - (L + Kus * v**2) * kappa) == 0
print("  等效转向 (1-kr)*delta_ref = (L+Kus v^2)kappa  OK")

print("\n" + "=" * 64)
print("验证 5: kr=0 退化为纯前轮 (100a)")
print("=" * 64)
e2_kr0 = simplify(beta_ref_code.subs(kr, 0))
e2_100a = m*lf*v**2/(Cr*L) * kappa
assert simplify(e2_kr0 - e2_100a) == 0
delta_kr0 = simplify(delta_ref_code.subs(kv, Kus/L).subs(kr, 0))
assert simplify(delta_kr0 - (L + Kus*v**2)*kappa) == 0
print("  e2_ss(kr=0) = ml_f v^2 kappa/(Cr L)   OK")
print("  delta_ref(kr=0) = (L + Kus v^2) kappa  OK")

print("\n" + "=" * 64)
print("验证 6: 数值一致性 (典型乘用车参数)")
print("=" * 64)
subs_num = {
    Cf: 80000, Cr: 80000, lf: 1.4, lr: 1.6, m: 1800, Iz: 3000,
    v: 15.0, kappa: 0.01, kr: Rational(-3, 10), delta_d: 0.02,
    e1: 0.1, e1d: 0.05, e2: 0.02, e2d: 0.01, steer: 0.05, d_steer: 0.001,
}
for i in range(5):
    val = abs(float(simplify(f_code[i] - f_doc[i]).subs(subs_num)))
    assert val < 1e-10
beta_num = float(beta_ref_code.subs(subs_num))
delta_num = float(delta_ref_code.subs(kv, Kus/L).subs(subs_num))
print(f"  SystemODE 数值差 < 1e-10  OK")
print(f"  beta_ref  = {beta_num:.6f} rad = {beta_num*180/3.14159:.4f} deg")
print(f"  delta_ref = {delta_num:.6f} rad = {delta_num*180/3.14159:.4f} deg")

print("\n" + "=" * 64)
print("所有验证通过")
print("=" * 64)
