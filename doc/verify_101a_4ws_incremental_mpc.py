"""
验证 4WS 比例后轮 + 前轮增量 + DOB 前轮扰动的扩展状态空间方程 (101a)
与 autogen MPC 代码 (auto_gen.h) 的一致性。

代码文件参考: mipilot/.../lat_mpc/interface/auto_gen.h
- Autogen::SystemODE
- Autogen::SystemODE_wrt_State_Gradient
- Autogen::SystemODE_wrt_Control_Gradient

运行: python3 doc/verify_101a_4ws_incremental_mpc.py
"""
from sympy import symbols, Matrix, simplify, zeros, Rational

# ============================================================
# 符号定义
# ============================================================
Cf, Cr, lf, lr, m, Iz = symbols('Cf Cr lf lr m Iz', positive=True)
v, kappa, kr = symbols('v kappa kr', real=True)
delta_d = symbols('delta_d', real=True)  # DOB 输出的前轮扰动 (rad)

# 状态: x = [e1, e1d, e2, e2d, steer]
e1, e1d, e2, e2d, steer = symbols('e1 e1d e2 e2d steer', real=True)
# 控制: u = d_steer (Δδ̇_f)
d_steer = symbols('d_steer', real=True)

L = lf + lr
eta = Iz - m * lf * lr
xi = Iz + m * lr**2

# ============================================================
# 文档 §2 紧凑形式: A_aug, B_eq, B_f, G_aug, B_u
# 来自 03c (4WS 后轴) + 04c (比例后轮 + 增量 + 扰动) + 积分器扩展
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

# 扩展到 5 维 (加上 δ_f^cmd 积分器)
A_aug = zeros(5, 5)
A_aug[0:4, 0:4] = A4
A_aug[0:4, 4]   = Beq4

Bd_aug = Matrix([Bf4[0], Bf4[1], Bf4[2], Bf4[3], 0])      # 扰动通道
G_aug  = Matrix([G4[0],  G4[1],  G4[2],  G4[3],  0])      # 曲率扰动通道
Bu     = Matrix([0, 0, 0, 0, 1])                          # 控制通道

x_aug = Matrix([e1, e1d, e2, e2d, steer])

# 文档形式 ODE
f_doc = A_aug * x_aug + Bu * d_steer + Bd_aug * delta_d + G_aug * (kappa * v)

# ============================================================
# 代码原始表达式 (从 auto_gen.h 直接逐字翻译)
# ============================================================
# 第 1 行 (lat_err 的导数)
f_code_1 = e1d

# 第 2 行 (lat_err_rate 的导数, 对应 ë₁)
f_code_2 = (
    e2 * ((Cf + Cr)/m - lr*(Cf*lf - Cr*lr)/Iz)
    + e2d * (Cf*lf*lr*(lf + lr)/(Iz*v) + (-Cf*lf - Cf*lr)/(m*v))
    + kappa * v * (Cf*lf*lr*(lf + lr)/(Iz*v) - v + (-Cf*lf - Cf*lr)/(m*v))
    + e1d * ((-Cf - Cr)/(m*v) + lr*(Cf*lf - Cr*lr)/(Iz*v))
    + steer * (Cf/m - Cf*lf*lr/Iz + kr*(Cr/m + Cr*lr**2/Iz))
    + delta_d * (Cf/m - Cf*lf*lr/Iz)
)

# 第 3 行 (heading_err 的导数)
f_code_3 = e2d

# 第 4 行 (heading_err_rate 的导数, 对应 ë₂)
f_code_4 = (
    -Cf*e2d*lf*(lf + lr)/(Iz*v)
    - Cf*kappa*lf*(lf + lr)/Iz
    + Cf*lf*delta_d/Iz
    + e2 * (Cf*lf - Cr*lr)/Iz
    + steer * (Cf*lf/Iz - Cr*kr*lr/Iz)
    + e1d * (-Cf*lf + Cr*lr)/(Iz*v)
)

# 第 5 行 (steer 的导数 = 控制输入)
f_code_5 = d_steer

f_code = Matrix([f_code_1, f_code_2, f_code_3, f_code_4, f_code_5])

# ============================================================
# 验证 1: ODE 逐行一致
# ============================================================
print("=" * 70)
print("验证 1: SystemODE 逐行与文档形式一致")
print("=" * 70)

row_names = ["ė₁", "ë₁", "ė₂", "ë₂", "δ̇_f^cmd"]
all_zero = True
for i in range(5):
    diff = simplify(f_code[i] - f_doc[i])
    status = "✓" if diff == 0 else "✗"
    print(f"  Row {i} ({row_names[i]:>8s}): diff = {diff}  {status}")
    if diff != 0:
        all_zero = False
assert all_zero, "ODE 不一致!"
print("✓ SystemODE 与文档形式完全一致")

# ============================================================
# 验证 2: SystemODE_wrt_State_Gradient (∂f/∂x)
# ============================================================
print()
print("=" * 70)
print("验证 2: SystemODE_wrt_State_Gradient = A_aug")
print("=" * 70)

# 对 5 维状态求雅可比
J_x_doc = f_doc.jacobian(x_aug)

# 代码中的雅可比 (按 auto_gen.h 中的非零项填写)
J_x_code = zeros(5, 5)
J_x_code[0, 1] = 1
J_x_code[1, 1] = (-Cf - Cr)/(m*v) + lr*(Cf*lf - Cr*lr)/(Iz*v)
J_x_code[1, 2] = (Cf + Cr)/m - lr*(Cf*lf - Cr*lr)/Iz
J_x_code[1, 3] = Cf*lf*lr*(lf + lr)/(Iz*v) + (-Cf*lf - Cf*lr)/(m*v)
J_x_code[1, 4] = Cf/m - Cf*lf*lr/Iz + kr*(Cr/m + Cr*lr**2/Iz)
J_x_code[2, 3] = 1
J_x_code[3, 1] = (-Cf*lf + Cr*lr)/(Iz*v)
J_x_code[3, 2] = (Cf*lf - Cr*lr)/Iz
J_x_code[3, 3] = -Cf*lf*(lf + lr)/(Iz*v)
J_x_code[3, 4] = Cf*lf/Iz - Cr*kr*lr/Iz

# 与 A_aug 比对
diff_A = simplify(J_x_code - A_aug)
print(f"  J_x_code - A_aug = {diff_A}")
assert diff_A == zeros(5, 5), "状态雅可比与 A_aug 不一致!"

# 与文档 jacobian 比对
diff_J = simplify(J_x_code - J_x_doc)
print(f"  J_x_code - J_x_doc = {diff_J}")
assert diff_J == zeros(5, 5), "状态雅可比与 ODE 自洽性失败!"
print("✓ 状态雅可比逐项与 A_aug 和文档一致")

# ============================================================
# 验证 3: SystemODE_wrt_Control_Gradient (∂f/∂u)
# ============================================================
print()
print("=" * 70)
print("验证 3: SystemODE_wrt_Control_Gradient = B_u")
print("=" * 70)

J_u_doc = f_doc.jacobian(Matrix([d_steer]))

J_u_code = Matrix([0, 0, 0, 0, 1])

diff_B = simplify(J_u_code - Bu)
print(f"  J_u_code - B_u = {diff_B.T}")
assert diff_B == zeros(5, 1)

diff_Ju = simplify(J_u_code - J_u_doc)
print(f"  J_u_code - J_u_doc = {diff_Ju.T}")
assert diff_Ju == zeros(5, 1)
print("✓ 控制雅可比与 B_u 和文档一致")

# ============================================================
# 验证 4: kr=0 退化为纯前轮 + 扰动 (相当于 03a + DOB)
# ============================================================
print()
print("=" * 70)
print("验证 4: kr=0 时 B_eq = B_f")
print("=" * 70)

Beq4_kr0 = Beq4.subs(kr, 0)
diff_kr0 = simplify(Beq4_kr0 - Bf4)
print(f"  B_eq(kr=0) - B_f = {diff_kr0.T}")
assert diff_kr0 == zeros(4, 1)
print("✓ kr=0 退化正确")

# ============================================================
# 验证 5: 扰动通道与控制通道的解耦
# ============================================================
print()
print("=" * 70)
print("验证 5: δ_d 走 B_f (不被 kr 缩放), δ_f^cmd 走 B_eq")
print("=" * 70)

# δ_d 系数 = ∂f/∂δ_d
df_d_dd_code = Matrix([
    f_code[i].diff(delta_d) for i in range(5)
])
df_d_dd_doc = Bd_aug

print(f"  ∂f/∂δ_d (code) = {df_d_dd_code.T}")
print(f"  Bd_aug         = {df_d_dd_doc.T}")
diff_dd = simplify(df_d_dd_code - df_d_dd_doc)
assert diff_dd == zeros(5, 1)
print("✓ 扰动通道 = B_f (前 4 行), 不含 kr")

# 验证 ∂f/∂δ_d 不依赖 kr
has_kr = any(df_d_dd_code[i].has(kr) for i in range(5))
print(f"  ∂f/∂δ_d 含有 kr: {has_kr}")
assert not has_kr
print("✓ 扰动通道与 kr 完全解耦")

# δ_f^cmd 系数 = ∂f/∂steer (即 J_x[:, 4])
df_d_steer = J_x_code[:, 4]
# 应该等于 [0; B_eq; 0]
expected_steer = Matrix([0, Beq4[1], 0, Beq4[3], 0])
diff_steer = simplify(df_d_steer - expected_steer)
print(f"  ∂f/∂steer - [0;B_eq;0] = {diff_steer.T}")
assert diff_steer == zeros(5, 1)
print("✓ 控制通道 = B_eq = B_f + kr*B_r")

# ============================================================
# 验证 6: 数值验证 (典型乘用车参数)
# ============================================================
print()
print("=" * 70)
print("验证 6: 数值验证")
print("=" * 70)

subs_numeric = {
    Cf: 80000, Cr: 80000,
    lf: 1.4, lr: 1.6,
    m: 1800, Iz: 3000,
    v: 15.0,
    kappa: 0.01,
    kr: Rational(-3, 10),  # -0.3 (低速反向)
    delta_d: 0.02,         # 1.15 deg
    e1: 0.1, e1d: 0.05,
    e2: 0.02, e2d: 0.01,
    steer: 0.05,
    d_steer: 0.001,
}

for i in range(5):
    diff_num = float(simplify(f_code[i] - f_doc[i]).subs(subs_numeric))
    print(f"  Row {i} 数值差: {diff_num:.3e}")
    assert abs(diff_num) < 1e-10

print("✓ 数值一致性验证通过")

# ============================================================
print()
print("=" * 70)
print("所有验证通过 ✓")
print("代码 auto_gen.h 中的 SystemODE / 状态雅可比 / 控制雅可比")
print("与文档 101a 的扩展状态空间模型完全一致")
print("=" * 70)
