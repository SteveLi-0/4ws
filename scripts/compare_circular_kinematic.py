"""
组1对比：反相转向（counter-steering）匀速定圆 — 运动学等效

1a: 4WS 后轮转向 δr = -δf
1b: 运动学等效前轮转角 δ_eq = δf - δr = 2δf, δr = 0
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt
from bicycle_model import (
    VehicleParams, simulate, kinematic_equivalent, const_steer,
)

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False

params = VehicleParams()
vx = 10.0
delta_f = 0.05
delta_r = -delta_f
delta_eq = kinematic_equivalent(delta_f, delta_r)
t_span = (0.0, 10.0)

# 1a: 4WS counter-steering
t_a, vy_a, r_a, psi_a, X_a, Y_a, beta_a = simulate(
    vx, const_steer(delta_f), const_steer(delta_r), params, t_span
)

# 1b: 2WS equivalent
t_b, vy_b, r_b, psi_b, X_b, Y_b, beta_b = simulate(
    vx, const_steer(delta_eq), const_steer(0.0), params, t_span
)

# Print steady-state comparison
print("=== 稳态比较 (取最后 1s 平均) ===")
n_tail = int(1.0 / 0.005)
for label, arr_a, arr_b, unit in [
    ("r", r_a, r_b, "rad/s"),
    ("β", beta_a, beta_b, "rad"),
    ("vy", vy_a, vy_b, "m/s"),
]:
    ss_a = np.mean(arr_a[-n_tail:])
    ss_b = np.mean(arr_b[-n_tail:])
    print(f"  {label:3s}: 4WS={ss_a:+.6f} {unit},  2WS={ss_b:+.6f} {unit},  Δ={ss_a-ss_b:+.6f}")

print(f"\n  理论 β 差异 ≈ δr = {delta_r:.4f} rad")
print(f"  实际 β 差异   = {np.mean(beta_a[-n_tail:]) - np.mean(beta_b[-n_tail:]):.6f} rad")

# Plot
fig, axes = plt.subplots(2, 3, figsize=(15, 9))
fig.suptitle("组1: 反相转向定圆 — 4WS vs 运动学等效 2WS (δ_eq=δf-δr)", fontsize=14)

label_4ws = "1a: 4WS (δr=-δf)"
label_2ws = "1b: 2WS (δ_eq=2δf)"

ax = axes[0, 0]
ax.plot(X_a, Y_a, label=label_4ws)
ax.plot(X_b, Y_b, "--", label=label_2ws)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("XY 轨迹")
ax.set_aspect("equal")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

ax = axes[0, 1]
ax.plot(t_a, np.degrees(r_a), label=label_4ws)
ax.plot(t_b, np.degrees(r_b), "--", label=label_2ws)
ax.set_xlabel("时间 [s]")
ax.set_ylabel("r [deg/s]")
ax.set_title("横摆角速度 r(t)")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

ax = axes[0, 2]
ax.plot(t_a, np.degrees(beta_a), label=label_4ws)
ax.plot(t_b, np.degrees(beta_b), "--", label=label_2ws)
ax.set_xlabel("时间 [s]")
ax.set_ylabel("β [deg]")
ax.set_title("质心侧偏角 β(t) — 核心差异")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

ax = axes[1, 0]
ax.plot(t_a, vy_a, label=label_4ws)
ax.plot(t_b, vy_b, "--", label=label_2ws)
ax.set_xlabel("时间 [s]")
ax.set_ylabel("vy [m/s]")
ax.set_title("侧向速度 vy(t)")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

ax = axes[1, 1]
ax.plot(t_a, np.degrees(psi_a), label=label_4ws)
ax.plot(t_b, np.degrees(psi_b), "--", label=label_2ws)
ax.set_xlabel("时间 [s]")
ax.set_ylabel("ψ [deg]")
ax.set_title("航向角 ψ(t)")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

ax = axes[1, 2]
t_input = [t_span[0], t_span[1]]
ax.step(t_input, [np.degrees(delta_f)] * 2, label=f"1a: δf={np.degrees(delta_f):.1f}°")
ax.step(t_input, [np.degrees(delta_r)] * 2, label=f"1a: δr={np.degrees(delta_r):.1f}°")
ax.step(t_input, [np.degrees(delta_eq)] * 2, "--", label=f"1b: δ_eq={np.degrees(delta_eq):.1f}°")
ax.set_xlabel("时间 [s]")
ax.set_ylabel("转角 [deg]")
ax.set_title("转向输入")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

plt.tight_layout()
output_dir = Path(__file__).resolve().parent.parent / "output"
output_dir.mkdir(exist_ok=True)
plt.savefig(output_dir / "compare_circular_kinematic.png", dpi=150)
plt.show()
