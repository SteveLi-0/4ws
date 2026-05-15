"""
组2对比：同相转向（same-direction）换道 — 横摆力矩等效

2a: 4WS 同相转向 δr = δf（蟹行换道）
2b: 横摆力矩等效前轮转角 δ_eq = δf - (lr·Cr)/(lf·Cf)·δr, δr = 0
"""

import numpy as np
import matplotlib.pyplot as plt
from bicycle_model import (
    VehicleParams, simulate, moment_equivalent, const_steer,
)

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False


def lane_change_steer(amplitude=0.03, t_start=1.0, duration=2.0):
    def _steer(t):
        if t_start <= t <= t_start + duration:
            return amplitude * np.sin(np.pi * (t - t_start) / duration)
        return 0.0
    return _steer


params = VehicleParams()
vx = 20.0
t_span = (0.0, 6.0)
steer_func = lane_change_steer(amplitude=0.03, t_start=1.0, duration=2.0)

# Compute equivalent steering
ratio = params.lr * params.Cr / (params.lf * params.Cf)
amplitude_eq = moment_equivalent(0.03, 0.03, params)
print(f"=== 横摆力矩等效 ===")
print(f"  lr·Cr/(lf·Cf) = {ratio:.4f}")
print(f"  δf=δr=0.03 rad → δ_eq = δf - {ratio:.4f}·δr = {amplitude_eq:.4f} rad ({np.degrees(amplitude_eq):.2f}°)")


def eq_steer_func(t):
    return moment_equivalent(steer_func(t), steer_func(t), params)


# 2a: 4WS same-direction
t_a, vy_a, r_a, psi_a, X_a, Y_a, beta_a = simulate(
    vx, steer_func, steer_func, params, t_span
)

# 2b: 2WS moment equivalent
t_b, vy_b, r_b, psi_b, X_b, Y_b, beta_b = simulate(
    vx, eq_steer_func, const_steer(0.0), params, t_span
)

# Print key metrics
print("\n=== 换道效果 ===")
print(f"  4WS 最终侧向位移 Y = {Y_a[-1]:.4f} m")
print(f"  2WS 最终侧向位移 Y = {Y_b[-1]:.4f} m")
print(f"  4WS 最大航向变化   = {np.degrees(np.max(np.abs(psi_a))):.4f} deg")
print(f"  2WS 最大航向变化   = {np.degrees(np.max(np.abs(psi_b))):.4f} deg")
print(f"  4WS 最大横摆角速度 = {np.degrees(np.max(np.abs(r_a))):.4f} deg/s")
print(f"  2WS 最大横摆角速度 = {np.degrees(np.max(np.abs(r_b))):.4f} deg/s")

# Compute steering input arrays for plotting
steer_arr = np.array([steer_func(ti) for ti in t_a])
delta_eq_arr = moment_equivalent(steer_arr, steer_arr, params)

# Plot
fig, axes = plt.subplots(2, 3, figsize=(15, 9))
fig.suptitle("组2: 同相转向换道 — 4WS vs 横摆力矩等效 2WS (δ_eq=δf-lr·Cr/(lf·Cf)·δr)", fontsize=13)

label_4ws = "2a: 4WS (δr=δf)"
label_2ws = f"2b: 2WS (δ_eq={1-ratio:.3f}·δf)"

ax = axes[0, 0]
ax.plot(X_a, Y_a, label=label_4ws)
ax.plot(X_b, Y_b, "--", label=label_2ws)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("XY 轨迹")
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
ax.set_title("质心侧偏角 β(t)")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

ax = axes[1, 0]
ax.plot(t_a, Y_a, label=label_4ws)
ax.plot(t_b, Y_b, "--", label=label_2ws)
ax.set_xlabel("时间 [s]")
ax.set_ylabel("Y [m]")
ax.set_title("侧向位移 Y(t)")
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
ax.plot(t_a, np.degrees(steer_arr), label="δf = δr")
ax.plot(t_a, np.degrees(delta_eq_arr), "--", label=f"δ_eq = {1-ratio:.3f}·δf")
ax.set_xlabel("时间 [s]")
ax.set_ylabel("转角 [deg]")
ax.set_title("转向输入对比")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("compare_lane_change_moment.png", dpi=150)
plt.show()
