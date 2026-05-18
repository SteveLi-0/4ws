"""
分析前后轮转角调度表 (front_rear_str_scheduler)

数据格式: speed (kph), f_str (front steer, deg), r_str (rear steer, deg)
"""

import numpy as np
import matplotlib.pyplot as plt
import csv
import os

# fmt: off
# 原始数据: (speed_kph, f_str_deg, r_str_deg)
raw_data = [
    # speed=0
    (0, 0.0000, 0.0000), (0, 3.6000, -0.4171), (0, 7.2000, -0.8686),
    (0, 10.8000, -1.3566), (0, 14.4000, -1.8833), (0, 18.0000, -2.4512),
    (0, 21.6000, -3.0628), (0, 25.2000, -3.7205), (0, 28.8000, -4.4274),
    (0, 32.4000, -5.1862), (0, 36.0000, -6.0000),
    # speed=5
    (5, 0.0000, 0.0000), (5, 3.6000, -0.4171), (5, 7.2000, -0.8686),
    (5, 10.8000, -1.3566), (5, 14.4000, -1.8833), (5, 18.0000, -2.4512),
    (5, 21.6000, -3.0628), (5, 25.2000, -3.7205), (5, 28.8000, -4.4274),
    (5, 32.4000, -5.1862), (5, 36.0000, -6.0000),
    # speed=10
    (10, 0.0000, 0.0000), (10, 3.6000, -0.4171), (10, 7.2000, -0.8686),
    (10, 10.8000, -1.3566), (10, 14.4000, -1.8833), (10, 18.0000, -2.4512),
    (10, 21.6000, -3.0628), (10, 25.2000, -3.7205), (10, 28.8000, -4.4274),
    (10, 32.4000, -5.1862), (10, 36.0000, -6.0000),
    # speed=15
    (15, 0.0000, 0.0000), (15, 3.6000, -0.4171), (15, 7.2000, -0.8686),
    (15, 10.8000, -1.3566), (15, 14.4000, -1.8833), (15, 18.0000, -2.4512),
    (15, 21.6000, -3.0628), (15, 25.2000, -3.7205), (15, 28.8000, -4.4274),
    (15, 32.4000, -5.1862), (15, 36.0000, -6.0000),
    # speed=20
    (20, 0.0000, 0.0000), (20, 3.6000, -0.2781), (20, 7.2000, -0.5790),
    (20, 10.8000, -0.9044), (20, 14.4000, -1.2555), (20, 18.0000, -1.6341),
    (20, 21.6000, -2.0418), (20, 25.2000, -2.4804), (20, 28.8000, -2.9516),
    (20, 32.4000, -3.4574), (20, 36.0000, -4.0000),
    # speed=25
    (25, 0.0000, 0.0000), (25, 3.6000, -0.1390), (25, 7.2000, -0.2895),
    (25, 10.8000, -0.4522), (25, 14.4000, -0.6278), (25, 18.0000, -0.8171),
    (25, 21.6000, -1.0209), (25, 25.2000, -1.2402), (25, 28.8000, -1.4758),
    (25, 32.4000, -1.7287), (25, 36.0000, -2.0000),
    # speed=30
    (30, 0.0000, 0.0000), (30, 3.6000, 0.0000), (30, 7.2000, 0.0000),
    (30, 10.8000, 0.0000), (30, 14.4000, 0.0000), (30, 18.0000, 0.0000),
    (30, 21.6000, 0.0000), (30, 25.2000, 0.0000), (30, 28.8000, 0.0000),
    (30, 32.4000, 0.0000), (30, 36.0000, 0.0000),
    # speed=35
    (35, 0.0000, 0.0000), (35, 3.6000, 0.0307), (35, 7.2000, 0.0613),
    (35, 10.8000, 0.0920), (35, 14.4000, 0.1226), (35, 18.0000, 0.1532),
    (35, 21.6000, 0.1838), (35, 25.2000, 0.2145), (35, 28.8000, 0.2451),
    (35, 32.4000, 0.2758), (35, 36.0000, 0.3064),
    # speed=40
    (40, 0.0000, 0.0000), (40, 3.6000, 0.0613), (40, 7.2000, 0.1227),
    (40, 10.8000, 0.1840), (40, 14.4000, 0.2455), (40, 18.0000, 0.3069),
    (40, 21.6000, 0.3682), (40, 25.2000, 0.4296), (40, 28.8000, 0.4910),
    (40, 32.4000, 0.5524), (40, 36.0000, 0.6137),
    # speed=45
    (45, 0.0000, 0.0000), (45, 3.6000, 0.0920), (45, 7.2000, 0.1840),
    (45, 10.8000, 0.2760), (45, 14.4000, 0.3681), (45, 18.0000, 0.4601),
    (45, 21.6000, 0.5700), (45, 25.2000, 0.7000), (45, 28.8000, 0.8200),
    (45, 32.4000, 0.9600), (45, 36.0000, 1.1000),
    # speed=50
    (50, 0.0000, 0.0000), (50, 3.6000, 0.1840), (50, 7.2000, 0.3681),
    (50, 10.8000, 0.5521), (50, 14.4000, 0.7361), (50, 18.0000, 0.9201),
    (50, 21.6000, 1.1042), (50, 25.2000, 1.2882), (50, 28.8000, 1.4722),
    (50, 32.4000, 1.6562), (50, 36.0000, 1.8403),
    # speed=55
    (55, 0.0000, 0.0000), (55, 3.6000, 0.2677), (55, 7.2000, 0.5354),
    (55, 10.8000, 0.8030), (55, 14.4000, 1.0707), (55, 18.0000, 1.3384),
    (55, 21.6000, 1.6061), (55, 25.2000, 1.8737), (55, 28.8000, 2.1414),
    (55, 32.4000, 2.4091), (55, 36.0000, 2.6768),
    # speed=60
    (60, 0.0000, 0.0000), (60, 3.6000, 0.3513), (60, 7.2000, 0.7026),
    (60, 10.8000, 1.0540), (60, 14.4000, 1.4053), (60, 18.0000, 1.7566),
    (60, 21.6000, 2.1079), (60, 25.2000, 2.4593), (60, 28.8000, 2.8106),
    (60, 32.4000, 3.1619), (60, 36.0000, 3.5132),
    # speed=65
    (65, 0.0000, 0.0000), (65, 3.6000, 0.4277), (65, 7.2000, 0.8554),
    (65, 10.8000, 1.2831), (65, 14.4000, 1.7108), (65, 18.0000, 2.1385),
    (65, 21.6000, 2.5662), (65, 25.2000, 2.9939), (65, 28.8000, 3.4216),
    (65, 32.4000, 3.8493), (65, 36.0000, 4.2770),
    # speed=70
    (70, 0.0000, 0.0000), (70, 3.6000, 0.5041), (70, 7.2000, 1.0081),
    (70, 10.8000, 1.5122), (70, 14.4000, 2.0163), (70, 18.0000, 2.5204),
    (70, 21.6000, 3.0244), (70, 25.2000, 3.5285), (70, 28.8000, 4.0326),
    (70, 32.4000, 4.5367), (70, 36.0000, 5.0407),
    # speed=75
    (75, 0.0000, 0.0000), (75, 3.6000, 0.5673), (75, 7.2000, 1.1347),
    (75, 10.8000, 1.7020), (75, 14.4000, 2.2694), (75, 18.0000, 2.8367),
    (75, 21.6000, 3.4041), (75, 25.2000, 3.9714), (75, 28.8000, 4.5388),
    (75, 32.4000, 5.1061), (75, 36.0000, 5.6735),
    # speed=80
    (80, 0.0000, 0.0000), (80, 3.6000, 0.6306), (80, 7.2000, 1.2612),
    (80, 10.8000, 1.8919), (80, 14.4000, 2.5225), (80, 18.0000, 3.1531),
    (80, 21.6000, 3.7837), (80, 25.2000, 4.4143), (80, 28.8000, 5.0450),
    (80, 32.4000, 5.6756), (80, 36.0000, 6.0000),
    # speed=85
    (85, 0.0000, 0.0000), (85, 3.6000, 0.6741), (85, 7.2000, 1.3481),
    (85, 10.8000, 2.0222), (85, 14.4000, 2.6963), (85, 18.0000, 3.3704),
    (85, 21.6000, 4.0444), (85, 25.2000, 4.7185), (85, 28.8000, 5.3926),
    (85, 32.4000, 6.0000), (85, 36.0000, 6.0000),
    # speed=90
    (90, 0.0000, 0.0000), (90, 3.6000, 0.7175), (90, 7.2000, 1.4351),
    (90, 10.8000, 2.1526), (90, 14.4000, 2.8701), (90, 18.0000, 3.5876),
    (90, 21.6000, 4.3052), (90, 25.2000, 5.0227), (90, 28.8000, 5.7402),
    (90, 32.4000, 6.0000), (90, 36.0000, 6.0000),
    # speed=95
    (95, 0.0000, 0.0000), (95, 3.6000, 0.7390), (95, 7.2000, 1.4780),
    (95, 10.8000, 2.2170), (95, 14.4000, 2.9560), (95, 18.0000, 3.6951),
    (95, 21.6000, 4.4341), (95, 25.2000, 5.1731), (95, 28.8000, 5.9121),
    (95, 32.4000, 6.0000), (95, 36.0000, 6.0000),
    # speed=100
    (100, 0.0000, 0.0000), (100, 3.6000, 0.7605), (100, 7.2000, 1.5210),
    (100, 10.8000, 2.2815), (100, 14.4000, 3.0420), (100, 18.0000, 3.8025),
    (100, 21.6000, 4.5630), (100, 25.2000, 5.3235), (100, 28.8000, 6.0000),
    (100, 32.4000, 6.0000), (100, 36.0000, 6.0000),
    # speed=110
    (110, 0.0000, 0.0000), (110, 3.6000, 0.7729), (110, 7.2000, 1.5458),
    (110, 10.8000, 2.3187), (110, 14.4000, 3.0917), (110, 18.0000, 3.8646),
    (110, 21.6000, 4.6375), (110, 25.2000, 5.4104), (110, 28.8000, 6.0000),
    (110, 32.4000, 6.0000), (110, 36.0000, 6.0000),
    # speed=120
    (120, 0.0000, 0.0000), (120, 3.6000, 0.7729), (120, 7.2000, 1.5458),
    (120, 10.8000, 2.3187), (120, 14.4000, 3.0917), (120, 18.0000, 3.8646),
    (120, 21.6000, 4.6375), (120, 25.2000, 5.4104), (120, 28.8000, 6.0000),
    (120, 32.4000, 6.0000), (120, 36.0000, 6.0000),
    # speed=140
    (140, 0.0000, 0.0000), (140, 3.6000, 0.7729), (140, 7.2000, 1.5458),
    (140, 10.8000, 2.3187), (140, 14.4000, 3.0917), (140, 18.0000, 3.8646),
    (140, 21.6000, 4.6375), (140, 25.2000, 5.4104), (140, 28.8000, 6.0000),
    (140, 32.4000, 6.0000), (140, 36.0000, 6.0000),
    # speed=285
    (285, 0.0000, 0.0000), (285, 3.6000, 0.7729), (285, 7.2000, 1.5458),
    (285, 10.8000, 2.3187), (285, 14.4000, 3.0917), (285, 18.0000, 3.8646),
    (285, 21.6000, 4.6375), (285, 25.2000, 5.4104), (285, 28.8000, 6.0000),
    (285, 32.4000, 6.0000), (285, 36.0000, 6.0000),
]
# fmt: on

# 转为 numpy 数组
data = np.array(raw_data)
speeds = data[:, 0]
f_str = data[:, 1]
r_str = data[:, 2]

# 获取唯一速度值
unique_speeds = np.unique(speeds)
# 获取唯一前轮转角值
unique_f_str = np.unique(f_str)

# ============================================================
# 保存为 CSV
# ============================================================
output_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output")
os.makedirs(output_dir, exist_ok=True)

csv_path = os.path.join(output_dir, "steering_scheduler_data.csv")
with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["speed_kph", "f_str_deg", "r_str_deg"])
    for row in raw_data:
        writer.writerow(row)
print(f"Data saved to: {csv_path}")

# ============================================================
# 图1: 后轮转角 vs 前轮转角 (不同速度)
# ============================================================
fig1, ax1 = plt.subplots(figsize=(10, 7))
cmap = plt.cm.coolwarm
norm = plt.Normalize(vmin=unique_speeds.min(), vmax=unique_speeds.max())

for spd in unique_speeds:
    mask = speeds == spd
    color = cmap(norm(spd))
    ax1.plot(f_str[mask], r_str[mask], "o-", color=color, markersize=3,
             label=f"{int(spd)} kph")

ax1.axhline(0, color="gray", linewidth=0.5, linestyle="--")
ax1.set_xlabel(r"Front steer $\delta_f$ (deg)")
ax1.set_ylabel(r"Rear steer $\delta_r$ (deg)")
ax1.set_title(r"Steering Scheduler: $\delta_r$ vs $\delta_f$ (various speeds)")
ax1.legend(loc="upper left", fontsize=7, ncol=2)
ax1.grid(True, alpha=0.3)
fig1.tight_layout()
fig1.savefig(os.path.join(output_dir, "scheduler_rear_vs_front.png"), dpi=150)
print("Fig1 saved: scheduler_rear_vs_front.png")

# ============================================================
# 图2: 后轮/前轮比值 (ratio) vs 速度
# ============================================================
fig2, ax2 = plt.subplots(figsize=(10, 6))

# 计算每个速度下的 ratio = r_str / f_str (排除 f_str=0)
for spd in unique_speeds:
    mask = (speeds == spd) & (f_str > 0)
    if mask.sum() > 0:
        ratios = r_str[mask] / f_str[mask]
        ax2.plot(f_str[mask], ratios, "o-", markersize=3,
                 color=cmap(norm(spd)), label=f"{int(spd)} kph")

ax2.axhline(0, color="gray", linewidth=0.5, linestyle="--")
ax2.set_xlabel(r"Front steer $\delta_f$ (deg)")
ax2.set_ylabel(r"Ratio $\delta_r / \delta_f$")
ax2.set_title(r"Rear/Front steer ratio vs $\delta_f$ (various speeds)")
ax2.legend(loc="lower right", fontsize=7, ncol=2)
ax2.grid(True, alpha=0.3)
fig2.tight_layout()
fig2.savefig(os.path.join(output_dir, "scheduler_ratio_vs_front.png"), dpi=150)
print("Fig2 saved: scheduler_ratio_vs_front.png")

# ============================================================
# 图3: 比值 vs 速度 (取 f_str=36 deg 时的比值作为代表)
# ============================================================
fig3, ax3 = plt.subplots(figsize=(10, 6))

# 对每个速度，取 f_str=36 deg 时的 ratio
ratios_at_max = []
for spd in unique_speeds:
    mask = (speeds == spd) & (np.isclose(f_str, 36.0))
    if mask.sum() > 0:
        ratio = r_str[mask][0] / f_str[mask][0]
        ratios_at_max.append((spd, ratio))

ratios_at_max = np.array(ratios_at_max)
ax3.plot(ratios_at_max[:, 0], ratios_at_max[:, 1], "bo-", markersize=6)
ax3.axhline(0, color="gray", linewidth=0.5, linestyle="--")
ax3.axvline(30, color="red", linewidth=1, linestyle="--", alpha=0.7, label="30 kph (zero crossing)")
ax3.set_xlabel("Speed (kph)")
ax3.set_ylabel(r"Ratio $\delta_r / \delta_f$ (at $\delta_f=36°$)")
ax3.set_title(r"Rear/Front steer ratio vs Speed (at $\delta_f=36°$)")
ax3.legend()
ax3.grid(True, alpha=0.3)
fig3.tight_layout()
fig3.savefig(os.path.join(output_dir, "scheduler_ratio_vs_speed.png"), dpi=150)
print("Fig3 saved: scheduler_ratio_vs_speed.png")

# ============================================================
# 图4: 3D 曲面图 - 后轮转角 = f(速度, 前轮转角)
# ============================================================
fig4 = plt.figure(figsize=(11, 8))
ax4 = fig4.add_subplot(111, projection="3d")

# 构建网格
speed_grid = np.array(sorted(unique_speeds))
f_str_grid = np.array(sorted(unique_f_str))
S, F = np.meshgrid(speed_grid, f_str_grid)
R = np.zeros_like(S, dtype=float)

for i, fs in enumerate(f_str_grid):
    for j, sp in enumerate(speed_grid):
        mask = (np.isclose(speeds, sp)) & (np.isclose(f_str, fs))
        if mask.sum() > 0:
            R[i, j] = r_str[mask][0]

surf = ax4.plot_surface(S, F, R, cmap="coolwarm", alpha=0.8, edgecolor="none")
ax4.set_xlabel("Speed (kph)")
ax4.set_ylabel(r"Front steer $\delta_f$ (deg)")
ax4.set_zlabel(r"Rear steer $\delta_r$ (deg)")
ax4.set_title(r"Rear steer scheduling surface: $\delta_r = f(speed, \delta_f)$")
fig4.colorbar(surf, shrink=0.6, label=r"$\delta_r$ (deg)")
fig4.tight_layout()
fig4.savefig(os.path.join(output_dir, "scheduler_3d_surface.png"), dpi=150)
print("Fig4 saved: scheduler_3d_surface.png")

# ============================================================
# 图5: 等效轴距分析 (有效转向角 = δ_f - δ_r)
# ============================================================
fig5, ax5 = plt.subplots(figsize=(10, 6))

for spd in unique_speeds:
    mask = speeds == spd
    effective_steer = f_str[mask] - r_str[mask]
    color = cmap(norm(spd))
    ax5.plot(f_str[mask], effective_steer, "o-", color=color, markersize=3,
             label=f"{int(spd)} kph")

# Reference line: pure front steer (r_str=0)
ax5.plot(unique_f_str, unique_f_str, "k--", linewidth=1.5, label=r"Front-only ($\delta_r=0$)")
ax5.set_xlabel(r"Front steer $\delta_f$ (deg)")
ax5.set_ylabel(r"Effective steer $\delta_f - \delta_r$ (deg)")
ax5.set_title(r"Effective steering angle ($\delta_f - \delta_r$) vs $\delta_f$")
ax5.legend(loc="upper left", fontsize=7, ncol=2)
ax5.grid(True, alpha=0.3)
fig5.tight_layout()
fig5.savefig(os.path.join(output_dir, "scheduler_effective_steer.png"), dpi=150)
print("Fig5 saved: scheduler_effective_steer.png")

# ============================================================
# 数值分析总结
# ============================================================
print("\n" + "=" * 70)
print("Steering Scheduler Analysis Summary")
print("=" * 70)

print(f"\nSpeed range: {unique_speeds.min():.0f} ~ {unique_speeds.max():.0f} kph")
print(f"Front steer range: {unique_f_str.min():.1f} ~ {unique_f_str.max():.1f} deg")
print(f"Rear steer range: {r_str.min():.4f} ~ {r_str.max():.4f} deg")

print("\n--- Ratio dr/df at df=36 deg for each speed ---")
print(f"{'Speed(kph)':>10} {'Ratio dr/df':>15} {'Direction':>12}")
for spd, ratio in ratios_at_max:
    direction = "opposite" if ratio < 0 else ("same" if ratio > 0 else "zero")
    print(f"{spd:>10.0f} {ratio:>15.4f} {direction:>12}")

print("\n--- Key characteristics ---")
print("1. Low speed (0~15 kph): rear steers opposite, ratio ~ -1/6, improves agility")
print("2. Transition (20~25 kph): opposite but decreasing magnitude")
print("3. Critical speed 30 kph: rear steer = 0 (equivalent to 2WS)")
print("4. Mid speed (35~45 kph): rear same direction, ratio small and growing")
print("5. High speed (>=50 kph): rear same direction, ratio approaching saturation")
print("6. Rear steer hard limit at +/-6 deg")

# Linearity check
print("\n--- Linearity analysis ---")
for spd in [0, 15, 30, 50, 80, 120]:
    mask = (speeds == spd) & (f_str > 0)
    if mask.sum() > 1:
        coeffs = np.polyfit(f_str[mask], r_str[mask], 1)
        r_pred = np.polyval(coeffs, f_str[mask])
        residual = np.max(np.abs(r_str[mask] - r_pred))
        print(f"  speed={spd:>3.0f} kph: slope={coeffs[0]:.4f}, "
              f"intercept={coeffs[1]:.4f}, max_residual={residual:.4f} deg")

plt.show()
