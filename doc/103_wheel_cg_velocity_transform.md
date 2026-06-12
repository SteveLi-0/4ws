# 轮速 ↔ 质心速度坐标变换公式总结

本文档总结 `tire_road_friction_estimator` 中两个文件的坐标变换公式：

- `vehicle_velocity_estimate.cc`：**轮 → 质心**，由实测轮速反推质心纵向速度。
- `tire_road_friction_estimate.cc`：**质心 → 轮**，由质心运动状态投影出各轮速度与侧偏角。

二者互为逆变换。

---

## 1. 符号定义

| 符号 | 代码字段 | 含义 |
|------|----------|------|
| `vx` | `trfe_input_.vx` / `mat_x_(0)` | 质心纵向速度（车体系 x） |
| `vy` | `vy_cal` | 质心侧向速度（自行车模型稳态解） |
| `ωz` | `yaw_rate` | 横摆角速度 |
| `δ` | `chassis.delta` | 等效前轮转角（方向盘折算后） |
| `lf / lr` | `veh_cnf.lf() / lr()` | 质心到前轴 / 后轴距离 |
| `L` | `veh_cnf.wheel_base()` | 轴距，`L = lf + lr` |
| `lw` | `veh_cnf.lw()` | 轮距（左右轮间距） |
| `m` | `veh_cnf.lon_mass()` | 整车质量 |
| `Cf / Cr` | `veh_cnf.cf() / cr()` | 前 / 后轴侧偏刚度 |
| `forward` | `moving_forward_` | 行驶方向符号：前进 +1，倒车 −1 |

轮序约定：`0=FL, 1=FR, 2=RL, 3=RR`。

---

## 2. 公共中间量

### 2.1 质心侧向速度（自行车稳态模型）

```
       1 − m·lf·vx² / (L·lr·Cr)
vy = ───────────────────────────────────── · (lr·δ / L) · vx_signed
     1 − m·(lf·Cf − lr·Cr)·vx² / (L²·Cr·Cf)
```

其中 `vx_signed = forward · vx`。velocity 文件中在分母绝对值 `> 1e-6` 时才计算，否则取 0。

### 2.2 左右前轮 Ackermann 修正角

定义半轴距比 `h = lw / (2L)`，当 `|sinδ| > 1e-6` 时：

```
δ_fl = atan2(sinδ,  cosδ − h·sinδ)
δ_fr = atan2(sinδ,  cosδ + h·sinδ)
```

否则 `δ_fl = δ_fr = δ`。

---

## 3. 质心 → 轮（`CalcTireStatus`）

### 3.1 轮心速度分量（车体系）

```
v1 = vx_signed − ωz·lw/2      // 左侧两轮纵向分量
v2 = vx_signed + ωz·lw/2      // 右侧两轮纵向分量
v3 = vy + ωz·lf               // 前轴处侧向分量
v4 = vy − ωz·lr               // 后轴处侧向分量
```

### 3.2 轮速（投影到各自轮平面）

```
vo_FL = | v1·cos(δ_fl) + v3·sin(δ_fl) |
vo_FR = | v2·cos(δ_fr) + v3·sin(δ_fr) |
vo_RL = | v1 |
vo_RR = | v2 |
```

### 3.3 轮胎侧偏角（用未做左右修正的 δ）

```
α_FL = atan2(−v1·sinδ + v3·cosδ,  |v1·cosδ + v3·sinδ|)
α_FR = atan2(−v2·sinδ + v3·cosδ,  |v2·cosδ + v3·sinδ|)
α_RL = atan2(v4,  |v1|)
α_RR = atan2(v4,  |v2|)
```

---

## 4. 轮 → 质心（`UpdateVelocityMeasure`）

输入为滤波后实测轮速 `ws_fl, ws_fr, ws_rl, ws_rr`，反解出每个轮单独给出的质心纵向速度估计 `tires[i].vx`。

记前轴侧向分量 `vy_front_hub = vy + ωz·lf`（即上文 v3）：

```
vx_FL = (ws_fl − forward·vy_front_hub·sin(δ_fl)) / cos(δ_fl) + forward·ωz·lw/2
vx_FR = (ws_fr − forward·vy_front_hub·sin(δ_fr)) / cos(δ_fr) − forward·ωz·lw/2
vx_RL =  ws_rl + forward·ωz·lw/2
vx_RR =  ws_rr − forward·ωz·lw/2
```

分母 `cos(δ_*)` 用 `kTrigEpsilon` 做除零保护：

```
cos_safe = (|cos δ| > kTrigEpsilon) ? cos δ : copysign(kTrigEpsilon, cos δ)
```

四个 `vx` 之后再经 Huber / 规则融合，并送入 EKF 得到最终车速。

---

## 5. 两者的逆变换关系

二者严格互逆。以左前轮为例：

正向（质心 → 轮）：
```
vo_FL = (vx_signed − ωz·lw/2)·cos(δ_fl) + vy_front_hub·sin(δ_fl)
```

令 `ws_fl = vo_FL`，反解 `vx_signed`：
```
vx_signed = (ws_fl − vy_front_hub·sin(δ_fl)) / cos(δ_fl) + ωz·lw/2
```

与第 4 节逆向公式完全一致。

要点：

- **friction 文件**：已知质心状态 → 求各轮速度 `vo`、侧偏角 `α`（用于滑移率、滑移角）。
- **velocity 文件**：已知各轮速度 → 反推质心纵向速度 `vx`。
- 倒车通过 `forward = −1` 翻转方向；后轮无转角，仅做 `±ωz·lw/2` 的轮距修正。
- 前轮额外考虑 Ackermann 左右修正角 `δ_fl / δ_fr` 与侧向分量 `vy_front_hub` 的投影。
