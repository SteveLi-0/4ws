# Steer Offset 下游影响模块分析

> 本文档梳理 `SteerOffsetEstimator` 输出的 `steer_offset` 信号在控制系统中的传播路径、各消费模块的用法，以及 4WS 升级后各模块需要的适应改动。

---

## 1. 输出结构与产生位置

### 数据结构

```cpp
// control/dependency/control_dependency.h:282
struct SystemIdentifyOut {
  double steer_offset{0.0};  // 前轮转向偏置 (deg)
  // ... 其他字段
};
```

### 产生位置

```cpp
// control/identification/identify_manager.cc:85
ctrl_input.identify.steer_offset = steer_offset_estimator_.Run(
    ctrl_input.identify.steer_offset, ctrl_input, steer_offset_debug);
```

---

## 2. 下游消费模块总览

```
SteerOffsetEstimator
        │
        ▼
  identify.steer_offset
        │
        ├──► IdentifyManager ──► chassis.delta（补偿后的前轮转角）
        │
        ├──► SteerCmdAfterProcess ──► str_final_cmd（最终转向指令）
        │
        ├──► SteerDobObserver ──► 扰动观测器内部模型
        │
        ├──► LatMPCController ──► MPC 首帧初始化
        │
        ├──► SmithPredictor ──► 延迟预测 steer buffer
        │
        ├──► ControlVisualize ──► debug 可视化
        │
        └──► HNOAControlComponent ──► 跨 session 持久化
```

---

## 3. 各模块详细分析

### 3.1 IdentifyManager（核心补偿点）

**文件**: `control/identification/identify_manager.cc`

**用法**:
```cpp
// line 85: offset 加到方向盘角度上，计算真实前轮转角
ctrl_input.chassis.delta = ControlConfig().StrAngToDeltaAngle(
    ctrl_input.chassis.str_angle + ctrl_input.identify.steer_offset);
```

**作用**: 将 sensor 报告的方向盘角度补偿为"真实"前轮转角，供下游所有需要 `chassis.delta` 的模块使用。

**4WS 适应**: 新增后轮补偿 `ctrl_input.chassis.delta_r += rear_steer_offset * M_PI / 180.0`

---

### 3.2 SteerCmdAfterProcess（指令输出补偿）

**文件**: `control/controller/lat_control/post_processor/steer_cmd_afterprocess.cc`

**用法**:
```cpp
// line 187: 从最终转向指令中减去 offset
local_view.str_final_cmd -= local_view.steer_offset;
```

**作用**: 控制器计算出的转向指令基于"真实"转角模型，但执行器侧存在 offset，因此下发指令时需要反向补偿。这是**前馈补偿的核心**：让执行器实际产生的转角等于期望转角。

**4WS 适应**: 若后轮指令也经过此后处理，需新增 `rear_str_final_cmd -= rear_steer_offset`

---

### 3.3 SteerDobObserver（转向扰动观测器）

**文件**: `control/controller/lat_control/observer/steer_dob_observer.cc`

**用法**:
```cpp
// line 99-101: 计算"无偏"转向角用于观测器模型
const double chassis_steer_wo_offset =
    control_dependency.chassis.str_angle +
    control_dependency.identify.steer_offset;
```

**作用**: DOB 需要知道实际作用在前轮的真实转角来计算模型预测输出，从而分离出外部扰动（如侧风、路面坡度）。如果不加 offset，扰动观测器会将 offset 引起的误差误判为外部扰动。

**4WS 适应**: DOB 当前仅建模前轮通道，暂不需改动。若后续 DOB 扩展为 4WS 模型，需引入 `rear_steer_offset`。

---

### 3.4 LatMPCController（横向 MPC 控制器）

**文件**: `control/controller/lat_control/lat_mpc/lat_mpc_controller.cc`

**用法**:
```cpp
// line 391: 首帧初始化 MPC 的转向请求值
mpc_out_.str_request =
    ctrl_input.chassis.str_angle + ctrl_input.identify.steer_offset;
```

**作用**: MPC 求解器首帧需要一个合理的初始转向值。使用补偿后的值确保 MPC 从正确的初始状态开始优化。

**4WS 适应**: 无直接改动需求（MPC 控制前轮）。但若 MPC 模型扩展纳入后轮状态，初始化需传入补偿后的 `delta_r`。

---

### 3.5 SmithPredictor（延迟补偿状态预测器）

**文件**: `control/controller/lat_control/state_predictor/smith_predictor.cc`

**用法**:
```cpp
// line 114-115: 初始化 steer buffer
deq_steer_.push_back(control_dependency_->chassis.str_angle +
                     local_view.steer_offset);

// line 154-155: 更新 steer buffer
deq_steer_.push_back(local_view.last_frame.str_final_cmd +
                     local_view.steer_offset);
```

**作用**: Smith 预测器通过维护一个转向历史 buffer 来补偿执行器延迟。buffer 中存储的应是"真实作用转角"，因此需要加上 offset。

**4WS 适应**: 若 SmithPredictor 内部模型纳入后轮动力学（当前未纳入），需维护 `rear_steer_buffer` 并加入 `rear_steer_offset` 补偿。

---

### 3.6 ControlVisualize（可视化调试）

**文件**: `control/common/control_visualize.cc`

**用法**:
```cpp
// line 50-51: 计算转向反馈
steer_feedback_ = lat_view.str_final_cmd + lat_view.steer_offset -
                  steer_ref_.predict_pp_str;

// line 80-81: 计算稳态横摆角速度
const double delta = ControlConfig().StrAngToDeltaAngle(
    lat_view.str_chassis + lat_view.steer_offset);
```

**作用**: 纯可视化/debug 用途，计算转向反馈误差和稳态模型值。

**4WS 适应**: 新增后轮 offset 的可视化字段。

---

### 3.7 LatController（横向控制器入口）

**文件**: `control/controller/lat_control/lat_controller.cc`

**用法**:
```cpp
// line 190: 将 offset 拷贝到 local_view 供子模块使用
local_view_.steer_offset = control_dependency_->identify.steer_offset;

// line 533: debug 输出
debug->set_str_zero_offset(local_view_.steer_offset);
```

**数据中转结构**:
```cpp
// control/controller/lat_control/lat_local_view.h:157
double steer_offset{0.0};
```

**作用**: LatController 是 steer_offset 向各子模块（MPC、SmithPredictor、DOB、AfterProcess）分发的枢纽。

**4WS 适应**: `LatLocalView` 需新增 `double rear_steer_offset{0.0}`，并在 LatController 中赋值。

---

### 3.8 HNOAControlComponent（持久化存储）

**文件**: `control/hnoa_control_component.cc`

**用法**:
```cpp
// line 84-103: 应用退出时持久化 offset 值
context->SetConfig("steer_offset", steer_offset);
// 下次启动时作为初始值传入 estimator
```

**作用**: 跨 power cycle 保持校准结果，避免每次冷启动都要重新收敛。

**4WS 适应**: 新增 `context->SetConfig("rear_steer_offset", rear_steer_offset)` 持久化后轮 offset。

---

## 4. 信号流总结图

```
┌─────────────────────────────────────────────────────────────────┐
│                        IdentifyManager                           │
│  steer_offset = estimator.Run(...)                              │
│  chassis.delta = StrAngToDeltaAngle(str_angle + steer_offset)   │
└───────────────────────────┬─────────────────────────────────────┘
                            │ ctrl_input.identify.steer_offset
                            ▼
┌───────────────────────────────────────────────────────────────────┐
│                         LatController                              │
│  local_view_.steer_offset = identify.steer_offset                 │
└───┬────────────┬─────────────┬──────────────┬─────────────────────┘
    │            │             │              │
    ▼            ▼             ▼              ▼
┌────────┐ ┌──────────┐ ┌───────────┐ ┌──────────────────┐
│LatMPC  │ │SmithPred │ │SteerDOB   │ │SteerCmdAfterProc │
│首帧初始│ │steer buf │ │模型真实角 │ │final_cmd -= ofs  │
└────────┘ └──────────┘ └───────────┘ └──────────────────┘
                                              │
                                              ▼
                                       下发给底盘 EPS
```

---

## 5. 4WS 升级影响汇总

| 模块 | 改动级别 | 说明 |
|------|---------|------|
| `SystemIdentifyOut` | 新增字段 | + `rear_steer_offset` |
| `IdentifyManager` | 中等 | 集成 4WS estimator，补偿 `delta_r` |
| `SteerCmdAfterProcess` | 小 | 后轮指令补偿（如有后轮指令输出） |
| `LatLocalView` | 新增字段 | + `rear_steer_offset` |
| `LatController` | 小 | 传递新字段 |
| `SteerDobObserver` | 暂不改 | 当前仅前轮模型 |
| `LatMPCController` | 暂不改 | 当前仅前轮模型 |
| `SmithPredictor` | 暂不改 | 当前仅前轮 buffer |
| `ControlVisualize` | 小 | 新增 debug 显示 |
| `HNOAControlComponent` | 小 | 持久化新字段 |
| `DependencyProcessor` | 小 | 补偿后的 `delta_r` 传播 |

---

## 6. 参考文件路径

- `control/identification/steer_offset_estimator/steer_offset_estimator.h`
- `control/identification/steer_offset_estimator/steer_offset_estimator.cc`
- `control/identification/identify_manager.cc`
- `control/dependency/control_dependency.h`
- `control/dependency/dependency_processor.cc`
- `control/controller/lat_control/lat_controller.cc`
- `control/controller/lat_control/lat_local_view.h`
- `control/controller/lat_control/lat_mpc/lat_mpc_controller.cc`
- `control/controller/lat_control/post_processor/steer_cmd_afterprocess.cc`
- `control/controller/lat_control/observer/steer_dob_observer.cc`
- `control/controller/lat_control/state_predictor/smith_predictor.cc`
- `control/common/control_visualize.cc`
- `control/hnoa_control_component.cc`
