# 调优交接文档 — LQR 曲线路径跟踪控制器

> 每次新对话开始时先读此文件，再运行 `python3 curve_tuning/analyze.py` 查看最新数据。

---

## 1. 项目背景（30秒速读）

**目标**：椭圆/样条曲线路径跟踪，p90 < 5mm，avg<3mm% > 65%
**控制器**：`lqr_curve_controller.cpp`
**控制方程**：ω = ω_ff + clamp(−K1·e_y − K2·e_θ + ω_i, ±feedback_limit)
**前馈**：ω_ff = v · κ（曲率从三点差分法计算，精度依赖路径点密度）
**速度**：固定 v_max=0.05 m/s（不自适应调度，与圆弧控制器不同）
**路径类型**：ELLIPSE（椭圆）或 SPLINE（B样条），由上层传入
**反馈限制**：单一比例 feedback_limit_ratio（无 before/after 区分）

---

## 2. 当前状态（每次测试后更新此节）

| 项目 | 状态 |
|------|------|
| **当前配置** | C00 基线 — 初始参数，未调优 |
| **yaml 状态** | `lqr_curve.yaml` 已添加圆弧移植参数 |
| **最优实测** | 无（尚未测试） |
| **调优状态** | 控制器算法已升级，待首次测试 |
| **移植状态** | ✅ 已移植圆弧控制器 8 项算法 |

---

## 3. 历史批次成绩表

| 批次 | batch_id | avg_p90 | avg<3mm% | pass | 关键变更 |
|------|----------|---------|----------|------|---------|
| C00 | — | — | — | —/— | 初始基线，待测 |

---

## 3.5 圆弧 vs 曲线 — 关键差异（对比分析结论）

| 差异项 | 圆弧控制器（C19）| 曲线控制器（修正后）| 说明 |
|--------|----------|----------|------|
| K1/K2 来源 | 直接指定 K1=6.0, K2=1.5 | ✅ 直接指定 K1=8.0, K2=1.5 | 计算值 K1=282.8 → tanh 退化为 bang-bang |
| 位置滤波 | 仅 Hampel | ✅ 仅 Hampel | SG 引入 ~200ms 延迟 = 10mm@v=0.05 滞后 |
| before_print | 0.30 | ✅ 0.20 (保守起调) | 比例区工作需要 prop_limit > K1×e_y |
| after_print | 0.50 | ✅ 0.20 (待测试调) | — |
| 曲率精度 | 精确 1/R | ⚠️ 三点差分（±3%误差） | 椭圆可精确计算，B样条有噪声 |
| e_theta参考 | exact_tangent（当前点几何切线）| ⚠️ 前瞻点切线（前置偏置）| 小半径曲线 R=0.3m 产生约 2-3° 系统误差 |
| 径向误差 | 几何距离到圆心 | ⚠️ Frenet前瞻投影 | 通用曲线无固定"圆心"，目前无法等效 |
| 速度调度 | 基于半径自适应 | ❌ 固定 0.05 m/s | 曲线物理约束，暂不改变 |

**已解决的高优先级问题（本次修正）**：K1/K2 bang-bang + SG 延迟 + 反馈限制过紧
**待后续调优的问题**：before/after_print 分段调优（C01+）、积分参数（C02+）

---

## 4. 当前 lqr_curve.yaml 关键参数（C00 基线）

```yaml
lqr:
  use_direct_gains: true
  K1_direct: 8.0   # 圆弧C19=6.0，曲线v更低适当提高；与before_print=0.20配合工作在比例区
  K2_direct: 1.5   # 与圆弧C19相同，临界阻尼特性
  enable_integral: false

velocity.max: 0.05 m/s
omega_max:    0.3 rad/s

feedback.limit_ratio:              0.10  # 兼容字段（新算法使用 before/after_print）
feedback.min_limit:                0.0
feedback.e_theta_lowpass_alpha:    0.82  # 移植自圆弧 C19
feedback.max_ey_jump_m:            0.005 # 移植自圆弧 C19
feedback.int_limit_ratio:          0.35  # 移植自圆弧 C19
feedback.limit_ratio_before_print: 0.20  # 参考圆弧0.30，保守起调0.20
feedback.limit_ratio_after_print:  0.20  # 待 C01+ 分段调优

lookahead.distance: 0.01m    # 总前瞻 = 0.01 + 0.2×0.05 = 0.02m (20mm)
lookahead.time:     0.2s

filter.savgol: 已移除（对齐圆弧修复7，消除200ms延迟）
filter.hampel: 窗口=5, k=3.0（保留，1帧延迟≈55ms）
```

**理论验证（K1=8, before_print=0.20, κ=2/m, v=0.05）**：
```
omega_ff     = 0.05 × 2.0 = 0.100 rad/s
prop_limit   = 0.100 × 0.20 = 0.020 rad/s
K1×e_y(3mm) = 8.0 × 0.003 = 0.024 rad/s
K2×e_θ(2°)  = 1.5 × 0.035 = 0.053 rad/s（K2主导）
omega_fb_raw ≈ -0.024 - 0.053 = -0.077 rad/s
ratio        = 0.077 / 0.020 = 3.85 → tanh(3.85) = 0.999 → 接近饱和但不是 bang-bang
─────────────────────────────────────
→ 若 e_θ≈0，e_y=3mm：ratio=1.2 → tanh=0.83 → 正常比例区 ✓
→ 若 e_y≈0，e_θ=2°：  ratio=2.6 → tanh=0.99 → 航向修正饱和（可接受）
```

### 已移植的圆弧控制器算法（C19 验证有效）

| 算法 | 说明 |
|------|------|
| e_theta IIR 低通滤波 | α=0.82，τ≈280ms，抑制航向误差高频噪声 |
| e_y 跳变检测 | 超过 5mm/周期 则冻结积分，防止 windup |
| tanh 软饱和 | 比例反馈路径替代硬 clamp，过渡平滑 |
| 双路径反馈解耦 | 比例(tanh+prop_limit) 与 积分(int_limit_ratio) 独立限幅 |
| 时变积分衰减 | decay^(ctrl_dt/T)，对控制频率抖动不敏感 |
| 实际 ctrl_dt 测量 | 用于时变衰减和平滑切换 |
| 50ms 精确采样 | 减少 CSV 体积，对齐圆弧控制器数据格式 |
| before/after 比例平滑切换 | τ=0.3s，tied to start_print 状态 |

**当前理论增益**（v=0.05 时）：
- K1 = √(200) / 0.05 = **282.8**（被 feedback_limit 大幅压制）
- K2 = √(2√(200×100) + 100) = **19.6**

---

## 5. 下一步优化计划

### 首次数据收集后的分析重点

| 分析项 | 意义 | 分析命令 |
|--------|------|---------|
| p90、avg<3mm% | 整体精度 | `analyze.py` |
| mean_signed_mm | 系统性偏左/偏右 | `analyze.py --curve 00` |
| 段分布（S1~S4）| 路径哪段精度差 | `analyze.py --curve 00` |
| 反馈饱和率 | feedback_limit 是否过紧 | 层3 自动计算 |
| std_mm | 位置噪声水平 | 层2 自动显示 |

### 优化方向（待数据验证）

| 优先级 | 方向 | 参数变更 | 预期收益 | 风险 |
|--------|------|---------|---------|------|
| 🔴 高 | feedback_limit 过紧 | `limit_ratio: 0.1→0.2` | 给 LQR 更多纠偏空间 | 可能引入抖动 |
| 🔴 高 | 系统性偏向 | 开启积分 `enable_integral: true` | 消除稳态偏心误差 | 需防 windup |
| 🟡 中 | 位置噪声大（std>2mm） | `savgol_window: 7→9` | 减少 e_y 噪声 | 延迟增大约27ms |
| 🟡 中 | 前馈不准（曲率噪声） | 调整 `lookahead_distance` | 平滑曲率估算 | 响应滞后 |

---

## 6. 已知约束（不得违反）

| # | 约束 | 原因 |
|---|------|------|
| 1 | v = 0.05 m/s 固定 | 机器人物理限制 |
| 2 | 每次只改一个参数 | 无法定位根因 |
| 3 | feedback_min_limit 保持 0.0 | 曲率→0 时前馈≈0，min_limit>0 产生无意义反馈 |
| 4 | 每次调参必须更新 optimization.id/parent_batch_id/change_note | 追踪优化链路 |
| 5 | 曲率由三点差分法计算，路径点间距应保持 3mm | 间距过大曲率失真，间距过小引入噪声 |

---

## 7. 工作流程

```bash
# 1. 分析最新批次（默认深析最差曲线）
python3 curve_tuning/analyze.py

# 2. 指定曲线深析
python3 curve_tuning/analyze.py --curve 00

# 3. 全历史对比
python3 curve_tuning/analyze.py --compare

# 4. 修改配置后
#    → 更新 lqr_curve.yaml 中的 optimization.id / parent_batch_id / change_note
#    → 更新 curve_tuning/curve_optimization_history.md
```

---

## 8. 文件索引

| 文件 | 作用 |
|------|------|
| `curve_tuning/analyze.py` | 主分析脚本（每次测试后第一件事） |
| `curve_tuning/tuning_guide.md` | 曲线调优方法论 |
| `curve_tuning/curve_optimization_history.md` | 历次批次详细记录 |
| `curve_tuning/HANDOFF.md` | **本文件**，每次新对话先读 |
| `src/xline_follow_controller/config/lqr_curve.yaml` | 当前运行配置 |
| `curve_tracking_latest/` | 最新批次采样数据（运行时生成） |
