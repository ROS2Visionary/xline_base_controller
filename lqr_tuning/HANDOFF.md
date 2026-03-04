# 调优交接文档 — LQR 直线跟随控制器

> 每次新对话开始时先读此文件，再运行 `python3 lqr_tuning/analyze.py` 查看最新数据。

---

## 1. 项目背景（30秒速读）

**目标**：水泥地面直线跟随，6条路径全部 p90 < 5mm，avg<3mm% > 65%。
**控制器**：LQR 单角速度控制，状态 [e_y, e_theta]，ω = -K1·e_y - K2·e_theta - Ki·∫e_y
**速度**：v=0.25 m/s（固定，**不得修改 walk_max/work_max**）
**限制**：max_angular_vel=0.066 rad/s（物理天花板，不得随意提高）

---

## 2. 当前状态（每次测试后更新此节）

| 项目 | 状态 |
|------|------|
| **当前配置** | B39 — `opt_20260304_b39_revert_tol_v1` |
| **line.yaml 状态** | 已写入，**待物理测试** |
| **历史最优** | B35（avg<3mm%=62.1%，pass=3/6）→ `line_best.yaml` |
| **最新实测批次** | B38（batch `1772652009287`，pass=2/6，avg<3mm%=50.1%）|
| **B39 预期** | 恢复到 B36 水平（avg<3mm%≈60%，pass≥3/6） |

---

## 3. 历史批次成绩表

| 批次 | batch_id | avg<3mm% | pass | 关键变更 |
|------|----------|---------|------|---------|
| B33 | 1772641030819 | 59.2% | 2/6 | 结构性修复基准 |
| B34 | 1772643134732 | 55.6% | 2/6 | 回退 |
| **B35** | **1772645333839** | **62.1%** | **3/6** | **历史最优** ← line_best.yaml |
| B36 | 1772648450356 | 60.5% | 3/6 | q2=0.42（K2: 1.072→1.109, ζ+） |
| B37 | 1772650459293 | 52.7% | 2/6 | align_tol 6→4mm + ey_start 4.8→3mm（双改回归） |
| B38 | 1772652009287 | 50.1% | 2/6 | 撤回 ey_start（仍 4mm） → 确认 align_tol=4mm 是根因 |
| **B39** | 待测 | 预计~60% | 预计3/6 | 撤回 align_tol 4→6mm，恢复 B36 基线 |

---

## 4. 当前 line.yaml 关键参数（B39）

```yaml
# 增益
q1: 0.39          # K1=2.50 @v=0.25
q2: 0.42          # K2=1.109（速度无关）
K1_max: 8.0
K1_min: 1.5

# K2 调制（重要约束：k2_min_floor_scale 必须 = k2_anti_cancel.scale）
k2_gate.ey_start: 0.0048   # 不得 < 0.004（低于此值会压制正常跟线阶段的 K2）
k2_gate.ey_end:   0.0180
k2_gate.min_scale: 0.62
k2_anti_cancel.scale: 0.45   # = k2_min_floor_scale（不变量！）
k2_min_floor_scale: 0.45     # = k2_anti_cancel.scale（不变量！）

# 积分
Ki: 0.42
integral_max: 0.015
integral_decay: 0.992

# 对齐
line_cross_track_tolerance: 0.0060   # B39 撤回（4mm 会导致 path_01 系统偏左2mm）

# 滤波
e_theta_lowpass.alpha: 0.82
ey_filter.lowpass.alpha: 0.87
```

**当前 ζ**（v=0.25，K2_eff=1.109）：**ζ = 0.701**（目标范围 0.65~0.85 ✓）

---

## 5. 下一步优化计划（B39 通过后执行）

### 优先级排序

| 优先级 | 方向 | 参数变更 | 预期收益 | 风险 |
|--------|------|---------|---------|------|
| 🔴 高 | 积分饱和（4/6路径>48%） | `integral_max: 0.015→0.020` | path_01/02/04/05 系统偏差改善 | 低 |
| 🟡 中 | K2 阻尼增强 | `q2: 0.42→0.46`（K2→1.143, ζ→0.723） | 所有路径 cancel 频率降低 | 低 |
| 🔴 高 | path_06 根因调查 | 查 path_05 出口→path_06 入口条件 ct[0] | 可能解锁 path_06 突破 | 需代码分析 |

**B40 建议**（B39 恢复 ≥60% 后）：先单独试 `integral_max→0.020`，观察 path_01/02/04/05 积分饱和率和<3mm%。

---

## 6. 已知的"坑"（绝对不要重蹈）

| # | 教训 | 不能做什么 |
|---|------|----------|
| 1 | align_tol=4mm 导致 path_01 系统偏左 2mm | 不要将 `line_cross_track_tolerance` 收紧到 <5mm |
| 2 | k2_gate.ey_start=3mm 误伤正常跟线（误差2-4mm在门控激活区） | ey_start 不得低于正常 e_y 分布的 p90（当前≈4.8mm） |
| 3 | floor ≠ anti_cancel 时 floor 覆盖 anti_cancel | `k2_min_floor_scale` 必须严格等于 `k2_anti_cancel.scale`（均为0.45） |
| 4 | K1↑ 不同步 K2↑ → ζ 单调下降 | K1 增大时必须同步增大 K2，公式 K2_new = K2_old × √(K1_new/K1_old) |
| 5 | 每次同时改多个参数 → 无法定位根因 | **每次只改一个参数** |

---

## 7. 工作流程

```bash
# 1. 分析最新批次
python3 lqr_tuning/analyze.py

# 2. 指定路径深析
python3 lqr_tuning/analyze.py --path 02

# 3. 全历史对比
python3 lqr_tuning/analyze.py --compare

# 4. 修改配置后标记批次
#    → 更新 line.yaml 中的 optimization.id / parent_batch_id / change_note
#    → 更新 lqr_tuning/line_optimization_history.md
#    → 若新配置超过最优：cp line.yaml line_best.yaml
```

---

## 8. 文件索引

| 文件 | 作用 |
|------|------|
| `lqr_tuning/analyze.py` | 主分析脚本（每次测试后第一件事） |
| `lqr_tuning/tuning_guide.md` | 完整调优方法论 + 历史教训 |
| `lqr_tuning/line_optimization_history.md` | 历次批次详细记录 |
| `lqr_tuning/HANDOFF.md` | **本文件**，每次新对话先读 |
| `src/xline_follow_controller/config/line.yaml` | 当前运行配置 |
| `src/xline_follow_controller/config/line_best.yaml` | 最优备份（B35） |
| `line_tracking_latest/` | 最新批次采样数据 |
