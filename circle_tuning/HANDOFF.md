# 调优交接文档 — LQR 圆弧/圆形跟踪控制器

> 每次新对话开始时先读此文件，再运行 `python3 circle_tuning/analyze.py` 查看最新数据。

---

## 1. 项目背景（30秒速读）

**目标**：圆弧/圆形路径跟踪，p90 < 5mm，avg<3mm% > 65%
**控制器**：`lqr_circle_controller.cpp`
**控制方程**：ω = ω_ff + clamp(−K1·e_y − K2·e_θ + ω_i, ±feedback_limit)
**前馈**：ω_ff = v · κ = v / R（圆弧跟踪的核心，直线控制器没有）
**速度调度**：基于半径自适应，R<0.5m→0.05m/s，每+0.1m加0.01m/s，上限0.10m/s
**半径**：任意传入（影响前馈精度和 K1 量级）

---

## 2. 当前状态（每次测试后更新此节）

| 项目 | 状态 |
|------|------|
| **当前配置** | C01 — limit_ratio_after_print 5%→20% |
| **yaml 状态** | `lqr_circle.yaml` 已更新，待物理测试 |
| **历史最优** | 无（C00 全部不通过） |
| **最新实测批次** | C00 — batch_id=1772922769017，avg_p90=13.64mm，0/5通过 |
| **数据采集状态** | ✅ 完整：samples含圆心坐标+real_angle_rad；metrics含p90_radial_mm等径向指标 |

---

## 3. 历史批次成绩表

| 批次 | batch_id | avg_p90_radial | avg<3mm%_radial | pass | 关键变更 |
|------|----------|----------------|-----------------|------|---------|
| C00 | 1772922769017 | 13.64mm | 38.3% | 0/5 | 初始基线，feedback_after=5% |
| C01 | 待测 | ? | ? | ?/5 | feedback_after 5%→20% |

---

## 4. 当前 lqr_circle.yaml 关键参数（C00）

```yaml
# 增益（use_direct_gains=false，从 q1/q2/r 计算）
q1: 200.0
q2: 300.0
r:  1.0

# 速度（调度由 velocity_schedule 节控制）
velocity.max: 0.10   # 调度上限
velocity.min: 0.05   # 调度下限

# 速度调度规则
velocity_schedule:
  enable_radius_schedule: true
  radius_threshold: 0.5    # m，低于此值用 v_min
  radius_step:      0.1    # m，每增加此值速度+velocity_step
  velocity_step:    0.01   # m/s

# 当前调度表：R=0.3→0.05，R=0.5→0.05，R=0.6→0.06，R=0.7→0.07，R=1.0→0.10(封顶)

# 前瞻
lookahead.distance: 0.015   # 15mm 基础前瞻
lookahead.time:     0.05    # 总前瞻 ≈ 18mm

# 反馈限制（圆弧核心机制）
feedback.limit_ratio_before_print: 0.20   # 喷墨前：前馈的20%
feedback.limit_ratio_after_print:  0.05   # 喷墨后：前馈的5%
feedback.min_limit: 0.0

# 积分（当前关闭）
enable_integral: false

# 控制频率
control.frequency: 18.0 Hz
```

**当前理论增益**（v=0.07, R=0.5m 示例）：
- ω_ff = 0.07 / 0.5 = 0.140 rad/s（前馈，与 R 成反比）
- K1 ≈ 202，K2 ≈ 28.1（但反馈被 cap 到前馈的 5%~20%）

---

## 5. 下一步优化计划（C00 数据收集后）

### 数据收集后的分析重点

| 分析项 | 意义 | 分析命令 |
|--------|------|---------|
| p90、avg<3mm% | 整体精度 | `analyze.py` |
| 径向偏差（mean_radial_mm）| 系统性偏内/偏外 | `analyze.py --circle 01` |
| 象限分布（Q1~Q4）| 圆弧哪段精度差 | `analyze.py --circle 01` |
| 反馈饱和率 | feedback_limit 是否过紧 | samples 列 omega_correction vs feedback_limit |
| 积分饱和情况 | 是否需要开启积分 | 看径向偏差是否系统性 |

### 优化方向（基于直线控制器经验，需数据验证）

| 优先级 | 方向 | 参数变更 | 预期收益 | 风险 |
|--------|------|---------|---------|------|
| 🔴 高 | feedback_limit 过紧 | `limit_ratio_after_print: 0.05→0.10` | 给 LQR 更多纠偏空间 | 可能引入抖动 |
| 🔴 高 | 系统性径向偏差 | 开启积分 `enable_integral: true` | 消除稳态偏心误差 | 需防 windup |
| 🟡 中 | 前瞻距离调整 | `lookahead_distance: 0.015→0.025` | 减少高频抖动 | 可能降低响应 |
| 🟡 中 | 位置噪声大 | 加强 Savitzky-Golay 滤波窗口 | 减少 e_y 噪声 | 延迟增大 |

---

## 6. 已知约束（不得违反）

| # | 约束 | 原因 |
|---|------|------|
| 1 | v = 0.05~0.10 m/s | 机器人物理限制 |
| 2 | 每次只改一个参数 | 无法定位根因 |
| 3 | feedback_min_limit 不设为正值 | 直线路径（κ=0）时前馈为0，min_limit>0会产生无意义反馈 |
| 4 | 每次调参必须更新 optimization.id/parent_batch_id/change_note | 追踪优化链路 |
| 5 | R=0.3m 时 ω_ff=0.233 rad/s，接近 omega_max=0.3 | 留给反馈的余量极小，该半径对参数变化最敏感 |
| 6 | 每批次最多支持 8 圈（kBatchSize=8） | 超过 8 圈会清空数据目录，5个半径完全安全 |

---

## 7. 工作流程

```bash
# 1. 分析最新批次（4圈数据）
python3 circle_tuning/analyze.py

# 2. 指定圆弧深析
python3 circle_tuning/analyze.py --circle 01

# 3. 全历史对比
python3 circle_tuning/analyze.py --compare

# 4. 修改配置后
#    → 更新 lqr_circle.yaml 中的 optimization.id / parent_batch_id / change_note
#    → 更新 circle_tuning/circle_optimization_history.md
```

---

## 8. 文件索引

| 文件 | 作用 |
|------|------|
| `circle_tuning/analyze.py` | 主分析脚本（每次测试后第一件事） |
| `circle_tuning/tuning_guide.md` | 圆弧调优方法论 |
| `circle_tuning/circle_optimization_history.md` | 历次批次详细记录 |
| `circle_tuning/HANDOFF.md` | **本文件**，每次新对话先读 |
| `src/xline_follow_controller/config/lqr_circle.yaml` | 当前运行配置 |
| `circle_tracking_latest/` | 最新批次采样数据（运行时生成） |
