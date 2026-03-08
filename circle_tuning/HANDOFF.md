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
| **当前配置** | ★ C19 最终锁定 — integral_max回退0.040，savgol保持7，after_print=0.50（C18振荡教训）|
| **yaml 状态** | `lqr_circle.yaml` 已锁定，`lqr_circle_best.yaml` 已更新为最终定稿 |
| **当前最优实测** | C13新地形 — 4/5通过，avg_p90=4.79mm ✓，已具备跨地形鲁棒性 |
| **调优状态** | **已收尾** — circle_02/03 Q4物理噪声本底为地形限制，不可优化，接受现状 |
| **控制器最优备份** | `circle_tuning/lqr_circle_best.yaml` — 随最优实测结果覆盖更新 |
| **数据采集状态** | ✅ 完整：samples含圆心坐标+real_angle_rad；metrics含p90_radial_mm等径向指标 |

---

## 3. 历史批次成绩表

| 批次 | batch_id | avg_p90_radial | avg<3mm%_radial | pass | 关键变更 |
|------|----------|----------------|-----------------|------|---------|
| C00 | 1772922769017 | 13.64mm | 38.3% | 0/5 | 初始基线，feedback_after=5% |
| C01 | 1772923713949 | 4.17mm | 76.6% | 3/5 | feedback_after 5%→20% |
| C02 | 1772927381988 | 3.89mm | 83.5% | 4/5 | feedback_after 20%→30% |
| C03 | 1772932226550 | 4.38mm | 78.7% | 4/5 | feedback_before 20%→30% |
| C04 | 1772934115424 | 14.16mm | 38.5% | 0/5 | tanh+5%→积分被淹没，精度崩溃 |
| C05 | 1772937524874 | 13.25mm | 23.1% | 0/3 | 符号修复+精确切线+双路反馈，但after=8%导致比例bang-bang，修正力严重不足 |
| C06 | 1772968332362 | 6.88mm | 64.3% | 2/5 | before=30%/after=30%（恢复C02水平），积分73~89%天花板，大半径偏外未修正 |
| C07 | 1772969500963 | 5.97mm | 65.3% | 2/5 | int_limit_ratio 0.20→0.35（对积分实际无效！state上限=25mrad/s已先触发；omega偏ff P90=30~33%，主因K2过大） |
| C08 | 1772971516780 | 6.79mm | 49.4% | 1/5 | K2=0.8→欠阻尼(ζ=0.54~0.73)，e_theta_std翻倍，路径波动σ=4-5mm，彻底失败 |
| C09 | 跳过 | — | — | —/5 | K2回退1.5(修复欠阻尼) + max_ey_jump_m 8→5mm(物理阈值修正)，叠入C10测试 |
| C10 | 1772973765536 | 5.15mm | 70.9% | 3/5 | e_theta低通alpha=0.82（硬饱和0~18%✓，omega±15%ff全部达标✓，3/5通过↑自1/5）|
| C11 | 1772975325358 | 5.36mm | 66.1% | 2/5 | velocity_step 0.01：circle_03改善✓，但circle_04/05同速也退步→批次环境噪声，circle_02 Q4物理异常(P90=9.94mm) |
| C12 | 1772976362881 | 5.41mm | 71.9% | 3/5 | limit_ratio→0.40：circle_04硬饱和1.3%✓,P90 5.88→4.26mm✓；circle_05 before=40%加剧入圆Q1 P90=11.48mm ✗ |
| C13 | 1772989187240(新地形) | 4.79mm | 68.1% | 4/5 | before=0.30精确修复R=0.405m入圆✓，配置跨地形鲁棒性验证✓，R=0.705m积分上限不足 |
| C14 | 1772990490356 | 6.80mm | 51.9% | 0/5 | integral_max=0.040：circle_03 bias+2.38→+1.16mm✓积分有效，但位置噪声std≈4mm(circle_01硬饱和2%仍P90=6.30mm)是失败主因 |
| C15 | 1772994025340 | 8.23mm | 42.0% | 0/5 | alpha=0.88失败：e_theta_std 32→56mrad(+75%!)，heading环路振荡，circle_04/05 bias方向翻转，硬饱和23→61% |
| C16 | 1772995210112 | 5.64mm | 64.8% | 1/5 | alpha恢复0.82 + after_print 0.40→0.50：硬饱和消除(33-36%→0-3%)，circle_05通过✓，std≈3.4mm是主瓶颈 |
| C17 | 1772996579382 | 5.73mm | 63.0% | 0/5 | savgol_window 7→9：无效，地形噪声更差(circle_05 std 2.48→3.26mm)；根因=积分state_cap限幅(circle_01所需0.00644>0.005) |
| C18 | 1772997858940 | 6.83mm | 55.0% | 0/5 | integral_max=0.056→circle_01振荡：std 3.28→5.37mm翻倍，e_theta_std 0.036→0.057rad翻倍，积分≈int_limit |
| C19 | 锁定 | — | — | — | 收尾锁定：integral_max回退0.040，保留after=0.50。最终配置=C16基础+C14积分上限 |

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
