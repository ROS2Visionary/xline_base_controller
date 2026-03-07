# 圆弧 LQR 调优指南

## 一、控制器结构速览

```
ω = ω_ff + clamp(ω_fb + ω_i, ±feedback_limit)
        │              │
        │         LQR 反馈 = −K1·e_y − K2·e_θ
        │         积分项   = −Ki·∫e_y
        │
        前馈 = v · κ = v / R
```

**圆弧与直线的核心区别**：
- 圆弧有前馈 ω_ff = v/R（常数），这是主要控制量
- 反馈被限制在前馈的 X% 以内（喷墨前 20%，喷墨后 5%）
- 直线曲率κ=0，前馈=0，纯依赖 LQR 反馈

---

## 二、三层漏斗分析法

### 层1：批次概览（`analyze.py`）
- 看 p90、avg<3mm%、通过率
- 看 mean_radial_mm（系统性偏内/偏外）

### 层2：单圆弧指标（`analyze.py --circle 01`）
- 象限分布 Q1~Q4：哪段圆弧精度差？
- 反馈饱和率：LQR 是否被 cap 过紧？

### 层3：样本信号（层2 自动触发）
- e_y 带符号均值：正/负系统性偏差
- 径向误差分布：偏外/偏内比例
- omega_correction vs feedback_limit：饱和情况

---

## 三、参数调优手册

### 3.1 反馈限制（feedback_limit_ratio）

| 参数 | 作用 | 调整方向 |
|------|------|---------|
| `limit_ratio_after_print` | 喷墨后反馈上限 | 默认5%，饱和率>30%时放宽到10% |
| `limit_ratio_before_print` | 喷墨前反馈上限 | 默认20%，通常足够 |
| `feedback_min_limit` | 反馈绝对下限 | **保持0**，非零会在直线段产生无意义反馈 |

**诊断**：
```
饱和率 > 30%  → 放宽 limit_ratio（+0.05）
饱和率 < 5%   → 可以收紧 limit_ratio（-0.05）
e_y 大但反馈被 cap → 放宽 limit 或增大积分
```

### 3.2 积分项（enable_integral + Ki）

**何时开启**：
- `mean_radial_mm` 的绝对值 > 2mm（系统性偏心）
- e_y 带符号均值稳定为正或负（单侧偏差）

**开启步骤**：
1. `enable_integral: true`
2. `Ki: 0.3`（初始值），`integral_max: 0.020`，`integral_decay: 0.99`
3. 观察 mean_radial_mm 是否收敛

**注意**：
- 圆弧积分**不应有** `start_line_aligned_` 条件（直线才需要）
- integral_max 太大会导致积分越过目标后过冲

### 3.3 前瞻距离（lookahead）

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `lookahead_distance` | 0.015m | 基础前瞻（15mm） |
| `lookahead_time` | 0.05s | 速度相关前瞻（v=0.07时加3.5mm） |
| 合计 | ~18mm | 非常短，适合慢速精确跟踪 |

**诊断**：
- e_y 高频振荡（标准差 > 2mm）→ 适当增大 lookahead（0.015→0.025）
- 响应滞后（象限间误差有相位差）→ 减小 lookahead

### 3.4 位置滤波（savgol_window）

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `savgol_window` | 7 | Savitzky-Golay 窗口（必须奇数） |
| `savgol_order` | 3 | 多项式阶数 |
| `hampel_window` | 5 | Hampel 异常值检测窗口 |
| `hampel_k` | 3.0 | Hampel 阈值（标准差倍数） |

**诊断**：
- e_y 标准差 > 1.5mm → 增大 sg_window（7→9→11，延迟代价约 +27ms）

### 3.5 增益（q1/q2/r 或 K1_direct/K2_direct）

**理论公式**（v=0.07 时）：
```
K1 = √(q1/r) / v    # v=0.07, q1=200 → K1≈202（被 feedback_limit cap）
K2 = √(2√(q1·q2)/r + q2/r)  # q1=200,q2=300 → K2≈28.1
```

**实际有效增益**（v=0.07, R=0.5, feedback_limit=5% of ff）：
```
feedback_limit = |ω_ff| × 0.05 = (0.07/0.5) × 0.05 = 0.007 rad/s
有效反馈最大值 ≈ 0.007 rad/s（对 7mm 误差 ≈ 1 rad/s 理论值的 0.7%）
```

**结论**：在喷墨后阶段，K1/K2 的实际影响几乎完全被 feedback_limit 遮蔽。
调 K1/K2 的意义在于**喷墨前阶段**（20% 限制时）和**积分饱和行为**。

---

## 四、调参工作流程

```
测试 → analyze.py → 确认最差指标 → 选1个参数调整
  → 更新 yaml (id/parent/note) → 测试 → analyze.py → 对比
```

**每次只改一个参数**（否则无法定位根因）

**yaml 必须更新的三个字段**：
```yaml
optimization:
  id: "circle_C01_increase_limit_ratio"
  parent_batch_id: "<上一次的 batch_id>"
  change_note: "limit_ratio_after_print: 0.05→0.10"
```

---

## 五、已知"坑"

| # | 教训 | 不能做什么 |
|---|------|----------|
| 1 | feedback_min_limit > 0 在直线段产生无意义反馈 | 保持 min_limit = 0.0 |
| 2 | 圆弧前馈 ω_ff = v/R，R 小时前馈大，feedback_limit 的绝对值也随之增大 | 不同半径时 feedback 表现不同，不能用同一批次数据推断所有半径 |
| 3 | K1/K2 在喷墨后被反馈限制遮蔽，改 q1/q2 意义不大 | 不要反复调 q1/q2，先确保 feedback_limit 合理 |
| 4 | 积分在圆弧控制器中无 start_line_aligned_ 条件 | 不应直接照搬直线控制器的积分逻辑 |

---

## 六、快速诊断图

```
p90 > 10mm ──→ 检查圆心准确性、v_max 范围
     ↓
5mm < p90 < 10mm ──→ 看 mean_radial_mm：
                        > 2mm → 开积分
                        < 2mm → 看饱和率：
                                  > 30% → 放宽 limit_ratio
                                  < 30% → 看象限分布：
                                            不均匀 → 检查入圆对准
                                            均匀差 → 增大 lookahead / 加强滤波
     ↓
p90 < 5mm ──→ 看 avg<3mm%：
                < 65% → 减小噪声（sg_window↑）或 开积分（如有系统偏差）
                ≥ 65% → 目标达成 ✓
```
