# 曲线 LQR 调优指南

## 一、控制器结构速览

```
ω = ω_ff + clamp(ω_fb + ω_i, ±feedback_limit)
        │              │
        │         LQR 反馈 = −K1·e_y − K2·e_θ
        │         积分项   = −Ki·∫e_y
        │
        前馈 = v · κ（κ 由三点差分法从路径点计算）
```

**曲线与圆弧的核心区别**：
- 圆弧曲率κ = 1/R（常数，精确）；曲线曲率κ由三点差分估算（有噪声）
- 曲线控制器速度固定 v=0.05m/s；圆弧有基于半径的自适应调度
- 曲线只有单一 feedback_limit_ratio；圆弧有 before/after 两档

---

## 二、三层漏斗分析法

### 层1：批次概览（`analyze.py`）
- 看 p90、avg<3mm%、通过率
- 看 mean_signed_mm（系统性偏左/偏右）

### 层2：单曲线指标（`analyze.py --curve 00`）
- 段分布 S1~S4：哪段路径精度差？
- 反馈饱和率：LQR 是否被 cap 过紧？
- std_mm：位置噪声水平

### 层3：样本信号（层2 自动触发）
- e_y 带符号均值：正/负系统性偏差
- omega_correction vs feedback_limit：饱和情况
- 换向频率：是否高频抖动

---

## 三、参数调优手册

### 3.1 反馈限制（feedback_limit_ratio）

| 参数 | 作用 | 调整方向 |
|------|------|---------|
| `feedback_limit_ratio` | 反馈上限（前馈的 X%） | 默认10%，饱和率>30%时放宽到20% |
| `feedback_min_limit` | 反馈绝对下限 | **保持0**，κ→0时前馈→0，min_limit>0产生无意义干扰 |

**诊断**：
```
饱和率 > 30%  → 放宽 limit_ratio（+0.05~0.10）
饱和率 < 5%   → 可以收紧 limit_ratio（-0.05）
p90 大但饱和率低 → 问题在曲率精度或位置噪声，不是 limit
```

**注意**：曲线曲率是变化的。在曲率大的段（弯度大），前馈大，feedback_limit 的绝对值也大；
在曲率小的段（近似直线），前馈接近0，feedback_limit → 0，反馈几乎无效。
若直线段误差大，考虑开启 feedback_min_limit（谨慎！）。

### 3.2 积分项（enable_integral + Ki）

**何时开启**：
- `mean_signed_mm` 的绝对值 > 2mm（系统性偏向）
- e_y 带符号均值稳定为正或负

**开启步骤**：
1. `enable_integral: true`
2. `Ki: 0.3`（初始值），`integral_max: 0.020`，`integral_decay: 0.99`
3. 观察 mean_signed_mm 是否收敛

**风险**：
- `integral_max` 太大 → 积分过冲
- 路径曲率变化大时积分可能滞后（积分衰减系数 0.99 已能减缓）

### 3.3 前瞻距离（lookahead）

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `lookahead.distance` | 0.01m | 基础前瞻（10mm） |
| `lookahead.time` | 0.2s | 速度相关前瞻（v=0.05时加10mm） |
| 合计 | ~20mm | 较短，适合慢速精确跟踪 |

**前瞻对曲率的影响**：
- 前瞻点处的曲率决定前馈值 ω_ff
- 前瞻过短：曲率估算受噪声影响大，前馈抖动
- 前瞻过长：对路径变化响应滞后，入弯过迟

**诊断**：
- 前馈高频抖动（换向频率>3Hz）→ 适当增大 lookahead（0.01→0.025）
- 入弯/出弯时误差大 → 可能前瞻过长，考虑减小

### 3.4 位置滤波（savgol_window）

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `savgol_window` | 7 | Savitzky-Golay 窗口（必须奇数） |
| `savgol_order` | 2 | 多项式阶数 |

**诊断**：
- `std_mm > 1.5mm` → 增大 sg_window（7→9→11，延迟代价约 +27ms）
- 增大窗口后 p90 反而变差 → 延迟已影响控制，不应继续增大

### 3.5 增益（q1/q2/r 或 K1_direct/K2_direct）

**理论公式**（v=0.05 时）：
```
K1 = √(q1/r) / v      # v=0.05, q1=200 → K1≈282.8（被 feedback_limit 大幅压制）
K2 = √(2√(q1·q2)/r + q2/r)   # q1=200,q2=100 → K2≈19.6
```

**实际有效增益**（v=0.05, feedback_limit_ratio=0.1，κ=2/m 示例）：
```
ω_ff = 0.05 × 2 = 0.100 rad/s
feedback_limit = 0.100 × 0.10 = 0.010 rad/s
有效反馈最大值 ≈ 0.010 rad/s（K1/K2 的实际影响被 limit 遮蔽）
```

**结论**：在曲率大的段，K1/K2 的实际影响几乎完全被 feedback_limit 遮蔽。
调整 K1/K2 意义有限，优先调 feedback_limit_ratio 和积分。

---

## 四、曲率精度问题（曲线特有）

曲线控制器使用三点差分法计算曲率，精度取决于路径点密度：

```
κ = (x'·y'' - y'·x'') / (x'² + y'²)^(3/2)
```

**常见问题**：
- 路径点间距过大（>5mm）→ 曲率估算误差大，前馈不准
- 路径点间距过小（<1mm）→ 数值噪声放大
- 控制器生成路径时采用 3mm 间距（`kPointSpacingMeters = 0.003`），已最优化

**诊断曲率质量**：
- 层3 中 `|Δω| P90 / |ω_ff|` > 30% → 前馈偏离过大，曲率估算有问题
- 换向频率 > 3Hz → 前馈在快速正负切换，可能是曲率噪声

---

## 五、调参工作流程

```
测试 → analyze.py → 确认最差指标 → 选1个参数调整
  → 更新 yaml (id/parent/note) → 测试 → analyze.py → 对比
```

**每次只改一个参数**（否则无法定位根因）

**yaml 必须更新的三个字段**：
```yaml
optimization:
  id: "curve_C01_increase_limit_ratio"
  parent_batch_id: "<上一次的 batch_id>"
  change_note: "feedback_limit_ratio: 0.10→0.20"
```

---

## 六、已知"坑"

| # | 教训 | 不能做什么 |
|---|------|----------|
| 1 | feedback_min_limit > 0 在直线段（κ≈0）产生干扰 | 除非明确需要，保持 min_limit = 0.0 |
| 2 | 曲率噪声会导致前馈抖动，增大 limit_ratio 会放大前馈噪声的影响 | 不要盲目放宽 limit_ratio，先分析换向频率 |
| 3 | K1/K2 在曲线段完全被 feedback_limit 遮蔽 | 不要反复调 q1/q2，先确保 feedback_limit 合理 |
| 4 | 前瞻点处的曲率决定前馈，前瞻距离影响曲率估算点 | lookahead 变化会联动影响前馈，改 lookahead 后需重测 |

---

## 七、快速诊断图

```
p90 > 10mm ──→ 检查路径点密度、v_max 范围、曲率计算
     ↓
5mm < p90 < 10mm ──→ 看 mean_signed_mm：
                        > 2mm → 开积分
                        < 2mm → 看饱和率：
                                  > 30% → 放宽 limit_ratio
                                  < 30% → 看段分布：
                                            S1/S4 差 → 检查入口对准和前馈精度
                                            均匀差 → 增大 lookahead / 加强滤波
     ↓
p90 < 5mm ──→ 看 avg<3mm%：
                < 65% → 减小噪声（sg_window↑）或 开积分（有系统偏差）
                ≥ 65% → 目标达成 ✓
```
