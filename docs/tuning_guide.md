# LQR 直线跟随控制器参数调优指南

> 版本：2026-03-04 | 适用代码：b33+
> 核心原则：**先诊断，后调参；每次只改一个变量；用数据验证而非凭感觉判断**

---

## 目录

1. [控制链路全景](#1-控制链路全景)
2. [数据文件结构与字段含义](#2-数据文件结构与字段含义)
3. [分析流程：三层漏斗法](#3-分析流程三层漏斗法)
4. [诊断决策树：症状→根因→参数](#4-诊断决策树症状根因参数)
5. [参数调优规则手册](#5-参数调优规则手册)
6. [正确的实验设计](#6-正确的实验设计)
7. [历史错误模式与教训](#7-历史错误模式与教训)
8. [快速参考：数值范围与调整步长](#8-快速参考数值范围与调整步长)

---

## 1. 控制链路全景

理解整个信号流是调参的前提。每一帧控制周期的完整处理链：

```
传感器位姿 (x, y, θ)
    │
    ▼
[位置滤波]  Hampel(窗口5) → SG滤波(窗口5, 3阶)
    │          ← filter.pos_hampel_* / filter.pos_savgol_*
    │
    ▼
[误差计算]  findNearestPoint → findLookaheadPoint → computeLQRErrors
    │          ← distance.lookahead, distance.lookahead_time
    │          输出：e_y_raw（横向误差）, e_theta_raw（航向误差）
    │
    ▼
[e_y 输入滤波]  rate_limit → lowpass
    │          ← ey_filter.rate_limit.*, ey_filter.lowpass.alpha
    │          输出：e_y_filtered
    │
    ▼
[e_theta 输入滤波]  lowpass（不平整地面专用）
    │          ← lqr_angular_control.e_theta_lowpass.*
    │          输出：e_theta_filtered（CSV 中 e_theta_rad）
    │
    ▼
[增益计算]  computeLQRGains(v)
    │          use_direct_gains=false: K1=√q1/v, K2=√(2√(q1q2)+q2)
    │          use_direct_gains=true:  K1=K1_direct, K2=K2_direct
    │          ← lqr_angular_control.q1/q2/r/K1_max/K1_min
    │
    ▼
[增益调制]  tail_schedule → k2_gate → k2_anti_cancel → k2_min_floor
    │          k1_eff = K1 × (1 + 0.25×schedule)   [仅长路径后段]
    │          k2_eff = K2 × gate_scale × anti_cancel_scale
    │          k2_eff = max(k2_eff, K2×k2_min_floor_scale)
    │
    ▼
[控制律]  ω_fb = -k1_eff × e_y - k2_eff × e_theta
    │      ω_i  = -Ki × ∫e_y·dt   [仅跟随阶段]
    │      ω    = ω_ff + ω_fb + ω_i
    │
    ▼
[输出滤波]  二阶平滑器(smoother) → 一阶低通(可选)
    │          ← lqr_angular_control.output_filter.*
    │
    ▼
[物理限幅]  applyAngularLimits(ω, dt)
    │          ← phase.following.max_angular_vel / max_angular_accel
    │
    ▼
角速度指令 → 底盘
```

**关键洞察**：调参时必须区分"信号在链路哪个位置出问题"。
同样是"横向误差大"，可能源于滤波、增益、限幅或积分的任一环节。

---

## 2. 数据文件结构与字段含义

### 2.1 batch_metrics.csv（批次汇总层）

每次 6 路径验证后追加一行。这是**第一层分析入口**。

| 字段 | 含义 | 正常范围 | 异常判断 |
|------|------|---------|---------|
| `path_length_m` | 路径长度 | 1.8m 或 7.8m | — |
| `p50_mm` | 横向误差中位数 | <3mm | >5mm 说明系统性偏差 |
| `p90_mm` | 横向误差 90 分位 | <5mm（目标） | >8mm 严重问题 |
| `ratio_lt_3mm` | <3mm 占比 | >0.70 | <0.50 需重点调 |
| `ratio_lt_5mm` | <5mm 占比 | >0.90 | <0.70 说明振荡严重 |
| `stable_ratio_lt_3mm` | 稳定段 <3mm 占比 | >0.70 | 与 ratio_lt_3mm 差值大说明振荡段集中 |
| `longest_breach_3_samples` | 超过 3mm 的最长连续段（采样点数） | <20 | >40 说明有持续漂移 |
| `longest_breach_5_samples` | 超过 5mm 的最长连续段 | <10 | >30 说明有大幅震荡 |
| `seg1_ratio_lt_3mm` | 前 1/3 段 <3mm 占比 | >0.70 | — |
| `seg3_ratio_lt_3mm` | 后 1/3 段 <3mm 占比 | >0.70 | 若 seg3 << seg1：后段漂移 |
| `seg3_stable_ratio_lt_5mm` | 后段稳定 <5mm | >0.80 | — |
| `drift_tail_minus_head_mm` | 后半段均值 - 前半段均值 | \|值\| <1.5mm | >3mm 单向漂移；振荡时接近 0 |
| `w_saturation_ratio` | 角速度饱和（触及 max_w）占比 | <0.05 | >0.10 说明 max_w 或 K1 过大 |
| `pass_p90_lt_5mm` | 本路径是否通过（p90<5mm） | 1 | — |

**关键列对**：
- `seg1_ratio_lt_3mm` vs `seg3_ratio_lt_3mm`：量化前后段差异，正差值=后段恶化
- `ratio_lt_3mm` vs `stable_ratio_lt_3mm`：差值小表示误差分布均匀；差值大表示有振荡峰值但均值还好
- `drift_tail_minus_head_mm`：**符号和幅度是漂移 vs 振荡的核心区分指标**

### 2.2 path_XX_metrics.csv（路径层）

单条路径的汇总，字段与 batch_metrics 相同但只有一行。用于单路径精细分析。

额外重要字段（来自 deep_analysis_by_path.csv）：

| 字段 | 含义 | 异常判断 |
|------|------|---------|
| `mean_cancel_ratio_logged` | 平均抵消比（cancel_ratio 均值） | >0.35 说明 K2 被频繁压制 |
| `severe_cancel_ratio` | cancel_ratio>0.5 的帧占比 | >0.10 值得关注 |
| `w_saturation_ratio` | 触及 max_w 的帧占比 | >0.10 说明控制受物理限制 |
| `tail_under3_ratio` | 后 1/4 段 <3mm 占比 | <0.30 后段严重恶化 |
| `objective_score` | 综合评分 | >0.75 较好，<0.50 需重优化 |

### 2.3 path_XX_samples.csv（采样层）

每 ~55ms 一行的原始采样。这是**定位根本原因的最细粒度数据**。

#### 完整字段含义表

| 字段 | 单位 | 含义 | 调参关联 |
|------|------|------|---------|
| `t_s` | s | 跟随阶段累计时间 | — |
| `x, y` | m | 机器人世界坐标 | — |
| `cross_track_mm` | mm | 到路径的横向误差（正=左偏） | 主要优化目标 |
| `linear_speed_mps` | m/s | 当前线速度 | 影响 K1（动态模式） |
| `angular_cmd_rps` | rad/s | 最终角速度指令（限幅后） | — |
| `path_yaw` | rad | 路径方向角 | — |
| `target_yaw` | rad | 制导目标航向 | — |
| `yaw_error_rad` | rad | 机器人航向 - 目标航向 | — |
| `lqr_mode` | 0/1 | 是否使用 LQR（0=PID） | — |
| `start_aligned` | 0/1 | 是否已进入跟随阶段 | 积分只在=1时积累 |
| `phase_max_w` | rad/s | 当前阶段最大角速度 | 限幅约束 |
| `k1` | — | 当前帧 K1 增益 | 动态模式下随速变化 |
| `k2` | — | 当前帧 K2_effective（所有调制后） | 越小阻尼越弱 |
| `cancel_ratio` | [0,1] | K1项与K2项的抵消比例 | >0.5 说明两项相互削弱 |
| `e_y_raw_mm` | mm | 未经滤波的横向误差 | 与 filtered 差大→滤波作用强 |
| `e_y_rate_limited_mm` | mm | 速率限制后的 e_y | 与 raw 差大→有跳变被平滑 |
| `e_y_filtered_mm` | mm | 低通后的 e_y（进入控制律） | 控制律实际使用值 |
| `e_theta_rad` | rad | 低通后的航向误差（进入控制律） | 控制律实际使用值 |
| `e_theta_raw_rad` | rad | 未经滤波的航向误差 | 与 e_theta 差大→地板缝扰动严重 |
| `omega_ff` | rad/s | 前馈项（直线路径=0） | — |
| `omega_fb` | rad/s | 反馈项 = -K1·e_y - K2·e_theta | 符号和幅度揭示控制行为 |
| `omega_i` | rad/s | 积分项 = -Ki·∫e_y | 偏置补偿量 |
| `omega_before_limits` | rad/s | 限幅前总角速度 | 与 angular_cmd 差大→被截幅 |
| `nearest_idx` | — | 最近路径点索引 | 除以总点数=路径进度 |
| `lookahead_idx` | — | 前瞻点索引 | — |
| `integral_state` | m·s | 积分器蓄积量 | 应缓慢变化；突变说明 windup |
| `tail_schedule` | [0,1] | 长路径后段调度系数 | >0 说明 K1 boost 生效 |
| `k2_floor_active` | 0/1 | K2 是否被下限约束 | 长时间=1 说明 min_floor 在工作 |
| `e_theta_raw_rad` | rad | e_theta 滤波前原始值 | 用于量化地板缝扰动幅度 |

---

## 3. 分析流程：三层漏斗法

### 第一层：batch_metrics.csv — 定位问题路径（2 分钟）

```
步骤 1：看哪条路径 pass_p90_lt_5mm=0（不达标）
步骤 2：按 p90_mm 降序排列，记录最差 2 条路径编号
步骤 3：对比 seg1_ratio_lt_3mm vs seg3_ratio_lt_3mm
         seg3 << seg1（差值 >0.15）→ 后段漂移/振荡恶化
         seg3 ≈ seg1                → 均匀分布，可能是系统偏差
步骤 4：看 drift_tail_minus_head_mm
         |drift| > 2mm，同号    → 单向漂移（积分不足或侧坡）
         |drift| < 1mm，p90 高   → 振荡（来回摆动，均值接近0）
步骤 5：看 w_saturation_ratio
         > 0.08                  → 物理限幅频繁，K1 过大或 max_w 太小
```

**示例（真实数据 batch_id=1772572362582）**：
```
path_01: p90=4.47mm ✓  seg1=0.34, seg3=0.78  drift=-1.22  saturation=0.000
path_02: p90=6.96mm ✗  seg1=0.95, seg3=0.59  drift=+1.41  saturation=0.003
path_03: p90=7.07mm ✗  seg1=0.42, seg3=0.62  drift=-0.90  saturation=0.006
path_04: p90=10.2mm ✗  seg1=0.38, seg3=0.16  drift=+3.91  saturation=0.070
path_05: p90=12.2mm ✗  seg1=0.48, seg3=0.21  drift=+2.79  saturation=0.141

诊断：
- path_04/05 最差，后段显著恶化（seg3 仅 0.16/0.21），有明显正向漂移（+3.9/+2.8mm）
- path_04/05 角速度饱和率高（7%/14%）→ K1 项占满 max_w，K2 阻尼被截幅
- path_02/03 中等，seg3 略好于 seg1（03 是这样）→ 主要是中段振荡
```

### 第二层：path_XX_metrics.csv + deep_analysis — 确认故障模式（5 分钟）

聚焦最差路径，查看：

```
步骤 1：mean_cancel_ratio_logged 和 severe_cancel_ratio
         > 0.35 / > 0.10 → K2 频繁被压制，阻尼结构有问题

步骤 2：w_saturation_ratio
         > 0.10 → 控制量受物理限制，需减小 K1 或放大 max_w

步骤 3：计算实际阻尼比
         ζ = k2_effective / (2 × √(v × k1))
         用 mean k1, mean k2, v=work_max 计算
         ζ < 0.5 → 欠阻尼，振荡根因

步骤 4：longest_breach_5_samples × 采样间隔(~0.055s) = 连续超差持续时间
         > 2s → 系统有持续无法收敛的段落
```

### 第三层：path_XX_samples.csv — 定位根本原因（10-15 分钟）

这一层需要对 CSV 进行可视化或统计分析。

#### 3.1 找到问题时间段

```python
# 伪代码：找连续超差段
import pandas as pd
df = pd.read_csv('path_04_samples.csv')
df['breach5'] = df['cross_track_mm'].abs() > 5
# 找到最长连续超差起点
breach_start = df[df['breach5'] & ~df['breach5'].shift(1).fillna(False)]['t_s']
```

**具体操作**：从 `longest_breach_5_t_start` 前后各取 3 秒的样本区间，重点分析。

#### 3.2 在问题时间段内检查以下关系

**诊断 A：确认是漂移还是振荡**
```
漂移特征：
  - cross_track_mm 在多帧内单调变化（同向增大）
  - e_y_filtered_mm 和 cross_track_mm 同向增大
  - omega_fb 应当对抗误差，但幅度不足
  - integral_state 缓慢积累

振荡特征：
  - cross_track_mm 正负交替，周期约 0.5-2s
  - e_theta_rad 与 cross_track_mm 反号（正在回摆）
  - omega_before_limits >> angular_cmd（被截幅）
  - cancel_ratio 高（K1 和 K2 互相抵消）
```

**诊断 B：确认 max_w 是否是瓶颈**
```python
# 检查截幅频率
saturation_frames = (df['omega_before_limits'].abs() > df['phase_max_w'] * 0.95).mean()
# > 0.10 即频繁截幅
```

当 `|omega_before_limits| > phase_max_w × 0.95` 时：
- 截幅发生，K2 项被物理截断，阻尼实际消失
- 此时增大 K1 只会更糟（加剧截幅）

**诊断 C：确认 K2 阻尼是否实际有效**
```python
# 计算每帧的有效阻尼比
df['zeta_eff'] = df['k2'] / (2 * (df['linear_speed_mps'] * df['k1']).pow(0.5))
# ζ < 0.5 的帧占比
underdamped_ratio = (df['zeta_eff'] < 0.5).mean()
```

**诊断 D：地板缝扰动的量化**
```python
# e_theta 滤波前后差异
df['etheta_noise'] = (df['e_theta_raw_rad'] - df['e_theta_rad']).abs()
# 突发幅度 > 0.02 rad 的帧
joint_frames = (df['etheta_noise'] > 0.02).mean()
```

**诊断 E：积分项是否正常工作**
```python
# 积分应在 start_aligned=1 后缓慢积累
following = df[df['start_aligned'] == 1]
# 检查 integral_state 是否单调漂移或反复清零
```

---

## 4. 诊断决策树：症状→根因→参数

### 症状一：后段横向误差持续增大（drift_tail_minus_head > 3mm，同号）

```
确认：
  └─ omega_before_limits ≈ angular_cmd？
       ├─ 否（omega_before_limits 常超 phase_max_w）
       │    → 根因：K1 过大，K1 项吃满 max_w，K2 阻尼被截幅
       │    → 对策：减小 K1（降 q1 或 K1_direct）或增大 phase.following.max_angular_vel
       │
       └─ 是（未饱和）
            └─ integral_state 趋势是否与误差同向？
                 ├─ 否（积分很小或反向）
                 │    → 根因：积分增益不足，无法消除侧坡稳态偏差
                 │    → 对策：增大 Ki，或确认 enable_integral=true
                 │
                 └─ 是（积分方向正确但误差仍增大）
                      → 根因：K1 太小，比例纠偏力不够
                      → 对策：增大 q1（动态模式）或 K1_direct
```

### 症状二：横向误差来回振荡（drift ≈ 0，但 p90 高）

```
确认：
  └─ cancel_ratio 是否常 > 0.35？
       ├─ 是
       │    └─ k2_floor_active 是否为 0？
       │         ├─ 是（k2_floor 不起作用）
       │         │    → 根因：k2_gate+anti_cancel 叠乘后 K2_eff 过小，阻尼消失
       │         │    → 对策：增大 k2_min_floor_scale 或放宽 k2_gate.min_scale
       │         │
       │         └─ 否（k2_floor 长期生效）
       │              → 根因：K2 本身不够大，floor 值也太低
       │              → 对策：增大 K2_direct 或调整 q2（提高 K2）
       │
       └─ 否（cancel_ratio 正常）
            └─ e_theta_raw 与 e_theta_rad 差异是否大？
                 ├─ 是（差值 > 0.02 rad 频发）
                 │    → 根因：地板缝产生 e_theta 高频扰动，alpha 不够小
                 │    → 对策：减小 e_theta_lowpass.alpha（加强滤波）
                 │
                 └─ 否
                      → 根因：ζ < 0.5（系统固有欠阻尼）
                      → 对策：减小 K1 + 增大 K2，使 ζ→0.65
```

### 症状三：起步对齐过慢（start_aligned=0 持续时间长）

```
检查：phase.alignment.line_cross_track_tolerance 和 line_heading_tolerance
      以及 phase.alignment.max_angular_vel

如果对齐阶段 cross_track_mm 一直在 6-8mm 附近震荡不收敛：
  → 根因：对齐阶段 max_angular_vel 不足，无法快速调向
  → 对策：增大 phase.alignment.max_angular_vel（当前 0.054）

如果对齐阶段误差已经 < 6mm 但 start_aligned 仍不翻转：
  → 根因：line_cross_track_tolerance 阈值太严
  → 对策：放宽 line_cross_track_tolerance（当前 0.006m）
```

### 症状四：开始跟随后立即出现大误差脉冲

```
确认：
  └─ 对齐→跟随切换时 integral_state 是否异常大？
       ├─ 是（> 0.004 m·s）
       │    → 根因：积分 windup（对齐阶段积累了偏差）
       │    → 验证：代码应有 start_line_aligned_ guard，检查是否生效
       │
       └─ 否
            └─ 切换后 k1 是否突变（动态模式）？
                 ├─ 是（对齐阶段低速 K1 高，跟随加速后 K1 骤降）
                 │    → 根因：K1 平滑器（k1_blend=0.35）跟随太慢
                 │    → 对策：可接受，但可适当增大 k1_blend 加快跟随
                 │
                 └─ 否
                      → 根因：前瞻距离切换导致参考点跳变
                      → 对策：检查 distance.lookahead 是否合理
```

### 症状五：长路径（>6m）后段特别差

```
确认：
  └─ tail_schedule 是否生效（CSV tail_schedule > 0）？
       ├─ 否（tail_schedule 全为 0）
       │    → 根因：path_length < 4m 未触发 tail_schedule
       │    → 对策：正常现象（短路径不需要）
       │
       └─ 是（tail_schedule > 0）
            └─ k2_floor_active 在后段是否长期为 1？
                 ├─ 是
                 │    → 良好：floor 在保护阻尼
                 │    → 但如果精度仍差：floor 值本身可能不够高
                 │
                 └─ 否（floor 未触发，k2 正常）
                      → 检查 drift_tail_minus_head_mm 方向
                      → 单向漂移 → 积分不足（增大 Ki 或 integral_max）
                      → 双向振荡 → ζ 不足（见症状二）
```

---

## 5. 参数调优规则手册

### 5.1 核心增益参数（最关键）

#### `q1`（动态模式）或 `K1_direct`（静态模式）
- **物理含义**：横向误差的纠偏增益。K1 越大，机器人对横向偏差的反应越激进。
- **影响**：`K1 = √(q1/r) / v`（动态模式），K1 变大 → ω_n 变大 → 收敛更快，**但 ζ 降低**
- **与 max_w 的约束**：`K1 × max_e_y_expected < phase_max_w × 0.6`（留 40% 给 K2 阻尼）
  - 当前 max_w=0.066，max_e_y≈10mm：K1 应 < 0.066×0.6/0.010 = **3.96**
  - 超过此值时 max_w 会截断 K2 项，阻尼消失
- **调整步长**：q1 每次调整 ±0.05（对应 K1 在 v=0.25 时变化 ±0.10）
- **调大时机**：后段单向漂移（积分也补不上），小误差段收敛太慢
- **调小时机**：w_saturation_ratio > 0.10，振荡发散

#### `q2`（动态模式）或 `K2_direct`（静态模式）
- **物理含义**：航向误差的阻尼增益。K2 越大，系统阻尼越强，超调越少。
- **影响**：`K2 = √(2√(q1q2) + q2)`（动态模式），K2 变大 → ζ 增大 → 更稳定
- **最优 K2**：`K2_opt = 2×ζ_target×√(v×K1)`，ζ_target=0.65
  - 当前配置：v=0.25, K1=2.50 → K2_opt = 2×0.65×√0.625 = 1.03 ✓
- **调整步长**：q2 每次调整 ±0.05（K2 变化约 ±0.05）
- **调大时机**：振荡（cancel_ratio 高，ζ < 0.5）
- **调小时机**：响应迟缓（过阻尼），误差收敛太慢

#### ζ（阻尼比）的计算与目标
```
ζ = K2_effective / (2 × √(v × K1))

目标：ζ ∈ [0.60, 0.80]
  ζ < 0.5  → 欠阻尼，振荡（历史问题根因）
  ζ ≈ 0.65 → 最优（快速且无超调）
  ζ > 1.0  → 过阻尼，响应迟缓

每次调参后必须重新计算 ζ，确认方向正确！
```

### 5.2 K1/K2 调制机制

#### `k2_gate`（横向误差门控）
- **正确理解**：误差 > ey_start 时逐渐减小 K2，**初衷是避免大误差段 K2 项干扰收敛**
- **实际问题**：误差 > ey_start 时 e_y/e_theta 异号是**正常阻尼**，不应被压制
- **调整原则**：
  - `ey_start` 应 ≥ 2×目标精度，当前 4.8mm 合理
  - `min_scale` 不能太小，当前 0.50；最低不建议 < 0.40
  - `ey_end` 设为 2~3×ey_start 合理，当前 12mm
- **何时调整 ey_start**：
  - 调大：振荡频繁发生在 3-6mm 区间 → 扩大保护区
  - 调小：大误差（>8mm）段完全没有收敛趋势 → 允许更早开始 K2 衰减

#### `k2_anti_cancel`（抵消抑制）
- **正确理解**：仅在 K1 项和 K2 项真正大幅抵消（cancel_ratio > kCancelStart）时才介入
- **危险操作**：scale 调得过小会剥夺所有阻尼
- **调整原则**：
  - `scale` 不应 < 0.35，当前 0.45 是安全值
  - `ey_threshold` 是触发最低 e_y，低于此值时即使异号也不介入（保护精细段）
- **何时禁用**：振荡问题主要来自地板缝（e_theta 扰动），而非真实抵消

#### `k2_min_floor_scale`（K2 绝对下限）
- **作用**：无论 gate + anti_cancel 叠乘多厉害，K2_eff 至少保留此比例
- **计算**：`K2_eff_min = K2_direct(或动态K2) × k2_min_floor_scale`
- **调整**：
  - 如果 k2_floor_active 长期为 1 且精度仍差 → floor 本身太低，增大此值
  - 当前 0.45 → K2_eff_min = 1.00×0.45 = 0.45 rad/s / (rad/s) 的有效 K2

### 5.3 积分参数

#### `Ki`（积分增益）
- **作用**：消除侧坡等产生的系统性稳态偏差
- **必要性**：`稳态误差 = (侧坡加速度) / (v × K1)`，K1=2.5, v=0.25 时 0.3°侧坡 → 36mm 稳态误差
- **调整步长**：每次 ±0.05
- **调大时机**：`drift_tail_minus_head > 2mm` 且 `integral_state` 未饱和
- **调小时机**：`integral_state` 频繁到达 `integral_max`（windup 风险）
- **windup 检查**：`integral_state / integral_max > 0.8` 超过 30% 的时间 → 减小 Ki

#### `integral_max`（积分上限）
- **含义**：积分器蓄积量上限，实际输出限幅 = Ki × integral_max
- **调整**：使最大积分输出 ≈ 0.5 × phase_max_w
  - 当前：Ki=0.38, integral_max=0.010 → 最大积分输出 0.0038 rad/s（很保守）
  - 若侧坡大：可增至 0.015~0.020

#### `integral_decay`（积分衰减）
- **含义**：每帧乘以此系数，防止历史偏差无限积累
- **时间常数**：τ = -dt / ln(decay)，decay=0.992, dt=0.055s → τ ≈ 6.8s
- **调小时机**：路径切换后积分历史引起初始抖动

### 5.4 e_theta 低通滤波

#### `e_theta_lowpass.alpha`
- **直觉**：alpha=1.0 = 完全不滤波；alpha=0 = 永远为 0（完全阻断）
- **地板缝特征频率**：约 10-20Hz（机器人通过一块地板约 50-100ms）
- **控制律响应频率**：约 1-3Hz（K2·e_theta 的有效带宽）
- **选择原则**：衰减 >10Hz 成分，保留 <3Hz 成分

| alpha | 截止频率（近似） | 50ms 脉冲衰减 | 建议场景 |
|-------|---------------|-------------|---------|
| 0.95 | ~3Hz | 1.5× | 平整地面 |
| 0.88 | ~2Hz | 2.5× | 轻微不平整 |
| **0.82** | **~1.5Hz** | **4×** | **标准不平整（当前）** |
| 0.72 | ~1Hz | 8× | 严重凹凸 |
| 0.60 | ~0.7Hz | 15× | 极端场景（影响正常跟线）|

**诊断方法**：
```python
# 检查 e_theta_raw 与 e_theta 的差
df['etheta_filter_effect'] = df['e_theta_raw_rad'] - df['e_theta_rad']
# 如果差值的 std > 0.015 rad → 地板缝扰动显著，alpha 可以再小
# 如果差值 std < 0.005 rad → 滤波强度已足够
```

### 5.5 e_y 输入滤波

#### `ey_filter.rate_limit.rate_factor`
- **作用**：限制 e_y 帧间变化速率，防止 Hampel 滤波器回退引起的阶跃污染 LQR 平滑器
- **计算**：`max_change = max(noise_floor, v × dt × rate_factor)`
- **调小时机**：e_y_raw 与 e_y_rate_limited 差值频繁（Hampel 跳变多）
- **调大时机**：e_y_rate_limited 跟不上真实运动（合法的快速偏移被限制）

#### `ey_filter.lowpass.alpha`
- **当前 0.87**：对每帧噪声约 7.5dB 衰减
- **调小时机**：e_y 信号噪声明显（e_y_rate_limited 高频抖动）
- **调大时机**：e_y_filtered 滞后于 e_y_rate_limited 过多（精细段响应慢）

### 5.6 物理限幅参数

#### `phase.following.max_angular_vel`（max_w）
- **当前 0.066 rad/s**：这是整个 LQR 性能的最大硬约束
- **重要性**：max_w 决定了 K1 的上限。`K1_max_safe = max_w × 0.6 / e_y_expected`
  - e_y_expected=10mm：K1 < 0.066×0.6/0.010 = **3.96**
- **调整注意**：增大 max_w 必须经过底盘稳定性验证，不能随意提高

#### `phase.following.max_angular_accel`
- **当前 5.2 rad/s²**：限制角速度的变化速率
- **影响**：过小会导致 LQR 输出被平滑掉，响应变迟缓
- **调整**：通常不需要调，除非角速度指令有明显锯齿

### 5.7 前瞻参数

#### `distance.lookahead` 和 `distance.lookahead_time`
- **总前瞻距离** = lookahead + lookahead_time × speed
  - 当前：0.158 + 0.014 × 0.25 = 0.1615m
- **前瞻距离的影响**：
  - 太小 → 参考点紧贴最近点 → 对路径噪声敏感，容易振荡
  - 太大 → 参考点超前过远 → e_y 计算偏差大，弯道处误差增大
- **调整**：一般不轻易动，若振荡与前瞻有关，先尝试 ±10% 的调整

---

## 6. 正确的实验设计

### 6.1 单变量原则（最重要）

**每次只改一个参数，观察一个指标。**

错误示例（历史上的问题）：
```
一次性同时修改：K1↑ + K2↓ + tail_schedule + ey_gate
结果：不知道是哪个变化带来了改善或恶化
```

正确流程：
```
1. 记录 baseline 的 6 项指标
2. 只修改 q1（K1），运行一个完整批次
3. 对比 6 项指标，确认改善/恶化
4. 若改善 → 记录为新 baseline，继续下一步
   若恶化 → 回滚，换方向或换参数
```

### 6.2 优化顺序（从根因到症状）

```
Step 1：确认 K1/K2 阻尼比 ζ ≥ 0.60
         └─ 计算：ζ = K2_eff / (2×√(v×K1))
         └─ 如果不满足，先调 q1/q2 达到目标 ζ

Step 2：确认 max_w 不成为瓶颈
         └─ w_saturation_ratio < 0.05
         └─ 如果频繁饱和，减小 K1 优先于增大 max_w

Step 3：确认积分项正常补偿系统性漂移
         └─ drift_tail_minus_head < 1.5mm（有积分时）
         └─ 如果漂移仍大，调 Ki

Step 4：处理地板缝高频扰动
         └─ 检查 e_theta_raw vs e_theta_rad 差异
         └─ 调整 e_theta_lowpass.alpha

Step 5：微调 k2_gate/k2_floor/k2_anti_cancel
         └─ 最后才动这些"补丁"参数
         └─ 此时 K1/K2/ζ 已正确，这些参数只做边界保护
```

### 6.3 每次实验的记录模板

```markdown
## 实验记录 yyyy-mm-dd / bNN

**变更**：
- 参数 X：旧值 → 新值
- 理由：...

**预期效果**：
- 哪条路径的哪个指标应该改善
- 改善的理论依据

**实验结果**（运行完整6路径后填写）：
| 路径 | p50_mm | p90_mm | <3mm% | drift_mm | saturation% |
|------|--------|--------|-------|----------|------------|
| 01   |        |        |       |          |            |
| 04   |        |        |       |          |            |
| 05   |        |        |       |          |            |

**结论**：
- 改善 / 恶化 / 无明显变化
- 原因分析（结合 samples 数据）
- 下一步行动
```

### 6.4 A/B 对比的正确方法

**绝对不能**：用不同环境、不同地面状态的数据做对比。

**正确对比**：同一批次连续运行，或者同地面立即重复。

**关注的是相对变化**，而不是绝对数值：
```
p90: 10.2mm → 8.5mm = -17% → 有效改善
p90: 10.2mm → 10.0mm = -2% → 在噪声范围内，无法判断
```

判断显著性的经验阈值：
- 单条路径 p90 改善 > 1mm → 可能有效
- 多条路径 p90 平均改善 > 0.8mm → 有效
- `<3mm%` 改善 > 3% → 有效

### 6.5 回滚策略

每次实验前备份 line.yaml：
```bash
cp config/line.yaml config/line_bNN_backup.yaml
```

使用 `optimization.id` 字段追踪每个版本，确保 batch_metrics.csv 中每行都有唯一 opt_id。

---

## 7. 历史错误模式与教训

### 错误一：把 LQR 阻尼误判为"反馈抵消"（30 轮优化的根本错误）

**现象**：观察到 `cancel_ratio` 高，认为 K1 项和 K2 项在"抵消"，于是降低 K2。

**错误根因**：
- `cancel_ratio` 高（e_y 与 e_theta 异号）发生在机器人向路径靠近的过程中
- 此时 K2 减小转向需求是**正确的阻尼行为**，不是干扰
- 删除 K2 等于删除阻尼 → 超调 → 超调后 cancel_ratio 更高 → 继续删 K2 → 恶性循环

**正确理解**：
```
cancel_ratio 高 = 阻尼机制正在工作
→ 应该检查 ζ 是否合理，而不是压制 K2
```

**教训**：在调参前，先计算当前阻尼比 ζ。ζ < 0.5 时不能通过降 K2 来解决问题。

### 错误二：单调提高 K1 试图提高精度

**历史路径**：
```
K1: 3.2 → 4.1 → 5.0 → 5.7（30轮优化）
K2: 0.95 → 0.90 → 0.87 → 0.84
ζ:  0.44 → 0.39 → 0.38 → 0.35（单调恶化）
```

**错误根因**：K1 增大 → ω_n 增大 → 为维持 ζ 需要同步增大 K2 → 但 K2 反而被降低 → ζ 单调下降。

**教训**：提高 K1 必须同时提高 K2，公式：`K2_new = K2_old × √(K1_new/K1_old)`

### 错误三：用多层机制叠乘压制 K2

**历史代码**：tail_schedule(×0.65) × k2_gate(×0.28) × anti_cancel(×0.23) = 0.042

**效果**：K2_eff = 0.84 × 0.042 = 0.035，ζ ≈ 0.015（近无阻尼）

**教训**：
- 每新增一层 K2 抑制机制，必须计算最坏情况叠乘值
- `K2_eff_min = K2 × 所有机制最小值乘积 ≥ K2 × 0.35`（经验下限）
- k2_min_floor_scale 是这一教训的直接产物

### 错误四：未区分不同路径的问题类型

**历史错误**：path_01（短路径）表现好，就认为配置已经足够好，忽略 path_04/05（长路径）恶化。

**教训**：
- 短路径（<2m）本质上不需要积分，因为误差没时间积累
- 长路径（>7m）是真正的考验：需要积分补偿侧坡，需要后段 K1 增强
- 优化目标必须明确：**6 路径全部 p90 < 5mm**，任何一条失败都需要改进

### 错误五：参数调整方向与实际效果相反

**场景**：path_04 后段误差大，尝试增大 K1（认为纠偏力不够）。

**实际效果**：K1 增大 → max_w 饱和率从 7% 升至 20% → K2 阻尼完全被截幅 → 误差反而更大。

**教训**：在调 K1 之前，先检查 `w_saturation_ratio`。若已 > 5%，增大 K1 只会更差。

---

## 8. 快速参考：数值范围与调整步长

### 动态增益模式（当前推荐）

| 参数 | 当前值 | 安全范围 | 步长 | 效果方向 |
|------|--------|---------|------|---------|
| `q1` | 0.39 | 0.20~0.60 | ±0.05 | ↑更快收敛但更易振荡 |
| `q2` | 0.31 | 0.15~0.50 | ±0.05 | ↑阻尼更强但响应更慢 |
| `K1_max` | 8.0 | 5.0~12.0 | ±1.0 | ↑允许低速更激进纠偏 |
| `K1_min` | 1.5 | 1.0~3.0 | ±0.3 | ↑高速段也保留基础纠偏 |

### K2 调制参数

| 参数 | 当前值 | 安全范围 | 步长 | 效果方向 |
|------|--------|---------|------|---------|
| `k2_gate.ey_start` | 0.0048 | 0.003~0.008 | ±0.001 | ↑扩大精细段保护区 |
| `k2_gate.ey_end` | 0.0120 | 0.008~0.020 | ±0.002 | ↑扩大门控过渡区 |
| `k2_gate.min_scale` | 0.50 | 0.35~0.70 | ±0.05 | ↑大误差段 K2 保留更多 |
| `k2_anti_cancel.scale` | 0.45 | 0.30~0.65 | ±0.05 | ↑减弱抵消抑制效果 |
| `k2_anti_cancel.ey_threshold` | 0.0020 | 0.001~0.004 | ±0.0005 | ↑更大误差时才触发 |
| `k2_min_floor_scale` | 0.45 | 0.30~0.60 | ±0.05 | ↑K2 绝对下限更高 |

### 积分参数

| 参数 | 当前值 | 安全范围 | 步长 | 效果方向 |
|------|--------|---------|------|---------|
| `Ki` | 0.38 | 0.10~0.60 | ±0.05 | ↑积分补偿更强，windup 风险更高 |
| `integral_max` | 0.010 | 0.005~0.025 | ±0.003 | ↑允许更大积分蓄积 |
| `integral_decay` | 0.992 | 0.985~0.998 | ±0.002 | ↑积分保持时间更长 |

### 滤波参数

| 参数 | 当前值 | 安全范围 | 步长 | 效果方向 |
|------|--------|---------|------|---------|
| `e_theta_lowpass.alpha` | 0.82 | 0.60~0.95 | ±0.03 | ↑延迟更小但滤波更弱 |
| `ey_filter.lowpass.alpha` | 0.87 | 0.75~0.95 | ±0.03 | ↑延迟更小但噪声更多 |
| `ey_filter.rate_limit.rate_factor` | 0.16 | 0.08~0.30 | ±0.02 | ↑允许更快的 e_y 变化 |

### 物理限幅参数

| 参数 | 当前值 | 安全范围 | 步长 | 效果方向 |
|------|--------|---------|------|---------|
| `phase.following.max_angular_vel` | 0.066 | 0.05~0.10 | ±0.005 | ↑允许更大角速度 |
| `phase.following.max_angular_accel` | 5.2 | 3.0~8.0 | ±0.5 | ↑角速度变化更快 |
| `phase.alignment.max_angular_vel` | 0.054 | 0.04~0.08 | ±0.005 | ↑对齐阶段调向更快 |

### 阻尼比速查表

当前配置（动态模式，K2≈1.00）：

| 速度 v | K1（理论） | K1（实际） | ζ |
|--------|---------|----------|---|
| 0.25 | 2.50 | 2.50 | **0.65** ← 目标 |
| 0.20 | 3.12 | 3.12 | 0.63 |
| 0.15 | 4.16 | 4.16 | 0.63 |
| 0.10 | 6.25 | 6.25 | 0.63 |
| 0.07 | 8.93 | **8.0**（截幅） | 0.66 |
| 0.05 | 12.5 | **8.0**（截幅） | 0.79 |

---

## 附录：常用诊断命令

```python
import pandas as pd
import numpy as np

df = pd.read_csv('line_tracking_latest/path_04_samples.csv')

# 1. 基本统计
print(df['cross_track_mm'].abs().describe())

# 2. 阻尼比分析
df['zeta'] = df['k2'] / (2 * np.sqrt(df['linear_speed_mps'] * df['k1']))
print(f"ζ 均值: {df['zeta'].mean():.3f}, <0.5 占比: {(df['zeta']<0.5).mean():.1%}")

# 3. max_w 饱和分析
df['saturated'] = df['omega_before_limits'].abs() > df['phase_max_w'] * 0.95
print(f"角速度饱和率: {df['saturated'].mean():.1%}")

# 4. 地板缝扰动分析（需要 b32+ 数据）
if 'e_theta_raw_rad' in df.columns:
    df['etheta_noise'] = (df['e_theta_raw_rad'] - df['e_theta_rad']).abs()
    print(f"e_theta 扰动 std: {df['etheta_noise'].std():.4f} rad")
    print(f"扰动 >0.02rad 帧: {(df['etheta_noise']>0.02).mean():.1%}")

# 5. 积分状态分析
following = df[df['start_aligned'] == 1.0]
if 'integral_state' in following.columns:
    print(f"积分均值: {following['integral_state'].mean():.5f} m·s")
    print(f"积分 std: {following['integral_state'].std():.5f} m·s")

# 6. 漂移分析（前后半段对比）
mid = len(df) // 2
head_mean = df.iloc[:mid]['cross_track_mm'].mean()
tail_mean = df.iloc[mid:]['cross_track_mm'].mean()
print(f"头段均值: {head_mean:.2f}mm, 尾段均值: {tail_mean:.2f}mm, 漂移: {tail_mean-head_mean:.2f}mm")

# 7. cancel_ratio 分析
print(f"cancel_ratio 均值: {df['cancel_ratio'].mean():.3f}")
print(f"cancel_ratio >0.5 占比: {(df['cancel_ratio']>0.5).mean():.1%}")
```
