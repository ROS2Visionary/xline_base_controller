# Line Optimization History

该文件记录每轮针对 `line_tracking_latest` 的分析结论与参数变更，配合
`line_tracking_latest/batch_metrics.csv` 的结构化字段（如 `optimization_id`、
`parent_batch_id`、`change_note`）形成闭环优化链路。

## 2026-02-28 / opt_20260228_b3_phase_unfreeze_v1
- Parent Batch: `1772304426886`
- Problem Summary:
  - 仅 `2/6` 路径满足 `P90 < 5mm`。
  - 失败路径出现长时间超差与角速度限幅利用不足并存，且对齐阶段限制可能滞留过久。
- Changes:
  - 提升阶段角速度能力：`phase.alignment.max_angular_vel/max_angular_accel`，
    `phase.following.max_angular_vel/max_angular_accel`。
  - 放宽起步对齐判据：`line_cross_track_tolerance`、`line_heading_tolerance`、
    `line_alignment_stable_count`。
  - 缩短前瞻：`distance.lookahead/lookahead_time`。
  - 回撤 K2 抑制强度：`k2_gate.{ey_start,ey_end,min_scale}`，并将
    `K2_direct` 调整为 `1.05`。
  - 记录结构升级：
    - 新增 `optimization.*` 配置并写入 metrics/batch 导出；
    - samples 新增 `start_aligned` 与 `phase_max_w` 字段。

## 2026-02-28 / opt_20260228_b4_cancel_guard_v2
- Parent Batch: `1772304426886`
- Deep Analysis (from `line_tracking_latest`):
  - `pass=1/6`，`path_02/03/04/05/06` 失败。
  - `path_06` 为持续偏差型：末段 `>5mm` 占比极高（长连续段）。
  - `path_02/04/05` 为峰值爆发型：中后段有明显尖峰且伴随角速度接近限幅。
  - 反馈项抵消信号明显：多条路径出现 `e_y` 与 `e_theta` 异号导致 `-K1*e_y` 与 `-K2*e_theta` 相互削弱。
- Changes:
  - 缩短前瞻：`distance.lookahead 0.16 -> 0.15`，`lookahead_time 0.02 -> 0.015`。
  - 提升角速度上限能力（不降线速度）：`phase.alignment/following.max_angular_vel 0.040 -> 0.042`，
    `max_angular_accel 2.8/2.4 -> 3.0/2.6`。
  - 下调 `K2_direct 1.05 -> 0.92`，减小反馈抵消风险。
  - 强化 `k2_gate`：`ey_start 0.0040 -> 0.0035`，`ey_end 0.0120 -> 0.0105`，`min_scale 0.45 -> 0.32`。
  - 重新启用 `k2_anti_cancel`：`enabled=true`，`scale=0.72`，`ey_threshold=0.0035`。

## 2026-02-28 / opt_20260228_b5_ratio_first_v1
- Parent Batch: `1772305246100`
- Deep Analysis (ratio-first + anti-spike):
  - 本轮 `pass=1/6`，且 `path_06` 末段存在持续超差（非单点突变）。
  - 目标重排为：先提升 `<3mm` 稳态覆盖，再提升 `<5mm` 稳态覆盖。
  - 使用“连续3采样点越界”规则过滤瞬时突变，避免误判。
- Changes:
  - 回收前瞻过短导致的整体漂移风险：`lookahead 0.15 -> 0.17`，
    `lookahead_time 0.015 -> 0.02`。
  - 提升横向纠偏主项：`K1_direct 4.8 -> 5.1`。
  - 重新平衡航向项：`K2_direct 0.92 -> 1.00`。
  - `k2_gate` 由“强抑制”回调为“中等抑制”：
    `ey_start 0.0035 -> 0.0042`，`ey_end 0.0105 -> 0.0130`，`min_scale 0.32 -> 0.45`。
  - `k2_anti_cancel` 继续启用且更精准触发：
    `scale 0.72 -> 0.58`，`ey_threshold 0.0035 -> 0.0028`。
  - e_y 抗突变滤波微调：`rate_factor 0.22 -> 0.18`，
    `lowpass alpha 0.90 -> 0.88`。
  - 数据结构升级（用于下一轮闭环优化）：
    - metrics/batch 新增 `ratio_lt_3mm`、`ratio_lt_5mm`、
      `stable_ratio_lt_3mm`、`stable_ratio_lt_5mm`、
      `longest_breach_3_samples`、`longest_breach_5_samples`。
    - 深度分析脚本新增上述字段及“`<3` 优先、`<5` 次优先”路径排序输出。

## 2026-02-28 / opt_20260228_b6_cancel_ratio_v1
- Parent Batch: `1772306202184`
- Deep Analysis (ratio-first + anti-spike):
  - `pass=2/6`，`path_01/05` 达标，`path_04` 接近达标（`p90=5.18mm`）。
  - 主问题集中在 `path_02/03/06`：长连续超差，且 `e_y>0` 与 `e_theta<0`
    导致反馈项抵消（非单点突变）。
  - 优化顺序继续执行：先拉升 `<3mm` 稳态覆盖，再拉升 `<5mm` 稳态覆盖。
- Changes:
  - 代码级精准优化：`k2_anti_cancel` 从“固定比例抑制”改为
    “按抵消比例动态抑制”（仅在抵消强时增强抑制）。
  - samples 数据结构升级：新增 `cancel_ratio` 字段，便于下轮直接按抵消强度分层调参。
  - 参数微调（小步）：
    - 前瞻轻微收紧：`lookahead 0.17 -> 0.165`，
      `lookahead_time 0.02 -> 0.018`。
    - `K2_direct 1.00 -> 0.98`。
    - `k2_gate`: `ey_start 0.0042 -> 0.0040`，`ey_end 0.0130 -> 0.0125`，
      `min_scale 0.45 -> 0.42`。
  - `k2_anti_cancel`: `scale 0.58 -> 0.45`，
    `ey_threshold 0.0028 -> 0.0025`。

## 2026-03-03 / opt_20260303_b7_ratio_recovery_v1
- Parent Batch: `1772555168435`
- Deep Analysis (ratio-first + anti-spike):
  - 本轮 `pass=2/6`，`path_01/03` 达标；`path_04/05/06` 及 `path_02` 未达标。
  - 以“连续3采样点越界”去除突变误判后，仍存在长连续超差：
    - `path_05`: `longest>5mm=63`（末段最差，`tail_stable<5=20.3%`）
    - `path_04`: `longest>5mm=40`
    - `path_06`: `longest>5mm=41`
    - `path_02`: `longest>5mm=35`
  - 样本级显示 `e_y` 与 `e_theta` 长时间异号，`cancel_ratio` 偏高（约 `0.34~0.49`），
    说明仍有明显反馈项抵消。
- Changes:
  - 目标顺序保持：先提升 `<3mm` 稳态覆盖，再提升 `<5mm` 稳态覆盖。
  - 提升横向主纠偏并继续压制抵消：
    - `K1_direct 5.1 -> 5.3`
    - `K2_direct 0.98 -> 0.90`
    - `k2_gate`: `ey_start 0.0040 -> 0.0038`，
      `ey_end 0.0125 -> 0.0115`，`min_scale 0.42 -> 0.36`
    - `k2_anti_cancel`: `scale 0.45 -> 0.35`，
      `ey_threshold 0.0025 -> 0.0022`
  - 轻微提升角速度能力（不降线速度）：
    - `phase.alignment.max_angular_vel/max_angular_accel: 0.042/3.0 -> 0.044/3.2`
    - `phase.following.max_angular_vel/max_angular_accel: 0.042/2.6 -> 0.044/2.8`
  - 略收紧前瞻与抗突变滤波：
    - `lookahead/lookahead_time: 0.165/0.018 -> 0.160/0.016`
    - `ey_filter.rate_factor: 0.18 -> 0.16`
    - `ey_filter.lowpass.alpha: 0.88 -> 0.87`

## 2026-03-03 / opt_20260303_b8_tail_drift_guard_v1
- Parent Batch: `1772556127438`
- Deep Analysis (ratio-first + anti-spike + tail drift):
  - 本轮 `pass=3/6`，整体提升，但仍出现“前段好、后段下降”：
    - `path_02`: `seg1 mean=1.998mm -> seg3 mean=6.306mm`，`tail_stable<5=20.3%`
    - `path_04`: `seg1 mean=1.131mm -> seg3 mean=5.473mm`，`tail_stable<5=37.5%`
    - `path_06`: `seg1 mean=0.672mm -> seg3 mean=6.111mm`，`tail_stable<5=27.9%`
  - 抗突变规则（连续3采样点越界）下仍有长连续超差，说明是持续漂移而非单点噪声。
  - `cancel_ratio` 仍偏高（约 `0.28~0.39`），抵消是后段劣化主因之一。
- Changes:
  - 数据结构增强（用于下一轮精准闭环）：
    - metrics/batch 新增分段字段：
      `seg1_ratio_lt_3mm`、`seg1_ratio_lt_5mm`、
      `seg3_ratio_lt_3mm`、`seg3_ratio_lt_5mm`、
      `seg3_stable_ratio_lt_5mm`、`drift_tail_minus_head_mm`。
    - 直接量化“后段退化”趋势，避免只看全局 P90 导致误判。
  - 参数继续按“先 `<3` 再 `<5`”小步回调：
    - `K1_direct 5.3 -> 5.6`
    - `K2_direct 0.90 -> 0.86`
    - `k2_gate`: `ey_start 0.0038 -> 0.0035`，
      `ey_end 0.0115 -> 0.0108`，`min_scale 0.36 -> 0.30`
    - `k2_anti_cancel`: `scale 0.35 -> 0.28`，
      `ey_threshold 0.0022 -> 0.0020`
  - 为后段纠偏预留更多角速度能力（不降线速度）：
    - `phase.alignment.max_angular_vel/max_angular_accel: 0.044/3.2 -> 0.046/3.4`
    - `phase.following.max_angular_vel/max_angular_accel: 0.044/2.8 -> 0.046/3.0`
  - 前瞻与 e_y 滤波细调：
    - `lookahead/lookahead_time: 0.160/0.016 -> 0.158/0.014`
    - `ey_filter.rate_factor: 0.16 -> 0.18`
    - `ey_filter.lowpass.alpha: 0.87 -> 0.89`

## 2026-03-03 / opt_20260303_b9_tail_response_v1
- Parent Batch: `1772557514137`
- Deep Analysis (ratio-first + anti-spike + tail drift):
  - 本轮 `pass=3/6`，但后段退化依旧显著：
    - `path_04`: `seg1<5=100% -> seg3<5=16.0%`，`stable_longest>5=48`
    - `path_06`: `seg1<5=100% -> seg3<5=51.5%`，`stable_longest>5=33`
    - `path_01`: `seg1<5=97.1% -> seg3<5=75.0%`
  - 样本级看后段多为低速持续偏差，角速度未饱和，问题更接近“纠偏响应滞后 + 反馈抵消残留”，
    而非物理限幅不足。
  - 抗突变规则（连续3采样点越界）后仍成立，排除单点噪声误判。
- Changes:
  - 目标顺序保持：优先提升 `<3mm` 稳态覆盖，再提升 `<5mm` 稳态覆盖。
  - 降低 e_y 处理滞后，增强后段响应：
    - `ey_filter.rate_factor 0.18 -> 0.15`
    - `ey_filter.lowpass.alpha 0.89 -> 0.86`
  - 小步加强横向主纠偏并继续削弱抵消：
    - `K1_direct 5.6 -> 5.8`
    - `K2_direct 0.86 -> 0.82`
    - `k2_gate`: `ey_start 0.0035 -> 0.0034`，
      `ey_end 0.0108 -> 0.0102`，`min_scale 0.30 -> 0.28`
    - `k2_anti_cancel`: `scale 0.28 -> 0.24`，
      `ey_threshold 0.0020 -> 0.0018`
  - 前瞻仅小步收紧：
    - `lookahead/lookahead_time: 0.158/0.014 -> 0.156/0.013`

## 2026-03-03 / opt_20260303_b10_tail_sat_rebalance_v1
- Parent Batch: `1772557974510`
- Deep Analysis (ratio-first + anti-spike + tail drift):
  - 本轮 `pass=4/6`，整体较上一轮提升，但 `path_04/06` 仍是后段主风险：
    - `path_04`: `seg3<5=16.2%`，`drift_tail_minus_head=+5.516mm`
    - `path_06`: `seg3<5=23.5%`，`drift_tail_minus_head=+5.737mm`
  - 在最长超差段内出现显著角速度饱和（约 `47%~53%`），
    且连续越界长度大（`45~49`），说明主要矛盾已从“纯抵消”转向
    “后段纠偏能力不足 + 上轮参数过激造成稳定性回退”。
  - 抗突变规则（连续3采样点越界）下结论一致，非瞬时噪声误判。
- Changes:
  - 撤回上轮过激增益回调（恢复 b8 稳态基线）：
    - `K1_direct 5.8 -> 5.6`
    - `K2_direct 0.82 -> 0.86`
    - `k2_gate`: `ey_start 0.0034 -> 0.0035`，
      `ey_end 0.0102 -> 0.0108`，`min_scale 0.28 -> 0.30`
    - `k2_anti_cancel`: `scale 0.24 -> 0.28`，
      `ey_threshold 0.0018 -> 0.0020`
  - 提升后段可用角速度能力（不降线速度）：
    - `phase.alignment.max_angular_vel: 0.046 -> 0.048`
    - `phase.following.max_angular_vel/max_angular_accel: 0.046/3.0 -> 0.050/3.4`
  - 恢复 e_y 响应与前瞻到更稳组合：
    - `ey_filter.rate_factor 0.15 -> 0.18`
    - `ey_filter.lowpass.alpha 0.86 -> 0.89`
    - `lookahead/lookahead_time 0.156/0.013 -> 0.158/0.014`
