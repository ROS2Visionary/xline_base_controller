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
