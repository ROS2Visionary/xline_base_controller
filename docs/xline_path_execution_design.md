# XLine 路径执行系统设计方案（xline_path_executor + xline_base_controller）

本文给出基于既有三端（xline_server / xline_path_planner / xline_mobile）的“路径按顺序逐段执行，并在 App 实时高亮反馈”的完整工程化方案。方案采用 ROS2 分层架构：由 xline_path_executor 通过 Action 将计划/段下发给 xline_base_controller 执行，利用 Action 的反馈/取消语义实现暂停、取消和进度回传；底层用 ros2_control 完成底盘轮级闭环。

## 1. 目标与约束
- 目标
  - 将 `planned_cad_2_transformed.json` 的路径按 `order` 顺序逐段下发到底盘并闭环执行。
  - App 端实时展示：当前段高亮、进度、完成/错误状态。
  - 支持暂停、恢复、取消、跳过段，以及断点续跑。
- 约束/前提
  - CAD 坐标单位多为 mm，机器人世界坐标使用 m；需一致的坐标对齐（CAD→map/base_link）。
  - 底盘侧使用 ros2_control（diff_drive_controller + 轮级 PID/硬件闭环）。
  - 若存在 Nav2，可替换控制层；若无，则在控制层自研 RPP + PID 发布 `/cmd_vel`。

## 2. 分层架构与职责

```
┌───────────────┐      ┌──────────────────────┐      ┌──────────────────┐
│ xline_mobile  │◀────▶│ xline_server         │◀────▶│ xline_path_executor│ (ActionClient)
└───────────────┘  WS   │  REST/WS + 状态桥接   │  ROS2 │  解析计划/下发/聚合反馈 │
                        └──────────────────────┘      └─────────▲────────┘
                                                               Action
                                                       ┌─────────┴────────┐
                                                       │ xline_base_controller │ (ActionServer)
                                                       │  RPP + PID → /cmd_vel │
                                                       └─────────▲────────┘
                                                              /cmd_vel
                                                    ┌──────────┴──────────┐
                                                    │   ros2_control       │
                                                    │ diff_drive + 硬件接口 │
                                                    └──────────┬──────────┘
                                                               车体
```

- xline_path_planner：输出规划 JSON（保持直线/圆等几何）。
- xline_path_executor（新包，ActionClient）：解析 JSON→构建 Plan/Segment→下发给 xline_base_controller；聚合 Feedback 上报给 xline_server。
- xline_base_controller（新包/节点，ActionServer）：执行器（RPP + PID），订阅位姿，发布 `/cmd_vel`；终点判据/暂停/取消/跳段处理。
- ros2_control：`diff_drive_controller` 消费 `/cmd_vel`，轮级闭环在硬件/控制器侧。
- xline_server：对外 REST/WS 接口，转发执行状态给 App；也可作为 ActionClient 代理（可选）。

## 3. 数据与坐标
- 单位：JSON 为 mm；ROS 使用 m。xline_path_executor 负责统一转换并写入 `frame_id`（建议 `map`）。
- 对齐：CAD→机器人坐标的仿射/SE(2) 变换（平移 + 旋转 + 尺度若已是 mm→m 则仅尺度 0.001）。参数通过 ROS 参数文件或专用 `alignment.yaml` 注入。
- 段分类：`work=true` 视作 WORK 段；`layer=TRANSITION` 视作 TRANSITION 段（可选执行/灰显）。

## 4. ROS2 接口规范（xline_msgs）

建议新建 `xline_msgs` 包，定义如下接口：

### 4.1 action/ExecutePlan.action
```
# Goal
xline_msgs/Plan plan
bool          dry_run         # 仅仿真/不发 /cmd_vel
float32       speed_scale     # 0..1 运行倍率
---
# Result
bool          success
uint32        completed_count
uint32        error_code
string        message
float32       total_time_s
---
# Feedback
uint32        current_index
string        current_uid
float32       progress        # 0..1 当前段进度
float32       dist_to_end_m
geometry_msgs/Pose2D pose
float32       v_lin
float32       v_ang
```

### 4.2 action/ExecuteSegment.action（可选，单段执行）
```
# Goal
xline_msgs/PathSegment segment
float32       speed_scale
---
# Result/Feedback 同 ExecutePlan 子集
```

### 4.3 msg/Plan.msg
```
string               plan_uid
xline_msgs/PathSegment[] segments
bool                 include_transitions
```

### 4.4 msg/PathSegment.msg
```
uint8  KIND_WORK=0
uint8  KIND_TRANSITION=1
uint8  GEOM_LINE=0
uint8  GEOM_ARC=1
uint8  GEOM_CIRCLE=2

string uid
uint32 order
uint8  kind
uint8  geometry_type
string frame_id           # 通常 map

# LINE
float64 sx
float64 sy
float64 ex
float64 ey

# ARC/CIRCLE
float64 cx
float64 cy
float64 radius
float64 start_rad
float64 end_rad
bool    clockwise

# 约束/期望
float32 v_target          # 目标线速度 m/s
float32 pos_tol_m         # 位置容差 m
float32 yaw_tol_rad       # 航向容差 rad
```

### 4.5 msg/SegmentProgress.msg（可选话题复用 Feedback 结构）
```
string        plan_uid
uint32        current_index
string        current_uid
float32       progress
float32       dist_to_end_m
geometry_msgs/Pose2D pose
float32       v_lin
float32       v_ang
```

### 4.6 srv 接口
```
# Pause.srv / Resume.srv / SkipSegment.srv / SetSpeedScale.srv
---
bool success
string message
```

说明：Action 原生支持 Cancel；Pause/Resume/Skip 建议用 Service（Goal 保持 active），体验更好。

## 5. xline_base_controller（ActionServer）

- 输入：`ExecutePlan`/`ExecuteSegment`、订阅 `/odom` 或 `/tf` 获取位姿。
- 输出：`/cmd_vel`（`geometry_msgs/Twist`）。
- 算法：Regulated Pure Pursuit（RPP）+ 横向/航向误差 PID 调节。
- 状态机：
  - `IDLE → EXECUTING → (PAUSED) → COMPLETED | ABORTED | FAILED`
  - Pause：置 `paused=true` 并发布零速；Resume：置 `paused=false` 继续；Cancel：立即零速并返回 `aborted`。
- 终止条件：`dist_to_end < pos_tol_m` 且 `|yaw_err| < yaw_tol_rad`；进入“就位模式”微调后判完成。
- 速率与限幅：`v ∈ [0, v_max]`、`|ω| ≤ ω_max`；含加速度/跃度限幅；PID 抗积分饱和（`i_clamp`）。
- TRANSITION 段：可配置更大容差/更高速度；也可选择跳过（由 Goal.plan.include_transitions 控制）。

RPP 要点：
- 最近点与前视点：从路径上找最近点，再沿弧长前推至 `lookahead_distance`；动态前视 `Ld = clamp(v * lookahead_time, min, max)`。
- 曲率与角速度：`κ = 2*sin(α)/Ld`，`ω = κ * v`，再叠加 `ω += k_theta*heading_err + k_cte*cross_track_err`。
- 速度调节（regulation）：曲率高/临近目标/障碍距离小→降低 `v`。

## 6. xline_path_executor（ActionClient）

- 解析 `planned_cad_2_transformed.json`：
  - 按 `order` 升序构造 `PathSegment[]`；`work=true`→WORK；`layer=TRANSITION`→TRANSITION。
  - 单位转换 mm→m；应用 CAD→map 变换；填充容差与目标速度。
- 下发与控制：
  - 调用 `ExecutePlan`；订阅 Feedback，聚合为 `segment_started/segment_progress/segment_completed` 等事件。
  - 暂停/恢复/跳过：调用对应 Service；取消：Action Cancel。
- 对外：可由 xline_server 通过 gRPC/ROS API 远程触发，或 executor 直接在 server 进程内调用 rclpy 作为桥接。

## 7. xline_server 集成（UI/APP 桥接）

WebSocket（Socket.IO）事件建议：
- `plan_loaded`：{ plan_uid, total_segments, work_count, transition_count }
- `segment_started`：{ plan_uid, uid, order, type, geometry, start_ts }
- `segment_progress`：{ uid, order, progress, dist_to_end_mm, v:{lin,ang}, pose:{x,y,heading}, ts }
- `segment_completed`：{ uid, order, duration_ms }
- `plan_completed`：{ plan_uid, duration_ms, success:true }
- `plan_paused` / `plan_resumed` / `plan_aborted` / `plan_error`

HTTP 控制面（示例）：
- POST `/api/v1/plan/load`
- POST `/api/v1/plan/start?include_transition=false`
- POST `/api/v1/plan/pause|resume|abort`
- POST `/api/v1/plan/skip_segment?order=…`
- GET  `/api/v1/plan/state`

## 8. ros2_control 集成（示例 YAML）

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    xline_diff_drive:
      type: diff_drive_controller/DiffDriveController
      left_wheel_names:  [left_wheel_joint]
      right_wheel_names: [right_wheel_joint]
      wheel_separation: 0.46
      wheel_radius: 0.075
      cmd_vel_timeout: 0.25
      base_frame_id: base_link
      odom_frame_id: odom
      enable_odom_tf: true

    # 若使用 effort 控制，可在此增加轮级 effort/velocity 控制器与 PID 参数
```

## 9. 参数建议（初始）
- 容差：WORK 段 `pos_tol 0.05–0.08 m`，TRANSITION 段 `0.12–0.20 m`；`yaw_tol 5–10°`。
- 速度：WORK 段 `v=0.30–0.45 m/s`；TRANSITION 段 `v=0.45–0.80 m/s`；`ω_max=1.0–1.5 rad/s`。
- RPP：`min_lookahead=0.4 m`，`max_lookahead=1.5 m`，`lookahead_time=1.0 s`。
- PID：`k_theta≈1.2`，`k_cte≈0.8`，`i_clamp≈0.3`，含加速度/跃度限制。

## 10. 状态机与时序

执行时序：
1) `xline_path_executor` 载入 JSON → 构建 `Plan`
2) 发送 `ExecutePlan.Goal` → `xline_base_controller` 接受
3) 控制器进入 EXECUTING：逐段执行，周期反馈 Feedback
4) 每段完成时产生一次 `segment_completed`，随后 `segment_started` 下一段
5) 全部完成 → Result.success=true → `plan_completed`
6) Pause/Resume/Skip/Cancel 通过 Service/Cancel 生效

## 11. 误差与异常处理
- 定位丢失：进入 `PAUSED` 并零速；等待定位恢复或人工干预。
- 超时未到达：按照段超时策略（减速/重试/人工确认）→ 失败则 `FAILED`。
- Cancel：立即零速并返回 `aborted`。
- 冗余安全：E-Stop 服务/话题，任何异常立即零速；心跳/看门狗避免失控。

## 12. TRANSITION 段策略
- 可选：执行（更快/容差宽）或跳过（仅 UI 灰显）。
- 参数：`include_transitions`（Plan 级别），或对每段 `kind` 独立控制。

## 13. 实施路线与里程碑
- M1：搭建 `xline_msgs`（action/msg/srv）；生成代码通过 colcon 编译。
- M2：实现 `xline_base_controller`（ActionServer）：先支持 LINE + WORK 段；订阅 `/odom`，发布 `/cmd_vel`。
- M3：实现 `xline_path_executor`（ActionClient）：解析 JSON → 下发 Plan → 打印/记录 Feedback。
- M4：xline_server 桥接：REST/WS → 调用 executor；推送 `segment_*`/`plan_*` 事件；App 高亮接入。
- M5：圆弧/圆段、TRANSITION 段、Pause/Resume/Skip；加速/限幅/抗积分完善。
- M6：实车联调、参数整定、容错/急停、断点续跑；完善文档与监控。

## 14. 测试与验收
- 仿真：记录/回放路径与位姿（bag），验证段进度与完成判据。
- HIL/实车：短路径（直线→转角→直线）验证；加入圆弧后测试曲率限速；App 高亮与反馈一致。
- 验收标准：
  - 计划能完整执行，所有段按序完成；
  - UI 实时显示当前段与进度；
  - Pause/Resume/Skip/Cancel 可靠；
  - 误差/超时处理符合预期；
  - 急停安全可用。

## 15. 可选变体与演进
- 使用 Nav2：将 `xline_base_controller` 替换为 Nav2 controller_server（Regulated Pure Pursuit），Action 改为下发 `nav_msgs/Path` 或 NavigateToPose 序列。
- 硬件桥接：若无需 ros2_control，可用 `xline_chassis_bridge` 直接对接厂商 SDK；其余层保持不变。
- 多机/并行：Action 的 plan_uid 区分任务；控制器与执行器支持命名空间隔离。

---

附：字段映射参考（从 JSON → PathSegment）

```
JSON line → PathSegment：
- uid:       组合 `order` + `id`（或哈希）
- order:     `line.order`
- kind:      `work=true` → WORK；`layer == "TRANSITION"` → TRANSITION
- geometry:  type:
   • `type==line` → GEOM_LINE：sx,sy = start.{x,y}*0.001，ex,ey = end.{x,y}*0.001
   • `type==arc/circle` → GEOM_ARC/GEOM_CIRCLE：按中心/半径/角度填充（单位 m / rad）
- v_target:  默认按 kind 区分；可全局参数或每段覆盖
- 容差:      pos_tol_m/yaw_tol_rad 由全局/图层策略给定
```

