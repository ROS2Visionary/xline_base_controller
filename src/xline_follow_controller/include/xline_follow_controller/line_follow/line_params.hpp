#pragma once

#include <string>

namespace xline
{
namespace follow_controller
{

// 调试参数 (debug.*)
struct LineDebugParams
{
  bool enabled = true;
  std::string raw_path;              ///< 原始路径导出目录
  std::string filtered_path;         ///< 滤波后路径导出目录
};

// 优化元信息 (optimization.*)
struct LineOptimizationParams
{
  std::string id = "baseline";       ///< 本轮优化标识
  std::string parent_batch_id;       ///< 基于哪个 batch_id 做的优化
  std::string change_note;           ///< 本轮变更摘要（建议用分号分隔）
};


// PID参数 (heading_pid.*)
struct LinePIDGains
{
  double kp = 1.0;
  double ki = 0.0;
  double kd = 0.1;
};

struct LineHeadingPIDParams
{
  LinePIDGains alignment;                ///< 对齐阶段PID
  LinePIDGains following;                ///< 跟随阶段PID
};

// 速度参数 (velocity.*)
struct LineVelocityParams
{
  // 速度限制
  double accel_min = 0.05;           ///< 加速时最小线速度 [m/s]
  double decel_min = 0.03;           ///< 减速时最小线速度 [m/s]
  double walk_max = 0.35;            ///< 步行模式最大速度 [m/s]
  double work_max = 0.35;            ///< 工作模式最大速度 [m/s]
  double alignment = 0.05;           ///< 对齐阶段速度 [m/s]
  // 调速参数
  double acce_factor = 0.0005;       ///< 加速因子
  double accel_sigmoid_k = 6.0;      ///< 加速sigmoid系数
  double decel_sigmoid_k = 10.0;     ///< 减速sigmoid系数
  double accel_sigmoid_center = 0.9; ///< 加速sigmoid中心点
  double decel_sigmoid_center = 0.3; ///< 减速sigmoid中心点
};

// 距离参数 (distance.*)
struct LineDistanceParams
{
  double alignment = 0.08;           ///< 对齐距离阈值 [m]
  double deceleration = 0.3;         ///< 减速距离（工作模式）[m]
  double acceleration = 0.15;        ///< 加速阶段距离 [m]
  double lookahead = 0.25;           ///< 前瞻基础距离 [m]
  double lookahead_time = 0.0;       ///< 前瞻时间系数 [s], 总前瞻 = lookahead + lookahead_time * speed
  double waypoint_tolerance = 0.005; ///< 路径点容差 [m]
};

// 原地旋转参数 (rotation.*)
struct LineRotationParams
{
  double max_w = 0.8;                ///< 最大角速度 [rad/s]
  double min_w = 0.15;               ///< 最小角速度 [rad/s]
  double factor = 1.2;               ///< 角速度调节因子
  double angle_threshold = 0.5;      ///< 角度阈值 [rad]
  double smooth_factor = 0.6;        ///< 平滑因子
};

// 滤波器参数 (filter.*)
struct LineFilterParams
{
  // 位置滤波
  int pos_hampel_window = 5;         ///< 位置Hampel窗口大小
  double pos_hampel_k = 3.0;         ///< 位置Hampel异常值阈值
  int pos_savgol_window = 7;         ///< 位置SG窗口大小
  int pos_savgol_order = 3;          ///< 位置SG多项式阶数
  // 角速度滤波
  int angular_hampel_window = 7;     ///< 角速度Hampel窗口大小
  double angular_hampel_k = 3.0;     ///< 角速度Hampel异常值阈值
};

// 阶段控制参数 (phase.*)

/// 单阶段参数
struct LinePhaseConfig
{
  double max_angular_vel = 0.065;    ///< 最大角速度 [rad/s]
  double max_angular_accel = 0.03;   ///< 最大角加速度 [rad/s²]
  double current_heading_weight = 0.0; ///< 当前航向权重
  double target_heading_weight = 1.0;  ///< 目标航向权重
  double filter_alpha = 0.95;        ///< 低通滤波系数
  double smoother_freq = 15.0;       ///< 平滑器频率 [Hz]
  double smoother_damping = 0.86;    ///< 平滑器阻尼比

  // 起步对齐 -> 加速的切换条件（主要用于直线跟随）
  double line_cross_track_tolerance = 0.02;   ///< 允许的横向误差(到直线) [m]
  double line_heading_tolerance = 0.08;       ///< 允许的航向误差(相对直线航向) [rad]
  int line_alignment_stable_count = 5;        ///< 连续满足次数（抗抖）
};

struct LinePhaseParams
{
  LinePhaseConfig alignment;             ///< 对齐阶段
  LinePhaseConfig following;             ///< 跟随阶段
};

// 直线路径"回线/进线"制导参数 (guidance.*)
struct LineGuidanceParams
{
  /// 制导模式：
  /// - weighted_yaw: 现有逻辑（path_ideal_yaw 与 path_direction_yaw 加权）
  /// - stanley: Stanley 侧向误差制导
  /// - hybrid: 对齐阶段用 alignment_mode，跟随阶段用 following_mode
  std::string mode = "weighted_yaw";

  // hybrid 子模式
  std::string alignment_mode = "stanley";       ///< 对齐阶段制导（默认 stanley）
  std::string following_mode = "weighted_yaw";  ///< 跟随阶段制导（默认 weighted_yaw）

  // stanley
  double stanley_k = 1.0;                ///< Stanley 增益 k
  double stanley_v0 = 0.05;              ///< 低速软化项 v0 [m/s]

  // 限制"截获角"，避免目标航向过于激进
  double theta_max = 0.7;                ///< 最大截获角 [rad]
};

// LQR 角速度控制参数 (lqr_angular_control.*)
struct LineLQROutputFilterParams
{
  bool enabled = false;              ///< 是否启用 LQR 输出滤波

  // 二阶平滑器（推荐：优先用这个做“手感”平滑）
  bool use_smoother = true;          ///< 是否启用二阶平滑器
  double smoother_freq = 15.0;       ///< 平滑器自然频率 [Hz]
  double smoother_damping = 0.8;     ///< 平滑器阻尼比

  // 一阶低通（可选：用于抑制高频噪声，alpha 越小越“糊”）
  bool use_lowpass = false;          ///< 是否启用一阶低通
  double lowpass_alpha = 0.95;       ///< 低通系数 alpha ∈ (0, 1]，越接近 1 越接近原始输出
};

struct LineLQRParams
{
  bool enabled = false;                  ///< 是否启用LQR角速度控制
  double q1 = 200.0;                     ///< 横向误差权重
  double q2 = 300.0;                     ///< 航向误差权重
  double r = 1.0;                        ///< 控制量权重
  bool use_direct_gains = false;         ///< 是否直接指定增益
  double K1_direct = 20.0;               ///< 横向误差增益（直接指定）
  double K2_direct = 15.0;               ///< 航向误差增益（直接指定）
  bool enable_integral = false;          ///< 是否启用横向误差积分项（LQI）
  double Ki = 0.5;                       ///< 积分增益
  double integral_max = 0.1;             ///< 积分项输出上限（rad/s）
  double integral_decay = 0.99;          ///< 积分衰减系数 [0,1]
  // 横向误差较大时，动态衰减航向项增益，避免 -K1*e_y 与 -K2*e_theta 互相抵消
  bool k2_gate_enabled = true;
  double k2_gate_start = 0.004;          ///< |e_y| <= start 时不衰减 [m]
  double k2_gate_end = 0.012;            ///< |e_y| >= end 时衰减到 min_scale [m]
  double k2_gate_min_scale = 0.35;       ///< K2 最小缩放比例 [0,1]
  // 当 e_y 与 e_theta 导致反馈项相互抵消时，进一步衰减 K2（仅在较大横向误差下触发）
  bool k2_anti_cancel_enabled = true;
  double k2_anti_cancel_scale = 0.60;    ///< 额外缩放比例 [0,1]
  double k2_anti_cancel_ey_threshold = 0.003; ///< |e_y| 触发阈值 [m]
  // 所有 K2 抑制机制（gate + anti_cancel + tail_schedule）作用后的绝对下限，
  // 相对于 K2_direct 的比例；0 表示不设下限。
  // 防止叠乘抑制把 LQR 阻尼项压至接近零，导致系统退化为无阻尼纯比例控制。
  double k2_min_floor_scale = 0.0;       ///< K2_effective 下限比例 [0,1]
  LineLQROutputFilterParams output_filter;  ///< LQR 输出滤波（单独一组，避免与 PID 干扰）
  double K1_max = 50;
  double K1_min = 13;
  // e_theta 一阶低通滤波（不平整地面专用）
  // 地板缝/接缝产生的瞬态航向扰动（~0.02-0.05 rad，时长 50-100ms）经 K2 放大后
  // 会引发 2-4mm 次生横向误差；低通 α=0.82 可衰减约 4x，对真实连续信号影响<5%
  bool e_theta_lowpass_enabled = false;  ///< 是否启用 e_theta 低通滤波（默认关，不影响历史行为）
  double e_theta_lowpass_alpha = 0.82;   ///< 低通系数 ∈ (0,1]，越接近 1 延迟越小（推荐 0.75-0.88）
};

// e_y 输入滤波参数 (ey_filter.*)
struct LineEyRateLimitParams
{
  bool enabled = true;               ///< 是否启用速率限制
  double noise_floor = 0.002;        ///< 噪声底噪阈值 [m]，帧间变化下限
  double rate_factor = 0.5;          ///< 运动学分量系数：max_change = max(noise_floor, v*dt*factor)
};

struct LineEyLowpassParams
{
  bool enabled = true;               ///< 是否启用一阶低通
  double alpha = 0.87;               ///< 低通系数 ∈ (0,1]，越接近 1 越接近原始值
};

struct LineEyFilterParams
{
  LineEyRateLimitParams rate_limit;  ///< e_y 速率限制（防止突变污染平滑器状态）
  LineEyLowpassParams lowpass;       ///< e_y 一阶低通（衰减常规高频噪声）
};

// 顶层参数结构体
struct LineParams
{
  LineDebugParams debug;                 ///< 调试参数
  LineOptimizationParams optimization;   ///< 优化元信息
  LineHeadingPIDParams heading_pid;      ///< 航向PID参数
  LineVelocityParams velocity;           ///< 速度参数
  LineDistanceParams distance;           ///< 距离参数
  LineRotationParams rotation;           ///< 旋转参数
  LineFilterParams filter;               ///< 滤波器参数
  LinePhaseParams phase;                 ///< 阶段控制参数
  LineGuidanceParams guidance;           ///< 制导参数（回线/进线）
  LineLQRParams lqr;                     ///< LQR 角速度控制参数
  LineEyFilterParams ey_filter;          ///< e_y 输入滤波参数
};

// 运行时参数（从PhaseConfig展开）
struct LinePhaseRuntimeParams
{
  double max_angular_vel;
  double max_angular_accel;
  double current_heading_weight;
  double target_heading_weight;
  double filter_alpha;
  double smoother_freq;
  double smoother_damping;
  double line_cross_track_tolerance;
  double line_heading_tolerance;
  int line_alignment_stable_count;

  static LinePhaseRuntimeParams fromConfig(const LinePhaseConfig& cfg)
  {
    LinePhaseRuntimeParams p;
    p.max_angular_vel = cfg.max_angular_vel;
    p.max_angular_accel = cfg.max_angular_accel;
    p.current_heading_weight = cfg.current_heading_weight;
    p.target_heading_weight = cfg.target_heading_weight;
    p.filter_alpha = cfg.filter_alpha;
    p.smoother_freq = cfg.smoother_freq;
    p.smoother_damping = cfg.smoother_damping;
    p.line_cross_track_tolerance = cfg.line_cross_track_tolerance;
    p.line_heading_tolerance = cfg.line_heading_tolerance;
    p.line_alignment_stable_count = cfg.line_alignment_stable_count;
    return p;
  }
};

}  // namespace follow_controller
}  // namespace xline
