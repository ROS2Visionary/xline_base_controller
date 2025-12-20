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
  double work_short = 0.2;           ///< 短路径工作速度 [m/s]
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
  double non_work_deceleration = 0.4;///< 减速距离（非工作模式）[m]
  double acceleration = 0.15;        ///< 加速阶段距离 [m]
  double lookahead = 0.25;           ///< 前瞻距离 [m]
  double waypoint_tolerance = 0.005; ///< 路径点容差 [m]
  double mini_path = 0.3;            ///< 最小路径距离 [m]
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

// 顶层参数结构体
struct LineParams
{
  LineDebugParams debug;                 ///< 调试参数
  LineHeadingPIDParams heading_pid;      ///< 航向PID参数
  LineVelocityParams velocity;           ///< 速度参数
  LineDistanceParams distance;           ///< 距离参数
  LineRotationParams rotation;           ///< 旋转参数
  LineFilterParams filter;               ///< 滤波器参数
  LinePhaseParams phase;                 ///< 阶段控制参数
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
