/**
 * @file rpp_params.hpp
 * @brief RPP控制器参数结构体定义
 *
 * 使用结构体组织配置参数，与嵌套YAML结构对应
 */

#pragma once

#include <string>

namespace xline
{
namespace follow_controller
{

/// 圆形路径动态速度参数
struct RPPCircleVelocityParams
{
  double reference_velocity = 0.08;  ///< 参考速度 [m/s]
  double reference_radius = 0.5;     ///< 参考半径 [m]
  double radius_exponent = 0.5;      ///< 半径指数 γ
  double min_velocity = 0.05;        ///< 最小速度 [m/s]
  double max_velocity = 0.15;        ///< 最大速度 [m/s]
};

/// 圆形路径动态前瞻参数
struct RPPCircleLookaheadParams
{
  double reference_lookahead = 0.15; ///< 参考前瞻距离 [m]
  double velocity_exponent = 1.0;    ///< 速度指数 α
  double radius_exponent = 0.0;      ///< 半径额外指数 β
  double min_dist = 0.08;            ///< 最小前瞻距离 [m]
  double max_dist = 0.25;            ///< 最大前瞻距离 [m]
};

/// 圆形路径固定参数（备用）
struct RPPCircleFixedParams
{
  double velocity = 0.08;            ///< 固定线速度 [m/s]
  double lookahead = 0.15;           ///< 固定前瞻距离 [m]
};

/// 圆形路径动态参数总配置
struct RPPCircleDynamicsParams
{
  bool enabled = true;               ///< 是否启用动态计算
  RPPCircleVelocityParams velocity;  ///< 速度参数
  RPPCircleLookaheadParams lookahead;///< 前瞻参数
  RPPCircleFixedParams fixed;        ///< 固定参数（备用）
};

// 目标到达判定参数
struct RPPGoalParams
{
  double dist_tol = 0.02;      ///< 位置容忍度 [m]
  double rotate_tol = 0.035;   ///< 角度容忍度 [rad]
};

// 前瞻控制参数
struct RPPLookaheadParams
{
  double time = 0.15;          ///< 前瞻时间 [s]
  double min_dist = 0.18;      ///< 最小前瞻距离 [m]
  double max_dist = 0.18;      ///< 最大前瞻距离 [m]
};

// 速度控制参数
struct RPPLinearVelocityParams
{
  double base = 0.06;          ///< 基准线速度 [m/s]
  double min = 0.06;           ///< 最小线速度 [m/s]
  double max = 0.3;            ///< 最大线速度 [m/s]
  double max_inc = 0.06;       ///< 最大加速度 [m/s²]
};

struct RPPAngularVelocityParams
{
  double min = 0.03;           ///< 最小角速度 [rad/s]
  double max = 1.0;            ///< 最大角速度 [rad/s]
  double max_inc = 0.06;       ///< 最大角加速度 [rad/s²]
};

struct RPPVelocityParams
{
  RPPLinearVelocityParams linear;
  RPPAngularVelocityParams angular;
};

// 路径跟踪约束参数
struct RPPApproachParams
{
  double dist = 0.05;          ///< 接近目标距离阈值 [m]
  double min_v = 0.07;         ///< 接近目标最小速度 [m/s]
};

struct RPPTrackingParams
{
  double regulated_min_radius = 0.5;  ///< 曲率约束半径阈值 [m]
  RPPApproachParams approach;
};

// 角速度平滑参数
struct RPPLowpassSmoothingParams
{
  double gain = 0.9;           ///< 低通滤波增益 [0-1]
};

struct RPPMovingAverageSmoothingParams
{
  int window_size = 1;         ///< 移动平均窗口大小
};

struct RPPSecondOrderSmoothingParams
{
  double freq = 15.0;          ///< 自然频率 [Hz]
  double damping = 0.9;        ///< 阻尼比 [-]
};

struct RPPSmoothingParams
{
  std::string type = "lowpass";            ///< 平滑类型
  double radius_offset = 0.0;              ///< 半径偏移量 [m]
  RPPLowpassSmoothingParams lowpass;
  RPPMovingAverageSmoothingParams moving_average;
  RPPSecondOrderSmoothingParams second_order;
};

// 滤波器参数
struct RPPPositionFilterParams
{
  bool enabled = true;         ///< 是否启用滤波器结构
  bool lowpass_active = false;         ///< 是否实际应用滤波
  double cutoff_freq = 1.5;    ///< 截止频率 [Hz]
  double sample_rate = 18.0;   ///< 采样率 [Hz]
  double output_limit = 0.3;   ///< 输出限幅 [m]
  double rate_limit = 0.3;     ///< 变化率限制 [m/s]
};

struct RPPAngularFilterParams
{
  bool enabled = true;             ///< 是否启用滤波器结构
  bool lowpass_active = true;              ///< 是否实际应用滤波
  bool use_offset_limit = true;    ///< 是否启用偏移限制
  double output_offset = 0.1;      ///< 输出偏移量 [rad/s]
  double cutoff_freq = 1.5;        ///< 截止频率 [Hz]
  double sample_rate = 18.0;       ///< 采样率 [Hz]
  double output_limit_rate = 1.05; ///< 输出限幅比率 [-]
  double rate_limit = 0.03;        ///< 变化率限制 [rad/s²]
};

struct RPPFilterParams
{
  RPPPositionFilterParams position;
  RPPAngularFilterParams angular;
};

// 运行模式参数
struct RPPModeParams
{
  bool low_speed = true;       ///< 低速模式开关
};

// 调试参数
struct RPPDebugParams
{
  bool enable_grid_map = true;         ///< 是否启用栅格图
  std::string grid_map_path = "grid_maps";  ///< 栅格图保存路径
};

// 偏差控制参数
struct RPPDeviationParams
{
  double start_factor = 0.5;   ///< 起始阶段放宽区间系数：start_factor × π [rad]
  double end_factor = 0.5;     ///< 结束阶段放宽区间系数：end_factor × π [rad]

  // 角速度偏差比例（相对 baseline_angular_velocity_ 的允许偏差）
  // - strict_ratio: 中间阶段的“严格约束”比例（越小越贴近 v/r）
  // - relaxed_ratio: 起始/结束放宽区间的“放宽约束”比例（通常大于 strict_ratio）
  double strict_ratio = 0.05;
  double relaxed_ratio = 0.1;
};

// RPP控制器完整参数结构体
struct RPPParams
{
  RPPGoalParams goal;
  RPPLookaheadParams lookahead;
  RPPVelocityParams velocity;
  RPPTrackingParams tracking;
  RPPSmoothingParams smoothing;
  RPPFilterParams filter;
  RPPModeParams mode;
  RPPDebugParams debug;
  RPPDeviationParams deviation;
};

// 路径策略参数结构体
struct RPPPathStrategyParams
{
  RPPGoalParams goal;
  RPPTrackingParams tracking;
  RPPSmoothingParams smoothing;
  RPPDeviationParams deviation;
  RPPCircleDynamicsParams circle_dynamics;
};

}  // namespace follow_controller
}  // namespace xline
