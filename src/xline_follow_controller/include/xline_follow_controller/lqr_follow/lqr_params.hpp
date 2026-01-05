#pragma once

#include <string>

namespace xline
{
namespace follow_controller
{

/**
 * @brief LQR控制器参数结构体
 *
 * 包含前馈+LQR控制器的所有可配置参数
 * 参数可通过YAML文件加载
 */
struct LQRParams
{
  // ================================
  // LQR 权重参数
  // ================================

  /// 横向误差权重 q1（状态权重矩阵Q的第一个对角元素）
  double q1 = 1.0;

  /// 航向误差权重 q2（状态权重矩阵Q的第二个对角元素）
  double q2 = 200.0;

  /// 控制量权重 r（控制权重）
  double r = 1.0;

  // ================================
  // 直接指定增益（推荐）
  // ================================

  /// 是否使用直接指定的增益（而不是从q1,q2,r计算）
  bool use_direct_gains = true;

  /// 横向误差增益 K1（直接指定）
  double K1_direct = 20.0;  // 对应 v=0.05 m/s

  /// 航向误差增益 K2（直接指定）
  double K2_direct = 15.0;

  // ================================
  // 积分项参数（LQI，可选）
  // ================================

  /// 是否启用积分项
  bool enable_integral = false;

  /// 积分增益
  double Ki = 0.5;

  /// 积分饱和限制 (rad/s)
  double integral_max = 0.1;

  /// 积分衰减系数（0-1之间，越小衰减越快）
  double integral_decay = 0.99;

  // ================================
  // 速度限制参数
  // ================================

  /// 最大线速度 (m/s)
  double v_max = 0.05;

  /// 最小线速度 (m/s)
  double v_min = 0.01;

  /// 最大角速度 (rad/s)
  double omega_max = 0.5;

  /// 最大角加速度 (rad/s²)
  double omega_dot_max = 1.0;

  // ================================
  // 机器人参数
  // ================================

  /// 轮距 (m)
  double wheel_base = 0.5;

  // ================================
  // 前瞻参数
  // ================================

  /// 前瞻距离 (m) - 基于弧长的前瞻
  double lookahead_distance = 0.02;

  /// 前瞻时间 (s) - 额外的时间前瞻
  double lookahead_time = 0.2;

  // ================================
  // 最近点搜索窗口
  // ================================

  /// 向后搜索窗口大小
  size_t search_window_back = 5;

  /// 向前搜索窗口大小
  size_t search_window_forward = 30;

  // ================================
  // 目标到达判定参数
  // ================================

  /// 距离容差 (m)
  double goal_dist_tolerance = 0.01;

  /// 角度容差 (rad)
  double goal_angle_tolerance = 0.05;

  // ================================
  // 控制参数
  // ================================

  /// 控制频率 (Hz)
  double control_frequency = 18.0;

  /// 控制周期 (s) - 从control_frequency自动计算
  double control_period = 1.0 / 18.0;

  // ================================
  // 原地旋转控制参数
  // ================================

  /// 旋转速度调节因子（sigmoid函数增益）
  double rotation_factor = 1.2;

  /// 旋转最大角速度 (rad/s)
  double rotation_max_w = 0.8;

  /// 旋转最小角速度 (rad/s)
  double rotation_min_w = 0.15;

  /// 角度阈值 (rad)
  double rotation_angle_threshold = 0.5;

  /// 平滑因子（小角度时的余弦平滑系数）
  double rotation_smooth_factor = 0.6;

  // ================================
  // 调试参数
  // ================================

  /// 是否启用调试输出
  bool enable_debug = false;

  /// 是否输出详细日志
  bool verbose = false;

  /// 是否启用栅格图可视化
  bool enable_grid_map = false;

  /// 栅格图保存路径
  std::string grid_map_path = "grid_maps";
};

}  // namespace follow_controller
}  // namespace xline
