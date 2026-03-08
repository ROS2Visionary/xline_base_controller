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
  // 路径类型参数
  // ================================

  // ================================
  // 反馈限制参数（分段控制）
  // ================================

  /// 喷墨前的反馈角速度限制比例（相对于前馈的百分比）
  /// 喷墨前可以设置较大值，允许更强的反馈调整以快速对齐路径
  double feedback_limit_ratio_before_print = 0.10;

  /// 喷墨后的反馈角速度限制比例（相对于前馈的百分比）
  /// 喷墨后应设置较小值，保证喷印质量的稳定性和精度
  double feedback_limit_ratio_after_print = 0.05;

  /// 最小反馈限制 (rad/s)，当前馈很小时使用
  double feedback_min_limit = 0.02;

  /// [已弃用] 统一的反馈限制比例（为了向后兼容保留）
  /// 如果配置文件中只有 limit_ratio，则同时应用到前后两段
  double feedback_limit_ratio = 0.05;

  /// 积分路径独立限制比例（相对于前馈的百分比）
  /// 积分路径与比例路径分离，可以提供更强的稳态误差修正能力
  /// 建议范围: 0.10 - 0.30 (10% - 30%)
  double int_limit_ratio = 0.20;

  /// e_y 跳变检测阈值 (m)
  /// 单个控制周期内横向误差变化超过此值时判定为定位异常/地面扰动跳变
  /// 触发时：限制跳变幅度，冻结积分更新（防止积分响应虚假扰动）
  /// 设为 0.0 关闭跳变检测
  double max_ey_jump_m = 0.0;

  /// e_theta 一阶低通滤波系数（过滤定位系统航向噪声）
  /// 1.0 = 关闭（直通）；0.82 = line控制器推荐值（tau≈280ms，@18Hz）
  /// DC(真实航向误差)完全保留；>0.5Hz高频噪声大幅衰减
  double e_theta_lowpass_alpha = 1.0;

  // ================================
  // 滤波器参数
  // ================================

  // 位置滤波器参数
  /// Hampel滤波器窗口大小
  int hampel_window = 5;

  /// Hampel滤波器异常值阈值（标准差倍数）
  double hampel_k = 3.0;

  /// Savitzky-Golay滤波器窗口大小
  int savgol_window = 7;

  /// Savitzky-Golay滤波器多项式阶数
  int savgol_order = 2;

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

  // ================================
  // 半径自适应速度调度
  // ================================

  /// 是否启用基于半径的速度自适应（false 时始终使用 v_max）
  bool enable_radius_schedule = true;

  /// 半径基准值 (m)：低于此值使用 v_min
  double radius_threshold = 0.5;

  /// 半径步长 (m)：每增加此值，速度增加一个 velocity_step
  double radius_step = 0.1;

  /// 速度步长 (m/s)：每个半径步长对应的速度增量
  double velocity_step = 0.01;

  // ================================
  // 优化元信息（调参记录）
  // ================================

  struct {
    /// 当前优化批次 ID（格式：circle_C00_baseline）
    std::string id              = "circle_C00_baseline";
    /// 父批次 ID（用于追踪优化链路）
    std::string parent_batch_id = "";
    /// 本次变更说明
    std::string change_note     = "初始基线";
  } optimization;
};

}  // namespace follow_controller
}  // namespace xline
