#pragma once

#include "xline_follow_controller/common/base_follow_controller.hpp"
#include "xline_follow_controller/lqr_follow/lqr_path_types.hpp"
#include "xline_follow_controller/lqr_follow/lqr_params.hpp"
#include "xline_follow_controller/common/yaml_parser.hpp"
#include "xline_follow_controller/common/path_utils.hpp"
#include "xline_follow_controller/common/follow_common.hpp"

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/utils.h>
#include <opencv2/opencv.hpp>

#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <chrono>
#include <mutex>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <numeric>

namespace xline
{
namespace follow_controller
{

/**
 * @brief 路径类型枚举
 */
enum class PathType
{
  CIRCLE, 
  ARC   
};


class LQRCircleController : public BaseFollowController
{
public:
  /**
   * @brief 构造函数
   */
  LQRCircleController();

  /**
   * @brief 析构函数
   */
  ~LQRCircleController() override = default;

  // ================================
  // BaseFollowController 接口实现
  // ================================

  /**
   * @brief 设置路径
   * @param orig_global_plan 原始全局路径
   * @return 是否成功设置
   */
  bool setPlan(const nav_msgs::msg::Path& orig_global_plan) override;

  /**
   * @brief 计算速度命令
   * @param pose 当前位姿
   * @param velocity 当前速度
   * @param cmd_vel 输出速度命令
   * @return 是否成功计算
   */
  bool computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                               const geometry_msgs::msg::Twist& velocity,
                               geometry_msgs::msg::TwistStamped& cmd_vel) override;

  /**
   * @brief 判断是否到达目标
   * @return 是否到达
   */
  bool isGoalReached() override;

  /**
   * @brief 取消当前任务
   * @return 是否成功取消
   */
  bool cancel() override;

  // ================================
  // LQR 特有接口
  // ================================

  /**
   * @brief 初始化控制器
   */
  void initialize();

  /**
   * @brief 从YAML文件更新参数
   * @param config_path 配置文件路径（相对于package的config目录）
   */
  void updateParameters(const std::string& config_path);

  /**
   * @brief 设置圆形路径角度范围（用于生成圆弧路径）
   * @param start_angle 起始角（rad）
   * @param end_angle 结束角（rad）
   */
  void setAngleRange(double start_angle, double end_angle);

  /**
   * @brief 生成并设置一条圆形（圆弧）路径
   * @param circle_center_x 圆心X (m)
   * @param circle_center_y 圆心Y (m)
   * @param circle_radius   半径 (m)
   * @param robot_pose      当前机器人位姿（用于生成切入点与航向）
   * @return 是否成功
   */
  bool setPlanForCircle(double circle_center_x, double circle_center_y,
                        double circle_radius,
                        const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief 获取当前LQR增益
   * @param K1 输出横向误差增益
   * @param K2 输出航向误差增益
   */
  void getGains(double& K1, double& K2) const;

  /**
   * @brief 获取调试信息字符串
   * @return 调试信息
   */
  std::string getDebugInfo() const;

private:
  // ================================
  // 核心计算函数
  // ================================

  /**
   * @brief 计算LQR增益
   * @param v 当前速度
   */
  void computeLQRGains(double v);

  /**
   * @brief 计算路径曲率
   */
  void computePathCurvature();

  /**
   * @brief 找到最近的路径点（改进的搜索算法）
   * @param x 当前X坐标
   * @param y 当前Y坐标
   * @return 最近点索引
   */
  size_t findNearestPoint(double x, double y);

  /**
   * @brief 查找前瞻点（参考RPP控制器实现）
   * 从最近点开始，沿路径累积距离直到达到前瞻距离，并进行插值
   * @param nearest_idx 最近点索引
   * @param lookahead_dist 前瞻距离
   * @return 前瞻点（插值后的位置）
   */
  PathPointWithCurvature findLookaheadPoint(size_t nearest_idx, double lookahead_dist);

  /**
   * @brief 计算前瞻距离
   * @param speed 当前速度
   * @return 前瞻距离
   */
  double getLookaheadDistance(double speed);

  /**
   * @brief 计算误差
   * @param current_x 当前X坐标
   * @param current_y 当前Y坐标
   * @param current_theta 当前航向角
   * @param ref 参考点
   * @param e_y 输出横向误差
   * @param e_theta 输出航向误差
   */
  void computeErrors(double current_x, double current_y, double current_theta,
                     const PathPointWithCurvature& ref,
                     double& e_y, double& e_theta);

  /**
   * @brief 应用控制量限幅
   * @param omega 角速度
   * @return 限幅后的角速度
   */
  double applyLimits(double omega);

  /**
   * @brief 角度归一化到 [-π, π]
   * @param angle 角度
   * @return 归一化后的角度
   */
  static double normalizeAngle(double angle);

  /**
   * @brief 执行航向预对准
   * @param current_pose 当前位姿
   * @param target_yaw 目标航向角
   * @param cmd_vel 输出速度命令
   * @return 是否完成预对准
   */
  bool performYawPrealignment(const geometry_msgs::msg::PoseStamped& current_pose,
                              double target_yaw,
                              geometry_msgs::msg::TwistStamped& cmd_vel);

  /**
   * @brief 计算旋转速度
   * @param angle_diff 角度差
   * @return 旋转速度
   */
  double calculateRotationVelocity(double angle_diff);

  /**
   * @brief 重置控制器状态
   */
  void reset();

  /**
   * @brief 处理等待状态（对齐完成后的延时）
   * @param cmd_vel 输出速度命令
   * @return 是否仍在等待中
   */
  bool handleWaitingState(geometry_msgs::msg::TwistStamped& cmd_vel);

  /**
   * @brief 根据圆心/半径/起始位姿生成圆弧路径（包含切入点）
   */
  nav_msgs::msg::Path generateCirclePath(double center_x, double center_y,
                                         double radius,
                                         const geometry_msgs::msg::PoseStamped& start_pose) const;

  // ================================
  // 数据采集与导出
  // ================================

  /**
   * @brief 准备新的圆弧采集槽位（每次 setPlanForCircle 调用）
   */
  void prepareCircleSlot();

  /**
   * @brief 按采样间隔记录一帧跟踪数据
   */
  void sampleTrackingData(double robot_x, double robot_y,
                          double cross_track_m, double radial_err_m,
                          double linear_speed, double angular_cmd,
                          double ref_theta, double e_theta,
                          double omega_ff, double omega_fb_raw,
                          double omega_correction, double omega_i,
                          double omega_before_limits, double feedback_limit,
                          double k1, double k2,
                          size_t nearest_idx, double accumulated_angle_rad,
                          bool is_printing, double integral_state);

  /**
   * @brief 导出本次圆弧的跟踪指标和采样数据到 CSV
   */
  void exportTrackingMetrics();

  /**
   * @brief 计算百分位数
   */
  double computePercentile(const std::vector<double>& values, double percentile) const;

  /**
   * @brief 计算绝对值低于阈值的样本比例
   */
  double computeShareBelowThreshold(const std::vector<double>& values, double threshold_m) const;

  /**
   * @brief 获取跟踪数据根目录（优先使用 XLINE_WS_ROOT 环境变量）
   */
  std::string getTrackingRecordDir() const;

  /**
   * @brief 根据圆弧半径计算调度线速度
   * @param radius 圆弧半径 (m)
   * @return 调度后的线速度，范围 [v_min, v_max]
   */
  double computeScheduledVelocity(double radius) const;

  /**
   * @brief 位置滤波（使用Hampel滤波器和Savitzky-Golay滤波器）
   * @param robot_pose 原始机器人位姿
   * @return 滤波后的机器人位姿
   */
  geometry_msgs::msg::PoseStamped filterRobotPose(const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief 初始化滤波器（使用参数文件中的配置）
   */
  void initializeFilters();

  // ================================
  // 栅格图可视化
  // ================================

  /**
   * @brief 初始化栅格图
   * @param path 路径
   */
  void initializeGridMap(const nav_msgs::msg::Path& path);

  /**
   * @brief 世界坐标转栅格坐标
   * @param x 世界X坐标
   * @param y 世界Y坐标
   * @return 栅格坐标
   */
  cv::Point worldToGrid(double x, double y);

  /**
   * @brief 在栅格图上绘制路径
   * @param path 路径
   * @param color 颜色
   * @param thickness 线宽
   */
  void drawPathOnGrid(const nav_msgs::msg::Path& path, const cv::Scalar& color, int thickness);

  /**
   * @brief 在栅格图上绘制机器人
   * @param pose 机器人位姿
   */
  void drawRobotOnGrid(const geometry_msgs::msg::PoseStamped& pose);

  /**
   * @brief 在栅格图上绘制前瞻点
   * @param lookahead_point 前瞻点
   */
  void drawLookaheadPointOnGrid(const PathPointWithCurvature& lookahead_point);

  /**
   * @brief 绘制栅格线
   */
  void drawGridLines();

  /**
   * @brief 保存栅格图
   */
  void saveGridMap();

  /**
   * @brief 在需要时更新栅格图
   * @param current_pose 当前位姿
   * @param lookahead_point 前瞻点
   */
  void updateGridMapIfNeeded(const geometry_msgs::msg::PoseStamped& current_pose,
                             const PathPointWithCurvature& lookahead_point);

  /**
   * @brief 判断点是否在栅格图内
   * @param pt 栅格坐标点
   * @return 是否在栅格图内
   */
  bool isPointInGrid(const cv::Point& pt);

  // ================================
  // 参数与状态
  // ================================

  LQRParams params_;  ///< 控制器参数

  /// 路径点（带曲率）
  std::vector<PathPointWithCurvature> path_;

  /// 当前LQR增益
  double K1_;  ///< 横向误差增益
  double K2_;  ///< 航向误差增益

  /// 控制状态
  bool initialized_;       ///< 是否已初始化
  bool goal_reached_;      ///< 是否到达目标
  size_t last_nearest_idx_; ///< 上次最近点索引（用于优化搜索）

  /// 航向预对准状态
  bool need_yaw_prealign_;  ///< 是否需要航向预对准
  bool yaw_prealign_done_;  ///< 航向预对准是否完成
  double target_yaw_;       ///< 目标航向角

  /// 等待状态（对齐完成后的延时）
  double wait_duration_;                        ///< 等待时长（秒）
  bool waiting_;                                ///< 是否正在等待
  std::chrono::steady_clock::time_point wait_start_time_;  ///< 等待开始时间

  /// 积分项状态
  double integral_e_y_;    ///< 横向误差积分

  /// e_y 跳变检测状态
  double last_e_y_;              ///< 上一周期控制误差（用于跳变检测）
  bool   last_e_y_initialized_;  ///< 是否已初始化上次 e_y

  /// e_theta 低通滤波状态
  double e_theta_filtered_ = 0.0;  ///< 低通滤波后的航向误差（一阶IIR）

  /// 喷墨触发预计算参数（setPlanForCircle 时一次性计算，避免每控制周期重复）
  double start_trigger_angle_ = 0.0;  ///< 喷墨触发累计角度阈值（角度达到此值→触发提前量）
  double start_lead_angle_    = 0.0;  ///< 喷墨提前量角度 = (v_scheduled/R) × print_start_delay

  /// 比例限制平滑过渡（防止 start_print 切换时 prop_limit 阶跃导致 omega 突变）
  double limit_ratio_smoothed_ = 0.1;  ///< 当前平滑后的 limit_ratio（一阶低通，时间常数 0.3s）

  /// 上次角速度（用于角加速度限幅）
  double last_omega_;

  /// 调试信息
  double debug_e_y_;       ///< 当前横向误差
  double debug_e_theta_;   ///< 当前航向误差
  double debug_omega_ff_;  ///< 前馈角速度
  double debug_omega_fb_;  ///< 反馈角速度
  double debug_ref_curvature_; ///< 参考曲率
  size_t debug_ref_index_;     ///< 参考点索引

  // ================================
  // 滤波器相关（与RPP保持一致）
  // ================================

  /// 位置滤波器
  SavitzkyGolayFilter sg_x_filter_ = SavitzkyGolayFilter(7, 2);
  SavitzkyGolayFilter sg_y_filter_ = SavitzkyGolayFilter(7, 2);
  HampelFilter h_x_filter = HampelFilter(5, 3.0);
  HampelFilter h_y_filter = HampelFilter(5, 3.0);

  // ================================
  // 栅格图可视化相关
  // ================================

  /// 栅格图
  cv::Mat grid_map_;

  /// 栅格分辨率 (m/pixel)
  double grid_resolution_ = 0.001;  // 1mm per pixel

  /// 栅格宽度 (m)
  double grid_width_ = 0.0;

  /// 栅格高度 (m)
  double grid_height_ = 0.0;

  /// 栅格原点X坐标 (m)
  double grid_origin_x_ = 0.0;

  /// 栅格原点Y坐标 (m)
  double grid_origin_y_ = 0.0;

  /// 上次栅格更新时间
  rclcpp::Time last_grid_update_time_;

  // ================================
  // 圆形路径参数（用于 setPlanForCircle）
  // ================================

  /// 路径类型（圆形或圆弧）
  PathType path_type_ = PathType::ARC;

  /// 圆形半径（用于 updateAccumulatedAngle 计算）
  double circle_radius_ = 0.0;

  double path_total_angle_ = 2.0 * M_PI;

  double target_total_angle_ = 2.0 * M_PI;

  /// 角度累计相关
  bool last_yaw_initialized_ = false;  ///< 是否已初始化上次航向角
  double last_yaw_ = 0.0;              ///< 上次航向角
  double accumulated_angle_ = 0.0;     ///< 累计角度

  /// 打印窗口参数
  bool print_window_initialized_ = false;   ///< 打印窗口是否已初始化
  double start_print_angle_ = 0.0;          ///< 开始打印触发角度
  double stop_print_start_angle_ = 0.0;     ///< 停止打印窗口起始角度
  double stop_print_end_angle_ = 0.0;       ///< 停止打印窗口结束角度

  // ================================
  // 圆心坐标（用于径向误差计算）
  // ================================
  double circle_center_x_ = 0.0;
  double circle_center_y_ = 0.0;

  // ================================
  // 数据采集状态
  // ================================

  /// 横向误差绝对值序列（单位：m，用于统计）
  std::vector<double> tracking_error_abs_m_;
  /// 径向误差序列（正=偏外，负=偏内，单位：m）
  std::vector<double> tracking_radial_err_m_;
  /// 每个样本对应的累计角度（用于角度分布分析）
  std::vector<double> tracking_angle_at_sample_;
  /// CSV 采样行缓存
  std::vector<std::string> tracking_sample_rows_;

  bool   tracking_metrics_exported_   = false; ///< 是否已导出指标
  double tracking_elapsed_time_s_     = 0.0;   ///< 跟踪已用时间（s）
  double next_tracking_sample_time_s_ = 0.0;   ///< 下次采样时刻（s）
  static constexpr double kTrackingSampleInterval = 0.05; ///< 采样间隔（50ms）

  // ================================
  // 批次管理
  // ================================
  std::string circle_batch_id_;         ///< 当前批次 ID（时间戳）
  int         circle_slot_              = 0;     ///< 当前圆弧槽位（1-based）
  bool        circle_batch_initialized_ = false; ///< 批次是否已初始化
  int         circle_count_in_batch_   = 0;     ///< 本批次已完成圆弧数

  // ================================
  // 控制周期计时
  // ================================
  std::chrono::steady_clock::time_point last_control_time_;
  bool last_control_time_initialized_ = false;

  /// 文件写入互斥锁
  mutable std::mutex file_mutex_;

  /**
   * @brief 更新累计角度
   * @param current_yaw 当前航向角
   * @return 是否完成圆形路径
   */
  bool updateAccumulatedAngle(double current_yaw);
};

}  // namespace follow_controller
}  // namespace xline
