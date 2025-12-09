/**
 * @file rpp_follow_controller.hpp
 * @brief Regulated Pure Pursuit 路径跟随控制器头文件
 *
 * 改进版 Pure Pursuit 路径跟随控制器，对应实现见 rpp_follow_controller.cpp。
 */

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "xline_follow_controller/yaml_parser.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "xline_follow_controller/base_follow_controller.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <filesystem>
#include "xline_follow_controller/follow_common.hpp"

namespace xline
{
namespace follow_controller
{

/**
 * @brief Regulated Pure Pursuit 路径跟随控制器
 *
 * 改进的 Pure Pursuit 算法，支持曲率/接近约束、位置/角速度滤波、圆形路径和后退模式等。
 */
class RPPController : public BaseFollowController
{
public:
  // ================================
  // 构造与析构
  // ================================

  RPPController();
  ~RPPController();

  // ================================
  // 公共接口
  // ================================

  /**
   * @brief 初始化控制器
   */
  void initialize();

  /**
   * @brief 更新控制器参数
   * @param file_path 配置文件路径（相对于包共享目录）
   */
  void updateParameters(std::string file_path);

  /**
   * @brief 设置圆形路径的角度范围
   * @param start_angle 起始角度
   * @param end_angle 终止角度
   */
  void setAngleRange(double start_angle, double end_angle);

  /**
   * @brief 设置圆形路径计划
   * @param circle_center_x 圆心x坐标
   * @param circle_center_y 圆心y坐标
   * @param circle_radius 圆半径
   * @param robot_pose 机器人当前位姿
   * @return 成功返回true
   */
  bool setPlanForCircle(double circle_center_x, double circle_center_y, double circle_radius,
                        const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief 设置路径计划
   * @param orig_global_plan 全局路径
   * @return 成功返回true
   */
  bool setPlan(const nav_msgs::msg::Path& orig_global_plan);

  /**
   * @brief 检查是否到达目标
   * @return 到达目标返回true
   */
  bool isGoalReached();

  /**
   * @brief 计算速度命令
   * @param pose 机器人当前位姿
   * @param velocity 机器人当前速度
   * @param cmd_vel 输出的速度命令
   * @return 成功返回true
   */
  bool computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                               const geometry_msgs::msg::Twist& velocity,
                               geometry_msgs::msg::TwistStamped& cmd_vel);

  /**
   * @brief 获取前瞻距离
   * @param speed 当前速度
   * @return 前瞻距离
   */
  double getLookAheadDistance(double speed);

  /**
   * @brief 设置后退跟随模式
   * @param back 是否启用后退模式
   */
  void setBackFollow(bool back);

  // ================================
  // 路径处理方法
  // ================================

  /**
   * @brief 裁剪全局路径
   * @param current_pose 当前位姿
   * @param global_plan 全局路径
   * @param pruned_plan 裁剪后的路径
   */
  void pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& current_pose,
                       const nav_msgs::msg::Path& global_plan,
                       std::vector<geometry_msgs::msg::PoseStamped>& pruned_plan);

  /**
   * @brief 获取前瞻点
   * @param lookahead_dist 前瞻距离
   * @param transformed_plan 变换后的路径
   * @param interpolate_after_goal 是否在目标后插值
   * @return 前瞻点位姿
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
      const double& lookahead_dist,
      const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
      bool interpolate_after_goal);

  // ================================
  // 速度约束方法
  // ================================

  /**
   * @brief 应用曲率约束
   * @param raw_linear_vel 原始线速度
   * @param curvature 曲率
   * @return 约束后的线速度
   */
  double applyCurvatureConstraint(const double raw_linear_vel, const double curvature);

  /**
   * @brief 应用接近约束
   * @param raw_linear_vel 原始线速度
   * @param robot_pose_global 机器人位姿
   * @param prune_plan 裁剪后的路径
   * @return 约束后的线速度
   */
  double applyApproachConstraint(const double raw_linear_vel,
                                 geometry_msgs::msg::PoseStamped robot_pose_global,
                                 const std::vector<geometry_msgs::msg::PoseStamped>& prune_plan);

  /**
   * @brief 线速度正则化
   * @param current_velocity 当前线速度
   * @param desired_velocity 期望线速度
   * @return 正则化后的线速度
   */
  double linearRegularization(double current_velocity, double desired_velocity);

  /**
   * @brief 角速度正则化
   * @param current_angular_vel 当前角速度
   * @param desired_angular_vel 期望角速度
   * @return 正则化后的角速度
   */
  double angularRegularization(double current_angular_vel, double desired_angular_vel);

  // ================================
  // 辅助计算方法
  // ================================

  /**
   * @brief 检查是否需要旋转以对齐路径
   */
  bool shouldRotateToPath(double angle_to_path, double tolerance);

  /**
   * @brief 规范化角度到[-π, π]
   */
  double regularizeAngle(double angle);

  /**
   * @brief 检查是否需要旋转以对齐目标
   */
  bool shouldRotateToGoal(const geometry_msgs::msg::PoseStamped& current_pose,
                          const geometry_msgs::msg::PoseStamped& goal_pose);

  /**
   * @brief 执行航向预对准
   */
  bool performYawPrealignment(const geometry_msgs::msg::PoseStamped& current_pose,
                              double target_yaw,
                              geometry_msgs::msg::TwistStamped& cmd_vel);

  /**
   * @brief 计算旋转速度
   */
  double calculateRotationVelocity(const double& angle_diff);

protected:
  /**
   * @brief 计算到前瞻点的角度差
   */
  double dphi(geometry_msgs::msg::PointStamped lookahead_pt,
              geometry_msgs::msg::PoseStamped robot_pose_global);

private:
  // ================================
  // 状态标志
  // ================================

  bool initialized_;          ///< 控制器是否已初始化
  bool goal_reached_;         ///< 是否已到达目标
  bool waiting_;              ///< 是否在等待状态
  bool is_circle_path;        ///< 是否为圆形路径
  bool back_follow_;          ///< 是否后退模式

  // ================================
  // 航向预对准相关
  // ================================

  bool need_yaw_prealign_;    ///< 是否需要航向预对准
  bool yaw_prealign_done_;    ///< 航向预对准是否完成
  double target_yaw_;         ///< 目标航向角

  // ================================
  // 圆形路径参数
  // ================================

  double circle_center_x_;    ///< 圆心x坐标
  double circle_center_y_;    ///< 圆心y坐标
  double circle_radius_;      ///< 圆半径
  double circle_entry_x_;     ///< 切入点x坐标
  double circle_entry_y_;     ///< 切入点y坐标
  double circle_start_angle;  ///< 圆形路径起始角度
  double circle_end_angle;    ///< 圆形路径结束角度
  double circle_total_angle;  ///< 圆形路径总角度
  double baseline_angular_velocity_for_circle_;  ///< 圆形路径基准角速度

  // 角度累计相关
  bool last_yaw_initialized_;   ///< 上次航向角是否已初始化
  double last_yaw_;             ///< 上次航向角
  double accumulated_angle_;    ///< 累计角度
  int angle_debug_counter_;     ///< 角度调试计数器

  // ================================
  // 控制参数
  // ================================

  double d_t_;                   ///< 控制时间间隔
  double regulated_min_radius_;  ///< 曲率约束最小半径阈值
  double approach_dist_;         ///< 接近目标阈值
  double approach_min_v_;        ///< 接近目标最小速度

  // 目标参数
  double goal_x_, goal_y_, goal_theta_;  ///< 目标位置和角度
  double goal_dist_tol_;    ///< 目标距离容忍度
  double rotate_tol_;       ///< 旋转角度容忍度

  // 前瞻参数
  double lookahead_time_;       ///< 前瞻时间
  double min_lookahead_dist_;   ///< 最小前瞻距离
  double max_lookahead_dist_;   ///< 最大前瞻距离

  // 线速度参数
  double max_v_;          ///< 最大线速度
  double min_v_;          ///< 最小线速度
  double max_v_inc_;      ///< 最大线速度增量
  double linear_speed_;   ///< 基准线速度

  // 角速度参数
  double max_w_;          ///< 最大角速度
  double min_w_;          ///< 最小角速度
  double max_w_inc_;      ///< 最大角速度增量

  // ================================
  // 路径与距离信息
  // ================================

  nav_msgs::msg::Path global_plan_;   ///< 全局路径
  double path_length_;                ///< 路径总长度
  double traversed_distance_;         ///< 已行驶距离
  double remaining_distance_;         ///< 剩余距离

  // ================================
  // 误差统计
  // ================================

  double max_error_;           ///< 最大横向误差
  double avg_error_;           ///< 平均横向误差
  double error_sum_;           ///< 误差累加
  int error_count_;            ///< 误差计数
  double current_lateral_error_;  ///< 当前横向误差
  double current_curvature_;      ///< 当前曲率

  // ================================
  // 滤波器
  // ================================

  SavitzkyGolayFilter sg_x_filter_ = SavitzkyGolayFilter(7, 2);
  SavitzkyGolayFilter sg_y_filter_ = SavitzkyGolayFilter(7, 2);
  HampelFilter h_x_filter = HampelFilter(7, 3.0);
  HampelFilter h_y_filter = HampelFilter(7, 3.0);

  FourthOrderLowpassFilter pos_x_filter_;
  FourthOrderLowpassFilter pos_y_filter_;
  FourthOrderLowpassFilter angle_vel_filter_;

  // ================================
  // 角速度平滑参数
  // ================================

  double previous_angular_vel_;             ///< 上次角速度
  double predicted_angular_vel_;            ///< 预测角速度
  double lowpass_angular_vel_filter_gain_;  ///< 低通滤波增益
  std::deque<double> angular_vel_history_;  ///< 角速度历史
  int angular_vel_history_size_;            ///< 历史记录大小
  double angle_to_path_prev_;               ///< 上次路径角度差
  double lookahead_dist_prev_;              ///< 上次前瞻距离
  std::string smoothing_type_ = "lowpass";  ///< 平滑类型

  // 二阶平滑器
  double angular_smoother_freq_;      ///< 自然频率
  double angular_smoother_damping_;   ///< 阻尼比
  SecondOrderSmoother second_order_filter_;

  // ================================
  // 滤波参数
  // ================================

  // 位置滤波参数
  double pos_cutoff_freq;
  double pos_sample_rate;
  double pos_output_limit;
  double pos_rate_limit;
  bool pos_use_biquad_cascade_;
  bool pos_use_biquad_cascade_filter_;
  bool low_speed_mode_;

  // 角速度滤波参数
  double angle_cutoff_freq;
  double angle_sample_rate;
  double angle_output_limit_rate;
  double angle_rate_limit;
  bool angle_use_biquad_cascade_;
  bool angle_use_biquad_cascade_filter_;
  bool angle_use_offset_limit_;
  double angle_output_offset_;

  // 圆形路径偏差参数
  double start_deviation_factor_;
  double end_deviation_factor_;
  double deviation_rate_;

  // 其他参数
  double radius_offset_;
  size_t last_closest_idx;

  // 速度状态
  geometry_msgs::msg::Twist current_velocity_;
  double desired_velocity_;

  // ================================
  // 栅格图相关
  // ================================

  bool enable_grid_map_;
  cv::Mat grid_map_;
  double grid_resolution_;
  double grid_width_;
  double grid_height_;
  double grid_origin_x_;
  double grid_origin_y_;
  std::string grid_map_path_;
  rclcpp::Time last_grid_update_time_;

  // ================================
  // 私有辅助方法 - 状态管理
  // ================================

  /**
   * @brief 重置控制器内部状态
   *
   * 清零目标到达标志、误差统计、路径长度等信息，
   * 一般在接收到新的全局路径或重新规划时调用，
   * 使控制器回到干净的初始状态。
   */
  void resetControllerState();

   /**
    * @brief 重置圆形路径相关状态
    *
    * 将累计转角、上一帧航向等与圆形轨迹跟随有关的
    * 中间变量全部恢复到初始值，避免不同圆形路径之间互相干扰。
    */
  void resetCirclePathState();

  // ================================
  // 私有辅助方法 - 路径处理
  // ================================

  /**
   * @brief 根据圆弧半径自适应调整速度参数
   *
   * 半径越小曲率越大，为保证安全与舒适，需要降低最大线速度、
   * 减小前瞻距离，从而提高跟踪精度并减小横向误差。
   *
   * @param radius 圆弧半径（单位：m）
   */
  void adjustSpeedForRadius(double radius);

  /**
   * @brief 根据给定圆心和半径生成圆形路径
   *
   * 该函数会根据当前机器人位姿计算切入点，使机器人先沿直线
   * 过渡到圆周上，然后再在圆周上匀速行驶，生成稠密的轨迹点。
   *
   * @param circle_center_x 圆心在世界坐标系下的 x
   * @param circle_center_y 圆心在世界坐标系下的 y
   * @param circle_radius 圆半径
   * @param robot_pose 当前机器人位姿
   * @return 生成的圆形路径（包含切入段 + 圆周段）
   */
  nav_msgs::msg::Path generateCirclePath(double circle_center_x, double circle_center_y,
                                          double circle_radius,
                                          const geometry_msgs::msg::PoseStamped& start_pose);

  /**
   * @brief 对原始路径进行插值和平滑
   *
   * 通过在相邻路径点之间插入中间点、重建朝向等方式，
   * 生成更加稠密且连续的路径，以减小纯跟踪算法的抖动。
   *
   * @param orig_path 原始全局路径
   * @return 平滑和细分后的路径
   */
  nav_msgs::msg::Path smoothAndSubdividePath(const nav_msgs::msg::Path& orig_path);

  /**
   * @brief 验证并清理路径中的无效点
   *
   * 将包含 NaN/Inf 的路径点剔除，保证后续计算中不会出现
   * 非法数值。如果清理后的路径为空，则返回 false。
   *
   * @return 路径有效返回 true，否则返回 false
   */
  bool validateAndCleanPath();

  /**
   * @brief 计算路径相关的统计信息
   *
   * 主要包括路径总长度、剩余距离、已行驶距离以及目标点
   * 的位置与朝向等，为速度规划和目标判定提供基础数据。
   */
  void calculatePathInfo();

  // ================================
  // 私有辅助方法 - 速度计算
  // ================================

  /**
   * @brief 初始化速度指令消息
   *
   * 将线速度和角速度清零，并填充时间戳与坐标系，
   * 作为后续控制量计算的输出基础。
   *
   * @param cmd_vel 速度指令消息
   */
  void initializeCommandVel(geometry_msgs::msg::TwistStamped& cmd_vel);

  /**
   * @brief 对机器人位姿进行滤波
   *
   * 通过 Hampel 滤波 + Savitzky-Golay 滤波 + 可选的
   * 四阶低通滤波，对位置进行平滑，抑制定位抖动和噪声。
   *
   * @param robot_pose 原始机器人位姿
   * @return 滤波后的机器人位姿
   */
  geometry_msgs::msg::PoseStamped filterRobotPose(const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief 在后退模式下对机器人位姿进行等效变换
   *
   * 将后退跟随等效为前进跟随：通过对航向角加 π 的方式
   * 统一控制逻辑，简化控制器实现。
   *
   * @param pose 原始位姿
   * @return 等效后的位姿（用于控制计算）
   */
  geometry_msgs::msg::PoseStamped adjustPoseForBackward(const geometry_msgs::msg::PoseStamped& pose);

  /**
   * @brief 检查位姿是否合法
   *
   * 检查位置信息和姿态四元数是否为有限数，避免进入数值异常状态。
   *
   * @param pose 待检查的位姿
   * @return 合法返回 true
   */
  bool isValidPose(const geometry_msgs::msg::PoseStamped& pose);

  /**
   * @brief 检查是否到达整体路径目标
   *
   * 根据当前位置与目标点的距离与角度误差判断是否停下，到达后会把速度指令清零。
   *
   * @param current_pose 当前机器人位姿（全局坐标系）
   * @param cmd_vel 输出的速度指令（可能被修改为 0）
   * @return 已到达目标返回 true
   */
  bool checkGoalReached(const geometry_msgs::msg::PoseStamped& current_pose,
                        geometry_msgs::msg::TwistStamped& cmd_vel);

  /**
   * @brief 检查圆形路径跟随是否完成
   *
   * 结合累计转角、总角度和当前位置判断圆弧是否跑完，并设置停止状态。
   *
   * @param current_pose 当前机器人位姿
   * @param cmd_vel 当前速度指令（可能被置零）
   * @return 完成圆形路径返回 true
   */
  bool checkCirclePathGoalReached(const geometry_msgs::msg::PoseStamped& current_pose,
                                  geometry_msgs::msg::TwistStamped& cmd_vel);

  /**
   * @brief 设置目标到达后的控制状态
   *
   * 将 `goal_reached_` 置为 true 并清零输出速度，保证机器人停住。
   *
   * @param cmd_vel 当前速度指令
   */
  void setGoalReachedState(geometry_msgs::msg::TwistStamped& cmd_vel);

  /**
   * @brief 获取备用前瞻点
   *
   * 前瞻插值失败时，从裁剪后的路径里挑一个兜底前瞻点，避免控制发散。
   *
   * @param pruned_plan 裁剪后的路径
   * @return 备用前瞻点位姿
   */
  geometry_msgs::msg::PoseStamped getFallbackLookaheadPoint(
      const std::vector<geometry_msgs::msg::PoseStamped>& pruned_plan);

  /**
   * @brief 计算机器人到前瞻点的角度误差
   *
   * 用当前位姿与前瞻点算期望朝向，与当前航向做差得到角度误差。
   *
   * @param lookahead_pose 前瞻点位姿
   * @param current_pose 当前机器人位姿
   * @return 角度误差（弧度）
   */
  double computeAngleToLookahead(const geometry_msgs::msg::PoseStamped& lookahead_pose,
                                  const geometry_msgs::msg::PoseStamped& current_pose);

  /**
   * @brief 根据角度误差和前瞻距离计算曲率
   *
   * 用 Pure Pursuit 公式把几何关系转成路径曲率，供后续约束使用。
   *
   * @param angle_to_lookahead 机器人到前瞻点的角度误差
   * @param lookahead_distance 前瞻距离
   * @return 路径曲率
   */
  double computeCurvature(double angle_to_lookahead, double lookahead_distance);

  /**
   * @brief 计算当前横向误差
   *
   * 用前瞻点和机器人几何关系近似出相对路径中心线的横向偏移。
   *
   * @param angle_to_lookahead 角度误差
   * @param lookahead_distance 前瞻距离
   * @return 横向误差（单位：m）
   */
  double computeLateralError(double angle_to_lookahead, double lookahead_distance);

  /**
   * @brief 更新横向误差统计量
   *
   * 根据当前横向误差更新 max/avg 等统计，用于日志和调试。
   */
  void updateErrorStatistics();

  /**
   * @brief 计算期望角速度
   *
   * 综合 Pure Pursuit 几何角速度、圆弧约束和角速度平滑，生成控制用目标角速度。
   *
   * @param desired_velocity 期望线速度
   * @param curvature 当前路径曲率
   * @param current_angular_vel 当前角速度
   * @param lookahead_distance 当前前瞻距离
   * @param angle_to_lookahead 当前角度误差
   * @param dt 控制周期
   * @param filter_reset 是否需要重置滤波器状态
   * @return 期望角速度（弧度/秒）
   */
  double computeDesiredAngularVelocity(double desired_velocity, double curvature,
                                        double current_angular_vel, double lookahead_distance,
                                        double angle_to_lookahead, double dt, bool& filter_reset);

  /**
   * @brief 根据期望速度生成最终控制指令
   *
   * 对期望 v/w 做正则化和安全检查后写入 `cmd_vel`，并处理后退模式等附加逻辑。
   *
   * @param cmd_vel 输出的速度指令
   * @param current_velocity 当前机器人速度
   * @param desired_velocity 期望线速度
   * @param desired_angular_velocity 期望角速度
   */
  void generateVelocityCommand(geometry_msgs::msg::TwistStamped& cmd_vel,
                                const geometry_msgs::msg::Twist& current_velocity,
                                double desired_velocity, double desired_angular_velocity);

  // ================================
  // 私有辅助方法 - 角速度平滑
  // ================================

  /**
   * @brief 对角速度进行多级平滑处理
   *
   * 包括低通/滑动平均/二阶平滑及可选四阶低通级联，在响应和抖动之间做平衡。
   *
   * @param current_angular_vel 当前角速度
   * @param desired_angular_vel 理想角速度
   * @param lookahead_dist 当前前瞻距离
   * @param angle_to_path 当前路径方向误差
   * @param dt 控制周期
   * @param is_reset 是否重置内部滤波状态
   * @return 平滑后的角速度
   */
  double smoothAngularVelocity(double current_angular_vel, double desired_angular_vel,
                               double lookahead_dist, double angle_to_path, double dt, bool is_reset);

  // ================================
  // 私有辅助方法 - 栅格图
  // ================================

  /**
   * @brief 使用路径信息初始化栅格图
   *
   * 根据路径计算边界框，创建 OpenCV 图像并画网格和路径，用于离线调试。
   *
   * @param path 当前全局路径
   */
  void initializeGridMap(const nav_msgs::msg::Path& path);

  /**
   * @brief 将世界坐标转换为栅格图像素坐标
   *
   * 根据栅格分辨率和原点，把 (x, y) 映射到图像中的 (u, v)。
   *
   * @param x 世界坐标系下的 x
   * @param y 世界坐标系下的 y
   * @return 对应的图像像素坐标
   */
  cv::Point worldToGrid(double x, double y);

  /**
   * @brief 在栅格图中绘制路径
   *
   * 依次连线路径点，并标记起点/终点，用不同颜色区分。
   *
   * @param path 要绘制的路径
   * @param color 线条颜色
   * @param thickness 线宽
   */
  void drawPathOnGrid(const nav_msgs::msg::Path& path, const cv::Scalar& color, int thickness);

  /**
   * @brief 在栅格图中绘制机器人位置
   *
   * 将机器人当前位姿转换为栅格坐标并标记出来，辅助调试轨迹。
   *
   * @param pose 机器人当前位姿
   */
  void drawRobotOnGrid(const geometry_msgs::msg::PoseStamped& pose);

  /**
   * @brief 在栅格图中绘制前瞻点
   *
   * 将当前前瞻点标记在栅格图上，方便看控制决策。
   *
   * @param lookahead_point 前瞻点在世界坐标系下的位置
   */
  void drawLookaheadPointOnGrid(const geometry_msgs::msg::Point& lookahead_point);

  /**
   * @brief 绘制栅格网格线
   *
   * 按设定分辨率画水平/垂直网格线，提供距离参考。
   */
  void drawGridLines();

  /**
   * @brief 将当前栅格图保存到磁盘
   *
   * 保存为图像文件（如 png），用于离线分析控制效果。
   */
  void saveGridMap();

  /**
   * @brief 按需更新栅格图（位置+前瞻点）
   *
   * 按时间间隔刷新机器人和前瞻点在图中的位置，避免频繁写盘。
   *
   * @param current_pose 当前位姿
   * @param lookahead_pose 当前前瞻点位姿
   */
  void updateGridMapIfNeeded(const geometry_msgs::msg::PoseStamped& current_pose,
                              const geometry_msgs::msg::PoseStamped& lookahead_pose);

  /**
   * @brief 判断像素点是否落在栅格图范围内
   *
   * @param pt 像素坐标
   * @return 在图像内部返回 true
   */
  bool isPointInGrid(const cv::Point& pt);

  // ================================
  // 私有辅助方法 - 性能监控
  // ================================

  /**
   * @brief 检查一次控制循环的计算耗时
   *
   * 若计算时间接近控制周期上限则打印警告，方便看性能瓶颈。
   *
   * @param start_time 本次计算开始时间
   */
  void checkComputationTime(const rclcpp::Time& start_time);
};

}  // namespace follow_controller
}  // namespace xline
