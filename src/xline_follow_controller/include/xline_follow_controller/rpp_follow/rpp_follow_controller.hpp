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

#include "xline_follow_controller/common/yaml_parser.hpp"
#include "xline_follow_controller/common/base_follow_controller.hpp"
#include "xline_follow_controller/common/follow_common.hpp"
#include "xline_follow_controller/rpp_follow/path_strategy.hpp"
#include "xline_follow_controller/rpp_follow/circle_path_strategy.hpp"
#include "xline_follow_controller/rpp_follow/curve_path_strategy.hpp"
#include "xline_follow_controller/rpp_follow/rpp_params.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>
#include <fstream>
#include <filesystem>
#include <memory>
#include <deque>

namespace xline
{
namespace follow_controller
{

/**
 * @brief Regulated Pure Pursuit 主控制器
 *
 * 这个类在整体上负责：
 * - 作为 BaseFollowController 的一个具体实现，对外提供 setPlan / computeVelocityCommands 等统一接口；
 * - 持有一套结构化的参数 RPPParams，并从 YAML 中加载/更新这些参数；
 * - 基于 PathStrategy 抽象，在“曲线策略”和“圆形策略”之间切换；
 * - 完成路径裁剪、前瞻点选择、速度约束（曲率/接近目标）以及角速度平滑等一整套 PP 控制流程；
 * - 可选地维护一张栅格图，用于路径、机器人轨迹和前瞻点的可视化调试。
 *
 * 头文件主要声明接口和成员变量，具体实现细节可参考 rpp_follow_controller.cpp。
 */
class RPPController : public BaseFollowController
{
public:

  // 构造/析构：只做基本成员初始化，详细初始化由 initialize() 完成。
  RPPController();
  ~RPPController();

  // 公共接口
  /// 初始化内部状态，只需要调一次。
  void initialize();

  /// 从 YAML 文件加载/更新所有参数。
  void updateParameters(std::string file_path);

  /// 设置需要走过的圆弧角度范围（弧度）。
  void setAngleRange(double start_angle, double end_angle);

  /// 构造一条"切入直线 + 圆弧"的轨迹。
  bool setPlanForCircle(double circle_center_x, double circle_center_y, double circle_radius,
                        const geometry_msgs::msg::PoseStamped& robot_pose);

  /// 设置全局路径（曲线路径）。
  bool setPlan(const nav_msgs::msg::Path& orig_global_plan);

  /// 是否已经满足终点与角度阈值。
  bool isGoalReached();

  /// 主入口：根据当前位置和速度给出一帧的 cmd_vel。
  bool computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                               const geometry_msgs::msg::Twist& velocity,
                               geometry_msgs::msg::TwistStamped& cmd_vel);

  // ================================
  // 纯几何/约束相关的工具接口
  // ================================

  /// 根据当前速度算前瞻距离。
  double getLookAheadDistance(double speed);

  /// 控制是否启用后退跟随。
  void setBackFollow(bool back);

  /// 获取当前策略类型名称
  std::string getCurrentStrategyType() const;

  /// 获取当前参数（只读）
  const RPPParams& getParams() const { return params_; }


  /// 从全局路径中裁掉已走过的部分，得到局部路径。
  void pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& current_pose,
                       const nav_msgs::msg::Path& global_plan,
                       std::vector<geometry_msgs::msg::PoseStamped>& pruned_plan);

  /// 在当前路径上寻找前瞻点，必要时做插值。
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
      const double& lookahead_dist,
      const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
      bool interpolate_after_goal);


  /// 根据曲率减小线速度。
  double applyCurvatureConstraint(const double raw_linear_vel, const double curvature);

  /// 接近目标时进一步拉低速度。
  double applyApproachConstraint(const double raw_linear_vel,
                                 geometry_msgs::msg::PoseStamped robot_pose_global,
                                 const std::vector<geometry_msgs::msg::PoseStamped>& prune_plan);

  /// 对线速度做简单的限幅与渐变。
  double linearRegularization(double current_velocity, double desired_velocity);

  /// 对角速度做简单的限幅与渐变。
  double angularRegularization(double current_angular_vel, double desired_angular_vel);


  // ================================
  // 航向对齐与角度相关工具
  // ================================

  /// 当前姿态和路径切线的夹角是否需要先单独旋转对齐。
  bool shouldRotateToPath(double angle_to_path, double tolerance);

  /// 将角度规范化到 [-π, π]。
  double regularizeAngle(double angle);

  /// 是否需要原地旋转对齐终点朝向。
  bool shouldRotateToGoal(const geometry_msgs::msg::PoseStamped& current_pose,
                          const geometry_msgs::msg::PoseStamped& goal_pose);

  /// 航向预对准逻辑，常见于圆轨迹切入前。
  bool performYawPrealignment(const geometry_msgs::msg::PoseStamped& current_pose,
                              double target_yaw,
                              geometry_msgs::msg::TwistStamped& cmd_vel);

  /// 根据角度误差计算一个合适的旋转速度。
  double calculateRotationVelocity(const double& angle_diff);

protected:
  // ================================
  // 几何工具与误差计算（供子类或扩展使用）
  // ================================

  /// 计算机器人当前朝向与前瞻点连线之间的角度差。
  double dphi(geometry_msgs::msg::PointStamped lookahead_pt,
              geometry_msgs::msg::PoseStamped robot_pose_global);

private:


  PathStrategy::UniquePtr path_strategy_;        ///< 当前路径策略
  PathStrategyType current_strategy_type_;       ///< 当前策略类型

  // 切换路径策略（在曲线策略和圆形策略之间切换）
  void switchStrategy(PathStrategyType type);

  RPPParams params_;           ///< 控制器参数（结构体形式）

  // 状态标志
  bool initialized_;          ///< 控制器是否已初始化
  bool goal_reached_;         ///< 是否已到达目标
  bool waiting_;              ///< 是否在等待状态
  bool back_follow_;          ///< 是否后退模式

  // 航向预对准相关
  bool need_yaw_prealign_;    ///< 是否需要航向预对准
  bool yaw_prealign_done_;    ///< 航向预对准是否完成
  double target_yaw_;         ///< 目标航向角

  // 控制参数（运行时缓存）
  double d_t_;                   ///< 控制时间间隔

  // 目标参数
  double goal_x_, goal_y_, goal_theta_;  ///< 目标位置和角度

  // 路径与距离信息
  nav_msgs::msg::Path global_plan_;   ///< 全局路径
  double path_length_;                ///< 路径总长度
  double traversed_distance_;         ///< 已行驶距离
  double remaining_distance_;         ///< 剩余距离

  // 误差统计
  double max_error_;           ///< 最大横向误差
  double avg_error_;           ///< 平均横向误差
  double error_sum_;           ///< 误差累加
  int error_count_;            ///< 误差计数
  double current_lateral_error_;  ///< 当前横向误差
  double current_curvature_;      ///< 当前曲率

  // 当前状态缓存
  geometry_msgs::msg::Twist current_velocity_;   ///< 当前速度
  double desired_velocity_;                       ///< 期望速度

  // 滤波器（位置与角速度，具体参数来自 RPPParams）
  SavitzkyGolayFilter sg_x_filter_ = SavitzkyGolayFilter(7, 2);
  SavitzkyGolayFilter sg_y_filter_ = SavitzkyGolayFilter(7, 2);
  HampelFilter h_x_filter = HampelFilter(5, 3.0);
  HampelFilter h_y_filter = HampelFilter(5, 3.0);

  FourthOrderLowpassFilter pos_x_filter_;
  FourthOrderLowpassFilter pos_y_filter_;
  FourthOrderLowpassFilter angle_vel_filter_;

  // 角速度平滑状态
  double previous_angular_vel_;             ///< 上次角速度
  double predicted_angular_vel_;            ///< 预测角速度
  std::deque<double> angular_vel_history_;  ///< 角速度历史
  double angle_to_path_prev_;               ///< 上次路径角度差
  double lookahead_dist_prev_;              ///< 上次前瞻距离

  // 二阶平滑器
  SecondOrderSmoother second_order_filter_;

  // 其他状态
  size_t last_closest_idx;

  // 栅格图相关
  double grid_resolution_;
  double grid_width_;
  double grid_height_;
  double grid_origin_x_;
  double grid_origin_y_;
  cv::Mat grid_map_;
  rclcpp::Time last_grid_update_time_;

  // 从YAML解析器加载参数到结构体
  // 注意：这里只做“参数 -> 结构体”的映射，不涉及运行时状态重置。
  void loadParamsFromYaml(const xline::YamlParser::YamlParser& parser);

  // 状态重置
  void resetControllerState();

  // 路径处理
  nav_msgs::msg::Path smoothAndSubdividePath(const nav_msgs::msg::Path& orig_path);
  bool validateAndCleanPath();
  void calculatePathInfo();

  // 速度计算与目标检查
  void initializeCommandVel(geometry_msgs::msg::TwistStamped& cmd_vel);
  geometry_msgs::msg::PoseStamped filterRobotPose(const geometry_msgs::msg::PoseStamped& robot_pose);
  geometry_msgs::msg::PoseStamped adjustPoseForBackward(const geometry_msgs::msg::PoseStamped& pose);
  bool isValidPose(const geometry_msgs::msg::PoseStamped& pose);
  bool checkGoalReached(const geometry_msgs::msg::PoseStamped& current_pose,
                        geometry_msgs::msg::TwistStamped& cmd_vel);
  void setGoalReachedState(geometry_msgs::msg::TwistStamped& cmd_vel);
  geometry_msgs::msg::PoseStamped getFallbackLookaheadPoint(
      const std::vector<geometry_msgs::msg::PoseStamped>& pruned_plan);
  double computeAngleToLookahead(const geometry_msgs::msg::PoseStamped& lookahead_pose,
                                  const geometry_msgs::msg::PoseStamped& current_pose);
  double computeCurvature(double angle_to_lookahead, double lookahead_distance);
  double computeLateralError(double angle_to_lookahead, double lookahead_distance);
  void updateErrorStatistics();
  double computeDesiredAngularVelocity(double desired_velocity, double curvature,
                                        double current_angular_vel, double lookahead_distance,
                                        double angle_to_lookahead, double dt, bool& filter_reset);
  /// 根据期望线速度/角速度和当前状态，生成最终的 cmd_vel 消息
  void generateVelocityCommand(geometry_msgs::msg::TwistStamped& cmd_vel,
                                const geometry_msgs::msg::Twist& current_velocity,
                                double desired_velocity, double desired_angular_velocity);

  // 角速度平滑：在期望角速度的基础上做一层动态滤波
  double smoothAngularVelocity(double current_angular_vel, double desired_angular_vel,
                               double lookahead_dist, double angle_to_path, double dt, bool is_reset);

  // 栅格图
  void initializeGridMap(const nav_msgs::msg::Path& path);
  cv::Point worldToGrid(double x, double y);
  void drawPathOnGrid(const nav_msgs::msg::Path& path, const cv::Scalar& color, int thickness);
  void drawRobotOnGrid(const geometry_msgs::msg::PoseStamped& pose);
  void drawLookaheadPointOnGrid(const geometry_msgs::msg::Point& lookahead_point);
  void drawGridLines();
  void saveGridMap();
  void updateGridMapIfNeeded(const geometry_msgs::msg::PoseStamped& current_pose,
                              const geometry_msgs::msg::PoseStamped& lookahead_pose);
  bool isPointInGrid(const cv::Point& pt);

  // 性能监控
  void checkComputationTime(const rclcpp::Time& start_time);
};

}  // namespace follow_controller
}  // namespace xline
