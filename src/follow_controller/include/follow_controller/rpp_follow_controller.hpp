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
#include "follow_controller/yaml_parser.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "daosnrs_common/geometry/base_path.hpp"
#include "follow_controller/base_follow_controller.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <filesystem>
#include "follow_controller/follow_common.hpp"

namespace xline
{
namespace follow_controller
{

class RPPController : public BaseFollowController
{
public:
  /**
   * @brief 构造函数
   */
  RPPController();

  /**
   * @brief 析构函数
   */
  ~RPPController();

  /**
   * @brief 初始化控制器
   */
  void initialize();

  /**
   * @brief 规划沿着圆形路径以左转方式切入的路径
   * @param circle_center_x 圆心x坐标
   * @param circle_center_y 圆心y坐标
   * @param circle_radius 圆半径
   * @param robot_pose 机器人当前位姿
   * @return 成功返回true，失败返回false
   */
  bool setPlanForCircle(double circle_center_x, double circle_center_y, double circle_radius,
                        const geometry_msgs::msg::PoseStamped& robot_pose);

  void setAngleRange(double start_angle, double end_anngle);

  /**
   * @brief 设置需要跟随的路径
   * @param orig_global_plan 全局路径
   * @return 是否成功设置路径
   */
  bool setPlan(const nav_msgs::msg::Path& orig_global_plan);

  /**
   * @brief 设置路径对象
   * @param path_object 路径对象（可以是任何继承自BasePath的类型）
   * @return 是否成功设置路径
   */
  bool setPlanForBasePath(const std::shared_ptr<daosnrs::geometry::BasePath>& path_object);

  /**
   * @brief 检查是否已到达目标
   * @return 是否已到达目标
   */
  bool isGoalReached();

  /**
   * @brief 计算速度命令
   * @param pose 机器人当前位姿
   * @param velocity 机器人当前速度
   * @param cmd_vel 输出的速度命令
   * @return 是否成功计算速度命令
   */
  bool computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
                               geometry_msgs::msg::TwistStamped& cmd_vel);

  /**
   * @brief 根据当前速度计算前瞻距离
   * @param speed 当前速度
   * @return 计算得到的前瞻距离
   */
  double getLookAheadDistance(double speed);

  /**
   * @brief 设置后退跟随模式
   * @param back 是否启用后退模式
   */
  void setBackFollow(bool back);


protected:
  /**
   * @brief 计算机器人与前瞻点之间的角度差
   * @param lookahead_pt 前瞻点
   * @param robot_pose_global 机器人在全局坐标系下的位姿
   * @return 角度差
   */
  double dphi(geometry_msgs::msg::PointStamped lookahead_pt, geometry_msgs::msg::PoseStamped robot_pose_global);

public:
  /**
   * @brief 应用曲率约束限制线速度
   * @param raw_linear_vel 原始线速度
   * @param curvature 路径曲率
   * @return 应用约束后的线速度
   */
  double applyCurvatureConstraint(const double raw_linear_vel, const double curvature);

  /**
   * @brief 应用接近目标约束限制线速度
   * @param raw_linear_vel 原始线速度
   * @param robot_pose_global 机器人在全局坐标系下的位姿
   * @param prune_plan 裁剪后的路径
   * @return 应用约束后的线速度
   */
  double applyApproachConstraint(const double raw_linear_vel, geometry_msgs::msg::PoseStamped robot_pose_global,
                                 const std::vector<geometry_msgs::msg::PoseStamped>& prune_plan);

  /**
   * @brief 获取前瞻点
   * @param lookahead_dist 前瞻距离
   * @param transformed_plan 变换后的路径
   * @param interpolate_after_goal 是否在目标之后插值
   * @return 前瞻点位姿
   */
  geometry_msgs::msg::PoseStamped
  getLookAheadPoint(const double& lookahead_dist, const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
                    bool interpolate_after_goal);

  /**
   * @brief 裁剪全局路径，移除已经通过的点
   * @param current_pose 当前位姿
   * @param global_plan 全局路径
   * @param pruned_plan 输出的裁剪后路径
   */
  void pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& current_pose, const nav_msgs::msg::Path& global_plan,
                       std::vector<geometry_msgs::msg::PoseStamped>& pruned_plan);

  /**
   * @brief 检查是否需要旋转以对齐路径
   * @param angle_to_path 与路径的角度差
   * @param tolerance 容忍的角度阈值
   * @return 是否需要旋转
   */
  bool shouldRotateToPath(double angle_to_path, double tolerance);

  /**
   * @brief 规范化角度到[-π, π]区间
   * @param angle 输入角度
   * @return 规范化后的角度
   */
  double regularizeAngle(double angle);

  /**
   * @brief 检查是否需要旋转以对齐目标
   * @param current_pose 当前位姿
   * @param goal_pose 目标位姿
   * @return 是否需要旋转
   */
  bool shouldRotateToGoal(const geometry_msgs::msg::PoseStamped& current_pose,
                          const geometry_msgs::msg::PoseStamped& goal_pose);

  /**
   * @brief 线速度正则化，限制加速度和速度范围
   * @param current_velocity 当前线速度
   * @param desired_velocity 期望线速度
   * @return 正则化后的线速度命令
   */
  double linearRegularization(double current_velocity, double desired_velocity);

  /**
   * @brief 角速度正则化，限制角加速度和角速度范围
   * @param current_angular_vel 当前角速度
   * @param desired_angular_vel 期望角速度
   * @return 正则化后的角速度命令
   */
  double angularRegularization(double current_angular_vel, double desired_angular_vel);

  /**
   * @brief 更新控制器参数
   */
  void updateParameters(std::string file_path);

  /**
   * @brief 执行航向预对准控制
   * @param current_pose 当前位姿
   * @param target_yaw 目标航向角
   * @param cmd_vel 输出的速度命令
   * @return 如果对准完成返回true，否则返回false
   */
  bool performYawPrealignment(const geometry_msgs::msg::PoseStamped& current_pose, double target_yaw,
                              geometry_msgs::msg::TwistStamped& cmd_vel);

  double calculateRotationVelocity(const double& angle_diff);

  /**
   * @brief 收集机器人位置用于半径计算
   * @param pose 当前机器人位姿
   */
  void collectPositionForRadiusCalculation(const geometry_msgs::msg::PoseStamped& pose);

  /**
   * @brief 收集所有喷墨口的位置
   * @param robot_pose 当前机器人位姿
   */
  void collectOffsetPointsPositions(const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief 将机器人坐标系偏置转换为全局坐标（使用TF变换）
   * @param robot_pose 机器人当前位姿
   * @param x_offset 机器人坐标系x偏置
   * @param y_offset 机器人坐标系y偏置
   * @param global_x 输出全局x坐标
   * @param global_y 输出全局y坐标
   */
  void robotToGlobalCoordinate(const geometry_msgs::msg::PoseStamped& robot_pose, double x_offset, double y_offset,
                               double& global_x, double& global_y);

  /**
   * @brief 使用最小二乘法拟合圆并计算半径
   * @param positions 收集的位置点
   * @param center_x 输出圆心x坐标
   * @param center_y 输出圆心y坐标
   * @param radius 输出半径
   * @return 拟合是否成功
   */
  bool fitCircleToPositions(const std::vector<std::pair<double, double>>& positions, double& center_x, double& center_y,
                            double& radius);

  /**
   * @brief 计算并保存所有喷墨口的半径
   */
  void calculateAndSaveOffsetPointsRadius();

  /**
   * @brief 添加喷墨口偏置点配置
   * @param x_offset 基于机器人坐标系的x偏置（米）
   * @param y_offset 基于机器人坐标系的y偏置（米）
   * @param name 喷墨口名称
   */
  void addOffsetPoint(double x_offset, double y_offset, const std::string& name);

  /**
   * @brief 设置半径日志文件路径
   * @param file_path CSV文件路径
   */
  void setRadiusLogFilePath(const std::string& file_path);

  /**
   * @brief 清空所有喷墨口偏置点配置
   */
  void clearOffsetPoints();

private:
  bool initialized_;   // 标记控制器是否已初始化
  bool goal_reached_;  // 标记是否已达到目标位置

  // 控制器参数
  double d_t_;                   // 控制时间间隔（时间步长）
  double regulated_min_radius_;  // 应用曲率约束的最小半径阈值
  double approach_dist_;         // 用于接近目标的阈值
  double approach_min_v_;        // 接近目标时的最小速度

  // 目标位置参数
  double goal_x_, goal_y_, goal_theta_;  // 目标位置的x、y坐标和偏航角

  // 控制参数
  double goal_dist_tol_;  // 目标距离容忍度
  double rotate_tol_;     // 旋转角度容忍度

  // 前瞻参数
  double lookahead_time_;      // 前瞻时间
  double min_lookahead_dist_;  // 最小前瞻距离
  double max_lookahead_dist_;  // 最大前瞻距离

  // 线速度参数
  double max_v_;         // 最大线速度
  double min_v_;         // 最小线速度
  double max_v_inc_;     // 最大线速度增量
  double linear_speed_;  // 基准线速度
  bool waiting_;         // 是否在等待

  // 角速度参数
  double max_w_;      // 最大角速度
  double min_w_;      // 最小角速度
  double max_w_inc_;  // 最大角速度增量

  // 全局路径
  nav_msgs::msg::Path global_plan_;

  // 路径距离信息
  double path_length_;         // 路径总长度
  double traversed_distance_;  // 已经行驶的距离
  double remaining_distance_;  // 剩余距离

  // 性能统计
  double max_error_;  // 最大横向误差
  double avg_error_;  // 平均横向误差
  double error_sum_;  // 误差累加值
  int error_count_;   // 误差计数

  double current_lateral_error_;  // 当前横向误差
  double current_curvature_;      // 当前曲率

  // 栅格图相关参数
  bool enable_grid_map_;       // 是否启用栅格图功能
  cv::Mat grid_map_;           // 栅格图
  double grid_resolution_;     // 栅格图分辨率(米/像素)
  double grid_width_;          // 栅格图宽度(米)
  double grid_height_;         // 栅格图高度(米)
  double grid_origin_x_;       // 栅格图原点x坐标(米)
  double grid_origin_y_;       // 栅格图原点y坐标(米)
  std::string grid_map_path_;  // 栅格图保存路径

  // 最后一次更新栅格图的时间
  rclcpp::Time last_grid_update_time_;

  // 用于角速度平滑控制
  double previous_angular_vel_;             // 上一个周期的角速度命令
  double predicted_angular_vel_;            // 预测的角速度
  double lowpass_angular_vel_filter_gain_;  // 角速度滤波增益
  std::deque<double> angular_vel_history_;  // 角速度历史记录
  int angular_vel_history_size_;            // 历史记录大小
  double angle_to_path_prev_;               // 上一个周期的路径角度差
  double lookahead_dist_prev_;              // 上一个周期的前瞻距离

  geometry_msgs::msg::Twist current_velocity_;
  double desired_velocity_;

  // 上一次的最近点索引
  size_t last_closest_idx;

  bool is_circle_path;

  SavitzkyGolayFilter sg_x_filter_ = SavitzkyGolayFilter(7, 2);
  SavitzkyGolayFilter sg_y_filter_ = SavitzkyGolayFilter(7, 2);
  HampelFilter h_x_filter = HampelFilter(7, 3.0);
  HampelFilter h_y_filter = HampelFilter(7, 3.0);

  double baseline_angular_velocity_for_circle_;
  double circle_center_x_;        // 圆心x坐标
  double circle_center_y_;        // 圆心y坐标
  double circle_radius_;          // 圆半径
  double target_yaw_;  // 目标航向
  bool need_yaw_prealign_;        // 是否需要预先对准航向
  bool yaw_prealign_done_;        // 是否已完成航向对准

  // 圆形路径半径计算相关
  bool collect_positions_for_circle_;                           // 是否收集位置用于半径计算
  std::vector<std::pair<double, double>> collected_positions_;  // 收集的机器人位置
  std::string radius_log_file_path_;                            // 半径日志文件路径
  double target_radius_;                                        // 目标半径
  double target_center_x_;                                      // 目标圆心x坐标
  double target_center_y_;                                      // 目标圆心y坐标
  rclcpp::Time last_position_collect_time_;                     // 上次收集位置的时间
  double position_collect_interval_;                            // 位置收集间隔(秒)

  // 机器人中心拟合结果
  double robot_center_fitted_x_ = 0.0;
  double robot_center_fitted_y_ = 0.0;
  double robot_center_fitted_radius_ = 0.0;
  bool robot_center_fit_success_ = false;

  double x_offset_;
  double y_offset_;

  // 喷墨口记录相关
  struct OffsetPoint
  {
    double x_offset;                                   // 基于机器人坐标系的x偏置
    double y_offset;                                   // 基于机器人坐标系的y偏置
    std::string name;                                  // 喷墨口名称
    std::vector<std::pair<double, double>> positions;  // 收集的全局坐标位置

    // 拟合结果
    double fitted_center_x = 0.0;  // 拟合圆心x坐标
    double fitted_center_y = 0.0;  // 拟合圆心y坐标
    double fitted_radius = 0.0;    // 拟合半径
    bool fit_success = false;      // 拟合是否成功
  };

  std::vector<OffsetPoint> offset_points_;  // 多个喷墨口配置

  // 用于圆形路径的角度累计
  bool last_yaw_initialized_;
  double last_yaw_;
  double accumulated_angle_;
  int angle_debug_counter_;

  // 圆形路径的切入点坐标
  double circle_entry_x_;
  double circle_entry_y_;

  double radius_offset_;

  // 二阶平滑器相关参数
  /*
  自然频率 (ωn)：控制系统响应速度，值越大响应越快
  阻尼比 (ζ)：控制震荡程度
  ζ < 1: 欠阻尼（有震荡）
  ζ = 1: 临界阻尼（最快无震荡响应）
  ζ > 1: 过阻尼（响应较慢但很平滑）
  */
  double angular_smoother_freq_;     // 自然频率
  double angular_smoother_damping_;  // 阻尼比

  // 二阶平滑器实例
  SecondOrderSmoother second_order_filter_;
  std::string smoothing_type_ = "lowpass";

  FourthOrderLowpassFilter pos_x_filter_;
  FourthOrderLowpassFilter pos_y_filter_;
  FourthOrderLowpassFilter angle_vel_filter_;

  double pos_cutoff_freq;
  double pos_sample_rate;
  double pos_output_limit;
  double pos_rate_limit;
  bool pos_use_biquad_cascade_;
  bool pos_use_biquad_cascade_filter_;
  bool low_speed_mode_;

  double angle_cutoff_freq;
  double angle_sample_rate;
  double angle_output_limit_rate;
  double angle_rate_limit;
  bool angle_use_biquad_cascade_;
  bool angle_use_biquad_cascade_filter_;
  bool angle_use_offset_limit_;
  double angle_output_offset_;

  double circle_start_angle;
  double circle_end_angle;
  double circle_total_angle;

  double start_deviation_factor_;
  double end_deviation_factor_;
  double deviation_rate_;

  // 后退模式相关
  bool back_follow_;  // 是否启用后退模式

  // 新增方法
  /**
   * @brief 初始化栅格图
   * @param path 全局路径
   */
  void initializeGridMap(const nav_msgs::msg::Path& path);

  /**
   * @brief 将全局坐标转换为栅格图坐标
   * @param x 全局x坐标
   * @param y 全局y坐标
   * @return 栅格图坐标(像素)
   */
  cv::Point worldToGrid(double x, double y);

  /**
   * @brief 绘制路径到栅格图
   * @param path 路径
   * @param color 颜色
   * @param thickness 线宽
   */
  void drawPathOnGrid(const nav_msgs::msg::Path& path, const cv::Scalar& color, int thickness);

  /**
   * @brief 绘制机器人位置到栅格图
   * @param pose 机器人当前位姿
   */
  void drawRobotOnGrid(const geometry_msgs::msg::PoseStamped& pose);

  /**
   * @brief 保存栅格图到文件
   */
  void saveGridMap();

  /**
   * @brief 在栅格图上绘制栅格线
   */
  void drawGridLines();

  /**
   * @brief 在栅格图上绘制前瞻点
   * @param lookahead_point 前瞻点坐标
   */
  void drawLookaheadPointOnGrid(const geometry_msgs::msg::Point& lookahead_point);

  /**
   * @brief 平滑角速度命令
   * @param current_angular_vel 当前角速度
   * @param desired_angular_vel 期望角速度
   * @param lookahead_dist 前瞻距离
   * @param angle_to_path 当前与路径的角度差
   * @param dt 控制周期
   * @return 经过平滑后的角速度命令
   */
  double smoothAngularVelocity(double current_angular_vel, double desired_angular_vel, double lookahead_dist,
                               double angle_to_path, double dt, bool is_reset);

  /**
   * @brief 将详细圆形数据保存到CSV文件
   * @param offset_point 喷墨口信息（包含拟合结果）
   */
  void saveDetailedCircleDataToCSV(const OffsetPoint& offset_point);
};

}  // namespace follow_controller
}  // namespace xline