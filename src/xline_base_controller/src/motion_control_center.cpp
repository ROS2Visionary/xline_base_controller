#include "xline_base_controller/motion_control_center.hpp"

#include <cmath>
#include <sstream>
#include <algorithm>

// 为 std::bind 预留占位符
using std::placeholders::_1;
using std::placeholders::_2;

namespace xline
{
namespace base_controller
{

/**
 * 构造函数
 * - 设置节点名称为 "motion_control_center"
 * - 创建 ExecutePlan 动作服务器，动作名为 "execute_plan"
 */
MotionControlCenter::MotionControlCenter(const rclcpp::NodeOptions & options)
  : rclcpp::Node("motion_control_center", options)
{
  // 创建 ExecutePlan 动作服务器：分别绑定 目标处理/取消处理/接受处理 回调
  action_server_ = rclcpp_action::create_server<ExecutePlan>(
      this,                                  // 节点指针
      "execute_plan",                       // 动作名
      std::bind(&MotionControlCenter::handleGoal, this, _1, _2),
      std::bind(&MotionControlCenter::handleCancel, this, _1),
      std::bind(&MotionControlCenter::handleAccepted, this, _1));

  // 创建位姿订阅器(订阅状态估计器发布的融合位姿)
  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robot_pose",                    // 话题名
      10,                                   // QoS 队列大小
      std::bind(&MotionControlCenter::poseCallback, this, _1));

  // 创建 cmd_vel 发布器(用于校准时控制机器人移动)
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",                           // 话题名
      10);                                  // QoS 队列大小

  // 创建定位校准服务客户端
  calibration_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/localization/calibrate_pose");      // 服务名

  RCLCPP_INFO(get_logger(), "MotionControlCenter 动作服务器已就绪: 'execute_plan'");
  RCLCPP_INFO(get_logger(), "位姿订阅器已创建: '/robot_pose'");
  RCLCPP_INFO(get_logger(), "cmd_vel 发布器已创建: '/cmd_vel'");
  RCLCPP_INFO(get_logger(), "校准服务客户端已创建: '/localization/calibrate_pose'");
  line_follow_controller_ = std::make_shared<xline::follow_controller::LineFollowController>();
  rpp_follow_controller_ = std::make_shared<xline::follow_controller::RPPController>();
  base_follow_controller_ = nullptr;
}

MotionControlCenter::~MotionControlCenter() = default;

/**
 * 目标处理回调：
 * - 检查是否有正在执行的action
 * - 基础合法性检查（plan_uid / plan_json 不能为空）
 * - 通过则返回 ACCEPT_AND_EXECUTE，进入执行流程
 */
rclcpp_action::GoalResponse
MotionControlCenter::handleGoal(const rclcpp_action::GoalUUID & uuid,
                                std::shared_ptr<const ExecutePlan::Goal> goal)
{
  (void)uuid;

  // 检查是否有action正在执行
  if (is_executing_.load()) {
    RCLCPP_WARN(get_logger(), "拒绝目标：当前有action正在执行中");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // 基础校验：goal 不能为空
  if (!goal) {
    RCLCPP_WARN(get_logger(), "拒绝目标：收到空的 goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // 内容校验：plan_uid 与 plan_json 不能为空
  if (goal->plan_uid.empty() || goal->plan_json.empty()) {
    RCLCPP_WARN(get_logger(), "拒绝目标：plan_uid 或 plan_json 为空");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "接受目标：plan_uid=%s, json长度=%zu",
              goal->plan_uid.c_str(), goal->plan_json.size());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * 取消处理回调：
 * - 当前示例中无条件接受取消请求
 */
rclcpp_action::CancelResponse
MotionControlCenter::handleCancel(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "收到取消请求，接受取消");
  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * 接受目标回调：
 * - 为避免阻塞 executor，启动独立线程执行 execute()
 */
void MotionControlCenter::handleAccepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  std::thread{std::bind(&MotionControlCenter::execute, this, goal_handle)}.detach();
}

/**
 * 执行主循环：
 * - 发布初始反馈（IDLE）
 * - 模拟 10 个 order 的执行，每 20Hz 发布一次反馈
 * - 支持客户端取消：若收到取消则返回 canceled 结果
 * - 正常完成后返回 success=true 的结果
 *
 * 注意：此处为示例/占位实现。真实系统应：
 * 1) 解析 goal->plan_json
 * 2) 按照计划调度底层控制器（路径跟随、速度/姿态控制等）
 * 3) 将实时状态转换为反馈并发布
 */
void MotionControlCenter::execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  // 设置执行标志
  is_executing_.store(true);
  // 使用RAII确保函数退出时清除标志
  auto cleanup = [this](void*) { is_executing_.store(false); };
  std::unique_ptr<void, decltype(cleanup)> guard(reinterpret_cast<void*>(1), cleanup);

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExecutePlan::Feedback>();
  auto result = std::make_shared<ExecutePlan::Result>();

  // 初始反馈：空闲状态
  feedback->current_order = 0;
  feedback->status = ExecutePlan::Feedback::STATUS_IDLE;
  goal_handle->publish_feedback(feedback);

  // 解析JSON字符串
  Json::CharReaderBuilder reader_builder;
  Json::Value root;
  std::string parse_errors;
  std::istringstream json_stream(goal->plan_json);

  if (!Json::parseFromStream(reader_builder, json_stream, &root, &parse_errors)) {
    result->success = false;
    result->error_message = "JSON解析失败: " + parse_errors;
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "JSON解析失败：%s", parse_errors.c_str());
    return;
  }

  // 提取lines数组
  if (!root.isMember("lines") || !root["lines"].isArray()) {
    result->success = false;
    result->error_message = "JSON中缺少lines数组";
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "JSON中缺少lines数组");
    return;
  }

  const Json::Value & lines = root["lines"];
  const uint32_t total_orders = lines.size();

  RCLCPP_INFO(get_logger(), "开始执行计划，共 %u 个路径元素", total_orders);

  // 遍历每个路径元素
  rclcpp::Rate loop_rate(20);  // 20Hz 发布反馈
  for (uint32_t order = 0; order < total_orders; ++order) {
    // 支持取消：收到取消则返回 canceled
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->error_message = "客户端取消执行";
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "目标已取消：plan_uid=%s", goal->plan_uid.c_str());
      return;
    }

    const Json::Value & line_element = lines[static_cast<int>(order)];
    std::string type = line_element["type"].asString();

    // 根据类型提取数据
    if (type == "line") {
      LineData line_data = extractLineData(line_element);
      RCLCPP_INFO(get_logger(),
                  "第 %u/%u 个元素[line]: 起点(%.2f, %.2f) -> 终点(%.2f, %.2f)",
                  order + 1, total_orders,
                  line_data.start_x, line_data.start_y,
                  line_data.end_x, line_data.end_y);
      // TODO: 使用line_data进行路径跟踪

    } else if (type == "circle") {
      CircleData circle_data = extractCircleData(line_element);
      RCLCPP_INFO(get_logger(),
                  "第 %u/%u 个元素[circle]: 圆心(%.2f, %.2f), 半径%.2f",
                  order + 1, total_orders,
                  circle_data.center_x, circle_data.center_y, circle_data.radius);
      // TODO: 使用circle_data进行路径跟踪

    } else if (type == "arc") {
      ArcData arc_data = extractArcData(line_element);
      RCLCPP_INFO(get_logger(),
                  "第 %u/%u 个元素[arc]: 圆心(%.2f, %.2f), 半径%.2f, 角度[%.2f, %.2f] rad",
                  order + 1, total_orders,
                  arc_data.center_x, arc_data.center_y, arc_data.radius,
                  arc_data.start_angle, arc_data.end_angle);
      // TODO: 使用arc_data进行路径跟踪

    } else {
      RCLCPP_WARN(get_logger(), "第 %u/%u 个元素: 未知类型 %s，跳过",
                  order + 1, total_orders, type.c_str());
    }

    // 发布反馈
    feedback->current_order = order + 1;
    feedback->status = ExecutePlan::Feedback::STATUS_EXECUTING;
    goal_handle->publish_feedback(feedback);

    // 模拟执行时间(实际应该等待路径跟踪完成)
    loop_rate.sleep();
  }

  // 执行完成：返回成功
  result->success = true;
  result->error_message = "";
  goal_handle->succeed(result);
  RCLCPP_INFO(get_logger(), "目标执行完成：plan_uid=%s", goal->plan_uid.c_str());
}

/**
 * 提取line数据
 * line包含: start{x,y}, end{x,y}
 */
MotionControlCenter::LineData MotionControlCenter::extractLineData(const Json::Value & line_obj)
{
  LineData data;
  data.start_x = line_obj["start"]["x"].asDouble();
  data.start_y = line_obj["start"]["y"].asDouble();
  data.end_x = line_obj["end"]["x"].asDouble();
  data.end_y = line_obj["end"]["y"].asDouble();

  RCLCPP_DEBUG(get_logger(),
               "提取Line数据: 起点(%.2f, %.2f) -> 终点(%.2f, %.2f)",
               data.start_x, data.start_y, data.end_x, data.end_y);
  return data;
}

/**
 * 提取circle数据
 * circle包含: start{x,y}作为圆心, radius
 */
MotionControlCenter::CircleData MotionControlCenter::extractCircleData(const Json::Value & circle_obj)
{
  CircleData data;
  data.center_x = circle_obj["start"]["x"].asDouble();
  data.center_y = circle_obj["start"]["y"].asDouble();
  data.radius = circle_obj["radius"].asDouble();

  RCLCPP_DEBUG(get_logger(),
               "提取Circle数据: 圆心(%.2f, %.2f), 半径%.2f",
               data.center_x, data.center_y, data.radius);
  return data;
}

/**
 * 提取arc数据
 * arc包含: center{x,y}, radius, start_angle, end_angle
 * 注意: JSON中角度为度,转换为弧度
 */
MotionControlCenter::ArcData MotionControlCenter::extractArcData(const Json::Value & arc_obj)
{
  ArcData data;
  data.center_x = arc_obj["center"]["x"].asDouble();
  data.center_y = arc_obj["center"]["y"].asDouble();
  data.radius = arc_obj["radius"].asDouble();

  // 度转弧度
  data.start_angle = arc_obj["start_angle"].asDouble() * M_PI / 180.0;
  data.end_angle = arc_obj["end_angle"].asDouble() * M_PI / 180.0;

  RCLCPP_DEBUG(get_logger(),
               "提取Arc数据: 圆心(%.2f, %.2f), 半径%.2f, 起始角%.2f rad, 结束角%.2f rad",
               data.center_x, data.center_y, data.radius, data.start_angle, data.end_angle);
  return data;
}

/**
 * 位姿数据回调函数
 * - 接收状态估计器融合后的位姿信息
 */
void MotionControlCenter::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(),
               "位姿数据 - 位置: [%.3f, %.3f, %.3f], 姿态: [%.3f, %.3f, %.3f, %.3f]",
               msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
               msg->pose.orientation.w, msg->pose.orientation.x,
               msg->pose.orientation.y, msg->pose.orientation.z);

  // TODO: 在此处添加位姿数据处理逻辑
  // 例如：更新控制器状态、路径跟踪计算等
}

/**
 * 执行定位系统校准
 * - 等待校准服务可用
 * - 异步调用校准服务
 * - 控制机器人沿直线前进
 * - 等待服务完成并返回结果
 */
bool MotionControlCenter::executeLocalizationCalibration(double linear_velocity, double duration)
{
  RCLCPP_INFO(get_logger(), "开始定位系统校准流程...");
  RCLCPP_INFO(get_logger(), "校准参数: 速度=%.2f m/s, 持续时间=%.1f秒", linear_velocity, duration);

  // 1. 等待校准服务可用
  RCLCPP_INFO(get_logger(), "等待校准服务 '/localization/calibrate_pose' 可用...");
  if (!calibration_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "校准服务不可用，请确保 localization 节点已启动");
    return false;
  }
  RCLCPP_INFO(get_logger(), "校准服务已就绪");

  // 2. 异步调用校准服务
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = calibration_client_->async_send_request(request);

  // 等待一小段时间，确保服务开始处理
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 3. 控制机器人沿直线前进
  RCLCPP_INFO(get_logger(), "控制机器人前进 %.1f 秒...", duration);
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = linear_velocity;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;

  // 以 20Hz 频率发布 cmd_vel
  rclcpp::Rate loop_rate(20);
  auto start_time = this->now();
  auto target_duration = rclcpp::Duration::from_seconds(duration);

  while ((this->now() - start_time) < target_duration) {
    cmd_vel_publisher_->publish(twist_msg);
    loop_rate.sleep();
  }

  // 4. 停止机器人
  RCLCPP_INFO(get_logger(), "停止机器人移动");
  twist_msg.linear.x = 0.0;
  cmd_vel_publisher_->publish(twist_msg);

  // 5. 等待校准服务返回结果
  RCLCPP_INFO(get_logger(), "等待校准服务完成...");

  // 等待最多5秒
  auto wait_status = future.wait_for(std::chrono::seconds(5));
  if (wait_status != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "校准服务超时");
    return false;
  }

  // 获取结果
  auto response = future.get();
  if (response->success) {
    RCLCPP_INFO(get_logger(), "校准成功: %s", response->message.c_str());
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "校准失败: %s", response->message.c_str());
    return false;
  }
}

}  // namespace base_controller
}  // namespace xline





