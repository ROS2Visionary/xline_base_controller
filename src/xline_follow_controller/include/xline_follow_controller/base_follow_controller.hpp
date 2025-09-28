#pragma once
#include <tf2_ros/buffer.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>



namespace xline
{

namespace follow_controller
{

class BaseFollowController: public rclcpp::Node
{
private:

public:
  

  BaseFollowController(): rclcpp::Node("base_follow_controller") {}

  virtual ~BaseFollowController() = default;

  virtual bool setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan){
    return false;
  }

  virtual bool setPlan(const nav_msgs::msg::Path& orig_global_plan){
    return false;
  }

  virtual bool computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                       const geometry_msgs::msg::Twist& velocity,
                                       geometry_msgs::msg::TwistStamped& cmd_vel) = 0;

  virtual bool isGoalReached() = 0;

  virtual bool cancel(){}

  virtual void setSpeedLimit(const double& speed_limit){}

protected:
};
}  // namespace follow_controller
}  // namespace xline
