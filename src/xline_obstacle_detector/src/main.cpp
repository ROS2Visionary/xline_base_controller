#include "xline_obstacle_detector/obstacle_detector_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<xline_obstacle_detector::ObstacleDetectorNode>();
        
        RCLCPP_INFO(rclcpp::get_logger("main"), 
                   "障碍物检测节点启动成功，开始处理数据...");
        
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), 
                    "节点启动失败: %s", e.what());
        return 1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "障碍物检测节点已关闭");
    rclcpp::shutdown();
    return 0;
}