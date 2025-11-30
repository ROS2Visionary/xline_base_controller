/**
 * @file inkjet_client_node.cpp
 * @brief 喷墨打印机 ROS 2 客户端节点实现
 */

#include "xline_base_controller/inkjet_client.hpp"
#include <chrono>

using namespace std::chrono_literals;
namespace xline
{
  namespace base_controller
  {
InkjetClient::InkjetClient() : Node("inkjet_client_node") {
    RCLCPP_INFO(this->get_logger(), "喷墨打印机客户端节点初始化中...");

    // 创建通用服务客户端
    send_command_client_ = this->create_client<xline_msgs::srv::PrinterCommand>(
        "printer/send_command"
    );

    // 创建快速命令服务客户端
    quick_command_client_ = this->create_client<xline_msgs::srv::QuickCommand>(
        "printer/quick_command"
    );

    // 创建文本设置服务客户端
    set_text_client_ = this->create_client<xline_msgs::srv::SetText>(
        "printer/set_text"
    );

    // 创建线段设置服务客户端
    set_line_client_ = this->create_client<xline_msgs::srv::SetLine>(
        "printer/set_line"
    );

    RCLCPP_INFO(this->get_logger(), "所有服务客户端已创建");
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "喷墨打印机客户端节点已启动");
    RCLCPP_INFO(this->get_logger(), "可用服务:");
    RCLCPP_INFO(this->get_logger(), "  - printer/send_command (通用命令)");
    RCLCPP_INFO(this->get_logger(), "  - printer/quick_command (快速命令)");
    RCLCPP_INFO(this->get_logger(), "============================================================");
}

bool InkjetClient::wait_for_all_services(std::chrono::seconds timeout) {
    RCLCPP_INFO(this->get_logger(), "等待服务可用（超时: %ld秒）...", timeout.count());

    struct ServiceInfo {
        rclcpp::ClientBase::SharedPtr client;
        std::string name;
    };

    std::vector<ServiceInfo> services = {
        {send_command_client_, "printer/send_command"},
        {quick_command_client_, "printer/quick_command"},
        {set_text_client_, "printer/set_text"},
        {set_line_client_, "printer/set_line"}
    };

    bool all_ready = true;
    for (const auto& service : services) {
        if (service.client->wait_for_service(timeout)) {
            RCLCPP_INFO(this->get_logger(), "  ✓ %s 可用", service.name.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "  ✗ %s 不可用", service.name.c_str());
            all_ready = false;
        }
    }

    return all_ready;
}

// ========== 通用命令服务方法 ==========

std::tuple<bool, std::string> InkjetClient::send_command(
    const std::string& printer_name,
    const std::string& json_data,
    std::chrono::seconds timeout
) {
    if (!send_command_client_->wait_for_service(1s)) {
        std::string msg = "printer/send_command 服务不可用";
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }

    auto request = std::make_shared<xline_msgs::srv::PrinterCommand::Request>();
    request->printer_name = printer_name;
    request->json_data = json_data;

    try {
        RCLCPP_INFO(this->get_logger(), "发送命令: printer=%s, json=%s", 
                    printer_name.c_str(), json_data.c_str());
        
        auto future = send_command_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "命令成功: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "命令失败: %s", response->message.c_str());
            }
            return {response->success, response->message};
        } else {
            std::string msg = "服务调用超时";
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            return {false, msg};
        }
    } catch (const std::exception& e) {
        std::string msg = std::string("发送命令异常: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }
}

// ========== 快速命令服务方法 ==========

std::tuple<bool, std::string> InkjetClient::quick_command(
    const std::string& printer_name,
    const std::string& action,
    int param,
    std::chrono::seconds timeout
) {
    if (!quick_command_client_->wait_for_service(1s)) {
        std::string msg = "printer/quick_command 服务不可用";
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }

    auto request = std::make_shared<xline_msgs::srv::QuickCommand::Request>();
    request->printer_name = printer_name;
    request->action = action;
    request->param = param;

    try {
        RCLCPP_INFO(this->get_logger(), "快速命令: printer=%s, action=%s, param=%d",
                    printer_name.c_str(), action.c_str(), param);
        
        auto future = quick_command_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "快速命令成功: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "快速命令失败: %s", response->message.c_str());
            }
            return {response->success, response->message};
        } else {
            std::string msg = "服务调用超时";
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            return {false, msg};
        }
    } catch (const std::exception& e) {
        std::string msg = std::string("快速命令异常: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }
}

// ========== 文本设置服务方法 ==========

std::tuple<bool, std::string> InkjetClient::set_text(
    const std::string& printer_name,
    const std::string& text,
    std::chrono::seconds timeout
) {
    if (!set_text_client_->wait_for_service(1s)) {
        std::string msg = "printer/set_text 服务不可用";
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }

    auto request = std::make_shared<xline_msgs::srv::SetText::Request>();
    request->printer_name = printer_name;
    request->text = text;

    try {
        RCLCPP_INFO(this->get_logger(), "设置文本: printer=%s, text=%s",
                    printer_name.c_str(), text.c_str());

        auto future = set_text_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "设置文本成功: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "设置文本失败: %s", response->message.c_str());
            }
            return {response->success, response->message};
        } else {
            std::string msg = "服务调用超时";
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            return {false, msg};
        }
    } catch (const std::exception& e) {
        std::string msg = std::string("设置文本异常: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }
}

// ========== 线段设置服务方法 ==========

std::tuple<bool, std::string> InkjetClient::set_line(
    const std::string& printer_name,
    int height,
    int width,
    int x,
    int y,
    std::chrono::seconds timeout
) {
    if (!set_line_client_->wait_for_service(1s)) {
        std::string msg = "printer/set_line 服务不可用";
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }

    auto request = std::make_shared<xline_msgs::srv::SetLine::Request>();
    request->printer_name = printer_name;
    request->height = height;
    request->width = width;
    request->x = x;
    request->y = y;

    try {
        RCLCPP_INFO(this->get_logger(),
                    "设置线段: printer=%s, height=%d, width=%d, x=%d, y=%d",
                    printer_name.c_str(), height, width, x, y);

        auto future = set_line_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "设置线段成功: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "设置线段失败: %s", response->message.c_str());
            }
            return {response->success, response->message};
        } else {
            std::string msg = "服务调用超时";
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            return {false, msg};
        }
    } catch (const std::exception& e) {
        std::string msg = std::string("设置线段异常: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }
}

// ========== 便捷方法 - 蜂鸣 ==========

std::tuple<bool, std::string> InkjetClient::beep(
    const std::string& printer_name, int times, std::chrono::seconds timeout
) {
    return quick_command(printer_name, "beep", times, timeout);
}

// ========== 便捷方法 - 打印控制 ==========

std::tuple<bool, std::string> InkjetClient::start_print(
    const std::string& printer_name, std::chrono::seconds timeout
) {
    return quick_command(printer_name, "start_print", 0, timeout);
}

std::tuple<bool, std::string> InkjetClient::stop_print(
    const std::string& printer_name, std::chrono::seconds timeout
) {
    return quick_command(printer_name, "stop_print", 0, timeout);
}


  } // namespace base_controller
} // namespace xline
