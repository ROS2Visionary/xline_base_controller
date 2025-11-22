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

    // 创建状态查询服务客户端（三个打印机）
    status_left_client_ = this->create_client<std_srvs::srv::Trigger>(
        "printer_left/status"
    );

    status_center_client_ = this->create_client<std_srvs::srv::Trigger>(
        "printer_center/status"
    );

    status_right_client_ = this->create_client<std_srvs::srv::Trigger>(
        "printer_right/status"
    );

    // 创建设置打印机启用状态服务客户端
    set_enabled_client_ = this->create_client<xline_msgs::srv::SetPrinterEnabled>(
        "printer/set_enabled"
    );

    // 创建设置打印机激活状态服务客户端
    set_active_client_ = this->create_client<xline_msgs::srv::SetPrinterActive>(
        "printer/set_active"
    );

    RCLCPP_INFO(this->get_logger(), "所有服务客户端已创建");
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "喷墨打印机客户端节点已启动");
    RCLCPP_INFO(this->get_logger(), "可用服务:");
    RCLCPP_INFO(this->get_logger(), "  - printer/send_command (通用命令)");
    RCLCPP_INFO(this->get_logger(), "  - printer/quick_command (快速命令)");
    RCLCPP_INFO(this->get_logger(), "  - printer/set_enabled (设置打印机自动连接)");
    RCLCPP_INFO(this->get_logger(), "  - printer/set_active (设置打印机激活状态)");
    RCLCPP_INFO(this->get_logger(), "  - printer_left/status (左打印机状态)");
    RCLCPP_INFO(this->get_logger(), "  - printer_center/status (中打印机状态)");
    RCLCPP_INFO(this->get_logger(), "  - printer_right/status (右打印机状态)");
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
        {set_enabled_client_, "printer/set_enabled"},
        {set_active_client_, "printer/set_active"},
        {status_left_client_, "printer_left/status"},
        {status_center_client_, "printer_center/status"},
        {status_right_client_, "printer_right/status"}
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
    const std::string& command,
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
    request->command = command;
    request->json_data = json_data;

    try {
        RCLCPP_INFO(this->get_logger(), "发送命令: printer=%s, cmd=%s", 
                    printer_name.c_str(), command.c_str());
        
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

// ========== 打印模式设置（通过 QuickCommand:set_print_mode） ==========

std::tuple<bool, std::string> InkjetClient::set_print_mode(
    const std::string& printer_name,
    int interval,
    int is_full_end,
    int mode,
    std::chrono::seconds timeout
) {
    if (!quick_command_client_->wait_for_service(1s)) {
        std::string msg = "printer/quick_command 服务不可用";
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }

    auto request = std::make_shared<xline_msgs::srv::QuickCommand::Request>();
    request->printer_name = printer_name;
    request->action = "set_print_mode";

    // 为兼容性保留 param 中的 interval
    request->param = interval;

    // 使用嵌套的 PrintModeConfig 传递完整参数
    request->print_mode.interval = interval;
    request->print_mode.is_full_end = is_full_end;
    request->print_mode.mode = mode;

    try {
        RCLCPP_INFO(
            this->get_logger(),
            "快速命令-设置打印模式: printer=%s, interval=%d, isFullEnd=%d, mode=%d",
            printer_name.c_str(), interval, is_full_end, mode
        );

        auto future = quick_command_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "设置打印模式成功: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "设置打印模式失败: %s", response->message.c_str());
            }
            return {response->success, response->message};
        } else {
            std::string msg = "服务调用超时";
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            return {false, msg};
        }
    } catch (const std::exception& e) {
        std::string msg = std::string("设置打印模式异常: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }
}

std::tuple<bool, std::string> InkjetClient::set_print_mode_left(
    int interval,
    int is_full_end,
    int mode,
    std::chrono::seconds timeout
) {
    return set_print_mode("left", interval, is_full_end, mode, timeout);
}

std::tuple<bool, std::string> InkjetClient::set_print_mode_center(
    int interval,
    int is_full_end,
    int mode,
    std::chrono::seconds timeout
) {
    return set_print_mode("center", interval, is_full_end, mode, timeout);
}

std::tuple<bool, std::string> InkjetClient::set_print_mode_right(
    int interval,
    int is_full_end,
    int mode,
    std::chrono::seconds timeout
) {
    return set_print_mode("right", interval, is_full_end, mode, timeout);
}

std::tuple<bool, std::string> InkjetClient::set_print_mode_all(
    int interval,
    int is_full_end,
    int mode,
    std::chrono::seconds timeout
) {
    return set_print_mode("all", interval, is_full_end, mode, timeout);
}

// ========== 单条线段测试（通过 QuickCommand:single_line） ==========

std::tuple<bool, std::string> InkjetClient::send_single_line(
    const std::string& printer_name,
    int height,
    int width,
    int x,
    int y,
    std::chrono::seconds timeout
) {
    if (!quick_command_client_->wait_for_service(1s)) {
        std::string msg = "printer/quick_command 服务不可用";
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }

    auto request = std::make_shared<xline_msgs::srv::QuickCommand::Request>();
    request->printer_name = printer_name;
    request->action = "single_line";

    // param 在 single_line 模式下未使用，保持为 0
    request->param = 0;

    // 使用嵌套的 SingleLineConfig 传递线段参数
    request->single_line.height = height;
    request->single_line.width = width;
    request->single_line.x = x;
    request->single_line.y = y;

    try {
        RCLCPP_INFO(
            this->get_logger(),
            "快速命令-单条线段: printer=%s, height=%d, width=%d, x=%d, y=%d",
            printer_name.c_str(), height, width, x, y
        );

        auto future = quick_command_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "单条线段测试命令成功: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "单条线段测试命令失败: %s", response->message.c_str());
            }
            return {response->success, response->message};
        } else {
            std::string msg = "服务调用超时";
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            return {false, msg};
        }
    } catch (const std::exception& e) {
        std::string msg = std::string("单条线段测试命令异常: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }
}

std::tuple<bool, std::string> InkjetClient::send_single_line_left(
    int height,
    int width,
    int x,
    int y,
    std::chrono::seconds timeout
) {
    return send_single_line("left", height, width, x, y, timeout);
}

std::tuple<bool, std::string> InkjetClient::send_single_line_center(
    int height,
    int width,
    int x,
    int y,
    std::chrono::seconds timeout
) {
    return send_single_line("center", height, width, x, y, timeout);
}

std::tuple<bool, std::string> InkjetClient::send_single_line_right(
    int height,
    int width,
    int x,
    int y,
    std::chrono::seconds timeout
) {
    return send_single_line("right", height, width, x, y, timeout);
}

std::tuple<bool, std::string> InkjetClient::send_single_line_all(
    int height,
    int width,
    int x,
    int y,
    std::chrono::seconds timeout
) {
    return send_single_line("all", height, width, x, y, timeout);
}

// ========== 便捷方法 - 蜂鸣 ==========

std::tuple<bool, std::string> InkjetClient::beep(
    const std::string& printer_name, int times, std::chrono::seconds timeout
) {
    return quick_command(printer_name, "beep", times, timeout);
}

std::tuple<bool, std::string> InkjetClient::beep_left(int times, std::chrono::seconds timeout) {
    return beep("left", times, timeout);
}

std::tuple<bool, std::string> InkjetClient::beep_center(int times, std::chrono::seconds timeout) {
    return beep("center", times, timeout);
}

std::tuple<bool, std::string> InkjetClient::beep_right(int times, std::chrono::seconds timeout) {
    return beep("right", times, timeout);
}

std::tuple<bool, std::string> InkjetClient::beep_all(int times, std::chrono::seconds timeout) {
    return beep("all", times, timeout);
}

// ========== 便捷方法 - 打印控制 ==========

std::tuple<bool, std::string> InkjetClient::start_print(
    const std::string& printer_name, std::chrono::seconds timeout
) {
    return quick_command(printer_name, "start_print", 0, timeout);
}

std::tuple<bool, std::string> InkjetClient::start_print_left(std::chrono::seconds timeout) {
    return start_print("left", timeout);
}

std::tuple<bool, std::string> InkjetClient::start_print_center(std::chrono::seconds timeout) {
    return start_print("center", timeout);
}

std::tuple<bool, std::string> InkjetClient::start_print_right(std::chrono::seconds timeout) {
    return start_print("right", timeout);
}

std::tuple<bool, std::string> InkjetClient::start_print_all(std::chrono::seconds timeout) {
    return start_print("all", timeout);
}

std::tuple<bool, std::string> InkjetClient::stop_print(
    const std::string& printer_name, std::chrono::seconds timeout
) {
    return quick_command(printer_name, "stop_print", 0, timeout);
}

std::tuple<bool, std::string> InkjetClient::stop_print_left(std::chrono::seconds timeout) {
    return stop_print("left", timeout);
}

std::tuple<bool, std::string> InkjetClient::stop_print_center(std::chrono::seconds timeout) {
    return stop_print("center", timeout);
}

std::tuple<bool, std::string> InkjetClient::stop_print_right(std::chrono::seconds timeout) {
    return stop_print("right", timeout);
}

std::tuple<bool, std::string> InkjetClient::stop_print_all(std::chrono::seconds timeout) {
    return stop_print("all", timeout);
}

// ========== 便捷方法 - 喷头清洗 ==========

std::tuple<bool, std::string> InkjetClient::clean_nozzle(
    const std::string& printer_name, int intensity, std::chrono::seconds timeout
) {
    return quick_command(printer_name, "clean_nozzle", intensity, timeout);
}

std::tuple<bool, std::string> InkjetClient::clean_nozzle_left(int intensity, std::chrono::seconds timeout) {
    return clean_nozzle("left", intensity, timeout);
}

std::tuple<bool, std::string> InkjetClient::clean_nozzle_center(int intensity, std::chrono::seconds timeout) {
    return clean_nozzle("center", intensity, timeout);
}

std::tuple<bool, std::string> InkjetClient::clean_nozzle_right(int intensity, std::chrono::seconds timeout) {
    return clean_nozzle("right", intensity, timeout);
}

std::tuple<bool, std::string> InkjetClient::clean_nozzle_all(int intensity, std::chrono::seconds timeout) {
    return clean_nozzle("all", intensity, timeout);
}

// ========== 便捷方法 - 墨盒查询 ==========

std::tuple<bool, std::string> InkjetClient::query_ink_level(
    const std::string& printer_name, std::chrono::seconds timeout
) {
    return quick_command(printer_name, "ink_level", 0, timeout);
}

std::tuple<bool, std::string> InkjetClient::query_ink_level_left(std::chrono::seconds timeout) {
    return query_ink_level("left", timeout);
}

std::tuple<bool, std::string> InkjetClient::query_ink_level_center(std::chrono::seconds timeout) {
    return query_ink_level("center", timeout);
}

std::tuple<bool, std::string> InkjetClient::query_ink_level_right(std::chrono::seconds timeout) {
    return query_ink_level("right", timeout);
}

std::tuple<bool, std::string> InkjetClient::query_ink_level_all(std::chrono::seconds timeout) {
    return query_ink_level("all", timeout);
}

// ========== 便捷方法 - 测试打印 ==========

std::tuple<bool, std::string> InkjetClient::test_print(
    const std::string& printer_name, std::chrono::seconds timeout
) {
    return quick_command(printer_name, "test_print", 0, timeout);
}

std::tuple<bool, std::string> InkjetClient::test_print_left(std::chrono::seconds timeout) {
    return test_print("left", timeout);
}

std::tuple<bool, std::string> InkjetClient::test_print_center(std::chrono::seconds timeout) {
    return test_print("center", timeout);
}

std::tuple<bool, std::string> InkjetClient::test_print_right(std::chrono::seconds timeout) {
    return test_print("right", timeout);
}

std::tuple<bool, std::string> InkjetClient::test_print_all(std::chrono::seconds timeout) {
    return test_print("all", timeout);
}

// ========== 状态查询服务方法 ==========

std::tuple<bool, std::optional<json>> InkjetClient::get_status_left(std::chrono::seconds timeout) {
    return get_status(status_left_client_, "printer_left", timeout);
}

std::tuple<bool, std::optional<json>> InkjetClient::get_status_center(std::chrono::seconds timeout) {
    return get_status(status_center_client_, "printer_center", timeout);
}

std::tuple<bool, std::optional<json>> InkjetClient::get_status_right(std::chrono::seconds timeout) {
    return get_status(status_right_client_, "printer_right", timeout);
}

std::map<std::string, std::tuple<bool, std::optional<json>>> InkjetClient::get_status_all(
    std::chrono::seconds timeout
) {
    return {
        {"left", get_status_left(timeout)},
        {"center", get_status_center(timeout)},
        {"right", get_status_right(timeout)}
    };
}

std::tuple<bool, std::optional<json>> InkjetClient::get_status(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
    const std::string& printer_name,
    std::chrono::seconds timeout
) {
    if (!client->wait_for_service(1s)) {
        std::string msg = printer_name + "/status 服务不可用";
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, std::nullopt};
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    try {
        RCLCPP_INFO(this->get_logger(), "查询 %s 状态", printer_name.c_str());
        
        auto future = client->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                try {
                    json status_json = json::parse(response->message);
                    RCLCPP_INFO(this->get_logger(), "%s 状态: %s", 
                               printer_name.c_str(), response->message.c_str());
                    return {true, status_json};
                } catch (const json::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "状态JSON解析失败: %s", e.what());
                    return {false, std::nullopt};
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "%s 状态查询失败: %s",
                           printer_name.c_str(), response->message.c_str());
                return {false, std::nullopt};
            }
        } else {
            std::string msg = "服务调用超时";
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            return {false, std::nullopt};
        }
    } catch (const std::exception& e) {
        std::string msg = std::string("状态查询异常: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, std::nullopt};
    }
}

// ========== 设置打印机启用状态服务方法 ==========

std::tuple<bool, std::string> InkjetClient::set_printer_enabled(
    const std::string& printer_name,
    bool enabled,
    std::chrono::seconds timeout
) {
    if (!set_enabled_client_->wait_for_service(1s)) {
        std::string msg = "printer/set_enabled 服务不可用";
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }

    auto request = std::make_shared<xline_msgs::srv::SetPrinterEnabled::Request>();
    request->printer_name = printer_name;
    request->enabled = enabled;

    try {
        RCLCPP_INFO(this->get_logger(), "设置打印机启用状态: printer=%s, enabled=%s",
                    printer_name.c_str(), enabled ? "true" : "false");
        
        auto future = set_enabled_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "设置成功: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "设置失败: %s", response->message.c_str());
            }
            return {response->success, response->message};
        } else {
            std::string msg = "服务调用超时";
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            return {false, msg};
        }
    } catch (const std::exception& e) {
        std::string msg = std::string("设置异常: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }
}

// 便捷方法 - 启用/禁用打印机自动连接

std::tuple<bool, std::string> InkjetClient::enable_printer_left(std::chrono::seconds timeout) {
    return set_printer_enabled("left", true, timeout);
}

std::tuple<bool, std::string> InkjetClient::disable_printer_left(std::chrono::seconds timeout) {
    return set_printer_enabled("left", false, timeout);
}

std::tuple<bool, std::string> InkjetClient::enable_printer_center(std::chrono::seconds timeout) {
    return set_printer_enabled("center", true, timeout);
}

std::tuple<bool, std::string> InkjetClient::disable_printer_center(std::chrono::seconds timeout) {
    return set_printer_enabled("center", false, timeout);
}

std::tuple<bool, std::string> InkjetClient::enable_printer_right(std::chrono::seconds timeout) {
    return set_printer_enabled("right", true, timeout);
}

std::tuple<bool, std::string> InkjetClient::disable_printer_right(std::chrono::seconds timeout) {
    return set_printer_enabled("right", false, timeout);
}

std::tuple<bool, std::string> InkjetClient::enable_all_printers(std::chrono::seconds timeout) {
    return set_printer_enabled("all", true, timeout);
}

std::tuple<bool, std::string> InkjetClient::disable_all_printers(std::chrono::seconds timeout) {
    return set_printer_enabled("all", false, timeout);
}

// ========== 设置打印机激活状态服务方法（功能控制层） ==========

std::tuple<bool, std::string> InkjetClient::set_printer_active(
    const std::string& printer_name,
    bool active,
    std::chrono::seconds timeout
) {
    if (!set_active_client_->wait_for_service(1s)) {
        std::string msg = "printer/set_active 服务不可用";
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }

    auto request = std::make_shared<xline_msgs::srv::SetPrinterActive::Request>();
    request->printer_name = printer_name;
    request->active = active;

    try {
        RCLCPP_INFO(this->get_logger(), "设置打印机激活状态: printer=%s, active=%s",
                    printer_name.c_str(), active ? "true" : "false");
        
        auto future = set_active_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "设置成功: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "设置失败: %s", response->message.c_str());
            }
            return {response->success, response->message};
        } else {
            std::string msg = "服务调用超时";
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            return {false, msg};
        }
    } catch (const std::exception& e) {
        std::string msg = std::string("设置异常: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }
}

// 便捷方法 - 激活/禁用打印机

std::tuple<bool, std::string> InkjetClient::activate_printer_left(std::chrono::seconds timeout) {
    return set_printer_active("left", true, timeout);
}

std::tuple<bool, std::string> InkjetClient::deactivate_printer_left(std::chrono::seconds timeout) {
    return set_printer_active("left", false, timeout);
}

std::tuple<bool, std::string> InkjetClient::activate_printer_center(std::chrono::seconds timeout) {
    return set_printer_active("center", true, timeout);
}

std::tuple<bool, std::string> InkjetClient::deactivate_printer_center(std::chrono::seconds timeout) {
    return set_printer_active("center", false, timeout);
}

std::tuple<bool, std::string> InkjetClient::activate_printer_right(std::chrono::seconds timeout) {
    return set_printer_active("right", true, timeout);
}

std::tuple<bool, std::string> InkjetClient::deactivate_printer_right(std::chrono::seconds timeout) {
    return set_printer_active("right", false, timeout);
}

std::tuple<bool, std::string> InkjetClient::activate_all_printers(std::chrono::seconds timeout) {
    return set_printer_active("all", true, timeout);
}

std::tuple<bool, std::string> InkjetClient::deactivate_all_printers(std::chrono::seconds timeout) {
    return set_printer_active("all", false, timeout);
}

  } // namespace base_controller
} // namespace xline
