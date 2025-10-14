#include "xline_inkjet_printer/inkjet_printer_node.hpp"
#include <rclcpp/executors.hpp>
#include <yaml-cpp/yaml.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>

namespace xline_inkjet_printer
{

InkjetPrinterNode::InkjetPrinterNode()
  : Node("inkjet_printer_node"),
    tcp_socket_(-1),
    is_connected_(false),
    should_stop_(false)
{
  RCLCPP_INFO(this->get_logger(), "喷码打印节点已启动");

  // 加载配置参数
  updateParameter();

  // 启动TCP连接线程
  tcp_thread_ = std::thread(&InkjetPrinterNode::tcpConnectionLoop, this);

  // 创建定时器，每秒检查连接状态
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&InkjetPrinterNode::timer_callback, this));
}

InkjetPrinterNode::~InkjetPrinterNode()
{
  should_stop_ = true;
  if (tcp_thread_.joinable()) {
    tcp_thread_.join();
  }
  closeTcpConnection();
}

void InkjetPrinterNode::updateParameter()
{
  try {
    // 使用相对路径：../config/tcp_config.yaml
    // 从可执行文件所在目录的上级目录的config文件夹中加载
    std::string config_path = "../config/tcp_config.yaml";

    RCLCPP_INFO(this->get_logger(), "正在加载配置文件: %s", config_path.c_str());

    YAML::Node config = YAML::LoadFile(config_path);

    tcp_ip_ = config["tcp"]["ip"].as<std::string>();
    tcp_port_ = config["tcp"]["port"].as<int>();
    reconnect_interval_ = config["tcp"]["reconnect_interval"].as<int>();
    connect_timeout_ = config["tcp"]["connect_timeout"].as<int>();

    RCLCPP_INFO(this->get_logger(), "配置加载成功 - IP: %s, Port: %d, 重连间隔: %ds",
                tcp_ip_.c_str(), tcp_port_, reconnect_interval_);
  }
  catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "YAML解析错误: %s", e.what());
    // 使用默认值
    tcp_ip_ = "127.0.0.1";
    tcp_port_ = 8888;
    reconnect_interval_ = 3;
    connect_timeout_ = 5;
    RCLCPP_WARN(this->get_logger(), "使用默认配置 - IP: %s, Port: %d",
                tcp_ip_.c_str(), tcp_port_);
  }
}

void InkjetPrinterNode::tcpConnectionLoop()
{
  while (!should_stop_) {
    if (!is_connected_) {
      RCLCPP_INFO(this->get_logger(), "尝试连接到 %s:%d...",
                  tcp_ip_.c_str(), tcp_port_);

      if (connectToServer()) {
        RCLCPP_INFO(this->get_logger(), "TCP连接成功！");
        is_connected_ = true;

        // 连接成功后，持续监控连接状态
        monitorConnection();
      }
      else {
        RCLCPP_WARN(this->get_logger(), "TCP连接失败，%d秒后重试...",
                    reconnect_interval_);
        std::this_thread::sleep_for(std::chrono::seconds(reconnect_interval_));
      }
    }
    else {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

bool InkjetPrinterNode::connectToServer()
{
  // 创建socket
  tcp_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_socket_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "创建socket失败");
    return false;
  }

  // 设置连接超时
  struct timeval timeout;
  timeout.tv_sec = connect_timeout_;
  timeout.tv_usec = 0;
  setsockopt(tcp_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(tcp_socket_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  // 配置服务器地址
  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(tcp_port_);

  if (inet_pton(AF_INET, tcp_ip_.c_str(), &server_addr.sin_addr) <= 0) {
    RCLCPP_ERROR(this->get_logger(), "无效的IP地址: %s", tcp_ip_.c_str());
    close(tcp_socket_);
    tcp_socket_ = -1;
    return false;
  }

  // 连接到服务器
  if (connect(tcp_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    close(tcp_socket_);
    tcp_socket_ = -1;
    return false;
  }

  return true;
}

void InkjetPrinterNode::monitorConnection()
{
  char buffer[1];
  while (!should_stop_ && is_connected_) {
    // 使用recv的MSG_PEEK标志来检查连接状态，不移除数据
    int result = recv(tcp_socket_, buffer, 1, MSG_PEEK | MSG_DONTWAIT);

    if (result == 0) {
      // 连接已关闭
      RCLCPP_WARN(this->get_logger(), "TCP连接已断开");
      is_connected_ = false;
      closeTcpConnection();
      break;
    }
    else if (result < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        // 连接错误
        RCLCPP_ERROR(this->get_logger(), "TCP连接错误: %s", strerror(errno));
        is_connected_ = false;
        closeTcpConnection();
        break;
      }
    }

    // 短暂休眠，避免CPU占用过高
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void InkjetPrinterNode::closeTcpConnection()
{
  if (tcp_socket_ >= 0) {
    close(tcp_socket_);
    tcp_socket_ = -1;
  }
}

void InkjetPrinterNode::timer_callback()
{
  if (is_connected_) {
    RCLCPP_INFO(this->get_logger(), "TCP连接状态: 已连接 [%s:%d]",
                tcp_ip_.c_str(), tcp_port_);
  }
  else {
    RCLCPP_INFO(this->get_logger(), "TCP连接状态: 未连接");
  }
}

}  // namespace xline_inkjet_printer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xline_inkjet_printer::InkjetPrinterNode>();

  // 使用多线程执行器
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
