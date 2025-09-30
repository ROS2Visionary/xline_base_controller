cmake_minimum_required(VERSION 3.8)
project(xline_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# 创建定位节点库
add_library(localization_node SHARED
  src/localization_node.cpp
)

ament_target_dependencies(localization_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(localization_node
  yaml-cpp
)

# 注册为可组合节点
rclcpp_components_register_node(localization_node
  PLUGIN "xline::localization::LocalizationNode"
  EXECUTABLE localization
)

# 安装
install(TARGETS localization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()