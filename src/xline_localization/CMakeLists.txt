cmake_minimum_required(VERSION 3.8)
project(xline_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
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
add_library(localization_component SHARED
  src/localization_node.cpp
)

ament_target_dependencies(localization_component
  ament_index_cpp
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(localization_component
  yaml-cpp
)

# 注册为可组合节点
rclcpp_components_register_node(localization_component
  PLUGIN "xline::localization::LocalizationNode"
  EXECUTABLE localization_node
)

# 安装
install(TARGETS localization_component
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