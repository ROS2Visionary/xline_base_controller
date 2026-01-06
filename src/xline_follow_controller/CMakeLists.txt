cmake_minimum_required(VERSION 3.8)
project(xline_follow_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(angles REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories（供本包内部使用）
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 策略模式库（RPP 共用）
add_library(path_strategy SHARED
  src/rpp_follow/path_strategy_factory.cpp
  src/rpp_follow/curve_path_strategy.cpp
  src/rpp_follow/circle_path_strategy.cpp
)

target_include_directories(path_strategy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(path_strategy
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

target_link_libraries(path_strategy
  yaml-cpp
)

# 直线路径跟随控制器库
add_library(line_follow_controller SHARED
  src/line_follow/line_follow_controller.cpp
)

target_include_directories(line_follow_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(line_follow_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  angles
)

target_link_libraries(line_follow_controller
  path_strategy
  ${OpenCV_LIBS}
  yaml-cpp
)

# RPP控制器库
add_library(rpp_follow_controller SHARED
  src/rpp_follow/rpp_follow_controller.cpp
)

target_include_directories(rpp_follow_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rpp_follow_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  angles
)

target_link_libraries(rpp_follow_controller
  path_strategy
  ${OpenCV_LIBS}
  yaml-cpp
)

# LQR控制器库
add_library(lqr_circle_controller SHARED
  src/lqr_follow/lqr_circle_controller.cpp
)

target_include_directories(lqr_circle_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lqr_circle_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
  angles
)

target_link_libraries(lqr_circle_controller
  yaml-cpp
)

# LQR曲线控制器库
add_library(lqr_curve_controller SHARED
  src/lqr_follow/lqr_curve_controller.cpp
)

target_include_directories(lqr_curve_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lqr_curve_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
  angles
)

target_link_libraries(lqr_curve_controller
  yaml-cpp
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装库
install(
  TARGETS path_strategy line_follow_controller rpp_follow_controller lqr_circle_controller lqr_curve_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装配置文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 导出
ament_export_include_directories(include)
ament_export_libraries(path_strategy line_follow_controller rpp_follow_controller lqr_circle_controller lqr_curve_controller)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  angles
  OpenCV
  Eigen3
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
