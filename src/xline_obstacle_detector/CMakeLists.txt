cmake_minimum_required(VERSION 3.8)
project(xline_obstacle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executable
add_executable(xline_obstacle_detector_node
  src/obstacle_detector_node.cpp
  src/main.cpp
)

target_include_directories(xline_obstacle_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(xline_obstacle_detector_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(xline_obstacle_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install
install(TARGETS xline_obstacle_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  # launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()