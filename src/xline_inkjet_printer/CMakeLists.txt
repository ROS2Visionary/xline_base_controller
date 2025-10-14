cmake_minimum_required(VERSION 3.8)
project(xline_inkjet_printer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# 添加头文件目录
include_directories(include)

# 添加可执行文件
add_executable(inkjet_printer_node src/inkjet_printer_node.cpp)
ament_target_dependencies(inkjet_printer_node rclcpp)

# 链接yaml-cpp库
target_link_libraries(inkjet_printer_node yaml-cpp)

# 设置目标的包含目录
target_include_directories(inkjet_printer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 安装可执行文件
install(TARGETS
  inkjet_printer_node
  DESTINATION lib/${PROJECT_NAME})

# 安装头文件
install(DIRECTORY include/
  DESTINATION include)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION lib/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出include目录
ament_export_include_directories(include)

ament_package()
