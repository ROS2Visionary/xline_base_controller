cmake_minimum_required(VERSION 3.8)
project(xline_follow_controller)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
 include
 include/${PROJECT_NAME}
 ${OpenCV_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)

set(dependencies
 rclcpp
 geometry_msgs
 visualization_msgs
 nav_msgs
 tf2
 tf2_ros
 tf2_geometry_msgs
 angles
 OpenCV
 yaml-cpp
)

add_library(${PROJECT_NAME} SHARED
 src/line_follow_controller.cpp
 src/rpp_follow_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS ${PROJECT_NAME}
 DESTINATION lib
)

## Mark cpp header files for installation
install(DIRECTORY include/
 DESTINATION include/
)

install(DIRECTORY
 config
 DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
