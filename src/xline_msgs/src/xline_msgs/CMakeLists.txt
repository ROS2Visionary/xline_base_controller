cmake_minimum_required(VERSION 3.8)
project(xline_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PrintModeConfig.msg"
  "action/ExecutePlan.action"
  "srv/LnCommand.srv"
  "srv/PrinterCommand.srv"
  "srv/QuickCommand.srv"
  "srv/SetPrinterEnabled.srv"
  "srv/SetPrinterActive.srv"
  "srv/ConfigurePrint.srv"
  "srv/MotorCommand.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
