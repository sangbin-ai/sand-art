cmake_minimum_required(VERSION 3.8)
project(sandart_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SandPoint.msg"
  "msg/SandStroke.msg"
  "msg/Bond.msg"
  "srv/PathPlanList.srv"
  "srv/TripleLayerdImages.srv"
  "srv/ProcessImage.srv"
  DEPENDENCIES sensor_msgs
)

ament_package()