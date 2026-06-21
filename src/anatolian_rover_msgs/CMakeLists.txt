cmake_minimum_required(VERSION 3.8)
project(anatolian_rover_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GeoGoal.msg"
  "msg/GpsStatus.msg"
  "msg/ImuStatus.msg"
  "msg/HeadingStatus.msg"
  "msg/MissionState.msg"
  "msg/MotorPulseCommand.msg"
  "msg/MotorStatus.msg"
  "msg/NavStatus.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
