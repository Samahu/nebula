cmake_minimum_required(VERSION 3.8)
project(ouster_msgs)

find_package(autoware_cmake REQUIRED)
autoware_package()

rosidl_generate_interfaces(
  ${PROJECT_NAME} "msg/OusterPacket.msg" "msg/OusterScan.msg"
  DEPENDENCIES builtin_interfaces std_msgs)

ament_package()
