cmake_minimum_required(VERSION 3.20)
project(nebula_ouster)

find_package(autoware_cmake REQUIRED)
find_package(Boost REQUIRED)
autoware_package()

add_library(
  ouster_ros_wrapper SHARED
  src/ouster_ros_wrapper.cpp
  src/decoder_wrapper.cpp
  src/hw_interface_wrapper.cpp
  src/hw_monitor_wrapper.cpp)

target_include_directories(
  ouster_ros_wrapper
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ouster_ros_wrapper
  PUBLIC Boost::headers
         diagnostic_updater::diagnostic_updater
         nebula_ouster_decoders::nebula_ouster_decoders
         nebula_ouster_hw_interfaces::nebula_ouster_hw_interfaces
         nebula_core_ros::nebula_core_ros)

ament_target_dependencies(
  ouster_ros_wrapper
  PUBLIC
  diagnostic_msgs
  ouster_msgs
  nebula_msgs
  rclcpp_components
  sensor_msgs)

rclcpp_components_register_node(
  ouster_ros_wrapper PLUGIN "OusterRosWrapper" EXECUTABLE ouster_ros_wrapper_node)

install(
  TARGETS ouster_ros_wrapper
  EXPORT export_ouster_ros_wrapper
  LIBRARY DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch schema DESTINATION share/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_ouster_ros_wrapper)

ament_export_dependencies(
  Boost
  diagnostic_msgs
  diagnostic_updater
  nebula_core_common
  nebula_ouster_common
  nebula_core_decoders
  nebula_ouster_decoders
  nebula_core_hw_interfaces
  nebula_ouster_hw_interfaces
  nebula_msgs
  nebula_core_ros
  rclcpp_components
  ouster_msgs
  sensor_msgs)

ament_package()
