cmake_minimum_required(VERSION 3.20)
project(nebula_ouster_common)

find_package(autoware_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
autoware_package()

add_library(nebula_ouster_common INTERFACE)

target_include_directories(
  nebula_ouster_common
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
target_link_libraries(nebula_ouster_common
                      INTERFACE nebula_core_common::nebula_core_common
                                nlohmann_json::nlohmann_json)

install(TARGETS nebula_ouster_common EXPORT export_nebula_ouster_common)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_nebula_ouster_common)
ament_export_dependencies(nebula_core_common nlohmann_json)

ament_package()
