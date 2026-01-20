cmake_minimum_required(VERSION 3.8)
project(onrobot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(onrobot_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Modbus TCP library
add_library(modbus_tcp_client SHARED
  src/modbus_tcp_client.cpp
)

target_include_directories(modbus_tcp_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(modbus_tcp_client
  rclcpp
)

# Base gripper driver library
add_library(onrobot_base SHARED
  src/onrobot_base.cpp
)

target_include_directories(onrobot_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(onrobot_base
  modbus_tcp_client
)

ament_target_dependencies(onrobot_base
  rclcpp
  onrobot_msgs
  std_msgs
)

# RG2/RG6 driver
add_library(rg_driver SHARED
  src/rg_driver.cpp
)

target_link_libraries(rg_driver
  onrobot_base
  modbus_tcp_client
)

ament_target_dependencies(rg_driver
  rclcpp
  onrobot_msgs
)

# 2FG driver
add_library(fg2_driver SHARED
  src/fg2_driver.cpp
)

target_link_libraries(fg2_driver
  onrobot_base
  modbus_tcp_client
)

ament_target_dependencies(fg2_driver
  rclcpp
  onrobot_msgs
)

# 3FG driver
add_library(fg3_driver SHARED
  src/fg3_driver.cpp
)

target_link_libraries(fg3_driver
  onrobot_base
  modbus_tcp_client
)

ament_target_dependencies(fg3_driver
  rclcpp
  onrobot_msgs
)

# 2FGP20 driver (with vacuum)
add_library(fgp20_driver SHARED
  src/fgp20_driver.cpp
)

target_link_libraries(fgp20_driver
  onrobot_base
  modbus_tcp_client
)

ament_target_dependencies(fgp20_driver
  rclcpp
  onrobot_msgs
)

# VG driver
add_library(vg_driver SHARED
  src/vg_driver.cpp
)

target_link_libraries(vg_driver
  onrobot_base
  modbus_tcp_client
)

ament_target_dependencies(vg_driver
  rclcpp
  onrobot_msgs
)

# MG10 driver
add_library(mg10_driver SHARED
  src/mg10_driver.cpp
)

target_link_libraries(mg10_driver
  onrobot_base
  modbus_tcp_client
)

ament_target_dependencies(mg10_driver
  rclcpp
  onrobot_msgs
)

# HEX-E/H QC driver
add_library(hex_driver SHARED
  src/hex_driver.cpp
)

target_link_libraries(hex_driver
  onrobot_base
  modbus_tcp_client
)

ament_target_dependencies(hex_driver
  rclcpp
  geometry_msgs
  onrobot_msgs
)

# Main node executable
add_executable(onrobot_node
  src/onrobot_node.cpp
)

target_link_libraries(onrobot_node
  onrobot_base
  modbus_tcp_client
  rg_driver
  fg2_driver
  fg3_driver
  fgp20_driver
  vg_driver
  mg10_driver
  hex_driver
)

ament_target_dependencies(onrobot_node
  rclcpp
  onrobot_msgs
)

# Install
install(TARGETS
  modbus_tcp_client
  onrobot_base
  rg_driver
  fg2_driver
  fg3_driver
  fgp20_driver
  vg_driver
  mg10_driver
  hex_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  onrobot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  modbus_tcp_client
  onrobot_base
  rg_driver
  fg2_driver
  fg3_driver
  fgp20_driver
  vg_driver
  mg10_driver
  hex_driver
)
ament_export_dependencies(
  rclcpp
  onrobot_msgs
)

ament_package()
