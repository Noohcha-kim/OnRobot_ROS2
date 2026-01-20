cmake_minimum_required(VERSION 3.8)
project(onrobot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Message and service files
rosidl_generate_interfaces(${PROJECT_NAME}
  # Common messages
  "msg/GripperStatus.msg"
  "msg/GripperCommand.msg"
  
  # RG2/RG6 messages
  "msg/RGStatus.msg"
  "msg/RGCommand.msg"
  
  # 2FG messages
  "msg/FGStatus.msg"
  "msg/FGCommand.msg"
  
  # 3FG messages
  "msg/FG3Status.msg"
  "msg/FG3Command.msg"
  
  # VG messages
  "msg/VGStatus.msg"
  "msg/VGCommand.msg"
  
  # MG10 messages
  "msg/MG10Status.msg"
  "msg/MG10Command.msg"
  
  # HEX sensor messages
  "msg/HEXWrench.msg"
  
  # Services
  "srv/SetGripperMode.srv"
  "srv/Calibrate.srv"
  "srv/GetStatus.srv"
  
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
