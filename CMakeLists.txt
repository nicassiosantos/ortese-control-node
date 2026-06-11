cmake_minimum_required(VERSION 3.8)
project(control_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(control_node
  src/control_node.cpp
  src/mg_motor_serial.cpp
)
ament_target_dependencies(control_node
  rclcpp
  std_msgs
  std_srvs
)

add_executable(trajectory_node
  src/trajectory_node.cpp
  src/mot_reader.cpp
  src/trajectory_proc.cpp
)
ament_target_dependencies(trajectory_node
  rclcpp
  std_msgs
)

install(TARGETS control_node trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()