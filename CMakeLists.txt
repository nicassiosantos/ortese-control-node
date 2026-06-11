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

# Ferramenta didatica: imprime os frames do protocolo (nao precisa de ROS nem motor)
add_executable(mostra_protocolo
  src/mostra_protocolo.cpp
)

# No de gravacao de telemetria em CSV
add_executable(logger_node
  src/logger_node.cpp
)
ament_target_dependencies(logger_node
  rclcpp
  std_msgs
)

# No de testes progressivos (gera referencias: hold/step/sine/ramp)
add_executable(test_node
  src/test_node.cpp
)
ament_target_dependencies(test_node
  rclcpp
  std_msgs
)

# No de teste de torque FIXO direto no motor (valida o protocolo, sem PID)
add_executable(test_torque_node
  src/test_torque_node.cpp
  src/mg_motor_serial.cpp
)
ament_target_dependencies(test_torque_node
  rclcpp
)

install(TARGETS
  control_node trajectory_node mostra_protocolo
  logger_node test_node test_torque_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/plotar_telemetria.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()