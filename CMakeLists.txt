cmake_minimum_required(VERSION 3.8)
project(control_node)

# -------------------------------------------------------------------
#  Opcao de baixo uso de memoria para compilar na Raspberry Pi.
#  -O2 consome MUITA RAM ao compilar e pode travar a Pi. Com
#  LOW_MEMORY=ON usamos -O1 (bem mais leve, perde pouco desempenho).
#  Compile assim na Pi:
#    colcon build --packages-select control_node \
#      --cmake-args -DLOW_MEMORY=ON
# -------------------------------------------------------------------
option(LOW_MEMORY "Reduz otimizacao para compilar com pouca RAM (Pi)" OFF)

# -------------------------------------------------------------------
#  Opcao de compilar SO o essencial (control_node + trajectory_node),
#  deixando os nos de teste de fora para reduzir a carga de compilacao.
#    colcon build --packages-select control_node \
#      --cmake-args -DLOW_MEMORY=ON -DBUILD_TESTS_TOOLS=OFF
# -------------------------------------------------------------------
option(BUILD_TESTS_TOOLS "Compila os nos de teste e ferramentas" ON)

if(LOW_MEMORY)
  set(OPT_FLAG -O1)
  message(STATUS "control_node: modo LOW_MEMORY (-O1)")
else()
  set(OPT_FLAG -O2)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra ${OPT_FLAG})
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

# ===== Executaveis ESSENCIAIS (sempre compilados) =====
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

# ===== Nos de TESTE e ferramentas (opcionais) =====
if(BUILD_TESTS_TOOLS)
  add_executable(mostra_protocolo
    src/mostra_protocolo.cpp
  )

  add_executable(logger_node
    src/logger_node.cpp
  )
  ament_target_dependencies(logger_node rclcpp std_msgs)

  add_executable(test_node
    src/test_node.cpp
  )
  ament_target_dependencies(test_node rclcpp std_msgs)

  add_executable(test_torque_node
    src/test_torque_node.cpp
    src/mg_motor_serial.cpp
  )
  ament_target_dependencies(test_torque_node rclcpp)

  install(TARGETS mostra_protocolo logger_node test_node test_torque_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(PROGRAMS scripts/plotar_telemetria.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()