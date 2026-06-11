// ============================================================================
//  test_torque_node.cpp
//
//  Teste de BAIXO NIVEL: manda um torque FIXO direto no motor (sem PID) e
//  imprime a resposta (angulo, velocidade, corrente, temperatura). Serve
//  para validar o PROTOCOLO e a resposta fisica do motor isoladamente,
//  antes de fechar a malha de controle.
//
//  Abre a serial diretamente (igual ao control_node). NAO rode junto com o
//  control_node -- os dois disputariam a porta.
//
//  Uso:
//    ros2 run control_node test_torque_node --ros-args \
//        -p torque_nm:=1.0 -p duration_s:=3.0
//
//  SEGURANCA: comeca com Motor ON, aplica o torque pelo tempo pedido, e ao
//  final zera o torque e desliga o motor. Use torques BAIXOS (0.5-2 N.m) e
//  o motor preso. Comecando baixo voce confirma o sinal e a escala sem risco.
// ============================================================================
#include <chrono>
#include <memory>
#include <thread>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "control_node/mg_motor_serial.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_torque_node");

  node->declare_parameter("port", "/dev/ttyUSB0");
  node->declare_parameter("baudrate", 115200);
  node->declare_parameter("motor_id", 1);
  node->declare_parameter("torque_nm", 1.0);
  node->declare_parameter("duration_s", 3.0);
  node->declare_parameter("rate_ms", 20);     // 50 Hz de leitura/print
  node->declare_parameter("kt", 2.0);
  node->declare_parameter("i_max", 12.0);
  node->declare_parameter("gear_ratio", 9.0);
  node->declare_parameter("debug_frames", false);

  control_node::MotorConfig cfg;
  cfg.port = node->get_parameter("port").as_string();
  cfg.baudrate = static_cast<int>(node->get_parameter("baudrate").as_int());
  cfg.motor_id = static_cast<uint8_t>(node->get_parameter("motor_id").as_int());
  cfg.kt = node->get_parameter("kt").as_double();
  cfg.i_max = node->get_parameter("i_max").as_double();
  cfg.gear_ratio = node->get_parameter("gear_ratio").as_double();
  cfg.debug_frames = node->get_parameter("debug_frames").as_bool();

  double torque = node->get_parameter("torque_nm").as_double();
  double duration = node->get_parameter("duration_s").as_double();
  int rate_ms = static_cast<int>(node->get_parameter("rate_ms").as_int());

  control_node::MGMotorSerial motor(cfg);
  if (!motor.open()) {
    RCLCPP_FATAL(node->get_logger(), "nao abriu a serial %s", cfg.port.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
    "Aplicando torque FIXO = %.2f N.m por %.1f s. Motor preso!",
    torque, duration);

  if (!motor.motorOn()) {
    RCLCPP_WARN(node->get_logger(),
      "motorOn nao confirmou resposta (segue mesmo assim)");
  }

  printf("\n%8s | %10s | %10s | %10s | %6s\n",
         "t[s]", "ang[deg]", "vel[deg/s]", "corr[A]", "T[C]");
  printf("---------|------------|------------|------------|------\n");

  auto t0 = node->now();
  control_node::MotorState st;
  double t = 0;
  while (rclcpp::ok() && t < duration) {
    if (motor.commandTorque(torque, st)) {
      // tambem le o angulo multi-loop para mostrar a posicao do eixo
      control_node::MotorState ang;
      if (motor.readMultiLoopAngle(ang)) st.position = ang.position;

      printf("%8.2f | %10.2f | %10.2f | %10.3f | %6d\n",
             t,
             st.position * 180.0 / M_PI,
             st.velocity * 180.0 / M_PI,
             st.current,
             st.temperature);
      fflush(stdout);
    } else {
      printf("%8.2f | (falha na transacao serial)\n", t);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(rate_ms));
    t = (node->now() - t0).seconds();
  }

  // seguranca: zera torque e desliga
  control_node::MotorState dummy;
  motor.commandTorque(0.0, dummy);
  motor.motorOff();
  motor.close();
  RCLCPP_INFO(node->get_logger(), "torque zerado, motor desligado.");

  rclcpp::shutdown();
  return 0;
}