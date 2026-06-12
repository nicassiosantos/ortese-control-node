// ============================================================================
//  control_node.cpp
//
//  No ROS 2 que fecha a malha de controle PID (IDB+PID) com controle de
//  TORQUE sobre o motor K-Tech MG8008E-i9 via RS485 serial, na arquitetura do projeto
//  do Marcio.
//
//  Segue o padrao dos nos existentes:
//   - Subscreve a referencia de marcha   : /control/reference (KneeReference)
//   - Publica o estado da malha           : /control/state     (ControlState)
//   - Expoe servico de habilitacao torque : /control/set_torque (SetTorque)
//
//  A cada ciclo (update_rate_ms): le encoder, roda PID, comanda torque.
//
//  OBS sobre interfaces: para nao alterar as do Marcio (que sao de posicao
//  para o Dynamixel), este no usa interfaces proprias. Se preferir reaproveitar
//  /actuator/state, basta trocar os tipos abaixo. Veja README.
// ============================================================================
#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "control_node/pid_controller.hpp"
#include "control_node/mg_motor_serial.hpp"

using namespace std::chrono_literals;

namespace control_node
{

class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("control_node")
  {
    // -------- Parametros (espelham os .yaml do projeto) --------
    declare_parameter("port", "/dev/ttyUSB0");       // conversor USB-RS485
    declare_parameter("baudrate", 115200);           // baud do motor
    declare_parameter("motor_id", 1);
    declare_parameter("update_rate_ms", 1);          // 1 ms = 1 kHz
    declare_parameter("tau_max", 20.0);              // MG8008E-i9
    declare_parameter("kt", 2.0);                    // Kt eixo de saida [N.m/A]
    declare_parameter("i_max", 12.0);                // corrente max [A]
    declare_parameter("gear_ratio", 9.0);            // reducao 9:1
    declare_parameter("temp_limit", 70);             // corte de seguranca [C]
    declare_parameter("debug_frames", false);        // imprime frames TX/RX hex
    declare_parameter("command_mode", "torque");     // "raw" ou "torque"
    declare_parameter("raw_max", 2000);              // escala do comando bruto

    // ganhos PID (do projeto por alocacao de polos)
    declare_parameter("kp", 190.6);
    declare_parameter("ki", 884.1);
    declare_parameter("kd", 16.07);

    // parametros fisicos da planta (para o feedforward IDB)
    declare_parameter("D11", 0.2158);
    declare_parameter("mgd", 7.848);
    declare_parameter("b", 0.5);
    declare_parameter("use_feedforward", true);

    // -------- Montar configuracoes --------
    MotorConfig mcfg;
    mcfg.port = get_parameter("port").as_string();
    mcfg.baudrate = static_cast<int>(get_parameter("baudrate").as_int());
    mcfg.motor_id = static_cast<uint8_t>(get_parameter("motor_id").as_int());
    mcfg.kt = get_parameter("kt").as_double();
    mcfg.i_max = get_parameter("i_max").as_double();
    mcfg.gear_ratio = get_parameter("gear_ratio").as_double();
    mcfg.debug_frames = get_parameter("debug_frames").as_bool();
    mcfg.command_mode = get_parameter("command_mode").as_string();
    mcfg.raw_max = static_cast<int>(get_parameter("raw_max").as_int());
    temp_limit_ = static_cast<int>(get_parameter("temp_limit").as_int());

    PIDConfig pcfg;
    pcfg.gains.kp = get_parameter("kp").as_double();
    pcfg.gains.ki = get_parameter("ki").as_double();
    pcfg.gains.kd = get_parameter("kd").as_double();
    pcfg.plant.D11 = get_parameter("D11").as_double();
    pcfg.plant.mgd = get_parameter("mgd").as_double();
    pcfg.plant.b = get_parameter("b").as_double();
    pcfg.tau_max = get_parameter("tau_max").as_double();
    pcfg.use_feedforward = get_parameter("use_feedforward").as_bool();
    int update_ms = static_cast<int>(get_parameter("update_rate_ms").as_int());
    pcfg.ts = update_ms / 1000.0;

    motor_ = std::make_unique<MGMotorSerial>(mcfg);
    pid_ = std::make_unique<PIDController>(pcfg);
    tau_max_ = pcfg.tau_max;

    // -------- Comunicacao ROS --------
    // referencia: [theta_ref, thetadot_ref, thetaddot_ref] em rad, rad/s, rad/s^2
    ref_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/control/reference", 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 1) th_ref_ = msg->data[0];
        if (msg->data.size() >= 2) thd_ref_ = msg->data[1];
        if (msg->data.size() >= 3) thdd_ref_ = msg->data[2];
      });

    // estado: [theta_meas, tau_cmd, erro, torque_est, temperatura]
    state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/control/state", 10);

    // servico para habilitar/desabilitar o torque (seguranca)
    set_torque_srv_ = create_service<std_srvs::srv::SetBool>(
      "/control/set_torque",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        torque_enabled_ = req->data;
        if (torque_enabled_) {
          res->success = motor_->motorOn();
          pid_->reset();
          res->message = res->success ? "torque habilitado" : "falha ao ligar";
        } else {
          motor_->commandTorque(0.0, last_state_);  // zera torque
          res->success = motor_->motorOff();
          res->message = "torque desabilitado";
        }
        RCLCPP_INFO(get_logger(), "set_torque=%d -> %s",
                    torque_enabled_, res->message.c_str());
      });

    // -------- Abrir porta serial --------
    if (!motor_->open()) {
      RCLCPP_FATAL(get_logger(), "nao foi possivel abrir a porta serial. Encerrando.");
      throw std::runtime_error("serial open failed");
    }
    RCLCPP_INFO(get_logger(),
                "control_node iniciado: %s @ %d baud, %d Hz, tau_max=%.1f N.m",
                mcfg.port.c_str(), mcfg.baudrate, 1000 / update_ms, tau_max_);
    RCLCPP_WARN(get_logger(), "torque DESABILITADO. Chame /control/set_torque "
                "com data:true para iniciar.");

    // -------- Loop de controle em tempo real --------
    timer_ = create_wall_timer(
      std::chrono::milliseconds(update_ms),
      std::bind(&ControlNode::controlStep, this));
  }

  ~ControlNode()
  {
    if (motor_ && motor_->isOpen()) {
      motor_->commandTorque(0.0, last_state_);
      motor_->motorOff();
      motor_->close();
    }
  }

private:
  void controlStep()
  {
    MotorState st;

    if (!torque_enabled_) {
      // so le o estado para telemetria, sem comandar torque
      motor_->readMultiLoopAngle(st);
      last_state_.position = st.position;
      publishState(st, 0.0, th_ref_ - st.position);
      return;
    }

    // corte de seguranca por temperatura (datasheet: protecao em 100 C;
    // aqui usamos um limite mais conservador configuravel)
    if (last_state_.valid && last_state_.temperature >= temp_limit_) {
      motor_->commandTorque(0.0, st);
      motor_->motorOff();
      torque_enabled_ = false;
      RCLCPP_ERROR(get_logger(),
                   "TEMPERATURA %d C >= limite %d C: torque desligado!",
                   last_state_.temperature, temp_limit_);
      return;
    }

    // 1) le o angulo atual do EIXO DE SAIDA (joelho) via multi-loop
    MotorState ang_state;
    if (motor_->readMultiLoopAngle(ang_state)) {
      last_state_.position = ang_state.position;
    }
    double th_meas = last_state_.position;

    // 2) roda o PID (IDB+PID) -> torque
    double tau = pid_->update(th_ref_, thd_ref_, thdd_ref_, th_meas);

    // 3) comanda o torque e recebe o estado (corrente, velocidade, temp)
    if (motor_->commandTorque(tau, st)) {
      st.position = last_state_.position;  // preserva o angulo multi-loop
      last_state_ = st;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "falha na transacao serial");
    }

    publishState(st, tau, pid_->lastError());
  }

  void publishState(const MotorState & st, double tau, double err)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      st.position,            // theta medido [rad]
      tau,                    // torque comandado [N*m]
      err,                    // erro [rad]
      st.torque_est,          // torque estimado pela corrente [N*m]
      static_cast<double>(st.temperature)
    };
    state_pub_->publish(msg);
  }

  std::unique_ptr<MGMotorSerial> motor_;
  std::unique_ptr<PIDController> pid_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_torque_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  double th_ref_{0.0}, thd_ref_{0.0}, thdd_ref_{0.0};
  double tau_max_{20.0};
  int    temp_limit_{70};
  bool   torque_enabled_{false};
  MotorState last_state_;
};

}  // namespace control_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<control_node::ControlNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("control_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}