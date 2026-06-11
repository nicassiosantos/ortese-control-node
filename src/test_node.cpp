// ============================================================================
//  test_node.cpp
//
//  No de TESTES PROGRESSIVOS para validar o controle e o protocolo, do mais
//  simples ao mais complexo. Publica em /control/reference o mesmo formato
//  do trajectory_node: [theta_ref, thetadot_ref, thetaddot_ref] (rad).
//
//  Modos (parametro 'mode'):
//    "hold"   : mantem um angulo fixo (degrau). Use angle_deg.
//    "step"   : alterna entre 0 e angle_deg a cada period_s (onda quadrada).
//    "sine"   : referencia senoidal lenta. Use amp_deg, freq_hz, offset_deg.
//    "ramp"   : sobe linearmente ate angle_deg em period_s e segura.
//
//  IMPORTANTE: este no NAO comanda torque direto. Ele so publica a
//  referencia; o control_node e quem fecha a malha com o motor. Para um
//  teste de TORQUE FIXO direto no motor (sem PID), use o test_torque_node
//  separado (ver mais abaixo no mesmo arquivo, compilado como executavel
//  proprio).
//
//  Uso:
//    ros2 run control_node test_node --ros-args -p mode:=sine \
//        -p amp_deg:=10.0 -p freq_hz:=0.2 -p offset_deg:=-20.0
// ============================================================================
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

namespace control_node
{

class TestNode : public rclcpp::Node
{
public:
  TestNode()
  : Node("test_node")
  {
    declare_parameter("mode", "hold");        // hold|step|sine|ramp
    declare_parameter("angle_deg", -20.0);    // alvo p/ hold/step/ramp
    declare_parameter("amp_deg", 10.0);       // amplitude p/ sine
    declare_parameter("freq_hz", 0.2);        // frequencia p/ sine
    declare_parameter("offset_deg", -20.0);   // centro da senoide
    declare_parameter("period_s", 4.0);       // periodo p/ step/ramp
    declare_parameter("rate_ms", 5);          // taxa de publicacao

    mode_ = get_parameter("mode").as_string();
    angle_ = deg2rad(get_parameter("angle_deg").as_double());
    amp_ = deg2rad(get_parameter("amp_deg").as_double());
    freq_ = get_parameter("freq_hz").as_double();
    offset_ = deg2rad(get_parameter("offset_deg").as_double());
    period_ = get_parameter("period_s").as_double();
    int rate = static_cast<int>(get_parameter("rate_ms").as_int());

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/control/reference", 10);

    t0_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(rate),
      std::bind(&TestNode::tick, this));

    RCLCPP_INFO(get_logger(), "test_node modo='%s'", mode_.c_str());
    if (mode_ == "sine") {
      RCLCPP_INFO(get_logger(),
        "senoide: amp=%.1f deg, freq=%.2f Hz, offset=%.1f deg",
        get_parameter("amp_deg").as_double(), freq_,
        get_parameter("offset_deg").as_double());
    }
  }

private:
  static double deg2rad(double d) { return d * M_PI / 180.0; }

  void tick()
  {
    double t = (now() - t0_).seconds();
    double th = 0, thd = 0, thdd = 0;

    if (mode_ == "hold") {
      th = angle_;                         // angulo fixo, derivadas zero
    }
    else if (mode_ == "step") {
      // onda quadrada entre 0 e angle_ a cada period_/2
      bool high = std::fmod(t, period_) < (period_ / 2.0);
      th = high ? angle_ : 0.0;
    }
    else if (mode_ == "sine") {
      double w = 2.0 * M_PI * freq_;
      th   = offset_ + amp_ * std::sin(w * t);
      thd  = amp_ * w * std::cos(w * t);
      thdd = -amp_ * w * w * std::sin(w * t);
    }
    else if (mode_ == "ramp") {
      if (t < period_) {
        th  = angle_ * (t / period_);
        thd = angle_ / period_;
      } else {
        th = angle_;
      }
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {th, thd, thdd};
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time t0_;
  std::string mode_;
  double angle_{0}, amp_{0}, freq_{0}, offset_{0}, period_{0};
};

}  // namespace control_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control_node::TestNode>());
  rclcpp::shutdown();
  return 0;
}