// ============================================================================
//  logger_node.cpp
//
//  No ROS 2 que grava a telemetria do controle em um arquivo CSV, para
//  analise posterior e geracao de graficos do TCC.
//
//  Assina /control/state (Float64MultiArray) com o layout:
//    [0] = posicao (theta) [rad]
//    [1] = torque comandado [N.m]
//    [2] = erro [rad]
//    [3] = torque estimado (pela corrente) [N.m]
//    [4] = temperatura [C]
//
//  E tambem /control/reference para registrar a referencia junto:
//    [0]=theta_ref [1]=thetadot_ref [2]=thetaddot_ref [rad,rad/s,rad/s^2]
//
//  Salva um CSV com colunas:
//    t, theta_ref, theta, erro, torque_cmd, torque_est, corrente, temp
//
//  Uso:
//    ros2 run control_node logger_node
//    ros2 run control_node logger_node --ros-args -p out_file:=/tmp/run1.csv
// ============================================================================
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

namespace control_node
{

class LoggerNode : public rclcpp::Node
{
public:
  LoggerNode()
  : Node("logger_node")
  {
    declare_parameter("out_file", "/tmp/control_log.csv");
    declare_parameter("kt", 2.0);   // para estimar corrente a partir do torque

    out_path_ = get_parameter("out_file").as_string();
    kt_ = get_parameter("kt").as_double();

    file_.open(out_path_);
    if (!file_.is_open()) {
      RCLCPP_FATAL(get_logger(), "nao consegui abrir %s para escrita",
                   out_path_.c_str());
      throw std::runtime_error("cannot open csv");
    }
    // cabecalho do CSV
    file_ << "t,theta_ref_deg,theta_deg,erro_deg,"
          << "torque_cmd_Nm,torque_est_Nm,corrente_A,temp_C\n";

    t0_ = now();

    state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/control/state", 50,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        onState(msg);
      });

    ref_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/control/reference", 50,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (!msg->data.empty()) last_ref_ = msg->data[0];
      });

    RCLCPP_INFO(get_logger(), "gravando telemetria em %s", out_path_.c_str());
    RCLCPP_INFO(get_logger(), "Ctrl-C para encerrar e fechar o arquivo.");
  }

  ~LoggerNode()
  {
    if (file_.is_open()) {
      file_.flush();
      file_.close();
      RCLCPP_INFO(get_logger(), "CSV salvo: %s (%d linhas)",
                  out_path_.c_str(), n_rows_);
    }
  }

private:
  void onState(const std_msgs::msg::Float64MultiArray::SharedPtr & msg)
  {
    if (msg->data.size() < 5) return;
    double t = (now() - t0_).seconds();

    double theta      = msg->data[0];   // rad
    double torque_cmd = msg->data[1];   // N.m
    double erro       = msg->data[2];   // rad
    double torque_est = msg->data[3];   // N.m
    double temp       = msg->data[4];   // C
    double corrente   = (kt_ > 1e-9) ? torque_est / kt_ : 0.0;  // A

    const double R2D = 180.0 / M_PI;
    file_ << t << ','
          << last_ref_ * R2D << ','
          << theta * R2D << ','
          << erro * R2D << ','
          << torque_cmd << ','
          << torque_est << ','
          << corrente << ','
          << temp << '\n';
    n_rows_++;

    // flush periodico para nao perder dados se travar
    if (n_rows_ % 50 == 0) file_.flush();
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_sub_;
  std::ofstream file_;
  std::string out_path_;
  double kt_{2.0};
  double last_ref_{0.0};
  rclcpp::Time t0_;
  int n_rows_{0};
};

}  // namespace control_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<control_node::LoggerNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("logger_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}