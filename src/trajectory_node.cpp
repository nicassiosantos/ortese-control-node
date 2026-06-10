// ============================================================================
//  trajectory_node.cpp
//
//  No ROS 2 que le um arquivo .mot do OpenSim, processa (filtro 6 Hz,
//  reamostragem, derivadas) e publica a trajetoria de referencia ponto a
//  ponto no topico /control/reference, que o control_node consome.
//
//  Publica [theta_ref, thetadot_ref, thetaddot_ref] a cada ciclo (update_ms),
//  na mesma taxa do controlador.
//
//  Parametros:
//   - mot_file       : caminho do .mot (ex.: /app/.../subject01_walk1.mot)
//   - column         : nome da coluna (default knee_angle_r)
//   - update_rate_ms : passo de publicacao [ms] (default 1 = 1 kHz)
//   - filter_hz      : corte do filtro Butterworth [Hz] (default 6)
//   - loop           : repete a marcha em ciclo (default true)
// ============================================================================
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "control_node/mot_reader.hpp"
#include "control_node/trajectory_proc.hpp"

using namespace std::chrono_literals;

namespace control_node
{

class TrajectoryNode : public rclcpp::Node
{
public:
  TrajectoryNode()
  : Node("trajectory_node")
  {
    declare_parameter("mot_file", "");
    declare_parameter("column", "knee_angle_r");
    declare_parameter("update_rate_ms", 1);
    declare_parameter("filter_hz", 6.0);
    declare_parameter("loop", true);

    std::string mot = get_parameter("mot_file").as_string();
    std::string col = get_parameter("column").as_string();
    int update_ms = static_cast<int>(get_parameter("update_rate_ms").as_int());
    double fc = get_parameter("filter_hz").as_double();
    loop_ = get_parameter("loop").as_bool();
    double ts = update_ms / 1000.0;

    if (mot.empty()) {
      RCLCPP_FATAL(get_logger(), "parametro 'mot_file' vazio. Informe o caminho do .mot");
      throw std::runtime_error("mot_file vazio");
    }

    // --- le o .mot ---
    MotData raw = MotReader::read(mot, col);
    if (!raw.ok) {
      RCLCPP_FATAL(get_logger(), "erro ao ler %s: %s", mot.c_str(), raw.error.c_str());
      throw std::runtime_error("falha ao ler .mot");
    }
    RCLCPP_INFO(get_logger(), "%s: %zu amostras, coluna '%s'",
                mot.c_str(), raw.time.size(), col.c_str());

    // --- processa (filtro 6 Hz, reamostra, deriva) ---
    traj_ = TrajectoryProc::process(raw.time, raw.values, ts, fc);
    RCLCPP_INFO(get_logger(), "trajetoria processada: %zu pontos a %.0f Hz, filtro %.1f Hz",
                traj_.size(), 1.0 / ts, fc);

    // estatisticas
    double tmin = 1e9, tmax = -1e9;
    for (double v : traj_.theta) { tmin = std::min(tmin, v); tmax = std::max(tmax, v); }
    RCLCPP_INFO(get_logger(), "theta_ref: min=%.1f max=%.1f deg",
                tmin * 180.0 / M_PI, tmax * 180.0 / M_PI);

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/control/reference", 10);

    idx_ = 0;
    timer_ = create_wall_timer(
      std::chrono::milliseconds(update_ms),
      std::bind(&TrajectoryNode::tick, this));
  }

private:
  void tick()
  {
    if (traj_.size() == 0) return;

    if (idx_ >= traj_.size()) {
      if (loop_) {
        idx_ = 0;   // recomeca a marcha
      } else {
        // mantem o ultimo ponto (segura a posicao final)
        idx_ = traj_.size() - 1;
      }
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      traj_.theta[idx_],
      traj_.theta_dot[idx_],
      traj_.theta_ddot[idx_]
    };
    pub_->publish(msg);
    ++idx_;
  }

  Trajectory traj_;
  size_t idx_{0};
  bool loop_{true};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace control_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<control_node::TrajectoryNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("trajectory_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
