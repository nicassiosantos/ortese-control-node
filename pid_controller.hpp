// ============================================================================
//  pid_controller.hpp
//
//  Controlador PID com arquitetura IDB+PID (feedforward de dinamica inversa
//  + feedback PID) para a ortese de joelho 1-DOF, gerando TORQUE.
//
//  Espelha exatamente a matematica validada nos modelos MATLAB/Simulink:
//    tau = tau_FF + tau_PID
//    tau_FF  = D11*thdd_ref + mgd*cos(th_ref) + b*thd_ref   (dinamica inversa)
//    tau_PID = Kp*e + Ki*∫e + Kd*de/dt   (com derivativo FILTRADO)
//
//  Inclui: derivativo filtrado (evita derivative kick), anti-windup por
//  clamping, e saturacao no torque maximo do motor.
// ============================================================================
#ifndef CONTROL_NODE__PID_CONTROLLER_HPP_
#define CONTROL_NODE__PID_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

namespace control_node
{

struct PIDGains
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
};

struct PlantParams
{
  double D11{0.2158};   // inercia efetiva [kg*m^2]
  double mgd{7.848};    // torque gravitacional maximo [N*m]
  double b{0.5};        // atrito viscoso [N*m*s/rad]
};

struct PIDConfig
{
  PIDGains gains;
  PlantParams plant;
  double ts{0.001};        // periodo de amostragem [s]
  double tau_max{20.0};    // torque maximo do motor [N*m] (MG8008E-i9)
  double tf_ratio{0.1};    // Tf = tf_ratio * ts (filtro derivativo)
  bool   use_feedforward{true};   // habilita o termo IDB (dinamica inversa)
};

class PIDController
{
public:
  explicit PIDController(const PIDConfig & cfg)
  : cfg_(cfg)
  {
    tf_ = cfg_.tf_ratio * cfg_.ts * 100.0;  // Tf absoluto; ver nota no .cpp
    reset();
  }

  void reset()
  {
    integral_ = 0.0;
    deriv_filt_ = 0.0;
    e_prev_ = 0.0;
    first_step_ = true;
  }

  void setGains(const PIDGains & g) { cfg_.gains = g; }
  void setTauMax(double t) { cfg_.tau_max = t; }

  // ------------------------------------------------------------------------
  //  Um passo do controlador.
  //   th_ref    : angulo de referencia [rad]
  //   thd_ref   : velocidade de referencia [rad/s]  (para o feedforward)
  //   thdd_ref  : aceleracao de referencia [rad/s^2](para o feedforward)
  //   th_meas   : angulo medido pelo encoder [rad]
  //  retorna: torque comandado [N*m], ja saturado em +/- tau_max
  // ------------------------------------------------------------------------
  double update(double th_ref, double thd_ref, double thdd_ref, double th_meas)
  {
    const double ts = cfg_.ts;
    const double e  = th_ref - th_meas;

    // --- termo integral ---
    integral_ += e * ts;

    // --- termo derivativo FILTRADO (filtro de 1a ordem) ---
    // D_k = (Tf*D_{k-1} + Kd*(e_k - e_{k-1})) / (Tf + Ts)
    if (first_step_) { e_prev_ = e; first_step_ = false; }
    deriv_filt_ = (tf_ * deriv_filt_ + cfg_.gains.kd * (e - e_prev_)) / (tf_ + ts);
    e_prev_ = e;

    // --- lei PID (feedback) ---
    double tau_pid = cfg_.gains.kp * e + cfg_.gains.ki * integral_ + deriv_filt_;

    // --- feedforward de dinamica inversa (IDB) ---
    double tau_ff = 0.0;
    if (cfg_.use_feedforward) {
      tau_ff = cfg_.plant.D11 * thdd_ref
             + cfg_.plant.mgd * std::cos(th_ref)
             + cfg_.plant.b   * thd_ref;
    }

    double tau_cmd = tau_ff + tau_pid;

    // --- saturacao no torque maximo do motor ---
    double tau_sat = std::clamp(tau_cmd, -cfg_.tau_max, cfg_.tau_max);

    // --- anti-windup por back-calculation: desfaz o excesso integral ---
    if (tau_cmd != tau_sat && cfg_.gains.ki > 1e-9) {
      integral_ += (tau_sat - tau_cmd) / cfg_.gains.ki;
    }

    last_tau_ = tau_sat;
    last_error_ = e;
    return tau_sat;
  }

  double lastTau()   const { return last_tau_; }
  double lastError() const { return last_error_; }

private:
  PIDConfig cfg_;
  double tf_{0.0};
  double integral_{0.0};
  double deriv_filt_{0.0};
  double e_prev_{0.0};
  bool   first_step_{true};
  double last_tau_{0.0};
  double last_error_{0.0};
};

}  // namespace control_node

#endif  // CONTROL_NODE__PID_CONTROLLER_HPP_
