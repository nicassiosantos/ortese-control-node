// ============================================================================
//  trajectory_proc.hpp
//
//  Processa a trajetoria bruta do .mot (mesmo pipeline do
//  preparar_referencia_opensim.m):
//    1. Filtro Butterworth 2a ordem passa-baixas (zero-phase, filtfilt)
//    2. Reamostragem para o passo Ts do controlador (interp linear/spline)
//    3. Derivacao numerica -> velocidade e aceleracao (tambem filtradas)
//
//  Resultado: vetores theta_ref, thetadot_ref, thetaddot_ref na taxa Ts,
//  prontos para publicar ponto a ponto.
// ============================================================================
#ifndef CONTROL_NODE__TRAJECTORY_PROC_HPP_
#define CONTROL_NODE__TRAJECTORY_PROC_HPP_

#include <vector>

namespace control_node
{

struct Trajectory
{
  double ts{0.001};                       // passo de tempo [s]
  std::vector<double> theta;              // [rad]
  std::vector<double> theta_dot;          // [rad/s]
  std::vector<double> theta_ddot;         // [rad/s^2]
  size_t size() const { return theta.size(); }
};

class TrajectoryProc
{
public:
  // raw_t, raw_v : tempo e valores brutos do .mot (em rad)
  // ts           : passo de saida [s]
  // fc           : corte do filtro Butterworth [Hz]
  static Trajectory process(const std::vector<double> & raw_t,
                            const std::vector<double> & raw_v,
                            double ts, double fc);

private:
  // Butterworth 2a ordem passa-baixas aplicado em filtfilt (zero-phase)
  static std::vector<double> butterFiltfilt(const std::vector<double> & x,
                                            double fc, double fs);
  // interpolacao linear de (t,v) na grade uniforme t_new
  static std::vector<double> interp(const std::vector<double> & t,
                                    const std::vector<double> & v,
                                    const std::vector<double> & t_new);
  // derivada numerica por diferencas centradas
  static std::vector<double> derivative(const std::vector<double> & x, double ts);
};

}  // namespace control_node

#endif  // CONTROL_NODE__TRAJECTORY_PROC_HPP_
