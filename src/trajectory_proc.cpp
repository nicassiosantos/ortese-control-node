// ============================================================================
//  trajectory_proc.cpp  -  filtro Butterworth, reamostragem e derivadas
// ============================================================================
#include "control_node/trajectory_proc.hpp"

#include <cmath>
#include <algorithm>

namespace control_node
{

// ---------------------------------------------------------------------------
//  Butterworth 2a ordem passa-baixas, aplicado em filtfilt (zero-phase).
//  Coeficientes via transformada bilinear com pre-warp do corte.
// ---------------------------------------------------------------------------
std::vector<double> TrajectoryProc::butterFiltfilt(
    const std::vector<double> & x, double fc, double fs)
{
  const size_t n = x.size();
  if (n < 4) return x;

  // pre-warp e coeficientes do Butterworth 2a ordem (bilinear)
  double wc = std::tan(M_PI * fc / fs);   // frequencia pre-warped
  double k1 = std::sqrt(2.0) * wc;
  double k2 = wc * wc;
  double a0 = k2 / (1.0 + k1 + k2);
  double a1 = 2.0 * a0;
  double a2 = a0;
  double b1 = 2.0 * (k2 - 1.0) / (1.0 + k1 + k2);
  double b2 = (1.0 - k1 + k2) / (1.0 + k1 + k2);

  auto applyOnce = [&](const std::vector<double> & in) {
    std::vector<double> out(in.size(), 0.0);
    double x1 = in[0], x2 = in[0], y1 = in[0], y2 = in[0];
    for (size_t i = 0; i < in.size(); ++i) {
      double xn = in[i];
      double yn = a0 * xn + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;
      out[i] = yn;
      x2 = x1; x1 = xn;
      y2 = y1; y1 = yn;
    }
    return out;
  };

  // filtfilt: filtra para frente, inverte, filtra de novo, inverte
  std::vector<double> fwd = applyOnce(x);
  std::reverse(fwd.begin(), fwd.end());
  std::vector<double> bwd = applyOnce(fwd);
  std::reverse(bwd.begin(), bwd.end());
  return bwd;
}

// ---------------------------------------------------------------------------
//  Interpolacao linear de (t,v) na grade t_new.
// ---------------------------------------------------------------------------
std::vector<double> TrajectoryProc::interp(
    const std::vector<double> & t, const std::vector<double> & v,
    const std::vector<double> & t_new)
{
  std::vector<double> out(t_new.size());
  size_t j = 0;
  for (size_t i = 0; i < t_new.size(); ++i) {
    double tq = t_new[i];
    while (j + 1 < t.size() && t[j + 1] < tq) ++j;
    if (j + 1 >= t.size()) { out[i] = v.back(); continue; }
    double t0 = t[j], t1 = t[j + 1];
    double frac = (t1 > t0) ? (tq - t0) / (t1 - t0) : 0.0;
    out[i] = v[j] + frac * (v[j + 1] - v[j]);
  }
  return out;
}

// ---------------------------------------------------------------------------
//  Derivada por diferencas centradas.
// ---------------------------------------------------------------------------
std::vector<double> TrajectoryProc::derivative(
    const std::vector<double> & x, double ts)
{
  const size_t n = x.size();
  std::vector<double> d(n, 0.0);
  if (n < 2) return d;
  d[0] = (x[1] - x[0]) / ts;
  d[n - 1] = (x[n - 1] - x[n - 2]) / ts;
  for (size_t i = 1; i + 1 < n; ++i) {
    d[i] = (x[i + 1] - x[i - 1]) / (2.0 * ts);
  }
  return d;
}

// ---------------------------------------------------------------------------
//  Pipeline completo.
// ---------------------------------------------------------------------------
Trajectory TrajectoryProc::process(
    const std::vector<double> & raw_t, const std::vector<double> & raw_v,
    double ts, double fc)
{
  Trajectory traj;
  traj.ts = ts;
  if (raw_t.size() < 2) return traj;

  double fs = 1.0 / ts;

  // 1) grade uniforme do primeiro ao ultimo instante
  double t0 = raw_t.front(), t1 = raw_t.back();
  size_t n = static_cast<size_t>((t1 - t0) / ts) + 1;
  std::vector<double> t_new(n);
  for (size_t i = 0; i < n; ++i) t_new[i] = t0 + i * ts;

  // 2) reamostra os angulos brutos para a grade
  std::vector<double> theta_rs = interp(raw_t, raw_v, t_new);

  // 3) filtra os angulos (6 Hz, zero-phase)
  traj.theta = butterFiltfilt(theta_rs, fc, fs);

  // 4) deriva -> velocidade, filtra
  std::vector<double> vd = derivative(traj.theta, ts);
  traj.theta_dot = butterFiltfilt(vd, fc, fs);

  // 5) deriva -> aceleracao, filtra
  std::vector<double> ad = derivative(traj.theta_dot, ts);
  traj.theta_ddot = butterFiltfilt(ad, fc, fs);

  return traj;
}

}  // namespace control_node
