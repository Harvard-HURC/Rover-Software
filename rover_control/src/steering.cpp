#include "rover_driver/steering.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace rover_driver {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Fits one module in place; returns true if it was reversed (odd k).
bool fit_module(ModuleCommand& m, double current, double limit) noexcept {
  if (!m.steer_defined) {
    m.steer_angle = std::isfinite(current) ? std::clamp(current, -limit, limit) : 0.0;
    m.wheel_rate = 0.0;
    return false;
  }
  const double psi = m.steer_angle;
  if (!std::isfinite(psi)) {
    return false;
  }
  // Every k with psi + k*pi in [-limit, limit]. The interval is longer than pi,
  // so it holds at least one.
  const int k_min = static_cast<int>(std::ceil((-limit - psi) / kPi));
  const int k_max = static_cast<int>(std::floor((limit - psi) / kPi));
  assert(k_min <= k_max);
  int best = k_min;
  for (int k = k_min + 1; k <= k_max; ++k) {
    if (std::abs(psi + k * kPi - current) < std::abs(psi + best * kPi - current)) {
      best = k;
    }
  }
  m.steer_angle = psi + best * kPi;
  const bool reversed = best % 2 != 0;
  if (reversed) {
    m.wheel_rate = -m.wheel_rate;
  }
  return reversed;
}

}  // namespace

PerWheel<ModuleCommand> fit_steer_range(const PerWheel<ModuleCommand>& cmd,
                                        const PerWheel<double>& current_steer,
                                        const RoverGeometry& g) noexcept {
  assert(g.steer_limit > kPi / 2);
  PerWheel<ModuleCommand> out = cmd;
  for (int i = 0; i < kNumWheels; ++i) {
    fit_module(out[i], current_steer[i], g.steer_limit);
  }
  return out;
}

PerWheel<ModuleCommand3d> fit_steer_range(const PerWheel<ModuleCommand3d>& cmd,
                                          const PerWheel<double>& current_steer,
                                          const RoverGeometry& g) noexcept {
  assert(g.steer_limit > kPi / 2);
  PerWheel<ModuleCommand3d> out = cmd;
  for (int i = 0; i < kNumWheels; ++i) {
    if (fit_module(out[i], current_steer[i], g.steer_limit)) {
      out[i].contact_angle = -out[i].contact_angle;
    }
  }
  return out;
}

}  // namespace rover_driver
