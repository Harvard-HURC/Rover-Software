// The parts of the rover's drivetrain (rover_drivetrain.cpp) that need no Gazebo: the wheel motor, the dig-in
// state, the friction rule of a wheel contact, the hub forces and the dust rate. Design spec
// docs/superpowers/specs/2026-10-06-urc-realism-design.md, sections 6.3-6.5; ported from the validated prototype
// sim/data/research/drive/prototype/cpp/rover_drive.cpp. sim/plugins/tests/ tests them without a simulation.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <gz/math/Vector3.hh>

#include "terrain_ground.hh"

namespace rover_sim::drive {

inline double Clamp(double x, double lo, double hi) { return std::min(std::max(x, lo), hi); }

/// One wheel's DC motor, gearbox, driveline and speed controller (spec 6.3 table; D20). Defaults: the
/// prototype's validated set.
struct MotorParams {
  double voltage = 24.0;          ///< [V] supply (Husky A200 motor, R [18])
  double resistance = 0.46;       ///< [ohm] (R [18])
  double kt = 0.0445;             ///< [N m/A] (R [18])
  double ke = 0.0445;             ///< [V s/rad] (R [18])
  double gear = 50.0;             ///< reduction (M: prototype)
  double efficiency = 0.8;        ///< gearbox (M: prototype)
  double rotor_inertia = 1.2e-5;  ///< [kg m2] at the motor; x gear^2 = 0.03 at the wheel (A)
  double free_current = 1.0;      ///< [A] no-load current: Coulomb friction (A)
  double output_friction = 0.05;  ///< [N m s/rad] viscous friction at the output (A)
  double current_limit = 20.0;    ///< [A] -> 35.6 N m at the wheel (M: prototype)
  double stiffness = 1500.0;      ///< [N m/rad] driveline (A: 12 mm x 0.1 m steel shaft)
  double damping = 2.0;           ///< [N m s/rad] driveline (A)
  double backlash = 0.026;        ///< [rad] total, 1.5 deg (A: IMS 0.8-2.5 deg [22])
  double kp = 4.0;                ///< [V/(rad/s)] (M: prototype tuning)
  double ki = 40.0;               ///< [V/rad] (M: prototype tuning)
  double speed_filter = 0.005;    ///< [s] time constant of the measured speed (A)
  int substeps = 4;               ///< motor integration steps per physics step
};

/// Smoothing speed of the motor's Coulomb friction [rad/s] (A: the prototype's).
constexpr double kFrictionSpeed = 0.05;

/// The motor of one wheel. Step() advances it by one physics step towards a wheel-speed setpoint and returns
/// the torque the driveline applies to the wheel; Gazebo applies it as the wheel joint's force.
class WheelMotor {
 public:
  double current = 0.0;    ///< [A] at the end of the step
  double torque = 0.0;     ///< [N m] on the wheel, mean over the step's substeps
  double voltage = 0.0;    ///< [V] commanded
  bool saturated = false;  ///< the voltage or the current limit held the controller back

  /// sp: setpoint [rad/s] after the ramp; wheel: the wheel's speed [rad/s]; dt [s].
  double Step(const MotorParams& p, double sp, double wheel, double dt) {
    const double reflected = p.rotor_inertia * p.gear * p.gear;  // motor inertia seen at the output
    measured_ += dt / (p.speed_filter + dt) * (output_ - measured_);
    const double error = sp - measured_;
    const double unsaturated = p.ke * p.gear * sp + p.kp * error + p.ki * integral_;  // feed-forward + PI
    voltage = Clamp(unsaturated, -p.voltage, p.voltage);
    const double h = dt / p.substeps, half = p.backlash / 2;
    double sum = 0.0;
    for (int k = 0; k < p.substeps; ++k) {
      current = Clamp((voltage - p.ke * p.gear * output_) / p.resistance, -p.current_limit, p.current_limit);
      const double motor = p.gear * p.efficiency * p.kt * current;
      const double friction =
          p.gear * p.kt * p.free_current * std::tanh(output_ / kFrictionSpeed) + p.output_friction * output_;
      // The shaft twist beyond the backlash dead band transmits torque; the wheel side is Gazebo's.
      double shaft = 0.0;
      if (twist_ > half) {
        shaft = std::max(0.0, p.stiffness * (twist_ - half) + p.damping * (output_ - wheel));
      } else if (twist_ < -half) {
        shaft = std::min(0.0, p.stiffness * (twist_ + half) + p.damping * (output_ - wheel));
      }
      output_ += h * (motor - shaft - friction) / reflected;
      twist_ += h * (output_ - wheel);
      sum += shaft;
    }
    saturated = (unsaturated > p.voltage && error > 0) || (unsaturated < -p.voltage && error < 0) ||
                (std::abs(current) >= p.current_limit && current * error > 0);
    if (!saturated) integral_ += error * dt;  // no wind-up against a limit
    torque = sum / p.substeps;
    return torque;
  }

  void Reset() { *this = WheelMotor(); }

 private:
  double output_ = 0.0;    // gearbox output speed [rad/s]
  double twist_ = 0.0;     // output angle minus wheel angle [rad]
  double integral_ = 0.0;  // [rad]
  double measured_ = 0.0;  // filtered output speed [rad/s]
};

/// A wheel's dig factor D (spec 6.5, D21): how far it has dug into loose ground, as total over static sinkage.
/// It scales that wheel's rolling resistance by D and its bulldozing by D^2.
///   dD/dt = gain x dig_rate / sinkage_m x slip speed - (D - 1) x hub speed / heal_length
/// On ground that digs D stays within [1, gain-scaled dig_max]; elsewhere it only heals. Within one ground this
/// is the spec's extra sinkage z_d = (D - 1) sinkage_m; across grounds the factor carries over (A).
struct Dig {
  double factor = 1.0;

  /// rate_gain, max_gain: the preset (mild 1, 1; strong 5, 4: sand at D_max 2.0, dig_rate 0.05, spec 6.5).
  void Step(const Traction& ground, double rate_gain, double max_gain, double slip, double hub_speed,
            double heal_length, double dt) {
    const double grow = ground.Digs() ? rate_gain * ground.dig_rate / ground.sinkage_m * slip : 0.0;
    // Implicit in the healing term: stable for any step and travel speed.
    factor = 1.0 + ((factor - 1.0) + dt * grow) / (1.0 + dt * hub_speed / heal_length);
    if (ground.Digs()) factor = std::min(factor, 1.0 + max_gain * (ground.dig_max - 1.0));
  }
};

// --- Friction of a wheel contact (spec 6.4) ---------------------------------------------------------------------

/// Stribeck: the static coefficient at rest, the kinetic one once sliding faster than v_stribeck.
inline double StribeckMu(const Traction& t, double slip, double v_stribeck) {
  const double x = slip / v_stribeck;
  return t.mu_k + (t.mu_s - t.mu_k) * std::exp(-x * x);
}

/// Smooth 2-D value noise in [-1, 1], the same for the same point and seed.
inline double ValueNoise(double x, double y, int seed) {
  const auto hash = [seed](int i, int j) {
    uint32_t h = static_cast<uint32_t>(i) * 374761393u + static_cast<uint32_t>(j) * 668265263u +
                 static_cast<uint32_t>(seed) * 2246822519u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return ((h ^ (h >> 16)) & 0xffffff) / double(0xffffff);
  };
  const int i = static_cast<int>(std::floor(x)), j = static_cast<int>(std::floor(y));
  const double fx = x - i, fy = y - j;
  const double sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy);
  const double low = hash(i, j) + (hash(i + 1, j) - hash(i, j)) * sx;
  const double high = hash(i, j + 1) + (hash(i + 1, j + 1) - hash(i, j + 1)) * sx;
  return 2 * (low + (high - low) * sy) - 1;
}

/// Spatial friction variation: mu x max(0.05, 1 + sigma (0.7 n(p/L) + 0.3 n(3p/L))) (A: sigma 0.2, L 0.3 m).
inline double NoiseFactor(double x, double y, double sigma, double length) {
  if (sigma <= 0) return 1.0;
  const double n = 0.7 * ValueNoise(x / length, y / length, 1) + 0.3 * ValueNoise(3 * x / length, 3 * y / length, 2);
  return std::max(0.05, 1 + sigma * n);
}

/// The unit vector along `v` within the plane of `normal`, or zero if it has no part there.
inline gz::math::Vector3d InPlane(const gz::math::Vector3d& v, const gz::math::Vector3d& normal) {
  const gz::math::Vector3d t = v - normal * v.Dot(normal);
  const double length = t.Length();
  return length > 1e-9 ? t / length : gz::math::Vector3d::Zero;
}

/// The friction box of a sticking contact is aligned with the load it is expected to hold: the previous step's
/// tangential force at the wheel, else the downhill pull of gravity, else the wheel's heading.
inline gz::math::Vector3d StickDirection(const gz::math::Vector3d& previous_force, double load,
                                         const gz::math::Vector3d& gravity, const gz::math::Vector3d& heading,
                                         const gz::math::Vector3d& normal) {
  const gz::math::Vector3d force = previous_force - normal * previous_force.Dot(normal);
  if (force.Length() > 1e-3 * std::max(load, 1.0)) return force.Normalized();
  const gz::math::Vector3d downhill = gravity - normal * gravity.Dot(normal);
  if (downhill.Length() > 1e-3 * gravity.Length()) return downhill.Normalized();
  return InPlane(heading, normal);
}

/// What the contact callback gives DART for one wheel contact.
struct ContactFriction {
  gz::math::Vector3d direction;  ///< first friction direction, in the contact plane
  double mu1 = 0.0, mu2 = 0.0;   ///< along and across it
  double compliance = 0.0;       ///< slip compliance, both directions [m/s/N]
};

/// Parameters of the friction rule (spec 6.2 <contact>).
struct ContactParams {
  double v_stribeck = 0.03;       ///< [m/s] (A)
  double v_align = 0.005;         ///< [m/s] below it a contact sticks (A)
  double perp_ratio = 0.0;        ///< mu across the slip while sliding: 0 is a friction circle
  double stick_perp_ratio = 0.3;  ///< across the expected load while sticking: holds within 4.4 % of mu
  double mu_noise = 0.2;          ///< (A)
  double mu_noise_length = 0.3;   ///< [m] (A)
};

/// The friction of one wheel contact. mu: the ground's coefficient at this contact (StribeckMu x NoiseFactor);
/// slip_velocity: the wheel's material point relative to the ground, in the contact plane; wheel_speed: |w r|
/// [m/s]; load: the wheel's load over its contact count [N]; stick: the sticking direction (StickDirection).
/// Sliding contacts get a friction circle (friction along the slip only) and the force-dependent slip of gz's
/// WheelSlip convention, slip x wheel speed / load, with no speed floor (D23); sticking contacts an aligned box
/// and no compliance, so a parked rover does not creep.
inline ContactFriction Friction(const ContactParams& p, const Traction& ground, double mu,
                                const gz::math::Vector3d& slip_velocity, double wheel_speed, double load,
                                const gz::math::Vector3d& stick) {
  ContactFriction out;
  const double s = slip_velocity.Length();
  out.mu1 = mu;
  if (s > p.v_align) {
    out.direction = slip_velocity / s;
    out.mu2 = mu * p.perp_ratio;
    out.compliance = ground.slip * wheel_speed / std::max(load, 5.0);
  } else {
    out.direction = stick;
    out.mu2 = mu * p.stick_perp_ratio;
  }
  return out;
}

// --- Hub forces (spec 6.5, D5) ------------------------------------------------------------------------------------

/// Rolling resistance along the wheel's heading and bulldozing along its axle, both in the contact plane, as
/// one force on the hub [N]: -crr D N tanh(v_f / v0) f - bulldoze D^2 N tanh(v_l / v0) a, v0 = rr_w0 x radius.
/// A hub force, not a joint torque: the speed controller would cancel a torque inside the joint (D5).
inline gz::math::Vector3d HubForce(const Traction& ground, double dig, double load, const gz::math::Vector3d& velocity,
                                   const gz::math::Vector3d& axle, const gz::math::Vector3d& normal, double v0) {
  const gz::math::Vector3d along = axle.Cross(normal).Normalized();
  const gz::math::Vector3d across = InPlane(axle, normal);
  return -along * (ground.crr * dig * load * std::tanh(velocity.Dot(along) / v0)) -
         across * (ground.bulldoze * dig * dig * load * std::tanh(velocity.Dot(across) / v0));
}

/// Dust behind a wheel (spec 6.5, A gains): particles per second.
struct DustParams {
  double speed_gain = 8.0;   ///< [1/m] per m/s of hub speed
  double slip_gain = 25.0;   ///< [1/m] per m/s of slip
  double max_rate = 40.0;    ///< [1/s]
  double min_speed = 0.05;   ///< [m/s] below it nothing is raised
};

/// 0 when the wheel neither rolls nor slips faster than min_speed, or is in the air (load 0).
inline double DustRate(const DustParams& p, const Traction& ground, double hub_speed, double slip, double dig,
                       double load) {
  if (load <= 0 || std::max(hub_speed, slip) < p.min_speed) return 0.0;
  return std::min(p.max_rate, ground.dust * (p.speed_gain * hub_speed + p.slip_gain * slip) * dig);
}

}  // namespace rover_sim::drive
