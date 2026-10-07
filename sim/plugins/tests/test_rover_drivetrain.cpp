// Unit tests of the drivetrain's Gazebo-free rules (rover_drivetrain.hh): the motor and driveline, the dig-in
// state, the friction of a wheel contact, the hub forces and the dust rate.
#include <cmath>

#include "check.hh"
#include "rover_drivetrain.hh"

using gz::math::Vector3d;
using rover_sim::Traction;
namespace drive = rover_sim::drive;

namespace {

constexpr double kDt = 0.001;              // [s] the worlds' physics step
constexpr double kWheelInertia = 0.028125;  // [kg m2] gen_model.wheel_inertia(2.5, 0.15, 0.1), about the axle

Traction Sand() {  // spec 5.6, mild dig
  Traction t;
  t.key = "sand";
  t.mu_s = t.mu_k = 0.52;
  t.crr = 0.20;
  t.bulldoze = 0.06;
  t.slip = 1.0;
  t.sinkage_m = 0.02;
  t.dig_rate = 0.01;
  t.dig_max = 1.25;
  t.dust = 0.8;
  return t;
}

/// Free speed: full voltage on a wheel that turns freely settles where the back-EMF and the motor's own
/// friction use up the supply: (V - R i) / (ke N), i = (i_free + b w / (N kt)) / efficiency.
void FreeSpeed() {
  const drive::MotorParams p;
  drive::WheelMotor motor;
  double wheel = 0.0;
  for (int k = 0; k < 4000; ++k) {
    wheel += kDt * motor.Step(p, 20.0, wheel, kDt) / kWheelInertia;
    CHECK(std::abs(motor.current) <= p.current_limit);
  }
  double w = 10.0;  // fixed point of the steady state
  for (int k = 0; k < 50; ++k) {
    const double i = (p.free_current + p.output_friction * w / (p.gear * p.kt)) / p.efficiency;
    w = (p.voltage - p.resistance * i) / (p.ke * p.gear);
  }
  CHECK_NEAR(wheel, w, 0.02 * w);
  CHECK(wheel < p.voltage / (p.ke * p.gear));  // 10.8 rad/s ideal (spec 6.3)
  CHECK(motor.saturated);                      // the setpoint is out of reach
}

/// Stall: a held wheel gets the current limit's torque, gear x efficiency x kt x I_lim = 35.6 N m.
void Stall() {
  const drive::MotorParams p;
  drive::WheelMotor motor;
  for (int k = 0; k < 3000; ++k) motor.Step(p, 10.0, 0.0, kDt);
  CHECK_NEAR(motor.current, p.current_limit, 1e-9);
  CHECK_NEAR(motor.torque, p.gear * p.efficiency * p.kt * p.current_limit, 0.3);
  // A lower current limit stalls at its own torque.
  drive::MotorParams low = p;
  low.current_limit = 7.0;
  drive::WheelMotor weak;
  for (int k = 0; k < 3000; ++k) weak.Step(low, -10.0, 0.0, kDt);
  CHECK_NEAR(weak.torque, -low.gear * low.efficiency * low.kt * low.current_limit, 0.3);
}

/// Backlash: the gearbox output turns through the 1.5 deg dead band before the shaft carries any torque
/// (about 6 ms here), then the torque builds.
void Backlash() {
  const drive::MotorParams p;
  drive::WheelMotor motor;
  for (int k = 0; k < 3; ++k) CHECK(motor.Step(p, 1.0, 0.0, kDt) == 0.0);
  double torque = 0.0;
  for (int k = 0; k < 30; ++k) torque = motor.Step(p, 1.0, 0.0, kDt);
  CHECK(torque > 1.0);
  drive::MotorParams tight = p;
  tight.backlash = 0.0;
  drive::WheelMotor direct;
  direct.Step(tight, 1.0, 0.0, kDt);
  CHECK(direct.Step(tight, 1.0, 0.0, kDt) > 0.0);
}

/// Dig-in: grows with slip at gain x dig_rate / sinkage per metre, stops at the capped D_max, heals by
/// exp(-travel / heal_length) on ground that does not dig.
void DigIn() {
  const Traction sand = Sand();
  drive::Dig mild;
  for (int k = 0; k < 500; ++k) mild.Step(sand, 1.0, 1.0, 0.4, 0.0, 0.3, kDt);
  CHECK_NEAR(mild.factor, 1.0 + 0.5 * 0.4 * 0.01 / 0.02, 1e-9);  // 0.5 s of slipping at 0.4 m/s
  for (int k = 0; k < 5000; ++k) mild.Step(sand, 1.0, 1.0, 0.4, 0.0, 0.3, kDt);
  CHECK_NEAR(mild.factor, 1.25, 1e-12);
  drive::Dig strong;
  for (int k = 0; k < 5000; ++k) strong.Step(sand, 5.0, 4.0, 0.4, 0.0, 0.3, kDt);
  CHECK_NEAR(strong.factor, 2.0, 1e-12);  // sand's strong preset: D_max 2.0, dig_rate 0.05 (spec 6.5)
  const Traction rock;  // digs nothing
  drive::Dig heal = strong;
  for (int k = 0; k < 2000; ++k) heal.Step(rock, 5.0, 4.0, 0.4, 0.5, 0.3, kDt);  // 1 m at 0.5 m/s
  CHECK_NEAR(heal.factor - 1.0, std::exp(-1.0 / 0.3), 0.002);
  CHECK(heal.factor < 1.05);
  drive::Dig still;
  for (int k = 0; k < 1000; ++k) still.Step(rock, 5.0, 4.0, 0.4, 0.0, 0.3, kDt);
  CHECK(still.factor == 1.0);
}

/// The friction of a contact: a circle along the slip with force-dependent slip while sliding, a box aligned
/// with the expected load and no compliance while sticking.
void Friction() {
  const drive::ContactParams p;
  const Traction sand = Sand();
  const Vector3d up = Vector3d::UnitZ, stick = Vector3d::UnitY;
  const auto sliding = drive::Friction(p, sand, 0.5, Vector3d(0.3, 0.4, 0.0), 0.4, 50.0, stick);
  CHECK_NEAR((sliding.direction - Vector3d(0.6, 0.8, 0.0)).Length(), 0.0, 1e-12);
  CHECK(sliding.mu1 == 0.5 && sliding.mu2 == 0.0);
  CHECK_NEAR(sliding.compliance, 1.0 * 0.4 / 50.0, 1e-12);
  const auto light = drive::Friction(p, sand, 0.5, Vector3d(0.3, 0.4, 0.0), 0.4, 1.0, stick);
  CHECK_NEAR(light.compliance, 0.4 / 5.0, 1e-12);  // loads under 5 N count as 5 N
  const auto stuck = drive::Friction(p, sand, 0.5, Vector3d(0.004, 0.0, 0.0), 0.4, 50.0, stick);
  CHECK(stuck.direction == stick && stuck.mu1 == 0.5 && stuck.compliance == 0.0);
  CHECK_NEAR(stuck.mu2, 0.15, 1e-12);

  // The sticking direction: the last tangential force, else downhill, else the heading.
  const Vector3d g(0, 0, -9.81), heading = Vector3d::UnitX;
  CHECK(drive::StickDirection(Vector3d(0, 5, 3), 100, g, heading, up) == Vector3d::UnitY);
  const Vector3d slope = Vector3d(-std::sin(0.3), 0, std::cos(0.3));  // rises towards +x
  const Vector3d downhill = drive::StickDirection(Vector3d::Zero, 100, g, heading, slope);
  CHECK_NEAR(downhill.Dot(slope), 0.0, 1e-12);
  CHECK(downhill.X() < 0 && downhill.Z() < 0);
  CHECK(drive::StickDirection(Vector3d(0, 0.01, 0), 100, g, heading, up) == Vector3d::UnitX);

  Traction slab;
  slab.mu_s = 1.0;
  slab.mu_k = 0.8;
  CHECK_NEAR(drive::StribeckMu(slab, 0.0, 0.03), 1.0, 1e-12);
  CHECK_NEAR(drive::StribeckMu(slab, 0.3, 0.03), 0.8, 1e-12);
  double lo = 2, hi = 0;
  for (int i = 0; i < 200; ++i) {
    const double f = drive::NoiseFactor(0.37 * i, -0.11 * i, 0.2, 0.3);
    lo = std::min(lo, f);
    hi = std::max(hi, f);
  }
  CHECK(lo >= 0.8 - 1e-9 && hi <= 1.2 + 1e-9 && hi - lo > 0.1);
  CHECK(drive::NoiseFactor(1.3, 2.1, 0.2, 0.3) == drive::NoiseFactor(1.3, 2.1, 0.2, 0.3));
  CHECK(drive::NoiseFactor(1.3, 2.1, 0.0, 0.3) == 1.0);
}

/// Rolling resistance opposes rolling along the heading, bulldozing (x D^2) sliding along the axle.
void HubForces() {
  const Traction sand = Sand();
  const Vector3d axle = Vector3d::UnitY, up = Vector3d::UnitZ;
  const Vector3d rolling = drive::HubForce(sand, 1.5, 100, Vector3d(0.5, 0, 0), axle, up, 0.03);
  CHECK_NEAR(rolling.X(), -0.20 * 1.5 * 100 * std::tanh(0.5 / 0.03), 1e-9);
  CHECK_NEAR(rolling.Y(), 0.0, 1e-9);
  const Vector3d sideways = drive::HubForce(sand, 1.5, 100, Vector3d(0, -0.5, 0), axle, up, 0.03);
  CHECK_NEAR(sideways.Y(), 0.06 * 2.25 * 100 * std::tanh(0.5 / 0.03), 1e-9);
  CHECK_NEAR(sideways.X(), 0.0, 1e-9);
  const Vector3d slope(-std::sin(0.3), 0, std::cos(0.3));
  CHECK_NEAR(drive::HubForce(sand, 1.0, 100, Vector3d(0.5, 0, 0.2), axle, slope, 0.03).Dot(slope), 0.0, 1e-9);
}

void Dust() {
  const drive::DustParams p;
  const Traction sand = Sand();
  CHECK(drive::DustRate(p, sand, 0.04, 0.01, 1.0, 100) == 0.0);  // too slow
  CHECK(drive::DustRate(p, sand, 0.5, 0.1, 1.0, 0.0) == 0.0);    // in the air
  CHECK_NEAR(drive::DustRate(p, sand, 0.5, 0.1, 1.0, 100), 0.8 * (8 * 0.5 + 25 * 0.1), 1e-9);
  CHECK(drive::DustRate(p, sand, 3.0, 2.0, 2.0, 100) == p.max_rate);
}

}  // namespace

int main() {
  FreeSpeed();
  Stall();
  Backlash();
  DigIn();
  Friction();
  HubForces();
  Dust();
  if (Failures() == 0) std::cout << "test_rover_drivetrain: all checks passed\n";
  return Failures() == 0 ? 0 : 1;
}
