// RoverDrive (research prototype, NOT repo code): wheel drivetrain + contact model for the
// skid-steer rover.
//
//  * <drive>motor</drive>: subscribes to <topic> (gz.msgs.Twist), runs per-wheel PI speed
//    control on a DC motor + gearbox (+ compliance/backlash) model and applies joint torques.
//    <drive>probe</drive>: applies nothing (use with DiffDrive), only logs.
//  * <rolling_resistance>: adds -Crr*N*r*tanh(w/w0) at each wheel joint (N from contacts).
//  * <contact>: customizes every wheel contact (gz-sim CollectContactSurfaceProperties event):
//      - friction "circle": fdir1 aligned with the contact slip velocity (box friction otherwise)
//      - Stribeck: mu = mu_k + (mu_s - mu_k) exp(-(v/v_s)^2)
//      - per-zone mu / Crr / slip compliance from rectangles <zone>xmin xmax ymin ymax mu_s mu_k crr slip</zone>
//  * <log>path</log>: binary float64 records per step (see kLogCols).
#include <gz/msgs/twist.pb.h>

#include <array>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/physics/Events.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace rover_sim {
namespace {
double clamp(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }

struct MotorParams {
  double V = 24, R = 0.46, kt = 0.0445, ke = 0.0445, N = 50, eta = 0.8, J_m = 1.2e-5;
  double I_lim = 20, i_free = 1.0, b_out = 0.05, k_s = 1500, c_s = 2.0, backlash = 0.026;
  double Kp = 4.0, Ki = 40.0, tau_meas = 0.005, accel = 8.0, w_max = 10.0, kff = -1;
  int substeps = 4;
};

struct WheelState {
  double w_o = 0, d = 0, integ = 0, w_meas = 0, sp = 0, i = 0, tau = 0;
  double Step(const MotorParams& p, double setpoint, double w_wheel, double dt) {
    const double J_r = p.J_m * p.N * p.N;
    const double target = clamp(setpoint, -p.w_max, p.w_max);
    if (p.accel > 0) {
      const double dmax = p.accel * dt;
      sp += clamp(target - sp, -dmax, dmax);
    } else {
      sp = target;
    }
    w_meas += dt / (p.tau_meas + dt) * (w_o - w_meas);
    const double e = sp - w_meas;
    const double ff = (p.kff < 0 ? p.ke * p.N : p.kff) * sp;
    const double u_unsat = ff + p.Kp * e + p.Ki * integ;
    const double u = clamp(u_unsat, -p.V, p.V);
    const double h = dt / p.substeps;
    double tau_sum = 0, cur = 0;
    const double half = p.backlash / 2;
    for (int k = 0; k < p.substeps; ++k) {
      cur = clamp((u - p.ke * p.N * w_o) / p.R, -p.I_lim, p.I_lim);
      const double tau_m = p.N * p.eta * p.kt * cur;
      const double tf = p.N * p.kt * p.i_free * std::tanh(w_o / 0.05) + p.b_out * w_o;
      double ts = 0;
      if (d > half) {
        ts = std::max(0.0, p.k_s * (d - half) + p.c_s * (w_o - w_wheel));
      } else if (d < -half) {
        ts = std::min(0.0, p.k_s * (d + half) + p.c_s * (w_o - w_wheel));
      }
      w_o += h * (tau_m - ts - tf) / J_r;
      d += h * (w_o - w_wheel);
      tau_sum += ts;
    }
    i = cur;
    const bool sat = (u_unsat > p.V && e > 0) || (u_unsat < -p.V && e < 0) ||
                     (std::abs(cur) >= p.I_lim && cur * e > 0);
    if (!sat) integ += e * dt;
    tau = tau_sum / p.substeps;
    return tau;
  }
};

// Smooth 2-D value noise in [-1, 1], deterministic per lattice cell.
double hash01(int x, int y, int seed) {
  uint32_t h = static_cast<uint32_t>(x) * 374761393u + static_cast<uint32_t>(y) * 668265263u +
               static_cast<uint32_t>(seed) * 2246822519u;
  h = (h ^ (h >> 13)) * 1274126177u;
  return ((h ^ (h >> 16)) & 0xffffff) / double(0xffffff);
}
double valueNoise(double x, double y, int seed) {
  const int xi = static_cast<int>(std::floor(x)), yi = static_cast<int>(std::floor(y));
  const double fx = x - xi, fy = y - yi;
  const double sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy);
  const double a = hash01(xi, yi, seed), b = hash01(xi + 1, yi, seed);
  const double c = hash01(xi, yi + 1, seed), d = hash01(xi + 1, yi + 1, seed);
  return 2 * ((a + (b - a) * sx) + ((c + (d - c) * sx) - (a + (b - a) * sx)) * sy) - 1;
}

struct Zone {
  double xmin, xmax, ymin, ymax, mu_s, mu_k, crr, slip;
};

constexpr int kWheels = 4;
const char* kWheelNames[kWheels] = {"fl", "rl", "fr", "rr"};
const double kSide[kWheels] = {+1, +1, -1, -1};
}  // namespace

class RoverDrive : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPreUpdate,
                   public gz::sim::ISystemPostUpdate {
 public:
  ~RoverDrive() override {
    if (log_) std::fclose(log_);
  }

  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& events) override {
    const gz::sim::Model model(entity);
    auto get = [&](const char* k, double def) { return sdf->Get<double>(k, def).first; };
    drive_ = sdf->Get<std::string>("drive", "probe").first;
    track_ = get("track", 0.8);
    radius_ = get("radius", 0.15);
    MotorParams& p = mp_;
    p.V = get("V", p.V); p.R = get("R", p.R); p.kt = get("kt", p.kt); p.ke = get("ke", p.ke);
    p.N = get("N", p.N); p.eta = get("eta", p.eta); p.J_m = get("J_m", p.J_m);
    p.I_lim = get("I_lim", p.I_lim); p.i_free = get("i_free", p.i_free); p.b_out = get("b_out", p.b_out);
    p.k_s = get("k_s", p.k_s); p.c_s = get("c_s", p.c_s); p.backlash = get("backlash", p.backlash);
    p.Kp = get("Kp", p.Kp); p.Ki = get("Ki", p.Ki); p.tau_meas = get("tau_meas", p.tau_meas);
    p.accel = get("accel", p.accel); p.w_max = get("w_max", p.w_max); p.kff = get("kff", p.kff);
    crr_ = get("rolling_resistance", 0.0);
    rr_w0_ = get("rr_w0", 0.2);
    contact_ = sdf->Get<bool>("contact", false).first;
    circle_ = sdf->Get<bool>("friction_circle", true).first;
    mu_s_ = get("mu_s", 1.0);
    mu_k_ = get("mu_k", 1.0);
    v_s_ = get("v_stribeck", 0.02);
    slip_ = get("slip_compliance", 0.0);
    perp_ratio_ = get("perp_ratio", 1.0);
    dump_ = sdf->Get<bool>("dump", false).first;
    noise_ = get("mu_noise", 0.0);
    lat_ratio_ = get("lat_ratio", 1.0);
    cfm_ = get("cfm", 0.0);
    erp_ = get("erp", -1.0);
    restitution_ = get("restitution", -1.0);
    rr_mode_ = sdf->Get<std::string>("rr_mode", "force").first;
    noise_l_ = get("mu_noise_scale", 0.3);
    v_align_ = get("v_align", 0.005);
    if (sdf->HasElement("zone")) {
      for (auto z = sdf->FindElement("zone"); z; z = z->GetNextElement("zone")) {
        std::istringstream in(z->Get<std::string>());
        Zone zz{};
        in >> zz.xmin >> zz.xmax >> zz.ymin >> zz.ymax >> zz.mu_s >> zz.mu_k >> zz.crr >> zz.slip;
        zones_.push_back(zz);
      }
    }
    for (int k = 0; k < kWheels; ++k) {
      const std::string w = kWheelNames[k];
      joints_[k] = gz::sim::Joint(model.JointByName(ecm, "wheel_" + w + "_joint"));
      links_[k] = gz::sim::Link(model.LinkByName(ecm, "wheel_" + w));
      joints_[k].EnablePositionCheck(ecm, true);
      joints_[k].EnableVelocityCheck(ecm, true);
      links_[k].EnableVelocityChecks(ecm, true);
      ecm.CreateComponent(joints_[k].Entity(), gz::sim::components::JointTransmittedWrench());
      for (auto c : links_[k].Collisions(ecm)) {
        collisions_[k] = c;
        ecm.CreateComponent(c, gz::sim::components::ContactSensorData());
        if (contact_) ecm.CreateComponent(c, gz::sim::components::EnableContactSurfaceCustomization(true));
      }
    }
    if (contact_) {
      conn_ = events.Connect<gz::sim::events::CollectContactSurfaceProperties>(
          [this](const gz::sim::Entity& c1, const gz::sim::Entity& c2, const gz::math::Vector3d& pt,
                 const std::optional<gz::math::Vector3d>, const std::optional<gz::math::Vector3d> normal,
                 const std::optional<double>, const size_t ncon,
                 gz::physics::SetContactPropertiesCallbackFeature::ContactSurfaceParams<
                     gz::sim::events::Policy>& params) { this->Customize(c1, c2, pt, normal, ncon, params); });
    }
    if (drive_ == "motor") {
      const auto topic = sdf->Get<std::string>("topic", "/model/rover/cmd_vel").first;
      node_.Subscribe(topic, &RoverDrive::OnCmd, this);
    }
    const auto log = sdf->Get<std::string>("log", "").first;
    if (!log.empty()) log_ = std::fopen(log.c_str(), "wb");
    gzmsg << "RoverDrive: drive=" << drive_ << " contact=" << contact_ << " zones=" << zones_.size()
          << " crr=" << crr_ << "\n";
  }

  void OnCmd(const gz::msgs::Twist& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    vx_ = msg.linear().x();
    wz_ = msg.angular().z();
  }

  const Zone* ZoneAt(double x, double y) const {
    for (const auto& z : zones_)
      if (x >= z.xmin && x <= z.xmax && y >= z.ymin && y <= z.ymax) return &z;
    return nullptr;
  }

  void Customize(const gz::sim::Entity& c1, const gz::sim::Entity& c2, const gz::math::Vector3d& pt,
                 const std::optional<gz::math::Vector3d>& normal, size_t ncon,
                 gz::physics::SetContactPropertiesCallbackFeature::ContactSurfaceParams<
                     gz::sim::events::Policy>& params) {
    int k = -1;
    for (int j = 0; j < kWheels; ++j)
      if (collisions_[j] == c1 || collisions_[j] == c2) k = j;
    if (k < 0) return;
    ++ncust_;
    // slip velocity of the wheel material point at the contact (ground is static)
    const gz::math::Vector3d r = pt - pos_[k];
    gz::math::Vector3d v = lin_[k] + ang_[k].Cross(r);
    gz::math::Vector3d n = normal ? *normal : gz::math::Vector3d::UnitZ;
    n.Normalize();
    const gz::math::Vector3d vt = v - n * v.Dot(n);
    const double s = vt.Length();
    double mu_s = mu_s_, mu_k = mu_k_, slip = slip_;
    if (const Zone* z = ZoneAt(pt.X(), pt.Y())) {
      mu_s = z->mu_s; mu_k = z->mu_k; slip = z->slip;
    }
    double mu = mu_k + (mu_s - mu_k) * std::exp(-(s / v_s_) * (s / v_s_));
    if (noise_ > 0) {
      const double n = 0.7 * valueNoise(pt.X() / noise_l_, pt.Y() / noise_l_, 1) +
                       0.3 * valueNoise(pt.X() * 3 / noise_l_, pt.Y() * 3 / noise_l_, 2);
      mu *= std::max(0.05, 1 + noise_ * n);
    }
    if (circle_ && s > v_align_) {
      const gz::math::Vector3d d = vt / s;
      if (lat_ratio_ != 1.0) {  // friction ellipse: mu_lat = lat_ratio * mu_lon
        const double dl = d.Dot(axle_[k]);
        const double dx2 = std::max(0.0, 1 - dl * dl);
        mu = mu / std::sqrt(dx2 + (dl / lat_ratio_) * (dl / lat_ratio_));
      }
      params.firstFrictionalDirection = Eigen::Vector3d(d.X(), d.Y(), d.Z());
      params.frictionCoeff = mu;
      params.secondaryFrictionCoeff = mu * perp_ratio_;
    } else {
      params.frictionCoeff = mu;
      params.secondaryFrictionCoeff = mu;
    }
    if (cfm_ > 0) params.constraintForceMixing = cfm_;
    if (erp_ >= 0) params.errorReductionParameter = erp_;
    if (restitution_ >= 0) params.restitutionCoeff = restitution_;
    if (slip > 0) {
      // unitless slip-ratio compliance -> DART [m/s/N] like gz WheelSlip: s * wheel speed / normal force
      const double vw = std::max(std::abs(w_[k] * radius_), 0.1);
      const double fn = std::max(fn_[k] / std::max<size_t>(ncon, 1), 5.0);
      params.slipCompliance = slip * vw / fn;
      params.secondarySlipCompliance = slip * vw / fn;
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (info.paused) return;
    dt_ = std::chrono::duration<double>(info.dt).count();
    for (int k = 0; k < kWheels; ++k) {
      if (auto p = links_[k].WorldPose(ecm)) {
        pos_[k] = p->Pos();
        axle_[k] = p->Rot().RotateVector(gz::math::Vector3d::UnitY);
      }
      if (auto v = links_[k].WorldLinearVelocity(ecm)) lin_[k] = *v;
      if (auto w = links_[k].WorldAngularVelocity(ecm)) ang_[k] = *w;
      auto v = joints_[k].Velocity(ecm);
      w_[k] = (v && !v->empty()) ? (*v)[0] : 0.0;
    }
    double vx, wz;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      vx = vx_;
      wz = wz_;
    }
    for (int k = 0; k < kWheels; ++k) {
      double tau = 0;
      if (drive_ == "motor") {
        const double sp = (vx - kSide[k] * wz * track_ / 2) / radius_;
        tau = ws_[k].Step(mp_, sp, w_[k], dt_);
      }
      double crr = crr_;
      if (const Zone* z = ZoneAt(pos_[k].X(), pos_[k].Y())) crr = z->crr;
      rr_[k] = -crr * fn_[k] * radius_ * std::tanh(w_[k] / rr_w0_);
      if (rr_mode_ == "force") {
        // compaction resistance as a horizontal force on the hub against its motion over the ground
        gz::math::Vector3d f = axle_[k].Cross(gz::math::Vector3d::UnitZ);
        if (f.Length() > 1e-6) {
          f.Normalize();
          const double vf = lin_[k].Dot(f);
          const double R = crr * fn_[k] * std::tanh(vf / (rr_w0_ * radius_));
          rr_[k] = -R * radius_;  // logged as the equivalent moment
          links_[k].AddWorldForce(ecm, -R * f);
        }
      } else if (rr_mode_ == "joint") {
        tau += rr_[k];
      } else if (rr_[k] != 0) {
        // the ground's resisting moment on the wheel, about its axle (world frame)
        links_[k].AddWorldWrench(ecm, gz::math::Vector3d::Zero, axle_[k] * rr_[k]);
      }
      applied_[k] = tau;
      if (drive_ == "motor" || (rr_mode_ == "joint" && crr != 0)) joints_[k].SetForce(ecm, {tau});
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override {
    if (info.paused) return;
    std::vector<double> rec;
    rec.push_back(std::chrono::duration<double>(info.simTime).count());
    for (int k = 0; k < kWheels; ++k) {
      double tq = NAN;
      if (auto w = ecm.Component<gz::sim::components::JointTransmittedWrench>(joints_[k].Entity()))
        tq = w->Data().torque().y();
      double fn = 0, ft = 0;
      int nc = 0;
      if (auto c = ecm.Component<gz::sim::components::ContactSensorData>(collisions_[k])) {
        for (const auto& contact : c->Data().contact()) {
          if (dump_ && info.iterations == 3000) {
            for (int i = 0; i < contact.position_size(); ++i) {
              const auto& wr = contact.wrench(i);
              std::cerr << "DUMP wheel " << kWheelNames[k] << " c1=" << contact.collision1().id() << " c2="
                    << contact.collision2().id() << " mine=" << collisions_[k] << " pos=" << contact.position(i).x()
                    << "," << contact.position(i).y() << "," << contact.position(i).z() << " n="
                    << (i < contact.normal_size() ? contact.normal(i).z() : -9) << " depth="
                    << (i < contact.depth_size() ? contact.depth(i) : -9) << " F1=" << wr.body_1_wrench().force().x()
                    << "," << wr.body_1_wrench().force().y() << "," << wr.body_1_wrench().force().z() << " F2="
                    << wr.body_2_wrench().force().x() << "," << wr.body_2_wrench().force().y() << ","
                    << wr.body_2_wrench().force().z() << "\n";
            }
          }
          for (int i = 0; i < contact.wrench_size(); ++i) {
            const auto& wr = contact.wrench(i);
            // body_1_wrench is on collision1; take the wheel's side
            const bool wheel_is_1 = contact.collision1().id() == collisions_[k];
            const auto& f = wheel_is_1 ? wr.body_1_wrench().force() : wr.body_2_wrench().force();
            gz::math::Vector3d n(0, 0, 1);
            if (i < contact.normal_size())
              n = gz::math::Vector3d(contact.normal(i).x(), contact.normal(i).y(), contact.normal(i).z());
            const gz::math::Vector3d F(f.x(), f.y(), f.z());
            fn += std::abs(F.Dot(n));
            ft += (F - n * F.Dot(n)).Length();
          }
          nc += contact.position_size();
        }
      }
      fn_[k] = fn;
      rec.insert(rec.end(), {tq, applied_[k], ws_[k].i, ws_[k].d, ws_[k].w_o, ws_[k].sp, fn, ft,
                             static_cast<double>(nc), rr_[k]});
    }
    rec.push_back(static_cast<double>(ncust_));
    if (log_) std::fwrite(rec.data(), sizeof(double), rec.size(), log_);
  }

 private:
  std::string drive_;
  double track_ = 0.8, radius_ = 0.15, dt_ = 0.001;
  MotorParams mp_;
  std::array<WheelState, kWheels> ws_{};
  std::array<gz::sim::Joint, kWheels> joints_;
  std::array<gz::sim::Link, kWheels> links_;
  std::array<gz::sim::Entity, kWheels> collisions_{};
  std::array<gz::math::Vector3d, kWheels> pos_, lin_, ang_, axle_;
  std::array<double, kWheels> w_{}, fn_{}, applied_{}, rr_{};
  double crr_ = 0, rr_w0_ = 0.2;
  bool contact_ = false, circle_ = true;
  double mu_s_ = 1, mu_k_ = 1, v_s_ = 0.02, slip_ = 0, perp_ratio_ = 1, v_align_ = 0.005;
  std::vector<Zone> zones_;
  long ncust_ = 0;
  bool dump_ = false;
  double noise_ = 0, noise_l_ = 0.3, lat_ratio_ = 1.0, cfm_ = 0, erp_ = -1, restitution_ = -1;
  std::string rr_mode_ = "ground";
  gz::common::ConnectionPtr conn_;
  gz::transport::Node node_;
  std::mutex mutex_;
  double vx_ = 0, wz_ = 0;
  FILE* log_ = nullptr;
};

}  // namespace rover_sim

GZ_ADD_PLUGIN(rover_sim::RoverDrive, gz::sim::System, rover_sim::RoverDrive::ISystemConfigure,
              rover_sim::RoverDrive::ISystemPreUpdate, rover_sim::RoverDrive::ISystemPostUpdate)
