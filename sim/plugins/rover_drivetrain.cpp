// RoverDrivetrain: a gz-sim system that drives the rover's wheels the way its motors would, and decides how
// every wheel contact grips (design spec docs/superpowers/specs/2026-10-06-urc-realism-design.md, section 6;
// port of the validated prototype sim/data/research/drive/prototype/cpp/rover_drive.cpp).
//
// Every 1 ms step:
// - cmd_vel (vx, wz) -> wheel-speed setpoints (vx -+ wz track track_multiplier / 2) / radius, left -, right +,
//   through a ramp (gz::math::SpeedLimiter); a command older than <cmd_timeout> counts as zero.
// - each wheel's DC motor, gearbox, driveline and PI speed controller (rover_drivetrain.hh) give a torque,
//   applied as the wheel joint's force. Gazebo's DiffDrive must not be in the same model: it makes each wheel a
//   velocity servo and overrides the torque (D2, D22).
// - every wheel contact, on any shape, gets its friction from the ground under it (contact callback, D1):
//   a friction circle along the slip while sliding, a box aligned with the expected load while sticking,
//   Stribeck on firm ground, smooth spatial noise and force-dependent slip (D3, D23). DART's default rule,
//   min(mu) per box direction, cannot turn a skid-steer rover in place, so no wheel contact is left to it.
//   The rule reads a wheel's spin through the controller's 5 ms speed filter (OnContact says why).
// - rolling resistance and bulldozing as hub forces, scaled by each wheel's dig-in state (D5, D21).
// - the dust emitters behind the rear wheels get a rate from speed, slip and the ground (D15).
// - no-slip odometry (gz::math::DiffDriveOdometry) and tf as DiffDrive publishes them, and the drivetrain's
//   state as JSON (spec 9.3).
//
// The ground: the world's ground map (terrain_ground.hh: ground.png under the heightmap, ground.json's
// collision map for the terrain model's other shapes); a plane is ground of the default surface; any other
// model grips with the mu its SDF <surface> sets (as mu_s = mu_k, crr 0.015, slip 0.05, spec 5.6) or as the
// object surface. Without a ground map the default and object surfaces come from this plugin's <surface>
// rows (gen_model.py writes them from the catalogue, sim/urc/terrains.py).
//
// Wheel loads come from each wheel joint's transmitted wrench (gate G8: within 2.6 % of contact loads, and
// cheaper): the contact force is minus the joint force, the wheel's weight and the hub force; its part along
// the contact normal is the load, the rest the tangential force a sticking contact is aligned with.
//
// A wheel's slip compliance is set per contact from that contact's own load in the previous step, not from
// the wheel's load over its contact count: a cylinder on the ground touches at both tread edges, and how the
// two share the load is the solver's choice. DART's Dantzig splits it evenly; its PGS leaves nearly all of
// it on the first contact it visits (measured 90 / 2 N), so with even shares the light contact's friction
// was capped at mu x 2 N, the wheel kept one contact's compliance and slipped 1.85x the design in sand (0.37
// instead of 0.20). The split comes from the same transmitted wrench: the contact forces' moment about the
// wheel's heading axis is sum(y_i N_i) + rho F_axial (y_i: a contact's offset along the axle, rho: the
// radius), which fixes the two edges' loads (more contacts: a linear pressure across the tread). The
// wheels' ContactSensorData would give the same for +11 % CPU time per step (measured on rover_test and
// Delivery).
//
// SDF: see gen_model.py (_add_drivetrain), which writes every element; units SI.

#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>

#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/DiffDriveOdometry.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/SpeedLimiter.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/physics/Events.hh>
#include <gz/transport/Node.hh>
#include <sdf/Collision.hh>
#include <sdf/Element.hh>
#include <sdf/Geometry.hh>

#include "rover_drivetrain.hh"
#include "terrain_ground.hh"

namespace rover_sim {
namespace {

using gz::math::Vector3d;
using Clock = std::chrono::steady_clock;

// Objects whose SDF sets a friction coefficient: mu_s = mu_k = that mu, with these (spec 5.6).
constexpr double kObjectCrr = 0.015;  // rubber on concrete (R [6])
constexpr double kObjectSlip = 0.05;  // as rock (A)

double Seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

double Child(const sdf::ElementConstPtr& parent, const char* group, const char* key, double fallback) {
  if (!parent->HasElement(group)) return fallback;
  return parent->FindElement(group)->Get<double>(key, fallback).first;
}

/// A wheel contact of the previous step and the load it carried, matched to this step's by position.
struct PastContact {
  Vector3d point;
  double load = 0.0;  // [N] along the contact normal
};

/// The farthest a contact point moves between steps and still counts as the same contact: 1 m/s x 1 ms is
/// 1 mm; the two tread edges of a wheel are its width (0.10 m) apart (A).
constexpr double kContactMatch = 0.02;  // [m]

/// The deepest contact of a wheel in one step.
struct Contact {
  bool touched = false;
  double depth = -std::numeric_limits<double>::infinity();
  const Traction* ground = nullptr;
  Vector3d point, normal = Vector3d::UnitZ;
  double slip = 0.0;  // [m/s] the wheel's material point over the ground
};

struct Wheel {
  std::string name;  // fl, rl, fr, rr
  double side = 1.0;  // +1 left, -1 right
  gz::sim::Joint joint;
  gz::sim::Link link;
  std::vector<gz::sim::Entity> collisions;
  double mass = 0.0;
  gz::math::Quaterniond joint_frame;  // the joint's frame in the wheel's
  gz::math::SpeedLimiter ramp;
  double setpoint = 0.0;  // [rad/s] after the ramp
  drive::WheelMotor motor;
  drive::Dig dig;
  // This step's kinematics (PreUpdate), read by the contact callback.
  gz::math::Pose3d pose;
  Vector3d velocity, angular, axle = Vector3d::UnitY;
  double speed = 0.0, angle = 0.0;  // joint [rad/s], [rad]
  double spin = 0.0;  // joint speed through the controller's speed filter [rad/s]
  // From the last PostUpdate: load [N] and the contact force's tangential part [N].
  double load = 0.0;
  Vector3d tangential;
  Vector3d hub_force;  // applied this step [N]
  Contact last, current;  // the previous step's contact, and the one the callback fills in this step
  std::vector<Vector3d> points;  // this step's contact points (the callback)
  std::vector<PastContact> past;  // the previous step's contacts with their loads (EdgeLoads)
  std::string dust_topic;
  gz::transport::Node::Publisher dust;
  bool emitting = false;
};

}  // namespace

class RoverDrivetrain : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate,
                        public gz::sim::ISystemPostUpdate,
                        public gz::sim::ISystemReset {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& events) override {
    model_ = gz::sim::Model(entity);
    const auto get = [&](const char* key, double fallback) { return sdf->Get<double>(key, fallback).first; };
    const auto text = [&](const char* key, const std::string& fallback) {
      return sdf->Get<std::string>(key, fallback).first;
    };
    cmd_timeout_ = get("cmd_timeout", cmd_timeout_);
    wall_timeout_ = text("cmd_timeout_clock", "wall") == "wall";
    track_ = get("track", track_) * get("track_multiplier", 1.0);
    radius_ = get("radius", radius_);
    odom_period_ = 1.0 / get("odom_publish_frequency", 50.0);
    state_period_ = 1.0 / get("state_rate", 50.0);
    dust_period_ = 1.0 / get("dust_rate", 10.0);
    frame_id_ = text("frame_id", "odom");
    child_frame_id_ = text("child_frame_id", "base_link");

    drive::MotorParams& m = motor_;
    m.voltage = Child(sdf, "motor", "voltage", m.voltage);
    m.resistance = Child(sdf, "motor", "resistance", m.resistance);
    m.kt = Child(sdf, "motor", "kt", m.kt);
    m.ke = Child(sdf, "motor", "ke", m.ke);
    m.gear = Child(sdf, "motor", "gear", m.gear);
    m.efficiency = Child(sdf, "motor", "efficiency", m.efficiency);
    m.rotor_inertia = Child(sdf, "motor", "rotor_inertia", m.rotor_inertia);
    m.free_current = Child(sdf, "motor", "free_current", m.free_current);
    m.output_friction = Child(sdf, "motor", "output_friction", m.output_friction);
    m.current_limit = Child(sdf, "motor", "current_limit", m.current_limit);
    m.stiffness = Child(sdf, "driveline", "stiffness", m.stiffness);
    m.damping = Child(sdf, "driveline", "damping", m.damping);
    m.backlash = Child(sdf, "driveline", "backlash", m.backlash);
    m.kp = Child(sdf, "controller", "kp", m.kp);
    m.ki = Child(sdf, "controller", "ki", m.ki);
    m.speed_filter = Child(sdf, "controller", "speed_filter", m.speed_filter);
    m.substeps = std::max(1, static_cast<int>(Child(sdf, "controller", "substeps", m.substeps)));
    accel_ = Child(sdf, "controller", "accel", 8.0);
    max_speed_ = Child(sdf, "controller", "max_speed", 10.0);

    drive::ContactParams& c = contact_;
    c.v_stribeck = Child(sdf, "contact", "v_stribeck", c.v_stribeck);
    c.v_align = Child(sdf, "contact", "v_align", c.v_align);
    c.perp_ratio = Child(sdf, "contact", "perp_ratio", c.perp_ratio);
    c.stick_perp_ratio = Child(sdf, "contact", "stick_perp_ratio", c.stick_perp_ratio);
    c.mu_noise = Child(sdf, "contact", "mu_noise", c.mu_noise);
    c.mu_noise_length = Child(sdf, "contact", "mu_noise_length", c.mu_noise_length);
    rr_speed_ = Child(sdf, "contact", "rr_w0", 0.05) * radius_;
    dig_heal_length_ = Child(sdf, "contact", "dig_heal_length", 0.3);
    if (sdf->HasElement("contact")) {
      const auto contact = sdf->FindElement("contact");
      dig_ = contact->Get<bool>("dig", true).first;
      default_key_ = contact->Get<std::string>("default_surface", default_key_).first;
      object_key_ = contact->Get<std::string>("object_surface", object_key_).first;
      for (auto row = contact->FindElement("surface"); row; row = row->GetNextElement("surface")) {
        Traction t;
        t.key = row->Get<std::string>("key", "").first;
        t.mu_s = row->Get<double>("mu_s", t.mu_s).first;
        t.mu_k = row->Get<double>("mu_k", t.mu_k).first;
        t.crr = row->Get<double>("crr", t.crr).first;
        t.bulldoze = row->Get<double>("bulldoze", t.bulldoze).first;
        t.slip = row->Get<double>("slip", t.slip).first;
        t.sinkage_m = row->Get<double>("sinkage_m", t.sinkage_m).first;
        t.dig_rate = row->Get<double>("dig_rate", t.dig_rate).first;
        t.dig_max = row->Get<double>("dig_max", t.dig_max).first;
        t.dust = row->Get<double>("dust", t.dust).first;
        sdf_surfaces_.push_back(t);
      }
    }
    dust_.speed_gain = Child(sdf, "dust_rule", "speed_gain", dust_.speed_gain);
    dust_.slip_gain = Child(sdf, "dust_rule", "slip_gain", dust_.slip_gain);
    dust_.max_rate = Child(sdf, "dust_rule", "max_rate", dust_.max_rate);
    dust_.min_speed = Child(sdf, "dust_rule", "min_speed", dust_.min_speed);

    for (auto e = sdf->FindElement("wheel"); e; e = e->GetNextElement("wheel")) {
      Wheel& w = wheels_.emplace_back();
      w.name = e->Get<std::string>("name", "").first;
      w.side = e->Get<std::string>("side", "left").first == "right" ? -1.0 : 1.0;
      const auto joint = e->Get<std::string>("joint", "").first;
      const auto link = e->Get<std::string>("link", "").first;
      w.joint = gz::sim::Joint(model_.JointByName(ecm, joint));
      w.link = gz::sim::Link(model_.LinkByName(ecm, link));
      if (!w.joint.Valid(ecm) || !w.link.Valid(ecm)) {
        gzerr << "RoverDrivetrain: wheel joint '" << joint << "' or link '" << link << "' is not in model '"
              << model_.Name(ecm) << "'; the drivetrain is disabled.\n";
        wheels_.clear();
        return;
      }
      w.ramp.SetMinVelocity(-max_speed_);
      w.ramp.SetMaxVelocity(max_speed_);
      if (accel_ > 0) {
        w.ramp.SetMinAcceleration(-accel_);
        w.ramp.SetMaxAcceleration(accel_);
      }
      if (const auto inertial = ecm.Component<gz::sim::components::Inertial>(w.link.Entity())) {
        w.mass = inertial->Data().MassMatrix().Mass();
      }
      w.joint_frame = w.joint.Pose(ecm).value_or(gz::math::Pose3d::Zero).Rot();
      w.joint.EnablePositionCheck(ecm, true);
      w.joint.EnableVelocityCheck(ecm, true);
      w.joint.EnableTransmittedWrenchCheck(ecm, true);
      w.link.EnableVelocityChecks(ecm, true);
      for (const auto collision : w.link.Collisions(ecm)) {
        w.collisions.push_back(collision);
        ecm.CreateComponent(collision, gz::sim::components::EnableContactSurfaceCustomization(true));
      }
    }
    for (auto e = sdf->FindElement("dust"); e; e = e->GetNextElement("dust")) {
      const auto link = e->Get<std::string>("wheel", "").first;
      for (auto& w : wheels_) {
        if (w.link.Name(ecm) == link) {
          w.dust_topic = e->Get<std::string>("topic", "").first;
          w.dust = node_.Advertise<gz::msgs::ParticleEmitter>(w.dust_topic);
        }
      }
    }
    if (wheels_.empty()) {
      gzerr << "RoverDrivetrain: no <wheel>; the drivetrain is disabled.\n";
      return;
    }

    odometry_.SetWheelParams(track_, radius_, radius_);
    node_.Subscribe(text("topic", "/model/rover/cmd_vel"), &RoverDrivetrain::OnCmd, this);
    odom_pub_ = node_.Advertise<gz::msgs::Odometry>(text("odom_topic", "/model/rover/odometry"));
    tf_pub_ = node_.Advertise<gz::msgs::Pose_V>(text("tf_topic", "/model/rover/tf"));
    state_pub_ = node_.Advertise<gz::msgs::StringMsg>(text("state_topic", "/model/rover/drivetrain"));
    connection_ = events.Connect<gz::sim::events::CollectContactSurfaceProperties>(
        [this](const gz::sim::Entity& c1, const gz::sim::Entity& c2, const Vector3d& point,
               const std::optional<Vector3d>, const std::optional<Vector3d> normal, const std::optional<double> depth,
               const size_t count,
               gz::physics::SetContactPropertiesCallbackFeature::ContactSurfaceParams<gz::sim::events::Policy>&
                   params) { this->OnContact(c1, c2, point, normal, depth, count, params); });
    enabled_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (!enabled_ || info.paused) return;
    const double dt = Seconds(info.dt), t = Seconds(info.simTime);
    if (dt <= 0) return;
    if (!world_ready_) FindGround(ecm);
    ClassifyCollisions(ecm);

    double vx, wz, age;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sim_time_ = t;
      vx = cmd_[0];
      wz = cmd_[1];
      age = wall_timeout_ ? Seconds(Clock::now() - cmd_wall_) : t - cmd_sim_;
    }
    if (cmd_timeout_ > 0 && age > cmd_timeout_) vx = wz = 0.0;
    applied_cmd_ = {vx, wz};

    for (auto& w : wheels_) {
      w.last = w.current;
      w.current = Contact();
      w.points.clear();
      if (const auto pose = w.link.WorldPose(ecm)) w.pose = *pose;
      w.axle = w.pose.Rot().RotateVector(Vector3d::UnitY);
      w.velocity = w.link.WorldLinearVelocity(ecm).value_or(Vector3d::Zero);
      w.angular = w.link.WorldAngularVelocity(ecm).value_or(Vector3d::Zero);
      const auto speed = w.joint.Velocity(ecm);
      const auto angle = w.joint.Position(ecm);
      w.speed = speed && !speed->empty() ? (*speed)[0] : 0.0;
      w.angle = angle && !angle->empty() ? (*angle)[0] : 0.0;
      w.spin += dt / (motor_.speed_filter + dt) * (w.speed - w.spin);

      double target = (vx - w.side * wz * track_ / 2) / radius_;
      w.ramp.LimitVelocity(target);
      w.ramp.LimitAcceleration(target, w.setpoint, info.dt);
      w.setpoint = target;
      w.joint.SetForce(ecm, {w.motor.Step(motor_, w.setpoint, w.speed, dt)});

      const Vector3d normal = w.last.touched ? w.last.normal : Vector3d::UnitZ;
      const double hub_speed = (w.velocity - normal * w.velocity.Dot(normal)).Length();
      if (dig_) {
        w.dig.Step(w.last.touched ? *w.last.ground : kCoulomb, w.last.slip, hub_speed, dig_heal_length_,
                   dt);  // in the air: heal only
      }
      w.hub_force = Vector3d::Zero;
      if (w.last.touched && w.load > 0) {
        w.hub_force = drive::HubForce(*w.last.ground, w.dig.factor, w.load, w.velocity, w.axle, normal, rr_speed_);
        w.link.AddWorldForce(ecm, w.hub_force);
      }
    }
    UpdateOdometry(info);
    if (t - last_state_ >= state_period_ - 1e-9) {
      last_state_ = t;
      PublishState(t);
    }
    if (t - last_dust_ >= dust_period_ - 1e-9) {
      last_dust_ = t;
      PublishDust();
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override {
    if (!enabled_ || info.paused) return;
    // Static balance of each wheel: contact force = -(joint force + weight + hub force) (gate G8).
    for (auto& w : wheels_) {
      w.load = 0.0;
      w.tangential = Vector3d::Zero;
      const auto wrench = ecm.Component<gz::sim::components::JointTransmittedWrench>(w.joint.Entity());
      if (!w.current.touched || !wrench) continue;
      const auto& f = wrench->Data().force();
      const auto& m = wrench->Data().torque();
      const auto pose = w.link.WorldPose(ecm).value_or(w.pose);
      const auto frame = pose.Rot() * w.joint_frame;
      const Vector3d joint = frame.RotateVector(Vector3d(f.x(), f.y(), f.z()));
      const Vector3d contact = -(joint + gravity_ * w.mass + w.hub_force);
      const Vector3d& n = w.current.normal;
      w.load = std::max(0.0, contact.Dot(n));
      w.tangential = contact - n * contact.Dot(n);
      // The contact forces' moment about the wheel's centre (the joint's origin): minus the joint's, as gravity
      // and the hub force act at the centre and the motor's torque is about the axle.
      EdgeLoads(w, -frame.RotateVector(Vector3d(m.x(), m.y(), m.z())), contact);
    }
  }

  void Reset(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager&) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd_ = {0.0, 0.0};
    }
    for (auto& w : wheels_) {
      w.setpoint = w.spin = 0.0;
      w.motor.Reset();
      w.dig = drive::Dig();
      w.load = 0.0;
      w.tangential = w.hub_force = Vector3d::Zero;
      w.last = w.current = Contact();
      w.points.clear();
      w.past.clear();
    }
    odometry_ = gz::math::DiffDriveOdometry();
    odometry_.SetWheelParams(track_, radius_, radius_);
    last_odom_ = last_state_ = last_dust_ = -std::numeric_limits<double>::infinity();
  }

 private:
  void OnCmd(const gz::msgs::Twist& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_ = {msg.linear().x(), msg.angular().z()};
    cmd_sim_ = sim_time_;
    cmd_wall_ = Clock::now();
  }

  /// The world's ground map and the surfaces that do not depend on where a wheel stands. Once, at the first
  /// step: by then every model of the world is in the ECM.
  void FindGround(const gz::sim::EntityComponentManager& ecm) {
    world_ready_ = true;
    gravity_ = gz::sim::World(gz::sim::worldEntity(ecm)).Gravity(ecm).value_or(Vector3d(0, 0, -9.80665));
    terrain_ = FindTerrainShape(ecm);
    ground_ = FindGroundMap(terrain_);
    const auto resolve = [&](const std::string& key) -> const Traction* {
      if (ground_) {
        if (const Traction* t = ground_->Type(key)) return t;
      }
      for (const auto& t : sdf_surfaces_) {
        if (t.key == key) return &t;
      }
      gzerr << "RoverDrivetrain: surface '" << key << "' is in neither the world's ground map nor the plugin's "
            << "<surface> rows; it grips as plain Coulomb mu 1\n";
      return &kCoulomb;
    };
    default_ = ground_ && ground_->Default() ? ground_->Default() : resolve(default_key_);
    object_ = ground_ && ground_->ObjectDefault() ? ground_->ObjectDefault() : resolve(object_key_);
    gzmsg << "RoverDrivetrain: "
          << (ground_ ? "ground map in " + terrain_.directory : std::string("no ground map")) << "; default surface "
          << default_->key << ", objects " << object_->key << "\n";
  }

  /// What every collision a wheel may touch is made of, worked out once per collision (spec 6.4). The
  /// heightmap is looked up per contact in OnContact().
  void ClassifyCollisions(const gz::sim::EntityComponentManager& ecm) {
    const auto classify = [&](const gz::sim::Entity& entity, const gz::sim::components::Collision*) {
      if (entity == terrain_.collision) return true;
      const auto* geometry = ecm.Component<gz::sim::components::Geometry>(entity);
      const auto* element = ecm.Component<gz::sim::components::CollisionElement>(entity);
      const auto* name = ecm.Component<gz::sim::components::Name>(entity);
      const Traction* surface = nullptr;
      if (geometry && geometry->Data().Type() == sdf::GeometryType::PLANE) {
        surface = default_;  // a plane is flat ground
      } else if (ground_ && terrain_.model != gz::sim::kNullEntity &&
                 gz::sim::topLevelModel(entity, ecm) == terrain_.model) {
        surface = ground_->Collision(name ? name->Data() : std::string());
      }
      if (!surface) {
        const std::optional<double> mu = element ? ExplicitMu(element->Data()) : std::nullopt;
        if (mu) {
          Traction t;
          t.key = "sdf_mu";
          t.mu_s = t.mu_k = *mu;
          t.crr = kObjectCrr;
          t.slip = kObjectSlip;
          surface = &owned_.emplace_back(t);
        } else {
          const bool terrain = terrain_.model != gz::sim::kNullEntity &&
                               gz::sim::topLevelModel(entity, ecm) == terrain_.model;
          surface = terrain ? default_ : object_;  // terrain shapes of a world without a ground map: ground
        }
      }
      surfaces_[entity] = surface;
      return true;
    };
    if (classified_) {
      ecm.EachNew<gz::sim::components::Collision>(classify);
    } else {
      ecm.Each<gz::sim::components::Collision>(classify);
      classified_ = true;
    }
  }

  /// The friction coefficient a collision's SDF sets (<surface><friction><ode><mu>), if it sets one.
  static std::optional<double> ExplicitMu(const sdf::Collision& collision) {
    sdf::ElementPtr e = collision.Element();
    for (const char* tag : {"surface", "friction", "ode", "mu"}) {
      if (!e || !e->HasElement(tag)) return std::nullopt;
      e = e->GetElement(tag);
    }
    return e->Get<double>();
  }

  const Traction& Surface(const gz::sim::Entity& other, const Vector3d& point) const {
    if (other == terrain_.collision) {
      const Traction* t = ground_ ? ground_->At(point.X(), point.Y()) : nullptr;
      return t ? *t : *default_;
    }
    const auto it = surfaces_.find(other);
    return it != surfaces_.end() ? *it->second : *object_;  // created this very step: an object
  }

  /// The contact callback (spec 6.4), for every contact of a collision with surface customisation.
  void OnContact(const gz::sim::Entity& c1, const gz::sim::Entity& c2, const Vector3d& point,
                 const std::optional<Vector3d>& normal, const std::optional<double>& depth, size_t count,
                 gz::physics::SetContactPropertiesCallbackFeature::ContactSurfaceParams<gz::sim::events::Policy>&
                     params) {
    Wheel* wheel = nullptr;
    gz::sim::Entity other = c2;
    for (auto& w : wheels_) {
      for (const auto c : w.collisions) {
        if (c == c1) wheel = &w;
        if (c == c2) {
          wheel = &w;
          other = c1;
        }
      }
    }
    if (!wheel || !default_) return;
    Wheel& w = *wheel;
    Vector3d n = normal.value_or(Vector3d::UnitZ).Normalized();
    if (n.Dot(w.pose.Pos() - point) < 0) n = -n;  // the ground's normal, into the wheel
    // The wheel's material point over the ground, with the wheel's spin read through the speed filter: the
    // 1 ms coupling of the contacts, the motor and DART makes a wheel chatter at ~250 Hz (+-0.1 rad/s parked),
    // which would otherwise flip the slip direction every step and keep the contact sliding (measured: a rover
    // parked across 29 deg regolith crept downhill at 2 cm/s).
    const Vector3d omega = w.angular - w.axle * (w.speed - w.spin);
    const Vector3d v = w.velocity + omega.Cross(point - w.pose.Pos());
    const Vector3d slip = v - n * v.Dot(n);
    const Traction& ground = Surface(other, point);
    const double mu = drive::StribeckMu(ground, slip.Length(), contact_.v_stribeck) *
                      drive::NoiseFactor(point.X(), point.Y(), contact_.mu_noise, contact_.mu_noise_length);
    const Vector3d stick = drive::StickDirection(w.tangential, w.load, gravity_, w.axle.Cross(n), n);
    const double share = Share(w, point, count);
    const auto f = drive::Friction(contact_, ground, mu, slip, std::abs(w.spin * radius_), share, stick);
    params.firstFrictionalDirection = Eigen::Vector3d(f.direction.X(), f.direction.Y(), f.direction.Z());
    params.frictionCoeff = f.mu1;
    params.secondaryFrictionCoeff = f.mu2;
    params.slipCompliance = f.compliance;
    params.secondarySlipCompliance = f.compliance;
    w.points.push_back(point);
    const double d = depth.value_or(0.0);
    if (!w.current.touched || d > w.current.depth) {
      w.current = Contact{true, d, &ground, point, n, slip.Length()};
    }
  }

  /// How this step's contacts of a wheel shared its load (the notes at the top): from the contact forces'
  /// moment about the wheel's centre and their sum, a load linear in each contact's offset y along the axle,
  /// N_i = N (alpha + beta y_i), with sum N_i = N and sum y_i N_i = moment . heading - rho F_axial; negative
  /// shares are clipped. Kept in w.past for the next step's contacts.
  void EdgeLoads(Wheel& w, const Vector3d& moment, const Vector3d& contact) const {
    w.past.clear();
    const size_t k = w.points.size();
    if (!k || w.load <= 0) return;
    const Vector3d& n = w.current.normal;
    const Vector3d heading = w.axle.Cross(n).Normalized();
    const Vector3d axle = n.Cross(heading);
    std::vector<double> y(k);
    double sy = 0.0, syy = 0.0, rho = 0.0;
    for (size_t i = 0; i < k; ++i) {
      y[i] = (w.points[i] - w.pose.Pos()).Dot(axle);
      sy += y[i];
      syy += y[i] * y[i];
      rho += (w.pose.Pos() - w.points[i]).Dot(n) / k;
    }
    const double target = (moment.Dot(heading) - rho * contact.Dot(axle)) / w.load;  // sum y_i N_i / N
    const double det = k * syy - sy * sy;
    double alpha = 1.0 / k, beta = 0.0;
    if (det > 1e-6) {
      alpha = (syy - sy * target) / det;
      beta = (k * target - sy) / det;
    }
    double total = 0.0;
    std::vector<double> share(k);
    for (size_t i = 0; i < k; ++i) total += share[i] = std::max(0.0, alpha + beta * y[i]);
    for (size_t i = 0; i < k; ++i) {
      w.past.push_back({w.points[i], total > 0 ? w.load * share[i] / total : w.load / k});
    }
  }

  /// The load a wheel contact carries: its own in the previous step (EdgeLoads), else (a new contact) an
  /// even share of the wheel's.
  static double Share(const Wheel& w, const Vector3d& point, size_t count) {
    const PastContact* nearest = nullptr;
    double best = kContactMatch;
    for (const auto& p : w.past) {
      const double d = p.point.Distance(point);
      if (d < best) {
        best = d;
        nearest = &p;
      }
    }
    return nearest ? nearest->load : w.load / std::max<size_t>(count, 1);
  }

  void UpdateOdometry(const gz::sim::UpdateInfo& info) {
    const auto now = Clock::time_point(info.simTime);
    if (!odometry_.Initialized()) {
      odometry_.Init(now);
      return;
    }
    double left = 0, right = 0, nl = 0, nr = 0;
    for (const auto& w : wheels_) {
      (w.side > 0 ? left : right) += w.angle;
      (w.side > 0 ? nl : nr) += 1;
    }
    odometry_.Update(gz::math::Angle(left / std::max(nl, 1.0)), gz::math::Angle(right / std::max(nr, 1.0)), now);
    const double t = Seconds(info.simTime);
    if (t - last_odom_ < odom_period_ - 1e-9) return;
    last_odom_ = t;
    gz::msgs::Odometry msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(gz::sim::convert<gz::msgs::Time>(info.simTime));
    auto* frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(frame_id_);
    auto* child = msg.mutable_header()->add_data();
    child->set_key("child_frame_id");
    child->add_value(child_frame_id_);
    msg.mutable_pose()->mutable_position()->set_x(odometry_.X());
    msg.mutable_pose()->mutable_position()->set_y(odometry_.Y());
    gz::msgs::Set(msg.mutable_pose()->mutable_orientation(),
                  gz::math::Quaterniond(0, 0, *odometry_.Heading()));
    msg.mutable_twist()->mutable_linear()->set_x(odometry_.LinearVelocity());
    msg.mutable_twist()->mutable_angular()->set_z(*odometry_.AngularVelocity());
    gz::msgs::Pose_V tf;
    auto* pose = tf.add_pose();
    pose->mutable_header()->CopyFrom(msg.header());
    pose->mutable_position()->CopyFrom(msg.pose().position());
    pose->mutable_orientation()->CopyFrom(msg.pose().orientation());
    odom_pub_.Publish(msg);
    tf_pub_.Publish(tf);
  }

  /// /model/rover/drivetrain (spec 9.3): rad/s, A, N m, V, m/s, N, D; slip and surface of the last step's
  /// deepest contact ("" in the air).
  void PublishState(double t) {
    std::ostringstream json;
    json << std::setprecision(6) << "{\"t\": " << t << ", \"cmd\": [" << applied_cmd_[0] << ", " << applied_cmd_[1]
         << "], \"wheels\": {";
    for (size_t k = 0; k < wheels_.size(); ++k) {
      const Wheel& w = wheels_[k];
      const Contact& c = w.last;
      json << (k ? ", " : "") << '"' << w.name << "\": {\"sp\": " << w.setpoint << ", \"w\": " << w.speed
           << ", \"i\": " << w.motor.current << ", \"tau\": " << w.motor.torque << ", \"u\": " << w.motor.voltage
           << ", \"sat\": " << (w.motor.saturated ? "true" : "false") << ", \"slip\": " << c.slip
           << ", \"load\": " << w.load << ", \"surface\": \"" << (c.touched ? c.ground->key : "") << "\", \"dig\": "
           << w.dig.factor << '}';
    }
    json << "}}";
    gz::msgs::StringMsg msg;
    msg.set_data(json.str());
    state_pub_.Publish(msg);
  }

  /// Each rear emitter's rate (spec 6.5): emitting only while dust rises; an "off" goes out once.
  void PublishDust() {
    for (auto& w : wheels_) {
      if (w.dust_topic.empty()) continue;
      const Vector3d normal = w.last.touched ? w.last.normal : Vector3d::UnitZ;
      const double hub = (w.velocity - normal * w.velocity.Dot(normal)).Length();
      const double rate =
          w.last.touched ? drive::DustRate(dust_, *w.last.ground, hub, w.last.slip, w.dig.factor, w.load) : 0.0;
      const bool emitting = rate > 0;
      if (!emitting && !w.emitting) continue;
      w.emitting = emitting;
      gz::msgs::ParticleEmitter msg;
      msg.mutable_emitting()->set_data(emitting);
      msg.mutable_rate()->set_data(static_cast<float>(rate));
      w.dust.Publish(msg);
    }
  }

  static inline const Traction kCoulomb{};  // plain Coulomb mu 1, no resistance, no dig-in: DART's default

  gz::sim::Model model_;
  bool enabled_ = false, world_ready_ = false, classified_ = false;
  double cmd_timeout_ = 0.5;
  bool wall_timeout_ = true;
  double track_ = 0.8, radius_ = 0.15, accel_ = 8.0, max_speed_ = 10.0, rr_speed_ = 0.03;
  double odom_period_ = 0.02, state_period_ = 0.02, dust_period_ = 0.1;
  double last_odom_ = -std::numeric_limits<double>::infinity();
  double last_state_ = -std::numeric_limits<double>::infinity();
  double last_dust_ = -std::numeric_limits<double>::infinity();
  std::string frame_id_, child_frame_id_;
  drive::MotorParams motor_;
  drive::ContactParams contact_;
  drive::DustParams dust_;
  bool dig_ = true;
  double dig_heal_length_ = 0.3;
  std::string default_key_ = "regolith", object_key_ = "manmade";
  std::vector<Traction> sdf_surfaces_;
  std::deque<Traction> owned_;  // explicit-mu objects: stable addresses for surfaces_
  const Traction* default_ = nullptr;
  const Traction* object_ = nullptr;
  TerrainShape terrain_;
  std::optional<GroundMap> ground_;
  std::unordered_map<gz::sim::Entity, const Traction*> surfaces_;
  Vector3d gravity_{0, 0, -9.80665};
  std::deque<Wheel> wheels_;  // in place: SpeedLimiter cannot be copied or moved
  gz::math::DiffDriveOdometry odometry_;
  std::array<double, 2> applied_cmd_{};

  gz::transport::Node node_;
  gz::transport::Node::Publisher odom_pub_, tf_pub_, state_pub_;
  gz::common::ConnectionPtr connection_;
  std::mutex mutex_;  // the command, written by gz-transport's thread
  std::array<double, 2> cmd_{};
  double sim_time_ = 0.0, cmd_sim_ = 0.0;
  Clock::time_point cmd_wall_ = Clock::now();
};

}  // namespace rover_sim

GZ_ADD_PLUGIN(rover_sim::RoverDrivetrain, gz::sim::System, rover_sim::RoverDrivetrain::ISystemConfigure,
              rover_sim::RoverDrivetrain::ISystemPreUpdate, rover_sim::RoverDrivetrain::ISystemPostUpdate,
              rover_sim::RoverDrivetrain::ISystemReset)
