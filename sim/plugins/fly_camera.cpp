// FlyCamera: a gz-sim system that flies its model (a camera) freely over the world, for the driver
// station's Fly view (sim/station) and the offline map (sim/tools/render_map.py). It is a system of its
// own rather than a mode of ChaseCamera: it needs the terrain heightmap and gz-rendering, and keeps the
// rover-eye code untouched (realism design, section 8.1, decision D16).
//
// Every physics step (sim time; nothing while paused; started over when sim time goes backwards):
//   1. Take the inputs that arrived since the last step (transport threads, under a mutex).
//   2. Target velocity = cmd (if newer than the deadman) x clamp(agl x speed_per_agl, min_speed,
//      max_speed) x the speed multiplier, turned by the yaw.
//   3. The velocity approaches it with a first-order lag (time_constant).
//   4. Look target += look deltas + rates x dt; the view follows with look_time_constant.
//   5. Position += velocity x dt, plus the target model's displacement in follow mode.
//   6. Floor = max(ground here, ground 0.5 s ahead) + clearance, approached with 0.1 s; never below
//      ground + min(0.3 m, clearance).
//   7. Clamp to the terrain plus margin, and to max_altitude above its highest point.
//   8. SetWorldPoseCmd (measured free: no cost over a static camera).
// Integrating here, not in a client, is what makes the picture smooth: picture motion varied 0.46 % from
// frame to frame, against 26-35 % for Python set_pose at 20 or 60 Hz (sim/data/research/flycam/
// measurements/smoothness_*.json).
//
// The ground is the world's visual heightmap (terrain_heightmap.hh, sampled as Gazebo draws it), clamped
// at its edge outside it; a world without one has flat ground at z = 0 and bounds of +-kFlatHalfExtent.
//
// Orthographic projection: SDF cannot ask for it (an orthographic <projection_type> or <lens> is ignored,
// measured), so a hook on events::SceneUpdate, which the Sensors system emits on its rendering thread,
// sets the rendering camera's projection whenever the width changes. It finds the camera by name each
// time and keeps no pointer: the camera's control block lives in the ogre2 plugin, which is unloaded
// before systems are destroyed, so a stored CameraPtr (even a weak one) crashes gz at shutdown. Not
// PreRender, Render or PostRender: connecting to any of them makes the Sensors system render every
// step, -35 to -45 % sim speed; SceneUpdate cost nothing measurable (render_hook_and_camera_cost.json).
// The hook is connected once, in Configure (EventT::Connect is not thread-safe). While orthographic the
// camera looks straight down and the window is w = 2 h tan(hfov / 2), h the height above the ground
// where orthographic began, so switching does not jump and climbing zooms out. Orthographic views show
// no cast shadows, and perspective ones none beyond ~500 m (measured).
//
// The model should have one link without gravity or collisions carrying the camera: nothing then
// changes its velocity, and only the pose commands move it (sim/viewers.py writes it).
//
// Topics (SDF <cmd_topic> ... <state_topic>; defaults below). Inputs with a NaN or infinite value are
// dropped: one would stick in the smoothed state until the world is reset.
//   cmd    gz.msgs.Twist: linear x, y, z = forward, left, up in units of the cruise speed (each clamped
//          to +-<fast>: 1 cruise, <fast> while the driver holds the fast key); angular.z = yaw rate
//          (left +), angular.y = pitch rate (down +) [rad/s]. Held only for <deadman>.
//   speed  gz.msgs.Double: the speed multiplier, clamped to [0.25, 4].
//   look   gz.msgs.Vector3d: x is added to the yaw, y to the pitch [rad] (the view follows smoothly).
//   goto   gz.msgs.Pose: fly to the pose (the camera looks along its x axis; roll is dropped) on a
//          smoothstep over clamp(distance / 50 m/s, 0.4, 1.5) s; a header data key "jump" makes it
//          instant. A non-zero cmd or a look cancels a flight.
//   mode   gz.msgs.StringMsg: "free" (stop following), "follow" (move with <target>, keeping the
//          offset), "top" (look straight down), "level" (look at the horizon), "stop" (halt now and
//          drop the held cmd), "ortho" (orthographic from here), "ortho <width>" (orthographic, the
//          height set for a <width> m window), "perspective".
//   state  gz.msgs.StringMsg, JSON at 10 Hz of sim time: t, mode (free or follow), x, y, z, yaw, pitch
//          [rad, pitch down +], agl, ground [m], speed (the multiplier), v [m/s], ortho (window width,
//          0 in perspective), goto (a flight is under way).
// SDF: <target> (default rover), <clearance> [m], <time_constant>, <look_time_constant> [s],
// <deadman> [s], <deadman_clock> wall (default: a stalled sim cannot keep a stale command alive) or sim
// (tests at real-time factor 0), <speed_per_agl> [1/s], <min_speed>, <max_speed> [m/s], <fast>,
// <max_altitude> [m above the highest terrain], <margin> [m beyond the terrain edge]. The start pose is
// the model's pose; a world reset (ISystemReset) or a jump back in sim time returns there.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Matrix4.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/transport/Node.hh>
#include <sdf/Camera.hh>
#include <sdf/Element.hh>
#include <sdf/Sensor.hh>

#include "terrain_heightmap.hh"

namespace rover_sim {

namespace {

constexpr double kLookAhead = 0.5;           // [s] the floor also lies under where the camera will be (A)
constexpr double kFloorTimeConstant = 0.1;   // [s] how fast the camera rises to the floor (A)
constexpr double kHardFloor = 0.3;           // [m] never closer to the ground than this (A)
constexpr double kMinPitch = -1.5;           // [rad] nearly straight up
constexpr double kMaxPitch = GZ_PI / 2;      // [rad] straight down (top-down and orthographic views)
constexpr double kMinScale = 0.25, kMaxScale = 4.0;  // speed multiplier range (section 8.1)
constexpr double kGotoSpeed = 50.0;          // [m/s] a goto takes distance / kGotoSpeed ...
constexpr double kGotoMin = 0.4, kGotoMax = 1.5;     // ... clamped to [s] (prototype, tuned by eye)
constexpr double kStatePeriod = 0.1;         // [s] of sim time between state messages
constexpr double kFlatHalfExtent = 10000.0;  // [m] bounds of a world without a heightmap (A)

double Wrap(double angle) { return std::remainder(angle, 2 * GZ_PI); }

double Lag(double dt, double time_constant) { return time_constant > 0 ? 1 - std::exp(-dt / time_constant) : 1.0; }

double Seconds(const std::chrono::steady_clock::duration& d) { return std::chrono::duration<double>(d).count(); }

bool Finite(std::initializer_list<double> values) {
  for (double v : values) {
    if (!std::isfinite(v)) {
      gzwarn << "FlyCamera: dropped an input that is not finite.\n";
      return false;
    }
  }
  return true;
}

// Yaw and pitch (down +) of a camera that looks along the pose's x axis; straight up or down the yaw
// comes from the camera's z axis (the top of its picture), so a top-down pose keeps its heading.
void YawPitch(const gz::math::Quaterniond& q, double& yaw, double& pitch) {
  const auto x = q.RotateVector(gz::math::Vector3d::UnitX);
  const double horizontal = std::hypot(x.X(), x.Y());
  pitch = std::atan2(-x.Z(), horizontal);
  if (horizontal > 1e-6) {
    yaw = std::atan2(x.Y(), x.X());
  } else {
    const auto z = q.RotateVector(gz::math::Vector3d::UnitZ) * (x.Z() < 0 ? 1.0 : -1.0);
    yaw = std::atan2(z.Y(), z.X());
  }
}

}  // namespace

class FlyCamera : public gz::sim::System,
                  public gz::sim::ISystemConfigure,
                  public gz::sim::ISystemPreUpdate,
                  public gz::sim::ISystemReset {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& events) override {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm)) {
      gzerr << "FlyCamera must be attached to a model; it is disabled.\n";
      return;
    }
    model_name_ = model_.Name(ecm);
    target_name_ = sdf->Get<std::string>("target", target_name_).first;
    clearance_ = std::max(0.0, sdf->Get<double>("clearance", clearance_).first);
    time_constant_ = sdf->Get<double>("time_constant", time_constant_).first;
    look_time_constant_ = sdf->Get<double>("look_time_constant", look_time_constant_).first;
    deadman_ = sdf->Get<double>("deadman", deadman_).first;
    const auto clock = sdf->Get<std::string>("deadman_clock", "wall").first;
    if (clock != "wall" && clock != "sim") {
      gzwarn << "FlyCamera: deadman_clock '" << clock << "' is neither wall nor sim; using wall.\n";
    }
    sim_deadman_ = clock == "sim";
    speed_per_agl_ = sdf->Get<double>("speed_per_agl", speed_per_agl_).first;
    min_speed_ = sdf->Get<double>("min_speed", min_speed_).first;
    max_speed_ = std::max(min_speed_, sdf->Get<double>("max_speed", max_speed_).first);
    fast_ = std::max(1.0, sdf->Get<double>("fast", fast_).first);
    max_altitude_ = sdf->Get<double>("max_altitude", max_altitude_).first;
    margin_ = sdf->Get<double>("margin", margin_).first;

    node_.Subscribe(sdf->Get<std::string>("cmd_topic", "/fly_camera/cmd").first, &FlyCamera::OnCmd, this);
    node_.Subscribe(sdf->Get<std::string>("speed_topic", "/fly_camera/speed").first, &FlyCamera::OnSpeed, this);
    node_.Subscribe(sdf->Get<std::string>("look_topic", "/fly_camera/look").first, &FlyCamera::OnLook, this);
    node_.Subscribe(sdf->Get<std::string>("goto_topic", "/fly_camera/goto").first, &FlyCamera::OnGoto, this);
    node_.Subscribe(sdf->Get<std::string>("mode_topic", "/fly_camera/mode").first, &FlyCamera::OnMode, this);
    state_ = node_.Advertise<gz::msgs::StringMsg>(sdf->Get<std::string>("state_topic", "/fly_camera/state").first);
    render_connection_ = events.Connect<gz::sim::events::SceneUpdate>([this] { OnSceneUpdate(); });
    enabled_ = true;
  }

  void Reset(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager&) override { Restart(); }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (!enabled_) {
      return;
    }
    if (!world_read_) {
      ReadWorld(ecm);
    }
    if (info.paused) {
      return;
    }
    const double now = Seconds(info.simTime);
    if (now < last_time_) {
      Restart();
    }
    last_time_ = now;
    if (!started_) {
      const auto pose = gz::sim::worldPose(model_.Entity(), ecm);  // the model's start pose after a reset
      position_ = pose.Pos();
      YawPitch(pose.Rot(), yaw_, pitch_);
      look_yaw_ = yaw_;
      look_pitch_ = pitch_;
      started_ = true;
    }
    const double dt = Seconds(info.dt);
    const gz::math::Vector3d before = position_;

    // 1. Inputs.
    gz::math::Vector3d cmd;
    double yaw_rate = 0, pitch_rate = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (cmd_ && cmd_->fresh) {
        cmd_->sim_stamp = now;
        cmd_->fresh = false;
      }
      const bool alive = cmd_ && (sim_deadman_ ? now - cmd_->sim_stamp < deadman_
                                               : Seconds(std::chrono::steady_clock::now() - cmd_->wall_stamp) < deadman_);
      if (alive) {
        cmd = cmd_->linear;
        yaw_rate = cmd_->yaw_rate;
        pitch_rate = cmd_->pitch_rate;
      }
      if (pending_look_yaw_ != 0 || pending_look_pitch_ != 0 || cmd != gz::math::Vector3d::Zero) {
        flight_.reset();
      }
      look_yaw_ += pending_look_yaw_;
      look_pitch_ += pending_look_pitch_;
      pending_look_yaw_ = pending_look_pitch_ = 0;
      if (pending_scale_) {
        scale_ = *pending_scale_;
        pending_scale_.reset();
      }
      for (const auto& mode : pending_modes_) {
        ApplyMode(mode, ecm);
      }
      pending_modes_.clear();
      if (pending_goto_) {
        StartGoto(pending_goto_->first, pending_goto_->second);
        pending_goto_.reset();
      }
    }

    // 2.-3. Velocity.
    const double ground = Ground(position_.X(), position_.Y());
    const double cruise = std::clamp((position_.Z() - ground) * speed_per_agl_, min_speed_, max_speed_);
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    const gz::math::Vector3d target = cruise * scale_ * gz::math::Vector3d(c * cmd.X() - s * cmd.Y(),
                                                                          s * cmd.X() + c * cmd.Y(), cmd.Z());
    velocity_ += Lag(dt, time_constant_) * (target - velocity_);

    // 4. Look.
    look_yaw_ += yaw_rate * dt;
    look_pitch_ = std::clamp(look_pitch_ + pitch_rate * dt, kMinPitch, kMaxPitch);
    if (ortho_) {
      look_pitch_ = kMaxPitch;
    }
    const double b = Lag(dt, look_time_constant_);
    yaw_ += b * (look_yaw_ - yaw_);  // both unwrapped: they accumulate turns
    pitch_ += b * (look_pitch_ - pitch_);

    // 5. Position: a flight, or the velocity; plus the target's displacement in follow mode.
    gz::math::Vector3d carried;
    if (follow_) {
      carried = FollowDisplacement(ecm);
    }
    if (flight_) {
      flight_->from += carried;
      flight_->to += carried;
      Fly(dt);
    } else {
      position_ += velocity_ * dt + carried;
    }
    if (ortho_) {
      pitch_ = look_pitch_ = kMaxPitch;
    }

    // 6. Floor, 7. bounds.
    const double here = Ground(position_.X(), position_.Y());
    const gz::math::Vector3d ahead = position_ + kLookAhead * velocity_;
    const double floor = std::max(here, Ground(ahead.X(), ahead.Y())) + clearance_;
    if (position_.Z() < floor) {
      position_.Z(position_.Z() + Lag(dt, kFloorTimeConstant) * (floor - position_.Z()));
    }
    position_.Z(std::max(position_.Z(), here + std::min(kHardFloor, clearance_)));
    position_.X(std::clamp(position_.X(), low_.X(), high_.X()));
    position_.Y(std::clamp(position_.Y(), low_.Y(), high_.Y()));
    position_.Z(std::min(position_.Z(), high_.Z()));
    if (ortho_) {
      ortho_width_ = 2 * std::max(position_.Z() - ortho_ground_, clearance_ + kHardFloor) * std::tan(hfov_ / 2);
    }

    // 8. Pose.
    model_.SetWorldPoseCmd(ecm, gz::math::Pose3d(position_, gz::math::Quaterniond(0, pitch_, yaw_)));
    speed_ = dt > 0 ? (position_ - before - carried).Length() / dt : 0.0;
    PublishState(now);
  }

 private:
  /// Everything back to the start pose (the world was reset or sim time went back).
  void Restart() {
    std::lock_guard<std::mutex> lock(mutex_);
    started_ = false;
    velocity_ = gz::math::Vector3d::Zero;
    speed_ = 0;
    scale_ = 1;
    follow_ = false;
    target_ = gz::sim::kNullEntity;
    ortho_ = false;
    ortho_width_ = 0;
    flight_.reset();
    cmd_.reset();
    pending_look_yaw_ = pending_look_pitch_ = 0;
    pending_scale_.reset();
    pending_modes_.clear();
    pending_goto_.reset();
    last_time_ = 0;
    last_state_ = -kStatePeriod;
  }

  /// Once: the terrain heightmap, the bounds and the camera's field of view.
  void ReadWorld(const gz::sim::EntityComponentManager& ecm) {
    world_read_ = true;
    terrain_ = FindTerrainHeightmap(ecm, HeightmapGeometry::kVisual);
    if (terrain_) {
      const auto top = *std::max_element(terrain_->heights.begin(), terrain_->heights.end());
      const auto half = gz::math::Vector3d(terrain_->size.X() / 2 + margin_, terrain_->size.Y() / 2 + margin_, 0);
      low_ = terrain_->origin - half;
      high_ = terrain_->origin + half;
      high_.Z(terrain_->origin.Z() + top + max_altitude_);
    } else {
      gzmsg << "FlyCamera: no terrain heightmap; the ground is flat at z = 0.\n";
      low_ = gz::math::Vector3d(-kFlatHalfExtent, -kFlatHalfExtent, 0);
      high_ = gz::math::Vector3d(kFlatHalfExtent, kFlatHalfExtent, max_altitude_);
    }
    ecm.Each<gz::sim::components::Camera, gz::sim::components::ParentEntity>(
        [&](const gz::sim::Entity&, const gz::sim::components::Camera* camera,
            const gz::sim::components::ParentEntity* parent) {
          if (gz::sim::topLevelModel(parent->Data(), ecm) != model_.Entity() || !camera->Data().CameraSensor()) {
            return true;
          }
          hfov_ = camera->Data().CameraSensor()->HorizontalFov().Radian();
          return false;
        });
    if (hfov_ <= 0) {
      gzwarn << "FlyCamera: model '" << model_name_ << "' carries no camera; orthographic views are off.\n";
    }
  }

  /// Ground height at (x, y): the heightmap, clamped at its edge outside it; 0 without one.
  double Ground(double x, double y) const {
    if (!terrain_) {
      return 0.0;
    }
    const double hx = terrain_->size.X() / 2 * (1 - 1e-9), hy = terrain_->size.Y() / 2 * (1 - 1e-9);
    return terrain_->Height(std::clamp(x, terrain_->origin.X() - hx, terrain_->origin.X() + hx),
                            std::clamp(y, terrain_->origin.Y() - hy, terrain_->origin.Y() + hy));
  }

  gz::math::Vector3d FollowDisplacement(const gz::sim::EntityComponentManager& ecm) {
    if (target_ == gz::sim::kNullEntity || !ecm.HasEntity(target_)) {
      target_ = gz::sim::World(gz::sim::worldEntity(ecm)).ModelByName(ecm, target_name_);
      if (target_ == gz::sim::kNullEntity) {
        gzwarn << "FlyCamera: no model named '" << target_name_ << "' to follow.\n";
        follow_ = false;
        return {};
      }
      target_position_ = gz::sim::worldPose(target_, ecm).Pos();
    }
    const auto now = gz::sim::worldPose(target_, ecm).Pos();
    const auto moved = now - target_position_;
    target_position_ = now;
    return moved;
  }

  void ApplyMode(const std::string& mode, const gz::sim::EntityComponentManager& ecm) {
    if (mode == "free") {
      follow_ = false;
    } else if (mode == "follow") {
      follow_ = true;
      target_ = gz::sim::kNullEntity;  // FollowDisplacement finds it and starts from where it is now
      FollowDisplacement(ecm);
    } else if (mode == "top") {
      look_pitch_ = kMaxPitch;
    } else if (mode == "level") {
      look_pitch_ = 0;
    } else if (mode == "stop") {
      velocity_ = gz::math::Vector3d::Zero;
      flight_.reset();
      cmd_.reset();
    } else if (mode == "perspective") {
      ortho_ = false;
      ortho_width_ = 0;
    } else if (mode == "ortho" || mode.rfind("ortho ", 0) == 0) {
      if (hfov_ <= 0) {
        gzwarn << "FlyCamera: no camera, so no orthographic view.\n";
        return;
      }
      const double here = Ground(position_.X(), position_.Y());
      if (!ortho_) {
        ortho_ground_ = here;
      }
      ortho_ = true;
      look_pitch_ = kMaxPitch;
      double width = 0;
      if (std::sscanf(mode.c_str(), "ortho %lf", &width) == 1 && width > 0 && std::isfinite(width)) {
        const double height = width / (2 * std::tan(hfov_ / 2));
        StartGoto(gz::math::Vector3d(position_.X(), position_.Y(), ortho_ground_ + height), yaw_, kMaxPitch, false);
      }
    } else {
      gzwarn << "FlyCamera: mode '" << mode << "' not understood (free, follow, top, level, stop, ortho [width], "
             << "perspective).\n";
    }
  }

  void StartGoto(const gz::math::Pose3d& pose, bool jump) {
    double yaw, pitch;
    YawPitch(pose.Rot(), yaw, pitch);
    StartGoto(pose.Pos(), yaw, pitch, jump);
  }

  void StartGoto(const gz::math::Vector3d& to, double yaw, double pitch, bool jump) {
    yaw = yaw_ + Wrap(yaw - yaw_);  // the short way round, in the unwrapped yaw
    pitch = ortho_ ? kMaxPitch : std::clamp(pitch, kMinPitch, kMaxPitch);
    velocity_ = gz::math::Vector3d::Zero;
    look_yaw_ = yaw;
    look_pitch_ = pitch;
    if (jump) {
      flight_.reset();
      position_ = to;
      yaw_ = yaw;
      pitch_ = pitch;
      return;
    }
    const double duration = std::clamp(to.Distance(position_) / kGotoSpeed, kGotoMin, kGotoMax);
    flight_ = Flight{position_, to, yaw_, yaw, pitch_, pitch, duration, 0.0};
  }

  /// One step of a goto: smoothstep in position and view.
  void Fly(double dt) {
    flight_->elapsed += dt;
    const double u = std::min(1.0, flight_->elapsed / flight_->duration);
    const double k = u * u * (3 - 2 * u);
    position_ = flight_->from + k * (flight_->to - flight_->from);
    yaw_ = look_yaw_ = flight_->yaw0 + k * (flight_->yaw1 - flight_->yaw0);
    pitch_ = look_pitch_ = flight_->pitch0 + k * (flight_->pitch1 - flight_->pitch0);
    if (u >= 1) {
      flight_.reset();
    }
  }

  void PublishState(double now) {
    if (now - last_state_ < kStatePeriod) {
      return;
    }
    last_state_ = now;
    const double ground = Ground(position_.X(), position_.Y());
    char json[512];
    std::snprintf(json, sizeof(json),
                  R"({"t": %.4f, "mode": "%s", "x": %.4f, "y": %.4f, "z": %.4f, "yaw": %.5f, "pitch": %.5f, )"
                  R"("agl": %.3f, "ground": %.3f, "speed": %.3f, "v": %.3f, "ortho": %.3f, "goto": %s})",
                  now, follow_ ? "follow" : "free", position_.X(), position_.Y(), position_.Z(), Wrap(yaw_), pitch_,
                  position_.Z() - ground, ground, scale_, speed_, ortho_ ? ortho_width_.load() : 0.0,
                  flight_ ? "true" : "false");
    gz::msgs::StringMsg msg;
    msg.set_data(json);
    state_.Publish(msg);
  }

  // Rendering thread (events::SceneUpdate): apply the projection when the width changed.
  void OnSceneUpdate() {
    const double width = ortho_width_.load();
    if (width == applied_width_) {
      return;
    }
    auto scene = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene) {
      return;
    }
    const std::string prefix = model_name_ + "::";
    gz::rendering::CameraPtr camera;
    for (unsigned int i = 0; i < scene->SensorCount() && !camera; ++i) {
      auto candidate = std::dynamic_pointer_cast<gz::rendering::Camera>(scene->SensorByIndex(i));
      if (candidate && candidate->Name().rfind(prefix, 0) == 0) {
        camera = candidate;
      }
    }
    if (!camera) {
      return;  // not created yet; try again at the next update
    }
    if (width > 0) {
      const double w = width, h = width * camera->ImageHeight() / camera->ImageWidth();
      const double n = camera->NearClipPlane(), f = camera->FarClipPlane();
      gz::math::Matrix4d m = gz::math::Matrix4d::Zero;
      m(0, 0) = 2 / w;
      m(1, 1) = 2 / h;
      m(2, 2) = -2 / (f - n);
      m(2, 3) = -(f + n) / (f - n);
      m(3, 3) = 1;
      camera->SetProjectionType(gz::rendering::CPT_ORTHOGRAPHIC);
      camera->SetProjectionMatrix(m);
    } else {
      camera->SetProjectionType(gz::rendering::CPT_PERSPECTIVE);  // also drops the custom matrix
    }
    applied_width_ = width;
  }

  // Transport threads: inputs wait under the mutex for the next step.
  void OnCmd(const gz::msgs::Twist& msg) {
    if (!Finite({msg.linear().x(), msg.linear().y(), msg.linear().z(), msg.angular().y(), msg.angular().z()})) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto clamp = [this](double v) { return std::clamp(v, -fast_, fast_); };
    cmd_ = Command{gz::math::Vector3d(clamp(msg.linear().x()), clamp(msg.linear().y()), clamp(msg.linear().z())),
                   msg.angular().z(), msg.angular().y(), std::chrono::steady_clock::now(), 0.0, true};
  }

  void OnSpeed(const gz::msgs::Double& msg) {
    if (!Finite({msg.data()})) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    pending_scale_ = std::clamp(msg.data(), kMinScale, kMaxScale);
  }

  void OnLook(const gz::msgs::Vector3d& msg) {
    if (!Finite({msg.x(), msg.y()})) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    pending_look_yaw_ += msg.x();
    pending_look_pitch_ += msg.y();
  }

  void OnGoto(const gz::msgs::Pose& msg) {
    const auto pose = gz::msgs::Convert(msg);
    if (!Finite({pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(),
                 pose.Rot().Z()})) {
      return;
    }
    bool jump = false;
    for (const auto& entry : msg.header().data()) {
      jump = jump || entry.key() == "jump";
    }
    std::lock_guard<std::mutex> lock(mutex_);
    pending_goto_ = std::make_pair(pose, jump);
  }

  void OnMode(const gz::msgs::StringMsg& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_modes_.push_back(msg.data());
  }

  struct Command {
    gz::math::Vector3d linear;  // forward, left, up [cruise speeds]
    double yaw_rate, pitch_rate;  // [rad/s]
    std::chrono::steady_clock::time_point wall_stamp;  // when it arrived
    double sim_stamp;  // [s] sim time of the first step that saw it
    bool fresh;  // not yet stamped with sim time
  };
  struct Flight {
    gz::math::Vector3d from, to;
    double yaw0, yaw1, pitch0, pitch1;
    double duration, elapsed;  // [s]
  };

  // Configuration.
  gz::sim::Model model_;
  std::string model_name_;
  std::string target_name_ = "rover";
  double clearance_ = 1.0;           // [m]
  double time_constant_ = 0.2;       // [s]
  double look_time_constant_ = 0.08; // [s]
  double deadman_ = 0.3;             // [s]
  bool sim_deadman_ = false;
  double speed_per_agl_ = 1.0;       // [1/s]
  double min_speed_ = 2.0, max_speed_ = 200.0;  // [m/s]
  double fast_ = 4.0;
  double max_altitude_ = 2000.0;     // [m] above the highest terrain
  double margin_ = 100.0;            // [m] beyond the terrain edge
  bool enabled_ = false;

  // The world, read once.
  bool world_read_ = false;
  std::optional<TerrainHeightmap> terrain_;
  gz::math::Vector3d low_, high_;  // bounds (low_.Z() unused: the floor bounds from below)
  double hfov_ = 0.0;              // [rad] of the model's camera; 0 without one

  // Flight state (sim thread; Restart also runs from Reset on the sim thread).
  bool started_ = false;
  gz::math::Vector3d position_, velocity_;
  double yaw_ = 0, pitch_ = 0;            // the view
  double look_yaw_ = 0, look_pitch_ = 0;  // where the view is going
  double scale_ = 1.0;
  double speed_ = 0.0;                    // [m/s] last step, own motion only
  bool follow_ = false;
  gz::sim::Entity target_ = gz::sim::kNullEntity;
  gz::math::Vector3d target_position_;
  bool ortho_ = false;
  double ortho_ground_ = 0.0;             // [m] ground height where orthographic began
  std::optional<Flight> flight_;
  double last_time_ = 0.0, last_state_ = -kStatePeriod;  // [s] sim time

  // Shared with the rendering thread.
  std::atomic<double> ortho_width_{0.0};  // [m] 0: perspective
  double applied_width_ = 0.0;            // rendering thread only
  gz::common::ConnectionPtr render_connection_;

  // Written by transport threads, taken in PreUpdate.
  std::mutex mutex_;
  std::optional<Command> cmd_;
  double pending_look_yaw_ = 0.0, pending_look_pitch_ = 0.0;
  std::optional<double> pending_scale_;
  std::vector<std::string> pending_modes_;
  std::optional<std::pair<gz::math::Pose3d, bool>> pending_goto_;

  gz::transport::Node node_;
  gz::transport::Node::Publisher state_;
};

}  // namespace rover_sim

GZ_ADD_PLUGIN(rover_sim::FlyCamera, gz::sim::System, rover_sim::FlyCamera::ISystemConfigure,
              rover_sim::FlyCamera::ISystemPreUpdate, rover_sim::FlyCamera::ISystemReset)
