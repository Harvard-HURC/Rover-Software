// ChaseCamera: a gz-sim system that moves its model (a camera) with another
// model, for the driver station (sim/station). Two kinds, chosen in the SDF:
//
// Chase (the default), like the chase camera of a driving game. Every step it
// places its model on a sphere around a look-at point <look_height> above the
// target model's origin, looking at the point: azimuth `yaw`, elevation
// `pitch` above the horizon, radius `distance`. In follow mode yaw is relative
// to the target's heading (0 = behind it), so the camera swings round when the
// target turns; in orbit mode yaw is a world azimuth (0 = east of the target)
// and the view stays put while the target turns. The view approaches its
// target with a first-order lag (<time_constant>), angles the short way round,
// so it moves smoothly. Pitch is clamped above the horizon: the camera never
// dips below the target.
//
// Eye (<mount> given), a first-person camera. The model rides on the target at
// <mount>, a point in the target's frame (the rover's camera pivot), and only
// turns about that point, like a pan-tilt head: `yaw` about the target's z
// (left +), then `pitch` about the turned y (down +). It is body-fixed: it
// pitches and rolls with the target. Nothing is smoothed, so it stays on the
// point (one physics step behind the target).
//
// The model should have one link without gravity or collisions: nothing then
// changes its velocity (zero), and only the pose commands move it.
//
// Topics (SDF <cmd_topic>, <mode_topic>, <look_topic>, <state_topic>):
//   cmd    chase, gz.msgs.Vector3d: x is added to yaw [rad], y to pitch [rad],
//          and the distance is multiplied by e^z (zoom)
//   mode   chase, gz.msgs.StringMsg: "follow" or "orbit" (both keep the
//          current view) or "reset" (follow at the start offset)
//   look   eye, gz.msgs.Vector3d: x is the yaw, y the pitch [rad] (absolute)
//   state  gz.msgs.StringMsg, JSON at 5 Hz: mode (follow, orbit or eye),
//          target, yaw, pitch, and the distance of a chase camera
// SDF: <target> model name (default rover), <look_height> [m], start view
// <yaw>, <pitch> [rad], <distance> [m], limits <min_pitch>, <max_pitch> [rad]
// (elevation of a chase camera, pitch of an eye), <min_yaw>, <max_yaw> [rad]
// (eye only: a chase camera's yaw goes all the way round), <min_distance>,
// <max_distance> [m], <time_constant> [s], <mount> [m].

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace rover_sim {

namespace {

double Wrap(double angle) { return std::remainder(angle, 2 * GZ_PI); }

}  // namespace

class ChaseCamera : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager&) override {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm)) {
      gzerr << "ChaseCamera must be attached to a model; it is disabled.\n";
      return;
    }
    target_name_ = sdf->Get<std::string>("target", target_name_).first;
    look_height_ = sdf->Get<double>("look_height", look_height_).first;
    start_yaw_ = sdf->Get<double>("yaw", start_yaw_).first;
    start_pitch_ = sdf->Get<double>("pitch", start_pitch_).first;
    start_distance_ = sdf->Get<double>("distance", start_distance_).first;
    min_pitch_ = sdf->Get<double>("min_pitch", min_pitch_).first;
    max_pitch_ = sdf->Get<double>("max_pitch", max_pitch_).first;
    min_distance_ = sdf->Get<double>("min_distance", min_distance_).first;
    max_distance_ = sdf->Get<double>("max_distance", max_distance_).first;
    time_constant_ = sdf->Get<double>("time_constant", time_constant_).first;
    eye_ = sdf->HasElement("mount");
    mount_ = sdf->Get<gz::math::Vector3d>("mount", mount_).first;
    min_yaw_ = sdf->Get<double>("min_yaw", min_yaw_).first;
    max_yaw_ = sdf->Get<double>("max_yaw", max_yaw_).first;
    const auto state_topic = sdf->Get<std::string>("state_topic", "/chase_camera/state").first;
    Reset();
    if (eye_) {
      node_.Subscribe(sdf->Get<std::string>("look_topic", "/eye_camera/look").first, &ChaseCamera::OnLook, this);
    } else {
      node_.Subscribe(sdf->Get<std::string>("cmd_topic", "/chase_camera/cmd").first, &ChaseCamera::OnCmd, this);
      node_.Subscribe(sdf->Get<std::string>("mode_topic", "/chase_camera/mode").first, &ChaseCamera::OnMode, this);
    }
    state_ = node_.Advertise<gz::msgs::StringMsg>(state_topic);
    enabled_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (!enabled_ || info.paused) {
      return;
    }
    if (info.simTime < last_time_) {  // the world was reset
      initialized_ = false;
      last_state_ = {};
    }
    last_time_ = info.simTime;
    if (target_ == gz::sim::kNullEntity || !ecm.HasEntity(target_)) {
      target_ = gz::sim::World(gz::sim::worldEntity(ecm)).ModelByName(ecm, target_name_);
      initialized_ = false;
      if (target_ == gz::sim::kNullEntity) {
        if (!warned_) {
          gzwarn << "ChaseCamera: no model named '" << target_name_ << "' yet.\n";
          warned_ = true;
        }
        return;
      }
    }
    const auto target_pose = gz::sim::worldPose(target_, ecm);
    if (eye_) {
      ApplyLook();
      model_.SetWorldPoseCmd(ecm, target_pose * gz::math::Pose3d(mount_, gz::math::Quaterniond(0, pitch_, yaw_)));
      PublishState(info.simTime);
      return;
    }
    const double heading = target_pose.Rot().Yaw();
    ApplyCommands(heading);

    const auto look = target_pose.Pos() + gz::math::Vector3d(0, 0, look_height_);
    const double azimuth = follow_ ? heading + GZ_PI + yaw_ : yaw_;
    if (!initialized_) {
      azimuth_ = azimuth;
      elevation_ = pitch_;
      range_ = distance_;
      look_ = look;
      initialized_ = true;
    } else {
      const double dt = std::chrono::duration<double>(info.dt).count();
      const double a = time_constant_ > 0 ? 1 - std::exp(-dt / time_constant_) : 1.0;
      azimuth_ = Wrap(azimuth_ + a * Wrap(azimuth - azimuth_));
      elevation_ += a * (pitch_ - elevation_);
      range_ += a * (distance_ - range_);
      look_ += a * (look - look_);
    }
    const gz::math::Vector3d offset(std::cos(elevation_) * std::cos(azimuth_),
                                    std::cos(elevation_) * std::sin(azimuth_), std::sin(elevation_));
    // Camera x points from the camera to the look-at point: pitched down by the
    // elevation, turned to the opposite azimuth.
    model_.SetWorldPoseCmd(ecm, gz::math::Pose3d(look_ + range_ * offset,
                                                 gz::math::Quaterniond(0, elevation_, azimuth_ + GZ_PI)));
    PublishState(info.simTime);
  }

 private:
  void Reset() {
    follow_ = true;
    yaw_ = eye_ ? std::clamp(start_yaw_, min_yaw_, max_yaw_) : start_yaw_;
    pitch_ = std::clamp(start_pitch_, min_pitch_, max_pitch_);
    distance_ = std::clamp(start_distance_, min_distance_, max_distance_);
  }

  void OnCmd(const gz::msgs::Vector3d& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_yaw_ += msg.x();
    pending_pitch_ += msg.y();
    pending_zoom_ += msg.z();
  }

  void OnMode(const gz::msgs::StringMsg& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg.data() == "follow" || msg.data() == "orbit" || msg.data() == "reset") {
      pending_mode_ = msg.data();
    } else {
      gzwarn << "ChaseCamera: unknown mode '" << msg.data() << "' (follow, orbit, reset).\n";
    }
  }

  void OnLook(const gz::msgs::Vector3d& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_look_ = gz::math::Vector2d(msg.x(), msg.y());
  }

  void ApplyLook() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_look_) {
      yaw_ = std::clamp(pending_look_->X(), min_yaw_, max_yaw_);
      pitch_ = std::clamp(pending_look_->Y(), min_pitch_, max_pitch_);
      pending_look_.reset();
    }
  }

  void ApplyCommands(double heading) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_mode_ == "reset") {
      Reset();
    } else if (!pending_mode_.empty() && (pending_mode_ == "follow") != follow_) {
      // Switch frames where the camera is now, so the view does not jump.
      follow_ = !follow_;
      yaw_ = initialized_ ? (follow_ ? azimuth_ - heading - GZ_PI : azimuth_)
                          : (follow_ ? yaw_ - heading - GZ_PI : yaw_ + heading + GZ_PI);
    }
    pending_mode_.clear();
    yaw_ = Wrap(yaw_ + pending_yaw_);
    pitch_ = std::clamp(pitch_ + pending_pitch_, min_pitch_, max_pitch_);
    distance_ = std::clamp(distance_ * std::exp(pending_zoom_), min_distance_, max_distance_);
    pending_yaw_ = pending_pitch_ = pending_zoom_ = 0;
  }

  void PublishState(const std::chrono::steady_clock::duration& now) {
    if (now - last_state_ < std::chrono::milliseconds(200)) {
      return;
    }
    last_state_ = now;
    char json[256];
    if (eye_) {
      std::snprintf(json, sizeof(json), R"({"mode": "eye", "target": "%s", "yaw": %.4f, "pitch": %.4f})",
                    target_name_.c_str(), yaw_, pitch_);
    } else {
      std::snprintf(json, sizeof(json),
                    R"({"mode": "%s", "target": "%s", "yaw": %.4f, "pitch": %.4f, "distance": %.3f})",
                    follow_ ? "follow" : "orbit", target_name_.c_str(), yaw_, pitch_, distance_);
    }
    gz::msgs::StringMsg msg;
    msg.set_data(json);
    state_.Publish(msg);
  }

  gz::sim::Model model_;
  gz::sim::Entity target_ = gz::sim::kNullEntity;
  std::string target_name_ = "rover";
  double look_height_ = 0.5;
  double start_yaw_ = 0.0;
  double start_pitch_ = 0.3;
  double start_distance_ = 5.0;
  double min_pitch_ = 0.05;
  double max_pitch_ = 1.45;
  double min_distance_ = 1.5;
  double max_distance_ = 80.0;
  double time_constant_ = 0.25;
  bool eye_ = false;
  gz::math::Vector3d mount_;  // eye: the camera's point in the target's frame
  double min_yaw_ = -GZ_PI;
  double max_yaw_ = GZ_PI;
  bool enabled_ = false;
  bool warned_ = false;

  // Commanded view (follow_: yaw relative to the heading; eye: the look) and
  // the smoothed one (chase only).
  bool follow_ = true;
  double yaw_ = 0.0;
  double pitch_ = 0.0;
  double distance_ = 0.0;
  bool initialized_ = false;
  double azimuth_ = 0.0;  // world azimuth from the look-at point to the camera
  double elevation_ = 0.0;
  double range_ = 0.0;
  gz::math::Vector3d look_;
  std::chrono::steady_clock::duration last_time_{};
  std::chrono::steady_clock::duration last_state_{};

  // Written by transport threads, applied in PreUpdate.
  std::mutex mutex_;
  double pending_yaw_ = 0.0;
  double pending_pitch_ = 0.0;
  double pending_zoom_ = 0.0;
  std::string pending_mode_;
  std::optional<gz::math::Vector2d> pending_look_;

  gz::transport::Node node_;
  gz::transport::Node::Publisher state_;
};

}  // namespace rover_sim

GZ_ADD_PLUGIN(rover_sim::ChaseCamera, gz::sim::System, rover_sim::ChaseCamera::ISystemConfigure,
              rover_sim::ChaseCamera::ISystemPreUpdate)
