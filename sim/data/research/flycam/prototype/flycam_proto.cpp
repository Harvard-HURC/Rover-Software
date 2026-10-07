// FlyCamProto: prototype of a free "inspection" camera for the driver station.
// NOT project code: a research prototype to measure smoothness, terrain
// clearance and orthographic projection in gz-sim 8 / ogre2.
//
// Every physics step it integrates a commanded velocity (first-order lag,
// deadman), clamps the camera above the terrain heightmap it finds in the ECM,
// and places its model with SetWorldPoseCmd. On the rendering thread
// (events::PreRender, emitted by the Sensors system) it can switch its camera
// sensor to an orthographic projection of a given window width.
//
// Topics:
//   /flycam/cmd    gz.msgs.Twist   linear: forward, left, up [m/s] in the yaw frame;
//                                  angular.z yaw rate, angular.y pitch rate (down +) [rad/s]
//   /flycam/look   gz.msgs.Vector3d x: add to yaw, y: add to pitch [rad]
//   /flycam/goto   gz.msgs.Pose    position + orientation (yaw/pitch used); jumps
//   /flycam/ortho  gz.msgs.Double  ortho window width [m]; <= 0 perspective
//   /flycam/state  gz.msgs.StringMsg JSON at 20 Hz

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/geospatial/ImageHeightmap.hh>
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
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
#include <sdf/Heightmap.hh>

namespace rover_sim {

class FlyCamProto : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& events) override {
    model_ = gz::sim::Model(entity);
    model_name_ = model_.Name(ecm);
    clearance_ = sdf->Get<double>("clearance", clearance_).first;
    tau_ = sdf->Get<double>("time_constant", tau_).first;
    deadman_ = sdf->Get<double>("deadman", deadman_).first;
    method_ = sdf->Get<std::string>("method", method_).first;
    only_on_change_ = sdf->Get<bool>("only_on_change", only_on_change_).first;
    look_tau_ = sdf->Get<double>("look_tau", look_tau_).first;
    const auto start = sdf->Get<gz::math::Pose3d>("start", gz::math::Pose3d(0, 0, 30, 0, 1.5707, 0)).first;
    pos_ = start.Pos();
    yaw_ = start.Rot().Yaw();
    pitch_ = start.Rot().Pitch();
    node_.Subscribe("/flycam/cmd", &FlyCamProto::OnCmd, this);
    node_.Subscribe("/flycam/look", &FlyCamProto::OnLook, this);
    node_.Subscribe("/flycam/goto", &FlyCamProto::OnGoto, this);
    node_.Subscribe("/flycam/ortho", &FlyCamProto::OnOrtho, this);
    state_ = node_.Advertise<gz::msgs::StringMsg>("/flycam/state");
    if (sdf->Get<bool>("ortho_hook", true).first) {
      const auto hook = sdf->Get<std::string>("hook_event", "prerender").first;
      if (hook == "sceneupdate") {
        render_conn_ = events.Connect<gz::sim::events::SceneUpdate>([this] { this->OnPreRender(); });
      } else if (hook == "postrender") {
        render_conn_ = events.Connect<gz::sim::events::PostRender>([this] { this->OnPreRender(); });
      } else if (hook == "render") {
        render_conn_ = events.Connect<gz::sim::events::Render>([this] { this->OnPreRender(); });
      } else {
        render_conn_ = events.Connect<gz::sim::events::PreRender>([this] { this->OnPreRender(); });
      }
      gzmsg << "FlyCamProto: render hook on " << hook << "\n";
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (!heightmap_tried_) {
      LoadHeightmap(ecm);
    }
    if (info.paused) {
      return;
    }
    const double dt = std::chrono::duration<double>(info.dt).count();
    const double now = std::chrono::duration<double>(info.simTime).count();
    gz::math::Vector3d target_v;
    double target_wz = 0, target_wy = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (cmd_time_wall_ && std::chrono::duration<double>(std::chrono::steady_clock::now() - *cmd_time_wall_).count() < deadman_) {
        target_v = cmd_v_;
        target_wz = cmd_wz_;
        target_wy = cmd_wy_;
      }
      if (!look_init_) {
        target_yaw_ = yaw_;
        target_pitch_ = pitch_;
        look_init_ = true;
      }
      target_yaw_ += pending_yaw_;
      target_pitch_ = std::clamp(target_pitch_ + pending_pitch_, -1.5, 1.5707);
      pending_yaw_ = pending_pitch_ = 0;
      if (pending_goto_) {
        pos_ = pending_goto_->Pos();
        yaw_ = target_yaw_ = pending_goto_->Rot().Yaw();
        pitch_ = target_pitch_ = pending_goto_->Rot().Pitch();
        vel_ = {};
        pending_goto_.reset();
      }
    }
    const double a = tau_ > 0 ? 1 - std::exp(-dt / tau_) : 1.0;
    vel_ += a * (target_v - vel_);
    wz_ += a * (target_wz - wz_);
    wy_ += a * (target_wy - wy_);
    target_yaw_ += wz_ * dt;
    target_pitch_ = std::clamp(target_pitch_ + wy_ * dt, -1.5, 1.5707);
    const double b = look_tau_ > 0 ? 1 - std::exp(-dt / look_tau_) : 1.0;
    yaw_ += b * (target_yaw_ - yaw_);   // unwrapped: both accumulate
    pitch_ += b * (target_pitch_ - pitch_);
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    pos_ += dt * gz::math::Vector3d(c * vel_.X() - s * vel_.Y(), s * vel_.X() + c * vel_.Y(), vel_.Z());
    const double ground = Ground(pos_.X(), pos_.Y());
    if (pos_.Z() < ground + clearance_) {
      pos_.Z(ground + clearance_);
    }
    const gz::math::Pose3d pose(pos_, gz::math::Quaterniond(0, pitch_, yaw_));
    if (!only_on_change_ || pose != last_pose_ || !posed_) {
      if (method_ == "pose") {
        // Static model: write the pose straight into the ECM (rendering reads it), no physics command.
        ecm.SetComponentData<gz::sim::components::Pose>(model_.Entity(), pose);
        ecm.SetChanged(model_.Entity(), gz::sim::components::Pose::typeId, gz::sim::ComponentState::PeriodicChange);
      } else {
        model_.SetWorldPoseCmd(ecm, pose);
      }
      last_pose_ = pose;
      posed_ = true;
    }
    if (now - last_state_ >= 0.05 || now < last_state_) {
      last_state_ = now;
      char json[320];
      std::snprintf(json, sizeof(json),
                    R"({"t": %.4f, "x": %.4f, "y": %.4f, "z": %.4f, "yaw": %.4f, "pitch": %.4f, "ground": %.4f, "ortho": %.2f, "cam": "%s"})",
                    now, pos_.X(), pos_.Y(), pos_.Z(), yaw_, pitch_, ground, ortho_width_.load(), camera_found_.c_str());
      gz::msgs::StringMsg msg;
      msg.set_data(json);
      state_.Publish(msg);
    }
  }

 private:
  void LoadHeightmap(gz::sim::EntityComponentManager& ecm) {
    ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>(
        [&](const gz::sim::Entity& e, const gz::sim::components::Visual*, const gz::sim::components::Geometry* g) {
          if (g->Data().Type() != sdf::GeometryType::HEIGHTMAP || !heights_.empty()) {
            return true;
          }
          const auto* hm = g->Data().HeightmapShape();
          const auto pose = gz::sim::worldPose(e, ecm);
          std::string uri = hm->Uri();
          const auto t0 = std::chrono::steady_clock::now();
          gz::common::ImageHeightmap img;
          if (img.Load(uri) != 0) {
            gzerr << "FlyCamProto: cannot load heightmap " << uri << "\n";
            return true;
          }
          size_ = hm->Size();
          origin_ = pose.Pos() + hm->Position();
          n_ = img.Width();
          // Heights come out normalised (pixel / format max); Gazebo stretches the image's own
          // maximum to the heightmap's size.z, so do the same. Row 0 is the image's top row (north).
          img.FillHeightMap(1, n_, size_, gz::math::Vector3d::One, false, heights_);
          const float top = *std::max_element(heights_.begin(), heights_.end());
          for (auto& h : heights_) {
            h = top > 0 ? h / top * size_.Z() : 0.0f;
          }
          const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
          gzmsg << "FlyCamProto: heightmap " << uri << " " << n_ << "x" << img.Height() << " size " << size_
                << " origin " << origin_ << " loaded in " << ms << " ms; heights " << heights_.size() << "\n";
          return false;
        });
    heightmap_tried_ = true;
  }

  // Bilinear terrain height at world (x, y); -inf outside the heightmap.
  double Ground(double x, double y) const {
    if (heights_.empty()) {
      return -1e9;
    }
    const double u = (x - origin_.X()) / size_.X() + 0.5;   // 0 west .. 1 east
    const double v = 0.5 - (y - origin_.Y()) / size_.Y();   // 0 north .. 1 south (image rows)
    if (u < 0 || u > 1 || v < 0 || v > 1) {
      return -1e9;
    }
    const double fx = u * (n_ - 1), fy = v * (n_ - 1);
    const int i = std::min<int>(n_ - 2, static_cast<int>(fx));
    const int j = std::min<int>(n_ - 2, static_cast<int>(fy));
    const double ax = fx - i, ay = fy - j;
    auto h = [&](int col, int row) { return heights_[row * n_ + col]; };
    return origin_.Z() + (1 - ay) * ((1 - ax) * h(i, j) + ax * h(i + 1, j)) +
           ay * ((1 - ax) * h(i, j + 1) + ax * h(i + 1, j + 1));
  }

  void OnPreRender() {
    const double width = ortho_width_.load();
    if (width == applied_width_) {
      return;
    }
    auto scene = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene) {
      return;
    }
    gz::rendering::CameraPtr camera;
    {
      for (unsigned int k = 0; k < scene->SensorCount(); ++k) {
        auto sensor = scene->SensorByIndex(k);
        auto cam = std::dynamic_pointer_cast<gz::rendering::Camera>(sensor);
        if (cam) {
          gzmsg << "FlyCamProto: render camera '" << cam->Name() << "'\n";
        }
        if (cam && cam->Name().find(model_name_ + "::") != std::string::npos) {
          camera = cam;
          camera_found_ = cam->Name();
        }
      }
      if (!camera) {
        return;
      }
    }
    if (width > 0) {
      const double w = width, h = width * camera->ImageHeight() / camera->ImageWidth();
      const double n = camera->NearClipPlane(), f = camera->FarClipPlane();
      gz::math::Matrix4d m = gz::math::Matrix4d::Zero;
      m(0, 0) = 2.0 / w;
      m(1, 1) = 2.0 / h;
      m(2, 2) = -2.0 / (f - n);
      m(2, 3) = -(f + n) / (f - n);
      m(3, 3) = 1.0;
      camera->SetProjectionType(gz::rendering::CPT_ORTHOGRAPHIC);
      camera->SetProjectionMatrix(m);
    } else {
      camera->SetProjectionType(gz::rendering::CPT_PERSPECTIVE);
    }
    gzmsg << "FlyCamProto: projection " << (width > 0 ? "orthographic " : "perspective ") << width << "\n";
    applied_width_ = width;
  }

  void OnCmd(const gz::msgs::Twist& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_v_.Set(msg.linear().x(), msg.linear().y(), msg.linear().z());
    cmd_wz_ = msg.angular().z();
    cmd_wy_ = msg.angular().y();
    cmd_time_wall_ = std::chrono::steady_clock::now();
  }
  void OnLook(const gz::msgs::Vector3d& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_yaw_ += msg.x();
    pending_pitch_ += msg.y();
  }
  void OnGoto(const gz::msgs::Pose& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_goto_ = gz::msgs::Convert(msg);
  }
  void OnOrtho(const gz::msgs::Double& msg) { ortho_width_ = msg.data(); }

  gz::sim::Model model_;
  std::string model_name_;
  double clearance_ = 1.0, tau_ = 0.2, deadman_ = 0.3;
  std::string method_ = "cmd";
  double look_tau_ = 0.0, target_yaw_ = 0.0, target_pitch_ = 0.0;
  bool look_init_ = false;
  bool only_on_change_ = false;
  bool posed_ = false;
  gz::math::Pose3d last_pose_;
  gz::math::Vector3d pos_, vel_;
  double yaw_ = 0, pitch_ = 0, wz_ = 0, wy_ = 0;
  double last_state_ = -1;
  bool heightmap_tried_ = false;
  std::vector<float> heights_;
  gz::math::Vector3d size_, origin_;
  unsigned int n_ = 0;

  std::mutex mutex_;
  gz::math::Vector3d cmd_v_;
  double cmd_wz_ = 0, cmd_wy_ = 0, pending_yaw_ = 0, pending_pitch_ = 0;
  std::optional<std::chrono::steady_clock::time_point> cmd_time_wall_;
  std::optional<gz::math::Pose3d> pending_goto_;
  std::atomic<double> ortho_width_{0.0};
  double applied_width_ = 0.0;
  // No member pointer to the rendering camera: its control block lives in the ogre2 plugin, which is
  // unloaded before systems are destroyed, so even a weak_ptr crashes the destructor.
  std::string camera_found_;
  gz::common::ConnectionPtr render_conn_;

  gz::transport::Node node_;
  gz::transport::Node::Publisher state_;
};

}  // namespace rover_sim

GZ_ADD_PLUGIN(rover_sim::FlyCamProto, gz::sim::System, rover_sim::FlyCamProto::ISystemConfigure,
              rover_sim::FlyCamProto::ISystemPreUpdate)
