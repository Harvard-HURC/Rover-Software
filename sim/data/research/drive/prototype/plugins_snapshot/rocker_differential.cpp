// RockerDifferential: a gz-sim system that couples two rocker joints the way
// a differential does.
//
// Every step it applies the same torque to both joints,
//   tau = -k (q_L + q_R) - c (qdot_L + qdot_R),
// a stiff spring-damper on the motion a differential forbids (both rockers
// turning the same way relative to the body). The allowed motion, opposite
// rotation, stays free. Equal torque on both joints is what a real
// differential does: it splits the reaction torque between the two sides.
//
// SDF: <left_joint>, <right_joint>, <stiffness> [N m/rad], <damping> [N m s/rad].
//
// Why not an SDF <mimic> joint: in Gazebo Harmonic only Bullet-Featherstone
// implements mimic, and Bullet cannot skid-steer this rover (sim/README.md).

#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace rover_sim {

class RockerDifferential : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager&) override {
    const gz::sim::Model model(entity);
    const auto left_name = sdf->Get<std::string>("left_joint");
    const auto right_name = sdf->Get<std::string>("right_joint");
    left_ = gz::sim::Joint(model.JointByName(ecm, left_name));
    right_ = gz::sim::Joint(model.JointByName(ecm, right_name));
    if (!left_.Valid(ecm) || !right_.Valid(ecm)) {
      gzerr << "RockerDifferential: <left_joint> '" << left_name << "' and <right_joint> '"
            << right_name << "' must both be joints of model '" << model.Name(ecm)
            << "'; the differential is disabled.\n";
      return;
    }
    stiffness_ = sdf->Get<double>("stiffness", stiffness_).first;
    damping_ = sdf->Get<double>("damping", damping_).first;
    for (auto* joint : {&left_, &right_}) {
      joint->EnablePositionCheck(ecm, true);
      joint->EnableVelocityCheck(ecm, true);
    }
    enabled_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (!enabled_ || info.paused) {
      return;
    }
    const auto ql = left_.Position(ecm);
    const auto qr = right_.Position(ecm);
    const auto vl = left_.Velocity(ecm);
    const auto vr = right_.Velocity(ecm);
    // Empty until physics fills the components during the first step.
    if (!ql || !qr || !vl || !vr || ql->empty() || qr->empty() || vl->empty() || vr->empty()) {
      return;
    }
    const double tau = -stiffness_ * ((*ql)[0] + (*qr)[0]) - damping_ * ((*vl)[0] + (*vr)[0]);
    left_.SetForce(ecm, {tau});
    right_.SetForce(ecm, {tau});
  }

 private:
  gz::sim::Joint left_;
  gz::sim::Joint right_;
  double stiffness_ = 5000.0;
  double damping_ = 100.0;
  bool enabled_ = false;
};

}  // namespace rover_sim

GZ_ADD_PLUGIN(rover_sim::RockerDifferential, gz::sim::System,
              rover_sim::RockerDifferential::ISystemConfigure,
              rover_sim::RockerDifferential::ISystemPreUpdate)
