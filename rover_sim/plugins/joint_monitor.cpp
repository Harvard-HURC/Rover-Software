// JointMonitor: a gz-sim system that reports a model's state for the URC
// referee.
//
// It publishes every joint's position and every link's world pose at a fixed
// rate (gz.msgs.Model; JointStatePublisher in this Gazebo publishes every 1 ms
// step), and for "press" joints (keys, buttons) an event the moment one is
// pressed, so a short key press is never missed between two state messages.
//
// SDF:
//   <topic>            state, default /model/<model>/state
//   <update_rate>      [Hz] joint states, default 50
//   <press_prefix>     joints whose names start with this are presses (repeatable)
//   <press_threshold>  [m or rad] press depth, default 0.002
//   <press_topic>      gz.msgs.StringMsg with the joint name, default /model/<model>/presses
// A press fires when the position rises above the threshold and re-arms once
// it falls below half of it.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace rover_sim {

class JointMonitor : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager&) override {
    const gz::sim::Model model(entity);
    model_name_ = model.Name(ecm);
    const std::string prefix = "/model/" + model_name_;
    const auto topic = sdf->Get<std::string>("topic", prefix + "/state").first;
    const auto press_topic = sdf->Get<std::string>("press_topic", prefix + "/presses").first;
    const double rate = sdf->Get<double>("update_rate", 50.0).first;
    period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / rate));
    threshold_ = sdf->Get<double>("press_threshold", threshold_).first;

    std::vector<std::string> prefixes;
    for (auto e = sdf->FindElement("press_prefix"); e; e = e->GetNextElement("press_prefix")) {
      prefixes.push_back(e->Get<std::string>());
    }
    for (const auto joint_entity : model.Joints(ecm)) {
      gz::sim::Joint joint(joint_entity);
      joint.EnablePositionCheck(ecm, true);
      const auto name = joint.Name(ecm).value_or("");
      bool press = false;
      for (const auto& p : prefixes) {
        press = press || name.rfind(p, 0) == 0;
      }
      joints_.push_back({joint, name, press, false});
    }
    for (const auto link_entity : model.Links(ecm)) {
      links_.emplace_back(link_entity);
    }
    states_ = node_.Advertise<gz::msgs::Model>(topic);
    presses_ = node_.Advertise<gz::msgs::StringMsg>(press_topic);
  }

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override {
    if (info.paused) {
      return;
    }
    if (info.simTime < last_) {  // the world was reset
      last_ = info.simTime - period_;
    }
    const bool publish = info.simTime - last_ >= period_;
    gz::msgs::Model msg;
    if (publish) {
      last_ = info.simTime;
      msg.set_name(model_name_);
      msg.mutable_header()->mutable_stamp()->CopyFrom(gz::sim::convert<gz::msgs::Time>(info.simTime));
    }
    for (auto& j : joints_) {
      const auto q = j.joint.Position(ecm);
      if (!q || q->empty()) {
        continue;
      }
      const double position = (*q)[0];
      if (j.press) {
        if (!j.down && position > threshold_) {
          j.down = true;
          gz::msgs::StringMsg event;
          event.set_data(j.name);
          presses_.Publish(event);
        } else if (j.down && position < threshold_ / 2) {
          j.down = false;
        }
      }
      if (publish) {
        auto* out = msg.add_joint();
        out->set_name(j.name);
        out->mutable_axis1()->set_position(position);
      }
    }
    if (publish) {
      for (const auto& link : links_) {
        const auto pose = link.WorldPose(ecm);
        if (pose) {
          auto* out = msg.add_link();
          out->set_name(link.Name(ecm).value_or(""));
          gz::msgs::Set(out->mutable_pose(), *pose);
        }
      }
      states_.Publish(msg);
    }
  }

 private:
  struct Watched {
    gz::sim::Joint joint;
    std::string name;
    bool press;
    bool down;
  };
  std::vector<Watched> joints_;
  std::vector<gz::sim::Link> links_;
  std::string model_name_;
  double threshold_ = 0.002;
  std::chrono::steady_clock::duration period_{};
  std::chrono::steady_clock::duration last_{};
  gz::transport::Node node_;
  gz::transport::Node::Publisher states_;
  gz::transport::Node::Publisher presses_;
};

}  // namespace rover_sim

GZ_ADD_PLUGIN(rover_sim::JointMonitor, gz::sim::System, rover_sim::JointMonitor::ISystemConfigure,
              rover_sim::JointMonitor::ISystemPostUpdate)
