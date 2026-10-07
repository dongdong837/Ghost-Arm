// Publish measured single-axis joint positions at 50 Hz, not every physics step.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/model.pb.h>
#include <ignition/plugin/Register.hh>
#include <chrono>
#include <vector>
#include <string>

namespace ghost_arm {
class JointFeedback : public ignition::gazebo::System,
                      public ignition::gazebo::ISystemConfigure,
                      public ignition::gazebo::ISystemPostUpdate {
  ignition::transport::Node node;
  ignition::transport::Node::Publisher publisher;
  std::vector<std::pair<std::string, ignition::gazebo::Entity>> joints;
  std::chrono::steady_clock::duration last{};
  bool sent = false;
 public:
  void Configure(const ignition::gazebo::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &,
                 ignition::gazebo::EntityComponentManager &ecm,
                 ignition::gazebo::EventManager &) override {
    ignition::gazebo::Model model(entity);
    for (const auto *name : {"tail_yaw", "tail_shoulder", "tail_elbow", "tail_wrist",
                             "finger_left_slide", "finger_right_slide"}) {
      auto id = model.JointByName(ecm, name);
      if (id == ignition::gazebo::kNullEntity) continue;
      if (!ecm.Component<ignition::gazebo::components::JointPosition>(id))
        ecm.CreateComponent(id, ignition::gazebo::components::JointPosition());
      joints.emplace_back(name, id);
    }
    publisher = node.Advertise<ignition::msgs::Model>("/ghost/arm/joint_states");
  }
  void PostUpdate(const ignition::gazebo::UpdateInfo &info,
                  const ignition::gazebo::EntityComponentManager &ecm) override {
    if (info.paused) return;
    // Align to the same 20 ms simulation grid as PosePublisher for exact-stamp FK checks.
    const auto period = std::chrono::milliseconds(20);
    const auto slot = info.simTime / period;
    if (sent && info.simTime >= last && slot == last / period) return;
    sent = true; last = info.simTime;
    ignition::msgs::Model message;
    message.set_name("ghost");
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    message.mutable_header()->mutable_stamp()->set_sec(ns / 1000000000);
    message.mutable_header()->mutable_stamp()->set_nsec(ns % 1000000000);
    for (const auto &joint : joints) {
      const auto *position = ecm.Component<ignition::gazebo::components::JointPosition>(joint.second);
      if (!position || position->Data().empty()) continue;
      auto *item = message.add_joint();
      item->set_name(joint.first); item->set_id(joint.second);
      item->mutable_axis1()->set_position(position->Data()[0]);
    }
    publisher.Publish(message);
  }
};
}
IGNITION_ADD_PLUGIN(ghost_arm::JointFeedback, ignition::gazebo::System,
                   ghost_arm::JointFeedback::ISystemConfigure,
                   ghost_arm::JointFeedback::ISystemPostUpdate)
