#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
#include <iostream>
#include <gz/transport/Node.hh>
#include <gz/transport/AdvertiseOptions.hh>
#include "load_pose_stamped.pb.h"
using namespace gz;
using namespace sim;

class MyPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
{
  public:
   MyPlugin() : node(new transport::Node())
   {
    publish = node->Advertise<gz_pose_plugin::LoadPoseStamped>("/load_pose");
   }

virtual void Configure(const Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       EntityComponentManager &_ecm,
                       EventManager &/*_eventMgr*/) override
{
  if (!_sdf->HasElement("load")) {
    std::cerr << "SDF element 'chassis' not found." << std::endl;
    return;
  }

  auto linkName = _sdf->Get<std::string>("load");
  std::cout << "Link name from SDF: " << linkName << std::endl;

  this->linkEntity = _ecm.EntityByComponents(
      components::ParentEntity(_entity),
      components::Name(linkName), components::Link());

  if (this->linkEntity == Entity()) {
    std::cerr << "Entity with name '" << linkName << "' not found." << std::endl;
  }
}

virtual void PostUpdate(const UpdateInfo &_info,
                        const EntityComponentManager &_ecm) override
{
    std::cout << "Debug: Entering PostUpdate." << std::endl;

    if (this->linkEntity == Entity()) {
        std::cerr << "Debug: Entity is still not initialized." << std::endl;
        return;
    }

    auto poseComp = _ecm.Component<components::Pose>(this->linkEntity);
    if (!poseComp) {
        std::cerr << "Debug: Pose component is null" << std::endl;
        return;
    }

    auto pose = poseComp->Data();
    std::cout << "Debug: Obtained Pose Data." << std::endl;

    gz_pose_plugin::LoadPoseStamped msg;
    std::cout << "Debug: Created LoadPoseStamped message object." << std::endl;

    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();
    std::cout << "Debug: Converted simulation time to nanoseconds." << std::endl;

    auto loadStamp = msg.mutable_header()->mutable_stamp();
    loadStamp->set_sec(ns / 1000000000);
    loadStamp->set_nsec(ns % 1000000000);
    std::cout << "Debug: Set timestamp in LoadHeader." << std::endl;

    (*msg.mutable_header()->mutable_data())["frame_id"] = "x500_0/load";
    (*msg.mutable_header()->mutable_data())["seq"] = "5438";
    std::cout << "Debug: Set other fields in LoadHeader." << std::endl;

    msg.mutable_pose()->mutable_position()->set_x(pose.Pos().X());
    msg.mutable_pose()->mutable_position()->set_y(pose.Pos().Y());
    msg.mutable_pose()->mutable_position()->set_z(pose.Pos().Z());
    std::cout << "Debug: Set Position in LoadPose." << std::endl;

    msg.mutable_pose()->mutable_orientation()->set_w(pose.Rot().W());
    msg.mutable_pose()->mutable_orientation()->set_x(pose.Rot().X());
    msg.mutable_pose()->mutable_orientation()->set_y(pose.Rot().Y());
    msg.mutable_pose()->mutable_orientation()->set_z(pose.Rot().Z());
    std::cout << "Debug: Set Orientation in LoadPose." << std::endl;

    std::cout << "Debug: Publishing message with the following data:" << std::endl;
    std::cout << "Debug: Timestamp (sec): " << loadStamp->sec() << ", (nsec): " << loadStamp->nsec() << std::endl;
    std::cout << "Debug: Frame ID: " << (*msg.mutable_header()->mutable_data())["frame_id"] << std::endl;
    std::cout << "Debug: Seq: " << (*msg.mutable_header()->mutable_data())["seq"] << std::endl;
    std::cout << "Debug: Position (X, Y, Z): (" 
              << msg.mutable_pose()->mutable_position()->x() << ", "
              << msg.mutable_pose()->mutable_position()->y() << ", "
              << msg.mutable_pose()->mutable_position()->z() << ")" << std::endl;
    std::cout << "Debug: Orientation (W, X, Y, Z): (" 
              << msg.mutable_pose()->mutable_orientation()->w() << ", "
              << msg.mutable_pose()->mutable_orientation()->x() << ", "
              << msg.mutable_pose()->mutable_orientation()->y() << ", "
              << msg.mutable_pose()->mutable_orientation()->z() << ")" << std::endl;

    publish.Publish(msg);
    std::cout << "Debug: Message published." << std::endl;
}


private:
  Entity linkEntity = Entity();
  std::unique_ptr<transport::Node> node;
  transport::Node::Publisher publish;
};

GZ_ADD_PLUGIN(MyPlugin,
              gz::sim::System,
              MyPlugin::ISystemConfigure,
              MyPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(MyPlugin, "gz::sim::systems::MyPlugin")
