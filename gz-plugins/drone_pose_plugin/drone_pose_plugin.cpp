#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>  // Include the Model component header
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
#include <iostream>
#include <gz/transport/Node.hh>
#include <gz/transport/AdvertiseOptions.hh>
#include "drone_pose_stamped.pb.h"

using namespace gz;
using namespace sim;

class dronePosePlugin
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
{
  public:
    dronePosePlugin() : node(new transport::Node())
    {
      publish = node->Advertise<drone_pose_plugin::DronePoseStamped>("/drone_pose");
    }

    virtual void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &/*_eventMgr*/) override
    {
      if (!_sdf->HasElement("model_name")) {
        std::cerr << "SDF element 'model_name' not found." << std::endl;
        return;
      }

      auto modelName = _sdf->Get<std::string>("model_name");
      std::cout << "Model name from SDF: " << modelName << std::endl;

      this->modelEntity = _ecm.EntityByComponents(
          components::Name(modelName), components::Model());

      if (this->modelEntity == Entity()) {
        std::cerr << "Entity with name '" << modelName << "' not found." << std::endl;
      }
    }

    virtual void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override
    {
      if (this->modelEntity == Entity()) {
        std::cerr << "Entity is still not initialized." << std::endl;
        return;
      }

      auto poseComp = _ecm.Component<components::Pose>(this->modelEntity);
      if (!poseComp) {
        std::cerr << "Pose component is null" << std::endl;
        return;
      }

      auto pose = poseComp->Data();
      //std::cout << "Model Pose: " << pose << std::endl;

      //if(publish && publish.HasConnections()){
        drone_pose_plugin::DronePoseStamped msg;
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();
    std::cout << "Debug: Converted simulation time to nanoseconds." << std::endl;

    auto loadStamp = msg.mutable_header()->mutable_stamp();
    loadStamp->set_sec(ns / 1000000000);
    loadStamp->set_nsec(ns % 1000000000);
    std::cout << "Debug: Set timestamp in DroneHeader." << std::endl;

    (*msg.mutable_header()->mutable_data())["frame_id"] = "x500_0";
    (*msg.mutable_header()->mutable_data())["seq"] = "5438";
    std::cout << "Debug: Set other fields in DroneHeader." << std::endl;

// Set the position and orientation in the Pose
msg.mutable_pose()->mutable_position()->set_x(pose.Pos().X());
msg.mutable_pose()->mutable_position()->set_y(pose.Pos().Y());
msg.mutable_pose()->mutable_position()->set_z(pose.Pos().Z());

msg.mutable_pose()->mutable_orientation()->set_w(pose.Rot().W());
msg.mutable_pose()->mutable_orientation()->set_x(pose.Rot().X());
msg.mutable_pose()->mutable_orientation()->set_y(pose.Rot().Y());
msg.mutable_pose()->mutable_orientation()->set_z(pose.Rot().Z());

        publish.Publish(msg);
    }
private:
    Entity modelEntity = Entity();
    std::unique_ptr<transport::Node> node;
    transport::Node::Publisher publish;
};

GZ_ADD_PLUGIN(dronePosePlugin,
              gz::sim::System,
              dronePosePlugin::ISystemConfigure,
              dronePosePlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(dronePosePlugin, "gz::sim::systems::dronePosePlugin")
