#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
#include <iostream>

using namespace gz;
using namespace sim;

class MyPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
{
virtual void Configure(const Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       EntityComponentManager &_ecm,
                       EventManager &/*_eventMgr*/) override
{
  auto modelName = _sdf->Get<std::string>("model_name");
  this->modelEntity = _ecm.EntityByComponents(
      components::Name(modelName), components::Model());

  if (this->modelEntity == Entity()) {
    std::cerr << "Entity with name '" << modelName << "' not found." << std::endl;
  }
}


  virtual void PostUpdate(const UpdateInfo &/*_info*/,
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
    std::cout << "Model Pose: " << pose << std::endl;
  }

private:
  Entity modelEntity = Entity();
};

GZ_ADD_PLUGIN(MyPlugin,
              gz::sim::System,
              MyPlugin::ISystemConfigure,
              MyPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(MyPlugin, "gz::sim::systems::MyPlugin")
