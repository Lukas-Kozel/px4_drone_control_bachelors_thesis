#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
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

  virtual void PostUpdate(const UpdateInfo &/*_info*/,
                          const EntityComponentManager &_ecm) override
  {
    if (this->linkEntity == Entity()) {
      std::cerr << "Entity is still not initialized." << std::endl;
      return;
    }

    auto poseComp = _ecm.Component<components::Pose>(this->linkEntity);
    if (!poseComp) {
      std::cerr << "Pose component is null" << std::endl;
      return;
    }

    auto pose = poseComp->Data();
    std::cout << "Link Pose: " << pose << std::endl;
  }

private:
  Entity linkEntity = Entity();
};

GZ_ADD_PLUGIN(MyPlugin,
              gz::sim::System,
              MyPlugin::ISystemConfigure,
              MyPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(MyPlugin, "gz::sim::systems::MyPlugin")
