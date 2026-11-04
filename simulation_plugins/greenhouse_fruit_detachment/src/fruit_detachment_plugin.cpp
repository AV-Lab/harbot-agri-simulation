#include <map>
#include <mutex>
#include <memory>
#include <string>
#include <ros/ros.h>
#include <functional>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <boost/thread/recursive_mutex.hpp>
#include <greenhouse_fruit_detachment/DetachFruit.h>


namespace gazebo
{
class GreenhouseFruitDetachment : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    if (!ros::isInitialized())
    {
      ROS_FATAL("Load libgazebo_ros_api_plugin.so before the fruit detachment plugin.");
      return;
    }

    this->world = world;
    this->plant_link = Param(sdf, "plant_link", "orange_stem");
    this->fruit_link = Param(sdf, "fruit_link", "body");
    this->plant_model = Param(sdf, "plant_model", "greenhouse");
    this->fruit_prefix = Param(sdf, "fruit_prefix", "orange");

    this->node.reset(new ros::NodeHandle("fruit_detachment"));
    this->service = this->node->advertiseService("detach", &GreenhouseFruitDetachment::OnDetach, this);
    this->update = event::Events::ConnectWorldUpdateBegin(std::bind(&GreenhouseFruitDetachment::CreateJoints, this));
  }

private:
  static std::string Param(sdf::ElementPtr sdf, const std::string &name, const std::string &fallback)
  {
    return sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
  }

  void CreateJoints()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->joints_created)
      return;

    physics::ModelPtr plant = this->world->ModelByName(this->plant_model);
    if (!plant)
      return;

    this->joints_created = true;
    physics::LinkPtr parent = plant->GetLink(this->plant_link);
    if (!parent)
    {
      ROS_ERROR("Fruit detachment: link %s not found in model %s", this->plant_link.c_str(), this->plant_model.c_str());
      return;
    }

    for (const physics::ModelPtr &fruit : this->world->Models())
    {
      const std::string name = fruit->GetName();
      physics::LinkPtr child = fruit->GetLink(this->fruit_link);
      if (name.rfind(this->fruit_prefix, 0) != 0 || !child)
        continue;

      physics::JointPtr joint = this->world->Physics()->CreateJoint("fixed", plant);
      joint->SetName(name + "_stem");
      joint->Load(parent, child, ignition::math::Pose3d::Zero);
      joint->Init();
      this->joints[name] = joint;
      this->fruits[name] = fruit;
    }

    ROS_INFO("Fruit detachment: attached %zu fruits to %s::%s", this->joints.size(), this->plant_model.c_str(), this->plant_link.c_str());
  }

  bool OnDetach(greenhouse_fruit_detachment::DetachFruit::Request &request, greenhouse_fruit_detachment::DetachFruit::Response &response)
  {
    boost::recursive_mutex::scoped_lock physics_lock(*this->world->Physics()->GetPhysicsUpdateMutex());
    std::lock_guard<std::mutex> lock(this->mutex);

    auto match = this->joints.find(request.fruit);
    if (match == this->joints.end())
    {
      response.success = false;
      response.message = (this->joints_created ? "no attached fruit named " : "fruits are not attached yet: ") + request.fruit;
      return true;
    }

    if (match->second)
    {
      match->second->Detach();
      match->second->Fini();
      match->second.reset();

      // Wake the fruit; a body that rested on its joint stays asleep otherwise.
      this->fruits[request.fruit]->SetEnabled(true);
      this->fruits[request.fruit]->SetGravityMode(true);
    }

    response.success = true;
    response.message = "detached " + request.fruit;
    return true;
  }

  physics::WorldPtr world;
  std::string plant_link;
  std::string fruit_link;
  std::string plant_model;
  std::string fruit_prefix;
  std::mutex mutex;
  bool joints_created = false;
  ros::ServiceServer service;
  event::ConnectionPtr update;
  std::unique_ptr<ros::NodeHandle> node;
  std::map<std::string, physics::JointPtr> joints;
  std::map<std::string, physics::ModelPtr> fruits;
};

GZ_REGISTER_WORLD_PLUGIN(GreenhouseFruitDetachment)
}
