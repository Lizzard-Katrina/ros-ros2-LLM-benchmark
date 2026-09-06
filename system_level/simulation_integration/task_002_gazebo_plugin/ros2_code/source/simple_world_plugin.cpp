#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
    // In ROS2 + gazebo_ros, the node is managed by gazebo_ros::Node
    // We just log from the constructor
    RCLCPP_INFO(rclcpp::get_logger("world_plugin_tutorial"), "Hello World!");
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Store the world pointer for later use
    this->world = _world;

    // Store the SDF element pointer
    this->sdf = _sdf;

    RCLCPP_INFO(rclcpp::get_logger("world_plugin_tutorial"),
      "WorldPluginTutorial: Load called. World plugin loaded successfully.");
  }

private:
  physics::WorldPtr world;
  sdf::ElementPtr sdf;
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}