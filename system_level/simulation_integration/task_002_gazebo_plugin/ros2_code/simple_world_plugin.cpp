#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    // Initialize the ROS node for Gazebo
    this->ros_node_ = gazebo_ros::Node::Get(_sdf);

    if (!rclcpp::ok())
    {
      RCLCPP_FATAL(this->ros_node_->get_logger(), "A ROS node for Gazebo has not been initialized, unable to load plugin.");
      return;
    }

    RCLCPP_INFO(this->ros_node_->get_logger(), "Hello World!");

    this->world_ = _world;

    // Implement plugin logic for ROS node communication and world interaction
    this->publisher_ = this->ros_node_->create_publisher<std_msgs::msg::String>("world_status", 10);

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WorldPluginTutorial::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    // Publish world statistics periodically
    auto sim_time = this->world_->SimTime();
    if (sim_time.sec % 10 == 0 && sim_time.nsec == 0) {
      std_msgs::msg::String msg;
      msg.data = "Simulation time: " + std::to_string(sim_time.Double());
      this->publisher_->publish(msg);
    }
  }

  gazebo_ros::Node::SharedPtr ros_node_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}