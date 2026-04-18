# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: simple_world_plugin.cpp
----------------------------
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    ROS_INFO("Hello World!");
  }

   // TODO: Implement plugin logic for ROS node communication and world interaction
   //END 
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}

----------------------------


# LLM Response

[FILENAME: simple_world_plugin.cpp]
#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

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
    // Make sure the ROS node for Gazebo has already been initialized
    if (!rclcpp::ok())
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("WorldPluginTutorial"), "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->ros_node_ = rclcpp::Node::make_shared("world_plugin_tutorial");
    RCLCPP_INFO(this->ros_node_->get_logger(), "Hello World from ROS2!");

    this->world_ = _world;

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WorldPluginTutorial::OnUpdate, this));
  }

  void OnUpdate()
  {
    // Example: Print simulation time every second
    static double last_print_time = 0.0;
    double current_time = this->world_->SimTime().Double();

    if (current_time - last_print_time >= 1.0)
    {
      RCLCPP_INFO(this->ros_node_->get_logger(), "Simulation time: %f", current_time);
      last_print_time = current_time;
    }
  }

private:
  rclcpp::Node::SharedPtr ros_node_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}