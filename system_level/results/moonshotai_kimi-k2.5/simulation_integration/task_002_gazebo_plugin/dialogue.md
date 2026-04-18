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
#include <gazebo/physics/World.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!rclcpp::ok())
    {
      RCLCPP_FATAL(rclcpp::get_logger("world_plugin_tutorial"), 
        "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        "Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package)");
      return;
    }

    RCLCPP_INFO(rclcpp::get_logger("world_plugin_tutorial"), "Hello World!");
  }

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    // TODO: Implement plugin logic for ROS node communication and world interaction
    node_ = rclcpp::Node::make_shared("world_plugin_tutorial");
    
    world_pub_ = node_->create_publisher<std_msgs::msg::String>("world_info", 10);
    world_ = world;
    
    timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        auto message = std_msgs::msg::String();
        message.data = "World: " + world_->Name();
        RCLCPP_INFO(node_->get_logger(), "%s", message.data.c_str());
        world_pub_->publish(message);
      });
    //END 
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr world_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  physics::WorldPtr world_;
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}