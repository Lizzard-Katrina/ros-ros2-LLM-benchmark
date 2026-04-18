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
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("world_plugin"), "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
      return;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("world_plugin"), "Hello World!");
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    // Initialize ROS node
    this->node_ = rclcpp::Node::make_shared("world_plugin");
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "World plugin loaded");

    // Create a publisher to publish messages
    this->pub_ = this->node_->create_publisher<std_msgs::msg::String>("world_topic", 10);

    // Create a timer to publish messages at regular intervals
    this->timer_ = this->node_->create_wall_timer(std::chrono::seconds(1), std::bind(&WorldPluginTutorial::PublishMessage, this));
  }

  void PublishMessage()
  {
    // Create a message to publish
    std_msgs::msg::String msg;
    msg.data = "Hello from world plugin!";

    // Publish the message
    this->pub_->publish(msg);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "Published message: " << msg.data);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}