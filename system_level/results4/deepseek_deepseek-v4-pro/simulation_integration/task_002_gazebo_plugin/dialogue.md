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
----------------------------
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
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("WorldPluginTutorial"),
        "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    RCLCPP_INFO(rclcpp::get_logger("WorldPluginTutorial"), "Hello World!");

    // Create a ROS2 node for communication and world interaction
    node_ = rclcpp::Node::make_shared("world_plugin_tutorial_node");

    // Example: Create a publisher to send world state information
    publisher_ = node_->create_publisher<std_msgs::msg::String>("world_status", 10);

    // Example: Create a timer to periodically publish world information
    timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&WorldPluginTutorial::OnTimer, this));

    // Example: Create a subscription to receive commands
    subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "world_command", 10,
      std::bind(&WorldPluginTutorial::OnCommandReceived, this, std::placeholders::_1));

    // Spin the node in a separate thread to process callbacks
    executor_thread_ = std::thread([this]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node_);
      executor.spin();
    });

    RCLCPP_INFO(rclcpp::get_logger("WorldPluginTutorial"), "World plugin initialized successfully");
  }

  ~WorldPluginTutorial()
  {
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
  }

private:
  void OnTimer()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "World plugin is running";
    publisher_->publish(msg);
    RCLCPP_DEBUG(rclcpp::get_logger("WorldPluginTutorial"), "Published world status");
  }

  void OnCommandReceived(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("WorldPluginTutorial"),
      "Received command: %s", msg->data.c_str());
    // Process world interaction commands here
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread executor_thread_;
};

GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}