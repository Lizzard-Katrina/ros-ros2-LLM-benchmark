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
#include <gazebo/common/Events.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <thread>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
      ros_initialized_here_ = true;
    }

    node_ = std::make_shared<rclcpp::Node>("world_plugin_tutorial");

    RCLCPP_INFO(node_->get_logger(), "Hello World!");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });
  }

  ~WorldPluginTutorial() override
  {
    update_connection_.reset();

    if (executor_)
    {
      executor_->cancel();
    }

    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }

    if (executor_ && node_)
    {
      executor_->remove_node(node_);
    }

    publisher_.reset();
    node_.reset();

    if (ros_initialized_here_ && rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    (void)_sdf;

    world_ = _world;

    publisher_ = node_->create_publisher<std_msgs::msg::String>("world_plugin/status", 10);

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WorldPluginTutorial::OnUpdate, this));

    RCLCPP_INFO(
      node_->get_logger(),
      "WorldPluginTutorial loaded for world [%s]",
      world_ ? world_->Name().c_str() : "unknown");
  }

private:
  void OnUpdate()
  {
    if (!world_ || !publisher_)
    {
      return;
    }

    const common::Time sim_time = world_->SimTime();

    if ((sim_time - last_publish_time_).Double() < 1.0)
    {
      return;
    }

    last_publish_time_ = sim_time;

    std_msgs::msg::String msg;
    msg.data = "World [" + world_->Name() + "] simulation time: " + std::to_string(sim_time.Double());
    publisher_->publish(msg);
  }

  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  common::Time last_publish_time_;
  bool ros_initialized_here_{false};
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}