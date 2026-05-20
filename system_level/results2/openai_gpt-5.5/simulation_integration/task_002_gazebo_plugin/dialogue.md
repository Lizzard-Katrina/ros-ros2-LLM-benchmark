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
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <string>
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
      owns_ros_context_ = true;
    }

    node_ = std::make_shared<rclcpp::Node>("world_plugin_tutorial");

    RCLCPP_INFO(node_->get_logger(), "Hello World!");
  }

  ~WorldPluginTutorial() override
  {
    if (executor_)
    {
      executor_->cancel();
    }

    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }

    service_.reset();
    node_.reset();

    if (owns_ros_context_ && rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void Load(physics::WorldPtr world, sdf::ElementPtr /*sdf*/) override
  {
    world_ = world;

    service_ = node_->create_service<std_srvs::srv::Trigger>(
      "world_plugin_tutorial/get_world_info",
      std::bind(
        &WorldPluginTutorial::GetWorldInfo,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    if (world_)
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "World plugin loaded for world [%s]",
        world_->Name().c_str());
    }
  }

private:
  void GetWorldInfo(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!world_)
    {
      response->success = false;
      response->message = "Gazebo world is not available";
      return;
    }

    response->success = true;
    response->message =
      "World: " + world_->Name() +
      ", models: " + std::to_string(world_->ModelCount()) +
      ", sim_time: " + std::to_string(world_->SimTime().Double());
  }

  physics::WorldPtr world_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  bool owns_ros_context_{false};
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}