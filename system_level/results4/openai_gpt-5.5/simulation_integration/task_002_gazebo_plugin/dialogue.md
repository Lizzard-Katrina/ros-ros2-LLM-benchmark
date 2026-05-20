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
    if (!rclcpp::is_initialized())
    {
      rclcpp::init(0, nullptr);
    }

    node_ = rclcpp::Node::make_shared("world_plugin_tutorial");

    RCLCPP_INFO(node_->get_logger(), "Hello World!");
  }

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;

    if (!node_)
    {
      if (!rclcpp::is_initialized())
      {
        rclcpp::init(0, nullptr);
      }
      node_ = rclcpp::Node::make_shared("world_plugin_tutorial");
    }

    timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      [this]()
      {
        if (this->world_)
        {
          RCLCPP_INFO_THROTTLE(
            this->node_->get_logger(),
            *this->node_->get_clock(),
            5000,
            "World [%s] simulation time: %.3f",
            this->world_->Name().c_str(),
            this->world_->SimTime().Double());
        }
      });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread(
      [this]()
      {
        executor_->spin();
      });
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

    if (executor_ && node_)
    {
      executor_->remove_node(node_);
    }

    timer_.reset();
    node_.reset();
  }

private:
  physics::WorldPtr world_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}