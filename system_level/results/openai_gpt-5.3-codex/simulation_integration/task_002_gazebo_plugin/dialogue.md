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
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <sstream>
#include <thread>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
    if (!rclcpp::is_initialized())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("world_plugin_tutorial"),
        "A ROS2 node for Gazebo has not been initialized, unable to load plugin. "
        "Load the Gazebo ROS2 initialization system plugin.");
      return;
    }

    ros_node_ = std::make_shared<rclcpp::Node>("world_plugin_tutorial");
    status_pub_ = ros_node_->create_publisher<std_msgs::msg::String>("world_plugin/status", 10);

    command_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
      "world_plugin/command", 10,
      std::bind(&WorldPluginTutorial::OnCommand, this, std::placeholders::_1));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(ros_node_);

    running_.store(true);
    spin_thread_ = std::thread([this]() {
      while (running_.load() && rclcpp::ok())
      {
        executor_->spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

    RCLCPP_INFO(ros_node_->get_logger(), "Hello World!");
  }

  ~WorldPluginTutorial() override
  {
    running_.store(false);
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }

    if (executor_ && ros_node_)
    {
      executor_->remove_node(ros_node_);
    }
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    (void)_sdf;
    world_ = _world;
    last_pub_sim_time_ = 0.0;

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WorldPluginTutorial::OnUpdate, this));

    if (ros_node_ && world_)
    {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "World plugin connected to world [%s]",
        world_->Name().c_str());
    }
  }

private:
  void OnUpdate()
  {
    if (!world_ || !status_pub_)
    {
      return;
    }

    const double sim_time = world_->SimTime().Double();
    if ((sim_time - last_pub_sim_time_) < 1.0)
    {
      return;
    }

    last_pub_sim_time_ = sim_time;

    std_msgs::msg::String msg;
    std::ostringstream ss;
    ss << "world=" << world_->Name() << ", sim_time=" << sim_time;
    msg.data = ss.str();
    status_pub_->publish(msg);
  }

  void OnCommand(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!world_ || !ros_node_ || !msg)
    {
      return;
    }

    if (msg->data == "reset")
    {
      world_->Reset();
      RCLCPP_INFO(ros_node_->get_logger(), "Received command: reset");
    }
    else if (msg->data == "pause")
    {
      world_->SetPaused(true);
      RCLCPP_INFO(ros_node_->get_logger(), "Received command: pause");
    }
    else if (msg->data == "resume")
    {
      world_->SetPaused(false);
      RCLCPP_INFO(ros_node_->get_logger(), "Received command: resume");
    }
  }

  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  std::thread spin_thread_;
  std::atomic<bool> running_{false};
  double last_pub_sim_time_{0.0};
};

GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}