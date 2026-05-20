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
    if (!rclcpp::is_initialized())
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("gazebo_ros"), "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package)");
      return;
    }

    RCLCPP_INFO(rclcpp::get_logger("gazebo_ros"), "Hello World!");
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    this->world_ = _world;
    if (!rclcpp::is_initialized())
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("gazebo_ros"), "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package)");
      return;
    }
    this->ros_node_ = std::make_shared<rclcpp::Node>("gazebo_world_plugin");
    this->world_update_event_ = this->world_->Events()->ConnectWorldUpdateBegin(
      std::bind(&WorldPluginTutorial::OnWorldUpdate, this));
    RCLCPP_INFO(this->ros_node_->get_logger(), "World plugin successfully loaded!");
  }

  void OnWorldUpdate()
  {
    sim_time_ = this->world_->SimTime();
    if (sim_time_.sec - last_update_time_.sec >= 1.0)
    {
      RCLCPP_INFO(this->ros_node_->get_logger(), "Sim time: %d.%d", sim_time_.sec, sim_time_.nsec);
      last_update_time_ = sim_time_;
    }
  }

private:
  physics::WorldPtr world_;
  event::ConnectionPtr world_update_event_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  common::Time sim_time_;
  common::Time last_update_time_;
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}