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

FILE_PATH: diff_drive_controller.cpp
----------------------------
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Bence Magyar, Enrique Fernández
 */

#include <cmath>
#include <diff_drive_controller/diff_drive_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf/transform_datatypes.h>
#include <urdf/urdfdom_compatibility.h>
#include <urdf_parser/urdf_parser.h>

static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x-vec2.x,2) +
                   std::pow(vec1.y-vec2.y,2) +
                   std::pow(vec1.z-vec2.z,2));
}

/*
* \brief Check that a link exists and has a geometry collision.
* \param link The link
* \return true if the link has a collision element with geometry
*/
static bool hasCollisionGeometry(const urdf::LinkConstSharedPtr& link)
{
  if (!link)
  {
    ROS_ERROR("Link pointer is null.");
    return false;
  }

  if (!link->collision)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
    return false;
  }

  if (!link->collision->geometry)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
    return false;
  }
  return true;
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const urdf::LinkConstSharedPtr& link)
{
  if (!hasCollisionGeometry(link))
  {
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
  {
    ROS_DEBUG_STREAM("Link " << link->name << " does not have cylinder geometry");
    return false;
  }

  return true;
}

/*
 * \brief Check if the link is modeled as a sphere
 * \param link Link
 * \return true if the link is modeled as a Sphere; false otherwise
 */
static bool isSphere(const urdf::LinkConstSharedPtr& link)
{
  if (!hasCollisionGeometry(link))
  {
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::SPHERE)
  {
    ROS_DEBUG_STREAM("Link " << link->name << " does not have sphere geometry");
    return false;
  }

  return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
static bool getWheelRadius(const urdf::LinkConstSharedPtr& wheel_link, double& wheel_radius)
{
  if (isCylinder(wheel_link))
  {
    wheel_radius = (static_cast<urdf::Cylinder*>(wheel_link->collision->geometry.get()))->radius;
    return true;
  }
  else if (isSphere(wheel_link))
  {
    wheel_radius = (static_cast<urdf::Sphere*>(wheel_link->collision->geometry.get()))->radius;
    return true;
  }

  ROS_ERROR_STREAM("Wheel link " << wheel_link->name << " is NOT modeled as a cylinder or sphere!");
  return false;
}

namespace diff_drive_controller{

  DiffDriveController::DiffDriveController()
    : open_loop_(false)
    , command_struct_()
    , wheel_separation_(0.0)
    , wheel_radius_(0.0)
    , wheel_separation_multiplier_(1.0)
    , left_wheel_radius_multiplier_(1.0)
    , right_wheel_radius_multiplier_(1.0)
    , cmd_vel_timeout_(0.5)
    , allow_multiple_cmd_vel_publishers_(true)
    , base_frame_id_("base_link")
    , odom_frame_id_("odom")
    , enable_odom_tf_(true)
    , wheel_joints_size_(0)
    , publish_cmd_(false)
    , publish_wheel_joint_controller_state_(false)
  {
  }

  bool DiffDriveController::init(hardware_interface::VelocityJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh)
  {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // Get joint names from the parameter server
    std::vector<std::string> left_wheel_names, right_wheel_names;
    if (!getWheelNames(controller_nh, "left_wheel", left_wheel_names) ||
        !getWheelNames(controller_nh, "right_wheel", right_wheel_names))
    {
      return false;
    }

    if (left_wheel_names.size() != right_wheel_names.size())
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#left wheels (" << left_wheel_names.size() << ") != " <<
          "#right wheels (" << right_wheel_names.size() << ").");
      return false;
    }
    else
    {
      wheel_joints_size_ = left_wheel_names.size();

      left_wheel_joints_.resize(wheel_joints_size_);
      right_wheel_joints_.resize(wheel_joints_size_);
    }

    // Odometry related:
    double publish_rate;
    controller_nh.param("publish_rate", publish_rate, 50.0);
    ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                          << publish_rate << "Hz.");
    publish_period_ = ros::Duration(1.0 / publish_rate);
    //TODO:
    //Declare and retrieve 'open_loop' (bool, default: false), 'velocity_rolling_window_size' (int, default: 10), 
    //and 'cmd_vel_timeout' (double, default: 0.5).
    //Assign values to class members: open_loop_, cmd_vel_timeout_, and call odometry_.setVelocityRollingWindowSize().
    //Log the 'cmd_vel_timeout' value using the ROS 2 logging API.
    // [Style Constraints]:
    //- Use node->declare_parameter<T>() for all declarations.
    //- Use node->get_parameter().as_bool()/as_int()/as_double() for assignment.
    //- Use RCLCPP_INFO_STREAM(node->get_logger(), ...) for logging.
    //END OF TODO
    controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

    controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Odometry frame_id set to " << odom_frame_id_);

    controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
    ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

    // Velocity and acceleration limits:
    controller_nh.param("linear/x/has_velocity_limits"    , limiter_lin_.has_velocity_limits    , limiter_lin_.has_velocity_limits    );
    controller_nh.param("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits, limiter_lin_.has_acceleration_limits);
    controller_nh.param("linear/x/has_jerk_limits"        , limiter_lin_.has_jerk_limits        , limiter_lin_.has_jerk_limits        );
    controller_nh.param("linear/x/max_velocity"           , limiter_lin_.max_velocity           ,  limiter_lin_.max_velocity          );
    controller_nh.param("linear/x/min_velocity"           , limiter_lin_.min_velocity           , -limiter_lin_.max_velocity          );
    controller_nh.param("linear/x/max_acceleration"       , limiter_lin_.max_acceleration       ,  limiter_lin_.max_acceleration      );
    controller_nhon_set_parameters_callback.param("linear/x/min_acceleration"       , limiter_lin_.min_acceleration       , -limiter_lin_.max_acceleration      );
    controller_nh.param("linear/x/max_jerk"               , limiter_lin_.max_jerk               ,  limiter_lin_.max_jerk              );
    controller_nh.param("linear/x/min_jerk"               , limiter_lin_.min_jerk               , -limiter_lin_.max_jerk              );

    controller_nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
    controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
    controller_nh.param("angular/z/has_jerk_limits"        , limiter_ang_.has_jerk_limits        , limiter_ang_.has_jerk_limits        );
    controller_nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/max_jerk"               , limiter_ang_.max_jerk               ,  limiter_ang_.max_jerk              );
    controller_nh.param("angular/z/min_jerk"               , limiter_ang_.min_jerk               , -limiter_ang_.max_jerk              );

    // Publish limited velocity:
    controller_nh.param("publish_cmd", publish_cmd_, publish_cmd_);

    // Publish wheel data:
    controller_nh.param("publish_wheel_joint_controller_state", publish_wheel_joint_controller_state_, publish_wheel_joint_controller_state_);

    // If either parameter is not available, we need to look up the value in the URDF
    bool lookup_wheel_separation = !controller_nh.getParam("wheel_separation", wheel_separation_);
    bool lookup_wheel_radius = !controller_nh.getParam("wheel_radius", wheel_radius_);

    if (!setOdomParamsFromUrdf(root_nh,
                              left_wheel_names[0],
                              right_wheel_names[0],
                              lookup_wheel_separation,
                              lookup_wheel_radius))
    {
      return false;
    }

    // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    const double ws  = wheel_separation_multiplier_   * wheel_separation_;
    const double lwr = left_wheel_radius_multiplier_  * wheel_radius_;
    const double rwr = right_wheel_radius_multiplier_ * wheel_radius_;
    odometry_.setWheelParams(ws, lwr, rwr);
    ROS_INFO_STREAM_NAMED(name_,
                          "Odometry params : wheel separation " << ws
                          << ", left wheel radius "  << lwr
                          << ", right wheel radius " << rwr);

    if (publish_cmd_)
    {
      cmd_vel_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(controller_nh, "cmd_vel_out", 100));
    }

    // Wheel joint controller state:
    if (publish_wheel_joint_controller_state_)
    {
      controller_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>(controller_nh, "wheel_joint_controller_state", 100));

      const size_t num_wheels = wheel_joints_size_ * 2;

      controller_state_pub_->msg_.joint_names.resize(num_wheels);

      controller_state_pub_->msg_.desired.positions.resize(num_wheels);
      controller_state_pub_->msg_.desired.velocities.resize(num_wheels);
      controller_state_pub_->msg_.desired.accelerations.resize(num_wheels);
      controller_state_pub_->msg_.desired.effort.resize(num_wheels);

      controller_state_pub_->msg_.actual.positions.resize(num_wheels);
      controller_state_pub_->msg_.actual.velocities.resize(num_wheels);
      controller_state_pub_->msg_.actual.accelerations.resize(num_wheels);
      controller_state_pub_->msg_.actual.effort.resize(num_wheels);

      controller_state_pub_->msg_.error.positions.resize(num_wheels);
      controller_state_pub_->msg_.error.velocities.resize(num_wheels);
      controller_state_pub_->msg_.error.accelerations.resize(num_wheels);
      controller_state_pub_->msg_.error.effort.resize(num_wheels);

      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        controller_state_pub_->msg_.joint_names[i] = left_wheel_names[i];
        controller_state_pub_->msg_.joint_names[i + wheel_joints_size_] = right_wheel_names[i];
      }

      vel_left_previous_.resize(wheel_joints_size_, 0.0);
      vel_right_previous_.resize(wheel_joints_size_, 0.0);
    }

    setOdomPubFields(root_nh, controller_nh);

    // Get the joint object to use in the realtime loop
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      ROS_INFO_STREAM_NAMED(name_,
                            "Adding left wheel with joint name: " << left_wheel_names[i]
                            << " and right wheel with joint name: " << right_wheel_names[i]);
      left_wheel_joints_[i] = hw->getHandle(left_wheel_names[i]);  // throws on failure
      right_wheel_joints_[i] = hw->getHandle(right_wheel_names[i]);  // throws on failure
    }

    sub_command_ = controller_nh.subscribe("cmd_vel", 1, &DiffDriveController::cmdVelCallback, this);

    // Initialize dynamic parameters
    DynamicParams dynamic_params;
    dynamic_params.left_wheel_radius_multiplier  = left_wheel_radius_multiplier_;
    dynamic_params.right_wheel_radius_multiplier = right_wheel_radius_multiplier_;
    dynamic_params.wheel_separation_multiplier   = wheel_separation_multiplier_;

    dynamic_params.publish_rate = publish_rate;
    dynamic_params.enable_odom_tf = enable_odom_tf_;

    dynamic_params_.writeFromNonRT(dynamic_params);

    // Initialize dynamic_reconfigure server
    DiffDriveControllerConfig config;
    config.left_wheel_radius_multiplier  = left_wheel_radius_multiplier_;
    config.right_wheel_radius_multiplier = right_wheel_radius_multiplier_;
    config.wheel_separation_multiplier   = wheel_separation_multiplier_;

    config.publish_rate = publish_rate;
    config.enable_odom_tf = enable_odom_tf_;

    dyn_reconf_server_ = std::make_shared<ReconfigureServer>(dyn_reconf_server_mutex_, controller_nh);

    // Update parameters
    dyn_reconf_server_mutex_.lock();
    dyn_reconf_server_->updateConfig(config);
    dyn_reconf_server_mutex_.unlock();

    dyn_reconf_server_->setCallback(
        std::bind(&DiffDriveController::reconfCallback, this, std::placeholders::_1, std::placeholders::_2));

    return true;
  }

  void DiffDriveController::update(const ros::Time& time, const ros::Duration& period)
  {
    // update parameter from dynamic reconf
    updateDynamicParams();

    // Apply (possibly new) multipliers:
    const double ws  = wheel_separation_multiplier_   * wheel_separation_;
    const double lwr = left_wheel_radius_multiplier_  * wheel_radius_;
    const double rwr = right_wheel_radius_multiplier_ * wheel_radius_;

    odometry_.setWheelParams(ws, lwr, rwr);

    // COMPUTE AND PUBLISH ODOMETRY
    if (open_loop_)
    {
      odometry_.updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, time);
    }
    else
    {
      double left_pos  = 0.0;
      double right_pos = 0.0;
      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        const double lp = left_wheel_joints_[i].getPosition();
        const double rp = right_wheel_joints_[i].getPosition();
        if (std::isnan(lp) || std::isnan(rp))
          return;

        left_pos  += lp;
        right_pos += rp;
      }
      left_pos  /= wheel_joints_size_;
      right_pos /= wheel_joints_size_;

      // Estimate linear and angular velocity using joint information
      odometry_.update(left_pos, right_pos, time);
    }

    // Publish odometry message
    if (last_state_publish_time_ + publish_period_ < time)
    {
      last_state_publish_time_ += publish_period_;
      // Compute and store orientation info
      const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

      // Populate odom message and publish
      if (odom_pub_->trylock())
      {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
        odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
        odom_pub_->msg_.pose.pose.orientation = orientation;
        odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinear();
        odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
        odom_pub_->unlockAndPublish();
      }

      // Publish tf /odom frame
      if (enable_odom_tf_ && tf_odom_pub_->trylock())
      {
        geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = odometry_.getX();
        odom_frame.transform.translation.y = odometry_.getY();
        odom_frame.transform.rotation = orientation;
        tf_odom_pub_->unlockAndPublish();
      }
    }

    // MOVE ROBOT
    // Retreive current velocity command and time step:
    Commands curr_cmd = *(command_.readFromRT());
    const double dt = (time - curr_cmd.stamp).toSec();

    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd.lin = 0.0;
      curr_cmd.ang = 0.0;
    }

    // Limit velocities and accelerations:
    const double cmd_dt(period.toSec());

    limiter_lin_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);
    limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd;

    // Publish limited velocity:
    if (publish_cmd_ && cmd_vel_pub_ && cmd_vel_pub_->trylock())
    {
      cmd_vel_pub_->msg_.header.stamp = time;
      cmd_vel_pub_->msg_.twist.linear.x = curr_cmd.lin;
      cmd_vel_pub_->msg_.twist.angular.z = curr_cmd.ang;
      cmd_vel_pub_->unlockAndPublish();
    }

    // Compute wheels velocities:
    const double vel_left  = (curr_cmd.lin - curr_cmd.ang * ws / 2.0)/lwr;
    const double vel_right = (curr_cmd.lin + curr_cmd.ang * ws / 2.0)/rwr;

    // Set wheels velocities:
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      left_wheel_joints_[i].setCommand(vel_left);
      right_wheel_joints_[i].setCommand(vel_right);
    }

    publishWheelData(time, period, curr_cmd, ws, lwr, rwr);
    time_previous_ = time;
  }

  void DiffDriveController::starting(const ros::Time& time)
  {
    brake();

    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;
    time_previous_ = time;

    odometry_.init(time);
  }

  void DiffDriveController::stopping(const ros::Time& /*time*/)
  {
    brake();
  }

  void DiffDriveController::brake()
  {
    const double vel = 0.0;
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      left_wheel_joints_[i].setCommand(vel);
      right_wheel_joints_[i].setCommand(vel);
    }
  }

  void DiffDriveController::cmdVelCallback(const geometry_msgs::Twist& command)
  {
    if (isRunning())
    {
      // check that we don't have multiple publishers on the command topic
      if (!allow_multiple_cmd_vel_publishers_ && sub_command_.getNumPublishers() > 1)
      {
        ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_, "Detected " << sub_command_.getNumPublishers()
            << " publishers. Only 1 publisher is allowed. Going to brake.");
        brake();
        return;
      }

      if(!std::isfinite(command.angular.z) || !std::isfinite(command.linear.x))
      {
        ROS_WARN_THROTTLE(1.0, "Received NaN in velocity command. Ignoring.");
        return;
      }

      command_struct_.ang   = command.angular.z;
      command_struct_.lin   = command.linear.x;
      command_struct_.stamp = ros::Time::now();
      command_.writeFromNonRT (command_struct_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                             << "Ang: "   << command_struct_.ang << ", "
                             << "Lin: "   << command_struct_.lin << ", "
                             << "Stamp: " << command_struct_.stamp);
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }

  bool DiffDriveController::getWheelNames(ros::NodeHandle& controller_nh,
                              const std::string& wheel_param,
                              std::vector<std::string>& wheel_names)
  {
      XmlRpc::XmlRpcValue wheel_list;
      if (!controller_nh.getParam(wheel_param, wheel_list))
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Couldn't retrieve wheel param '" << wheel_param << "'.");
        return false;
      }

      if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        if (wheel_list.size() == 0)
        {
          ROS_ERROR_STREAM_NAMED(name_,
              "Wheel param '" << wheel_param << "' is an empty list");
          return false;
        }

        for (int i = 0; i < wheel_list.size(); ++i)
        {
          if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
          {
            ROS_ERROR_STREAM_NAMED(name_,
                "Wheel param '" << wheel_param << "' #" << i <<
                " isn't a string.");
            return false;
          }
        }

        wheel_names.resize(wheel_list.size());
        for (int i = 0; i < wheel_list.size(); ++i)
        {
          wheel_names[i] = static_cast<std::string>(wheel_list[i]);
        }
      }
      else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        wheel_names.push_back(wheel_list);
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Wheel param '" << wheel_param <<
            "' is neither a list of strings nor a string.");
        return false;
      }

      return true;
  }

  bool DiffDriveController::setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
                             const std::string& left_wheel_name,
                             const std::string& right_wheel_name,
                             bool lookup_wheel_separation,
                             bool lookup_wheel_radius)
  {
    if (!(lookup_wheel_separation || lookup_wheel_radius))
    {
      // Short-circuit in case we don't need to look up anything, so we don't have to parse the URDF
      return true;
    }

    // Parse robot description
    const std::string model_param_name = "robot_description";
    bool res = root_nh.hasParam(model_param_name);
    std::string robot_model_str="";
    if (!res || !root_nh.getParam(model_param_name,robot_model_str))
    {
      ROS_ERROR_NAMED(name_, "Robot description couldn't be retrieved from param server.");
      return false;
    }

    urdf::ModelInterfaceSharedPtr model(urdf::parseURDF(robot_model_str));

    urdf::JointConstSharedPtr left_wheel_joint(model->getJoint(left_wheel_name));
    urdf::JointConstSharedPtr right_wheel_joint(model->getJoint(right_wheel_name));

    if (!left_wheel_joint)
    {
      ROS_ERROR_STREAM_NAMED(name_, left_wheel_name
                             << " couldn't be retrieved from model description");
      return false;
    }

    if (!right_wheel_joint)
    {
      ROS_ERROR_STREAM_NAMED(name_, right_wheel_name
                             << " couldn't be retrieved from model description");
      return false;
    }

    if (lookup_wheel_separation)
    {
      // Get wheel separation
      ROS_INFO_STREAM("left wheel to origin: " << left_wheel_joint->parent_to_joint_origin_transform.position.x << ","
                      << left_wheel_joint->parent_to_joint_origin_transform.position.y << ", "
                      << left_wheel_joint->parent_to_joint_origin_transform.position.z);
      ROS_INFO_STREAM("right wheel to origin: " << right_wheel_joint->parent_to_joint_origin_transform.position.x << ","
                      << right_wheel_joint->parent_to_joint_origin_transform.position.y << ", "
                      << right_wheel_joint->parent_to_joint_origin_transform.position.z);

      wheel_separation_ = euclideanOfVectors(left_wheel_joint->parent_to_joint_origin_transform.position,
                                             right_wheel_joint->parent_to_joint_origin_transform.position);

    }

    if (lookup_wheel_radius)
    {
      // Get wheel radius
      if (!getWheelRadius(model->getLink(left_wheel_joint->child_link_name), wheel_radius_))
      {
        ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve " << left_wheel_name << " wheel radius");
        return false;
      }
    }

    return true;
  }

  void DiffDriveController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    /* * TODO (Task 3):
 * 1. Declare and retrieve 'pose_covariance_diagonal' and 'twist_covariance_diagonal' as double arrays.
 * 2. Validate that each retrieved vector has exactly 6 elements.
 * 3. If validation fails, throw a std::invalid_argument with the message "diagonal size must be 6".
 * 4. Store the vectors into local std::vector<double> variables for later mapping.
 * * [Style Constraints]:
 * - Must use std::vector<double> as the underlying data structure.
 * - Use node->declare_parameter<std::vector<double>>().
 * - Use explicit size() check and throw statement.
 */
    //END of TODO
    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = odom_frame_id_;
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = {
        static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5]) };
    odom_pub_->msg_.twist.twist.linear.y  = 0;
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = {
        static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5]) };
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
  }

  void DiffDriveController::reconfCallback(DiffDriveControllerConfig& config, uint32_t /*level*/)
  {
    DynamicParams dynamic_params;
    dynamic_params.left_wheel_radius_multiplier  = config.left_wheel_radius_multiplier;
    dynamic_params.right_wheel_radius_multiplier = config.right_wheel_radius_multiplier;
    dynamic_params.wheel_separation_multiplier   = config.wheel_separation_multiplier;

    dynamic_params.publish_rate = config.publish_rate;

    dynamic_params.enable_odom_tf = config.enable_odom_tf;

    dynamic_params_.writeFromNonRT(dynamic_params);

    ROS_INFO_STREAM_NAMED(name_, "Dynamic Reconfigure:\n" << dynamic_params);
  }

  void DiffDriveController::updateDynamicParams()
  {
    // Retreive dynamic params:
    const DynamicParams dynamic_params = *(dynamic_params_.readFromRT());

    left_wheel_radius_multiplier_  = dynamic_params.left_wheel_radius_multiplier;
    right_wheel_radius_multiplier_ = dynamic_params.right_wheel_radius_multiplier;
    wheel_separation_multiplier_   = dynamic_params.wheel_separation_multiplier;

    publish_period_ = ros::Duration(1.0 / dynamic_params.publish_rate);
    enable_odom_tf_ = dynamic_params.enable_odom_tf;
  }

  void DiffDriveController::publishWheelData(const ros::Time& time, const ros::Duration& period, Commands& curr_cmd,
          double wheel_separation, double left_wheel_radius, double right_wheel_radius)
  {
    if (publish_wheel_joint_controller_state_ && controller_state_pub_->trylock())
    {
      const double cmd_dt(period.toSec());

      // Compute desired wheels velocities, that is before applying limits:
      const double vel_left_desired  = (curr_cmd.lin - curr_cmd.ang * wheel_separation / 2.0) / left_wheel_radius;
      const double vel_right_desired = (curr_cmd.lin + curr_cmd.ang * wheel_separation / 2.0) / right_wheel_radius;
      controller_state_pub_->msg_.header.stamp = time;

      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        const double control_duration = (time - time_previous_).toSec();

        const double left_wheel_acc = (left_wheel_joints_[i].getVelocity() - vel_left_previous_[i]) / control_duration;
        const double right_wheel_acc = (right_wheel_joints_[i].getVelocity() - vel_right_previous_[i]) / control_duration;

        // Actual
        controller_state_pub_->msg_.actual.positions[i]     = left_wheel_joints_[i].getPosition();
        controller_state_pub_->msg_.actual.velocities[i]    = left_wheel_joints_[i].getVelocity();
        controller_state_pub_->msg_.actual.accelerations[i] = left_wheel_acc;
        controller_state_pub_->msg_.actual.effort[i]        = left_wheel_joints_[i].getEffort();

        controller_state_pub_->msg_.actual.positions[i + wheel_joints_size_]     = right_wheel_joints_[i].getPosition();
        controller_state_pub_->msg_.actual.velocities[i + wheel_joints_size_]    = right_wheel_joints_[i].getVelocity();
        controller_state_pub_->msg_.actual.accelerations[i + wheel_joints_size_] = right_wheel_acc;
        controller_state_pub_->msg_.actual.effort[i+ wheel_joints_size_]         = right_wheel_joints_[i].getEffort();

        // Desired
        controller_state_pub_->msg_.desired.positions[i]    += vel_left_desired * cmd_dt;
        controller_state_pub_->msg_.desired.velocities[i]    = vel_left_desired;
        controller_state_pub_->msg_.desired.accelerations[i] = (vel_left_desired - vel_left_desired_previous_) * cmd_dt;
        controller_state_pub_->msg_.desired.effort[i]        = std::numeric_limits<double>::quiet_NaN();

        controller_state_pub_->msg_.desired.positions[i + wheel_joints_size_]    += vel_right_desired * cmd_dt;
        controller_state_pub_->msg_.desired.velocities[i + wheel_joints_size_]    = vel_right_desired;
        controller_state_pub_->msg_.desired.accelerations[i + wheel_joints_size_] = (vel_right_desired - vel_right_desired_previous_) * cmd_dt;
        controller_state_pub_->msg_.desired.effort[i+ wheel_joints_size_]         = std::numeric_limits<double>::quiet_NaN();

        // Error
        controller_state_pub_->msg_.error.positions[i]     = controller_state_pub_->msg_.desired.positions[i] -
                                                                              controller_state_pub_->msg_.actual.positions[i];
        controller_state_pub_->msg_.error.velocities[i]    = controller_state_pub_->msg_.desired.velocities[i] -
                                                                              controller_state_pub_->msg_.actual.velocities[i];
        controller_state_pub_->msg_.error.accelerations[i] = controller_state_pub_->msg_.desired.accelerations[i] -
                                                                              controller_state_pub_->msg_.actual.accelerations[i];
        controller_state_pub_->msg_.error.effort[i]        = controller_state_pub_->msg_.desired.effort[i] -
                                                                              controller_state_pub_->msg_.actual.effort[i];

        controller_state_pub_->msg_.error.positions[i + wheel_joints_size_]     = controller_state_pub_->msg_.desired.positions[i + wheel_joints_size_] -
                                                                                                   controller_state_pub_->msg_.actual.positions[i + wheel_joints_size_];
        controller_state_pub_->msg_.error.velocities[i + wheel_joints_size_]    = controller_state_pub_->msg_.desired.velocities[i + wheel_joints_size_] -
                                                                                                   controller_state_pub_->msg_.actual.velocities[i + wheel_joints_size_];
        controller_state_pub_->msg_.error.accelerations[i + wheel_joints_size_] = controller_state_pub_->msg_.desired.accelerations[i + wheel_joints_size_] -
                                                                                                   controller_state_pub_->msg_.actual.accelerations[i + wheel_joints_size_];
        controller_state_pub_->msg_.error.effort[i+ wheel_joints_size_]         = controller_state_pub_->msg_.desired.effort[i + wheel_joints_size_] -
                                                                                                   controller_state_pub_->msg_.actual.effort[i + wheel_joints_size_];

        // Save previous velocities to compute acceleration
        vel_left_previous_[i] = left_wheel_joints_[i].getVelocity();
        vel_right_previous_[i] = right_wheel_joints_[i].getVelocity();
        vel_left_desired_previous_ = vel_left_desired;
        vel_right_desired_previous_ = vel_right_desired;
      }

      controller_state_pub_->unlockAndPublish();
    }
  }

} // namespace diff_drive_controller

PLUGINLIB_EXPORT_CLASS(diff_drive_controller::DiffDriveController, controller_interface::ControllerBase);

----------------------------


# LLM Response

[FILENAME: diff_drive_controller.cpp]
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 *  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Bence Magyar, Enrique Fernández
 */

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <diff_drive_controller/diff_drive_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <urdf/urdfdom_compatibility.h>
#include <urdf_parser/urdf_parser.h>

static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x-vec2.x,2) +
                   std::pow(vec1.y-vec2.y,2) +
                   std::pow(vec1.z-vec2.z,2));
}

/*
* \brief Check that a link exists and has a geometry collision.
* \param link The link
* \return true if the link has a collision element with geometry
*/
static bool hasCollisionGeometry(const urdf::LinkConstSharedPtr& link)
{
  auto logger = rclcpp::get_logger("diff_drive_controller");

  if (!link)
  {
    RCLCPP_ERROR(logger, "Link pointer is null.");
    return false;
  }

  if (!link->collision)
  {
    RCLCPP_ERROR_STREAM(logger, "Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
    return false;
  }

  if (!link->collision->geometry)
  {
    RCLCPP_ERROR_STREAM(logger, "Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
    return false;
  }
  return true;
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const urdf::LinkConstSharedPtr& link)
{
  if (!hasCollisionGeometry(link))
  {
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("diff_drive_controller"), "Link " << link->name << " does not have cylinder geometry");
    return false;
  }

  return true;
}

/*
 * \brief Check if the link is modeled as a sphere
 * \param link Link
 * \return true if the link is modeled as a Sphere; false otherwise
 */
static bool isSphere(const urdf::LinkConstSharedPtr& link)
{
  if (!hasCollisionGeometry(link))
  {
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::SPHERE)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("diff_drive_controller"), "Link " << link->name << " does not have sphere geometry");
    return false;
  }

  return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
static bool getWheelRadius(const urdf::LinkConstSharedPtr& wheel_link, double& wheel_radius)
{
  if (isCylinder(wheel_link))
  {
    wheel_radius = (static_cast<urdf::Cylinder*>(wheel_link->collision->geometry.get()))->radius;
    return true;
  }
  else if (isSphere(wheel_link))
  {
    wheel_radius = (static_cast<urdf::Sphere*>(wheel_link->collision->geometry.get()))->radius;
    return true;
  }

  RCLCPP_ERROR_STREAM(rclcpp::get_logger("diff_drive_controller"), "Wheel link " << wheel_link->name << " is NOT modeled as a cylinder or sphere!");
  return false;
}

namespace diff_drive_controller{

  DiffDriveController::DiffDriveController()
    : open_loop_(false)
    , command_struct_()
    , wheel_separation_(0.0)
    , wheel_radius_(0.0)
    , wheel_separation_multiplier_(1.0)
    , left_wheel_radius_multiplier_(1.0)
    , right_wheel_radius_multiplier_(1.0)
    , cmd_vel_timeout_(0.5)
    , allow_multiple_cmd_vel_publishers_(true)
    , base_frame_id_("base_link")
    , odom_frame_id_("odom")
    , enable_odom_tf_(true)
    , wheel_joints_size_(0)
    , publish_cmd_(false)
    , publish_wheel_joint_controller_state_(false)
  {
  }

  bool DiffDriveController::init(hardware_interface::VelocityJointInterface* hw,
            rclcpp::Node::SharedPtr root_nh,
            rclcpp::Node::SharedPtr controller_nh)
  {
    auto node = controller_nh;
    const std::string complete_ns = controller_nh->get_namespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // Get joint names from the parameter server
    std::vector<std::string> left_wheel_names, right_wheel_names;
    if (!getWheelNames(controller_nh, "left_wheel", left_wheel_names) ||
        !getWheelNames(controller_nh, "right_wheel", right_wheel_names))
    {
      return false;
    }

    if (left_wheel_names.size() != right_wheel_names.size())
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
          "#left wheels (" << left_wheel_names.size() << ") != " <<
          "#right wheels (" << right_wheel_names.size() << ").");
      return false;
    }
    else
    {
      wheel_joints_size_ = left_wheel_names.size();

      left_wheel_joints_.resize(wheel_joints_size_);
      right_wheel_joints_.resize(wheel_joints_size_);
    }

    // Odometry related:
    double publish_rate;
    controller_nh->declare_parameter<double>("publish_rate", 50.0);
    publish_rate = controller_nh->get_parameter("publish_rate").as_double();
    RCLCPP_INFO_STREAM(node->get_logger(), "Controller state will be published at "
                          << publish_rate << "Hz.");
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);

    node->declare_parameter<bool>("open_loop", false);
    node->declare_parameter<int>("velocity_rolling_window_size", 10);
    node->declare_parameter<double>("cmd_vel_timeout", 0.5);

    open_loop_ = node->get_parameter("open_loop").as_bool();
    const int velocity_rolling_window_size = node->get_parameter("velocity_rolling_window_size").as_int();
    cmd_vel_timeout_ = node->get_parameter("cmd_vel_timeout").as_double();

    odometry_.setVelocityRollingWindowSize(static_cast<size_t>(velocity_rolling_window_size));
    RCLCPP_INFO_STREAM(node->get_logger(), "Velocity commands will be considered old if they are older than "
                                           << cmd_vel_timeout_ << "s.");

    controller_nh->declare_parameter<std::string>("base_frame_id", base_frame_id_);
    base_frame_id_ = controller_nh->get_parameter("base_frame_id").as_string();
    RCLCPP_INFO_STREAM(node->get_logger(), "Base frame_id set to " << base_frame_id_);

    controller_nh->declare_parameter<std::string>("odom_frame_id", odom_frame_id_);
    odom_frame_id_ = controller_nh->get_parameter("odom_frame_id").as_string();
    RCLCPP_INFO_STREAM(node->get_logger(), "Odometry frame_id set to " << odom_frame_id_);

    controller_nh->declare_parameter<bool>("enable_odom_tf", enable_odom_tf_);
    enable_odom_tf_ = controller_nh->get_parameter("enable_odom_tf").as_bool();
    RCLCPP_INFO_STREAM(node->get_logger(), "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

    // Velocity and acceleration limits:
    controller_nh->declare_parameter<bool>("linear/x/has_velocity_limits", limiter_lin_.has_velocity_limits);
    limiter_lin_.has_velocity_limits = controller_nh->get_parameter("linear/x/has_velocity_limits").as_bool();
    controller_nh->declare_parameter<bool>("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits);
    limiter_lin_.has_acceleration_limits = controller_nh->get_parameter("linear/x/has_acceleration_limits").as_bool();
    controller_nh->declare_parameter<bool>("linear/x/has_jerk_limits", limiter_lin_.has_jerk_limits);
    limiter_lin_.has_jerk_limits = controller_nh->get_parameter("linear/x/has_jerk_limits").as_bool();
    controller_nh->declare_parameter<double>("linear/x/max_velocity", limiter_lin_.max_velocity);
    limiter_lin_.max_velocity = controller_nh->get_parameter("linear/x/max_velocity").as_double();
    controller_nh->declare_parameter<double>("linear/x/min_velocity", -limiter_lin_.max_velocity);
    limiter_lin_.min_velocity = controller_nh->get_parameter("linear/x/min_velocity").as_double();
    controller_nh->declare_parameter<double>("linear/x/max_acceleration", limiter_lin_.max_acceleration);
    limiter_lin_.max_acceleration = controller_nh->get_parameter("linear/x/max_acceleration").as_double();
    controller_nh->declare_parameter<double>("linear/x/min_acceleration", -limiter_lin_.max_acceleration);
    limiter_lin_.min_acceleration = controller_nh->get_parameter("linear/x/min_acceleration").as_double();
    controller_nh->declare_parameter<double>("linear/x/max_jerk", limiter_lin_.max_jerk);
    limiter_lin_.max_jerk = controller_nh->get_parameter("linear/x/max_jerk").as_double();
    controller_nh->declare_parameter<double>("linear/x/min_jerk", -limiter_lin_.max_jerk);
    limiter_lin_.min_jerk = controller_nh->get_parameter("linear/x/min_jerk").as_double();

    controller_nh->declare_parameter<bool>("angular/z/has_velocity_limits", limiter_ang_.has_velocity_limits);
    limiter_ang_.has_velocity_limits = controller_nh->get_parameter("angular/z/has_velocity_limits").as_bool();
    controller_nh->declare_parameter<bool>("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits);
    limiter_ang_.has_acceleration_limits = controller_nh->get_parameter("angular/z/has_acceleration_limits").as_bool();
    controller_nh->declare_parameter<bool>("angular/z/has_jerk_limits", limiter_ang_.has_jerk_limits);
    limiter_ang_.has_jerk_limits = controller_nh->get_parameter("angular/z/has_jerk_limits").as_bool();
    controller_nh->declare_parameter<double>("angular/z/max_velocity", limiter_ang_.max_velocity);
    limiter_ang_.max_velocity = controller_nh->get_parameter("angular/z/max_velocity").as_double();
    controller_nh->declare_parameter<double>("angular/z/min_velocity", -limiter_ang_.max_velocity);
    limiter_ang_.min_velocity = controller_nh->get_parameter("angular/z/min_velocity").as_double();
    controller_nh->declare_parameter<double>("angular/z/max_acceleration", limiter_ang_.max_acceleration);
    limiter_ang_.max_acceleration = controller_nh->get_parameter("angular/z/max_acceleration").as_double();
    controller_nh->declare_parameter<double>("angular/z/min_acceleration", -limiter_ang_.max_acceleration);
    limiter_ang_.min_acceleration = controller_nh->get_parameter("angular/z/min_acceleration").as_double();
    controller_nh->declare_parameter<double>("angular/z/max_jerk", limiter_ang_.max_jerk);
    limiter_ang_.max_jerk = controller_nh->get_parameter("angular/z/max_jerk").as_double();
    controller_nh->declare_parameter<double>("angular/z/min_jerk", -limiter_ang_.max_jerk);
    limiter_ang_.min_jerk = controller_nh->get_parameter("angular/z/min_jerk").as_double();

    // Publish limited velocity:
    controller_nh->declare_parameter<bool>("publish_cmd", publish_cmd_);
    publish_cmd_ = controller_nh->get_parameter("publish_cmd").as_bool();

    // Publish wheel data:
    controller_nh->declare_parameter<bool>("publish_wheel_joint_controller_state", publish_wheel_joint_controller_state_);
    publish_wheel_joint_controller_state_ = controller_nh->get_parameter("publish_wheel_joint_controller_state").as_bool();

    // If either parameter is not available, we need to look up the value in the URDF
    bool lookup_wheel_separation = !controller_nh->has_parameter("wheel_separation");
    controller_nh->declare_parameter<double>("wheel_separation", wheel_separation_);
    wheel_separation_ = controller_nh->get_parameter("wheel_separation").as_double();

    bool lookup_wheel_radius = !controller_nh->has_parameter("wheel_radius");
    controller_nh->declare_parameter<double>("wheel_radius", wheel_radius_);
    wheel_radius_ = controller_nh->get_parameter("wheel_radius").as_double();

    if (!setOdomParamsFromUrdf(root_nh,
                              left_wheel_names[0],
                              right_wheel_names[0],
                              lookup_wheel_separation,
                              lookup_wheel_radius))
    {
      return false;
    }

    // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    const double ws  = wheel_separation_multiplier_   * wheel_separation_;
    const double lwr = left_wheel_radius_multiplier_  * wheel_radius_;
    const double rwr = right_wheel_radius_multiplier_ * wheel_radius_;
    odometry_.setWheelParams(ws, lwr, rwr);
    RCLCPP_INFO_STREAM(node->get_logger(),
                          "Odometry params : wheel separation " << ws
                          << ", left wheel radius "  << lwr
                          << ", right wheel radius " << rwr);

    if (publish_cmd_)
    {
      cmd_vel_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>(controller_nh, "cmd_vel_out", 100));
    }

    // Wheel joint controller state:
    if (publish_wheel_joint_controller_state_)
    {
      controller_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>(controller_nh, "wheel_joint_controller_state", 100));

      const size_t num_wheels = wheel_joints_size_ * 2;

      controller_state_pub_->msg_.joint_names.resize(num_wheels);

      controller_state_pub_->msg_.desired.positions.resize(num_wheels);
      controller_state_pub_->msg_.desired.velocities.resize(num_wheels);
      controller_state_pub_->msg_.desired.accelerations.resize(num_wheels);
      controller_state_pub_->msg_.desired.effort.resize(num_wheels);

      controller_state_pub_->msg_.actual.positions.resize(num_wheels);
      controller_state_pub_->msg_.actual.velocities.resize(num_wheels);
      controller_state_pub_->msg_.actual.accelerations.resize(num_wheels);
      controller_state_pub_->msg_.actual.effort.resize(num_wheels);

      controller_state_pub_->msg_.error.positions.resize(num_wheels);
      controller_state_pub_->msg_.error.velocities.resize(num_wheels);
      controller_state_pub_->msg_.error.accelerations.resize(num_wheels);
      controller_state_pub_->msg_.error.effort.resize(num_wheels);

      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        controller_state_pub_->msg_.joint_names[i] = left_wheel_names[i];
        controller_state_pub_->msg_.joint_names[i + wheel_joints_size_] = right_wheel_names[i];
      }

      vel_left_previous_.resize(wheel_joints_size_, 0.0);
      vel_right_previous_.resize(wheel_joints_size_, 0.0);
    }

    setOdomPubFields(root_nh, controller_nh);

    // Get the joint object to use in the realtime loop
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      RCLCPP_INFO_STREAM(node->get_logger(),
                            "Adding left wheel with joint name: " << left_wheel_names[i]
                            << " and right wheel with joint name: " << right_wheel_names[i]);
      left_wheel_joints_[i] = hw->getHandle(left_wheel_names[i]);  // throws on failure
      right_wheel_joints_[i] = hw->getHandle(right_wheel_names[i]);  // throws on failure
    }

    sub_command_ = controller_nh->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&DiffDriveController::cmdVelCallback, this, std::placeholders::_1));

    // Initialize dynamic parameters
    DynamicParams dynamic_params;
    dynamic_params.left_wheel_radius_multiplier  = left_wheel_radius_multiplier_;
    dynamic_params.right_wheel_radius_multiplier = right_wheel_radius_multiplier_;
    dynamic_params.wheel_separation_multiplier   = wheel_separation_multiplier_;

    dynamic_params.publish_rate = publish_rate;
    dynamic_params.enable_odom_tf = enable_odom_tf_;

    dynamic_params_.writeFromNonRT(dynamic_params);

    return true;
  }

  void DiffDriveController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
  {
    // update parameter from dynamic reconf
    updateDynamicParams();

    // Apply (possibly new) multipliers:
    const double ws  = wheel_separation_multiplier_   * wheel_separation_;
    const double lwr = left_wheel_radius_multiplier_  * wheel_radius_;
    const double rwr = right_wheel_radius_multiplier_ * wheel_radius_;

    odometry_.setWheelParams(ws, lwr, rwr);

    // COMPUTE AND PUBLISH ODOMETRY
    if (open_loop_)
    {
      odometry_.updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, time);
    }
    else
    {
      double left_pos  = 0.0;
      double right_pos = 0.0;
      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        const double lp = left_wheel_joints_[i].getPosition();
        const double rp = right_wheel_joints_[i].getPosition();
        if (std::isnan(lp) || std::isnan(rp))
          return;

        left_pos  += lp;
        right_pos += rp;
      }
      left_pos  /= wheel_joints_size_;
      right_pos /= wheel_joints_size_;

      // Estimate linear and angular velocity using joint information
      odometry_.update(left_pos, right_pos, time);
    }

    // Publish odometry message
    if (last_state_publish_time_ + publish_period_ < time)
    {
      last_state_publish_time_ += publish_period_;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, odometry_.getHeading());
      const geometry_msgs::msg::Quaternion orientation = tf2::toMsg(q);

      // Populate odom message and publish
      if (odom_pub_->trylock())
      {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
        odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
        odom_pub_->msg_.pose.pose.orientation = orientation;
        odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinear();
        odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
        odom_pub_->unlockAndPublish();
      }

      // Publish tf /odom frame
      if (enable_odom_tf_ && tf_odom_pub_->trylock())
      {
        geometry_msgs::msg::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = odometry_.getX();
        odom_frame.transform.translation.y = odometry_.getY();
        odom_frame.transform.rotation = orientation;
        tf_odom_pub_->unlockAndPublish();
      }
    }

    // MOVE ROBOT
    // Retreive current velocity command and time step:
    Commands curr_cmd = *(command_.readFromRT());
    const double dt = (time - curr_cmd.stamp).seconds();

    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd.lin = 0.0;
      curr_cmd.ang = 0.0;
    }

    // Limit velocities and accelerations:
    const double cmd_dt(period.seconds());

    limiter_lin_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);
    limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd;

    // Publish limited velocity:
    if (publish_cmd_ && cmd_vel_pub_ && cmd_vel_pub_->trylock())
    {
      cmd_vel_pub_->msg_.header.stamp = time;
      cmd_vel_pub_->msg_.twist.linear.x = curr_cmd.lin;
      cmd_vel_pub_->msg_.twist.angular.z = curr_cmd.ang;
      cmd_vel_pub_->unlockAndPublish();
    }

    // Compute wheels velocities:
    const double vel_left  = (curr_cmd.lin - curr_cmd.ang * ws / 2.0)/lwr;
    const double vel_right = (curr_cmd.lin + curr_cmd.ang * ws / 2.0)/rwr;

    // Set wheels velocities:
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      left_wheel_joints_[i].setCommand(vel_left);
      right_wheel_joints_[i].setCommand(vel_right);
    }

    publishWheelData(time, period, curr_cmd, ws, lwr, rwr);
    time_previous_ = time;
  }

  void DiffDriveController::starting(const rclcpp::Time& time)
  {
    brake();

    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;
    time_previous_ = time;

    odometry_.init(time);
  }

  void DiffDriveController::stopping(const rclcpp::Time& /*time*/)
  {
    brake();
  }

  void DiffDriveController::brake()
  {
    const double vel = 0.0;
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      left_wheel_joints_[i].setCommand(vel);
      right_wheel_joints_[i].setCommand(vel);
    }
  }

  void DiffDriveController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr command)
  {
    if (isRunning())
    {
      if(!std::isfinite(command->angular.z) || !std::isfinite(command->linear.x))
      {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger(name_), *rclcpp::Clock::make_shared(), 1000, "Received NaN in velocity command. Ignoring.");
        return;
      }

      command_struct_.ang   = command->angular.z;
      command_struct_.lin   = command->linear.x;
      command_struct_.stamp = rclcpp::Clock().now();
      command_.writeFromNonRT(command_struct_);
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name_),
                             "Added values to command. "
                             << "Ang: "   << command_struct_.ang << ", "
                             << "Lin: "   << command_struct_.lin << ", "
                             << "Stamp: " << command_struct_.stamp.seconds());
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger(name_), "Can't accept new commands. Controller is not running.");
    }
  }

  bool DiffDriveController::getWheelNames(rclcpp::Node::SharedPtr controller_nh,
                              const std::string& wheel_param,
                              std::vector<std::string>& wheel_names)
  {
      if (!controller_nh->has_parameter(wheel_param))
      {
        controller_nh->declare_parameter<std::vector<std::string>>(wheel_param, std::vector<std::string>());
      }

      rclcpp::Parameter param = controller_nh->get_parameter(wheel_param);
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
      {
        wheel_names = param.as_string_array();

        if (wheel_names.empty())
        {
          RCLCPP_ERROR_STREAM(controller_nh->get_logger(),
              "Wheel param '" << wheel_param << "' is an empty list");
          return false;
        }
      }
      else if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      {
        wheel_names.push_back(param.as_string());
      }
      else
      {
        RCLCPP_ERROR_STREAM(controller_nh->get_logger(),
            "Wheel param '" << wheel_param <<
            "' is neither a list of strings nor a string.");
        return false;
      }

      return true;
  }

  bool DiffDriveController::setOdomParamsFromUrdf(rclcpp::Node::SharedPtr root_nh,
                             const std::string& left_wheel_name,
                             const std::string& right_wheel_name,
                             bool lookup_wheel_separation,
                             bool lookup_wheel_radius)
  {
    if (!(lookup_wheel_separation || lookup_wheel_radius))
    {
      // Short-circuit in case we don't need to look up anything, so we don't have to parse the URDF
      return true;
    }

    // Parse robot description
    const std::string model_param_name = "robot_description";
    std::string robot_model_str="";
    if (!root_nh->has_parameter(model_param_name))
    {
      root_nh->declare_parameter<std::string>(model_param_name, "");
    }
    robot_model_str = root_nh->get_parameter(model_param_name).as_string();
    if (robot_model_str.empty())
    {
      RCLCPP_ERROR(root_nh->get_logger(), "Robot description couldn't be retrieved from param server.");
      return false;
    }

    urdf::ModelInterfaceSharedPtr model(urdf::parseURDF(robot_model_str));

    urdf::JointConstSharedPtr left_wheel_joint(model->getJoint(left_wheel_name));
    urdf::JointConstSharedPtr right_wheel_joint(model->getJoint(right_wheel_name));

    if (!left_wheel_joint)
    {
      RCLCPP_ERROR_STREAM(root_nh->get_logger(), left_wheel_name
                             << " couldn't be retrieved from model description");
      return false;
    }

    if (!right_wheel_joint)
    {
      RCLCPP_ERROR_STREAM(root_nh->get_logger(), right_wheel_name
                             << " couldn't be retrieved from model description");
      return false;
    }

    if (lookup_wheel_separation)
    {
      // Get wheel separation
      RCLCPP_INFO_STREAM(root_nh->get_logger(), "left wheel to origin: " << left_wheel_joint->parent_to_joint_origin_transform.position.x << ","
                      << left_wheel_joint->parent_to_joint_origin_transform.position.y << ", "
                      << left_wheel_joint->parent_to_joint_origin_transform.position.z);
      RCLCPP_INFO_STREAM(root_nh->get_logger(), "right wheel to origin: " << right_wheel_joint->parent_to_joint_origin_transform.position.x << ","
                      << right_wheel_joint->parent_to_joint_origin_transform.position.y << ", "
                      << right_wheel_joint->parent_to_joint_origin_transform.position.z);

      wheel_separation_ = euclideanOfVectors(left_wheel_joint->parent_to_joint_origin_transform.position,
                                             right_wheel_joint->parent_to_joint_origin_transform.position);

    }

    if (lookup_wheel_radius)
    {
      // Get wheel radius
      if (!getWheelRadius(model->getLink(left_wheel_joint->child_link_name), wheel_radius_))
      {
        RCLCPP_ERROR_STREAM(root_nh->get_logger(), "Couldn't retrieve " << left_wheel_name << " wheel radius");
        return false;
      }
    }

    return true;
  }

  void DiffDriveController::setOdomPubFields(rclcpp::Node::SharedPtr root_nh, rclcpp::Node::SharedPtr controller_nh)
  {
    std::vector<double> pose_cov_list_default = {0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 0.03};
    std::vector<double> twist_cov_list_default = {0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 0.03};

    controller_nh->declare_parameter<std::vector<double>>("pose_covariance_diagonal", pose_cov_list_default);
    controller_nh->declare_parameter<std::vector<double>>("twist_covariance_diagonal", twist_cov_list_default);

    std::vector<double> pose_cov_list = controller_nh->get_parameter("pose_covariance_diagonal").as_double_array();
    std::vector<double> twist_cov_list = controller_nh->get_parameter("twist_covariance_diagonal").as_double_array();

    if (pose_cov_list.size() != 6 || twist_cov_list.size() != 6)
    {
      throw std::invalid_argument("diagonal size must be 6");
    }

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = odom_frame_id_;
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = {
        static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5]) };
    odom_pub_->msg_.twist.twist.linear.y  = 0;
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = {
        static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5]) };
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
  }

  void DiffDriveController::reconfCallback(DiffDriveControllerConfig& config, uint32_t /*level*/)
  {
    DynamicParams dynamic_params;
    dynamic_params.left_wheel_radius_multiplier  = config.left_wheel_radius_multiplier;
    dynamic_params.right_wheel_radius_multiplier = config.right_wheel_radius_multiplier;
    dynamic_params.wheel_separation_multiplier   = config.wheel_separation_multiplier;

    dynamic_params.publish_rate = config.publish_rate;

    dynamic_params.enable_odom_tf = config.enable_odom_tf;

    dynamic_params_.writeFromNonRT(dynamic_params);

    RCLCPP_INFO_STREAM(rclcpp::get_logger(name_), "Dynamic Reconfigure:\n" << dynamic_params);
  }

  void DiffDriveController::updateDynamicParams()
  {
    // Retreive dynamic params:
    const DynamicParams dynamic_params = *(dynamic_params_.readFromRT());

    left_wheel_radius_multiplier_  = dynamic_params.left_wheel_radius_multiplier;
    right_wheel_radius_multiplier_ = dynamic_params.right_wheel_radius_multiplier;
    wheel_separation_multiplier_   = dynamic_params.wheel_separation_multiplier;

    publish_period_ = rclcpp::Duration::from_seconds(1.0 / dynamic_params.publish_rate);
    enable_odom_tf_ = dynamic_params.enable_odom_tf;
  }

  void DiffDriveController::publishWheelData(const rclcpp::Time& time, const rclcpp::Duration& period, Commands& curr_cmd,
          double wheel_separation, double left_wheel_radius, double right_wheel_radius)
  {
    if (publish_wheel_joint_controller_state_ && controller_state_pub_->trylock())
    {
      const double cmd_dt(period.seconds());

      // Compute desired wheels velocities, that is before applying limits:
      const double vel_left_desired  = (curr_cmd.lin - curr_cmd.ang * wheel_separation / 2.0) / left_wheel_radius;
      const double vel_right_desired = (curr_cmd.lin + curr_cmd.ang * wheel_separation / 2.0) / right_wheel_radius;
      controller_state_pub_->msg_.header.stamp = time;

      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        const double control_duration = (time - time_previous_).seconds();

        const double left_wheel_acc = (left_wheel_joints_[i].getVelocity() - vel_left_previous_[i]) / control_duration;
        const double right_wheel_acc = (right_wheel_joints_[i].getVelocity() - vel_right_previous_[i]) / control_duration;

        // Actual
        controller_state_pub_->msg_.actual.positions[i]     = left_wheel_joints_[i].getPosition();
        controller_state_pub_->msg_.actual.velocities[i]    = left_wheel_joints_[i].getVelocity();
        controller_state_pub_->msg_.actual.accelerations[i] = left_wheel_acc;
        controller_state_pub_->msg_.actual.effort[i]        = left_wheel_joints_[i].getEffort();

        controller_state_pub_->msg_.actual.positions[i + wheel_joints_size_]     = right_wheel_joints_[i].getPosition();
        controller_state_pub_->msg_.actual.velocities[i + wheel_joints_size_]    = right_wheel_joints_[i].getVelocity();
        controller_state_pub_->msg_.actual.accelerations[i + wheel_joints_size_] = right_wheel_acc;
        controller_state_pub_->msg_.actual.effort[i+ wheel_joints_size_]         = right_wheel_joints_[i].getEffort();

        // Desired
        controller_state_pub_->msg_.desired.positions[i]    += vel_left_desired * cmd_dt;
        controller_state_pub_->msg_.desired.velocities[i]    = vel_left_desired;
        controller_state_pub_->msg_.desired.accelerations[i] = (vel_left_desired - vel_left_desired_previous_) * cmd_dt;
        controller_state_pub_->msg_.desired.effort[i]        = std::numeric_limits<double>::quiet_NaN();

        controller_state_pub_->msg_.desired.positions[i + wheel_joints_size_]    += vel_right_desired * cmd_dt;
        controller_state_pub_->msg_.desired.velocities[i + wheel_joints_size_]    = vel_right_desired;
        controller_state_pub_->msg_.desired.accelerations[i + wheel_joints_size_] = (vel_right_desired - vel_right_desired_previous_) * cmd_dt;
        controller_state_pub_->msg_.desired.effort[i+ wheel_joints_size_]         = std::numeric_limits<double>::quiet_NaN();

        // Error
        controller_state_pub_->msg_.error.positions[i]     = controller_state_pub_->msg_.desired.positions[i] -
                                                                              controller_state_pub_->msg_.actual.positions[i];
        controller_state_pub_->msg_.error.velocities[i]    = controller_state_pub_->msg_.desired.velocities[i] -
                                                                              controller_state_pub_->msg_.actual.velocities[i];
        controller_state_pub_->msg_.error.accelerations[i] = controller_state_pub_->msg_.desired.accelerations[i] -
                                                                              controller_state_pub_->msg_.actual.accelerations[i];
        controller_state_pub_->msg_.error.effort[i]        = controller_state_pub_->msg_.desired.effort[i] -
                                                                              controller_state_pub_->msg_.actual.effort[i];

        controller_state_pub_->msg_.error.positions[i + wheel_joints_size_]     = controller_state_pub_->msg_.desired.positions[i + wheel_joints_size_] -
                                                                                                   controller_state_pub_->msg_.actual.positions[i + wheel_joints_size_];
        controller_state_pub_->msg_.error.velocities[i + wheel_joints_size_]    = controller_state_pub_->msg_.desired.velocities[i + wheel_joints_size_] -
                                                                                                   controller_state_pub_->msg_.actual.velocities[i + wheel_joints_size_];
        controller_state_pub_->msg_.error.accelerations[i + wheel_joints_size_] = controller_state_pub_->msg_.desired.accelerations[i + wheel_joints_size_] -
                                                                                                   controller_state_pub_->msg_.actual.accelerations[i + wheel_joints_size_];
        controller_state_pub_->msg_.error.effort[i+ wheel_joints_size_]         = controller_state_pub_->msg_.desired.effort[i + wheel_joints_size_] -
                                                                                                   controller_state_pub_->msg_.actual.effort[i + wheel_joints_size_];

        // Save previous velocities to compute acceleration
        vel_left_previous_[i] = left_wheel_joints_[i].getVelocity();
        vel_right_previous_[i] = right_wheel_joints_[i].getVelocity();
        vel_left_desired_previous_ = vel_left_desired;
        vel_right_desired_previous_ = vel_right_desired;
      }

      controller_state_pub_->unlockAndPublish();
    }
  }

} // namespace diff_drive_controller

PLUGINLIB_EXPORT_CLASS(diff_drive_controller::DiffDriveController, controller_interface::ControllerBase);