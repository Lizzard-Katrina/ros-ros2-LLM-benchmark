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

FILE_PATH: generic_robot_state_node.cpp
----------------------------
/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "industrial_robot_client/robot_state_interface.h"

using industrial_robot_client::robot_state_interface::RobotStateInterface;

int main(int argc, char** argv)
{
// TODO: Initialize the Robot State Interface and start the execution loop.
// Note: The system will not provide feedback without proper initialization.
//END OF TODO
}

----------------------------

FILE_PATH: joint_trajectory_interface.cpp
----------------------------
/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include "industrial_robot_client/joint_trajectory_interface.h"
#include "simple_message/joint_traj_pt.h"
#include "industrial_utils/param_utils.h"

using namespace industrial_utils::param;
using industrial::simple_message::SimpleMessage;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;
namespace SpecialSeqValues = industrial::joint_traj_pt::SpecialSeqValues;
typedef industrial::joint_traj_pt::JointTrajPt rbt_JointTrajPt;
typedef trajectory_msgs::JointTrajectoryPoint  ros_JointTrajPt;

namespace industrial_robot_client
{
namespace joint_trajectory_interface
{

#define ROS_ERROR_RETURN(rtn,...) do {ROS_ERROR(__VA_ARGS__); return(rtn);} while(0)

bool JointTrajectoryInterface::init(std::string default_ip, int default_port)
{
  std::string ip;
  int port;

  // override IP/port with ROS params, if available
  ros::param::param<std::string>("robot_ip_address", ip, default_ip);
  ros::param::param<int>("~port", port, default_port);

  // check for valid parameter values
  if (ip.empty())
  {
    ROS_ERROR("No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
    return false;
  }
  if (port <= 0)
  {
    ROS_ERROR("No valid robot IP port found.  Please set ROS '~port' param");
    return false;
  }

  char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
  ROS_INFO("Joint Trajectory Interface connecting to IP address: '%s:%d'", ip_addr, port);
  default_tcp_connection_.init(ip_addr, port);
  free(ip_addr);

  return init(&default_tcp_connection_);
}

bool JointTrajectoryInterface::init(SmplMsgConnection* connection)
{
  std::vector<std::string> joint_names;
  if (!getJointNames("controller_joint_names", "robot_description", joint_names))
  {
    ROS_ERROR("Failed to initialize joint_names.  Aborting");
    return false;
  }

  return init(connection, joint_names);
}

bool JointTrajectoryInterface::init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                                    const std::map<std::string, double> &velocity_limits)
{
// TODO: Retrieve joint names from the parameter server and finish initialization.
  // Hint: Use the "controller_joint_names" and "robot_description" parameters.
  // END of TODO
}

JointTrajectoryInterface::~JointTrajectoryInterface()
{  
  trajectoryStop();
  this->sub_joint_trajectory_.shutdown();
}

bool JointTrajectoryInterface::jointTrajectoryCB(industrial_msgs::CmdJointTrajectory::Request &req,
                                                 industrial_msgs::CmdJointTrajectory::Response &res)
{
  trajectory_msgs::JointTrajectoryPtr traj_ptr(new trajectory_msgs::JointTrajectory);
  *traj_ptr = req.trajectory;  // copy message data
  this->jointTrajectoryCB(traj_ptr);

  // no success/fail result from jointTrajectoryCB.  Assume success.
  res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;

  return true;  // always return true.  To distinguish between call-failed and service-unavailable.
}

void JointTrajectoryInterface::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  ROS_INFO("Receiving joint trajectory message");

  // check for STOP command
  if (msg->points.empty())
  {
    ROS_INFO("Empty trajectory received, canceling current trajectory");
    trajectoryStop();
    return;
  }

  // convert trajectory into robot-format
  std::vector<JointTrajPtMessage> robot_msgs;
  if (!trajectory_to_msgs(msg, &robot_msgs))
    return;

  // send command messages to robot
  send_to_robot(robot_msgs);
}

bool JointTrajectoryInterface::trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj, std::vector<JointTrajPtMessage>* msgs)
{
  msgs->clear();

  // check for valid trajectory
  if (!is_valid(*traj))
    return false;

  for (size_t i=0; i<traj->points.size(); ++i)
  {
    ros_JointTrajPt rbt_pt, xform_pt;
    double vel, duration;

    // select / reorder joints for sending to robot
    if (!select(traj->joint_names, traj->points[i], this->all_joint_names_, &rbt_pt))
      return false;

    // transform point data (e.g. for joint-coupling)
    if (!transform(rbt_pt, &xform_pt))
      return false;

    // reduce velocity to a single scalar, for robot command
    if (!calc_speed(xform_pt, &vel, &duration))
      return false;

    JointTrajPtMessage msg = create_message(i, xform_pt.positions, vel, duration);
    msgs->push_back(msg);
  }

  return true;
}

bool JointTrajectoryInterface::select(const std::vector<std::string>& ros_joint_names, const ros_JointTrajPt& ros_pt,
                      const std::vector<std::string>& rbt_joint_names, ros_JointTrajPt* rbt_pt)
{
  ROS_ASSERT(ros_joint_names.size() == ros_pt.positions.size());

  // initialize rbt_pt
  *rbt_pt = ros_pt;
  rbt_pt->positions.clear(); rbt_pt->velocities.clear(); rbt_pt->accelerations.clear();

  for (size_t rbt_idx=0; rbt_idx < rbt_joint_names.size(); ++rbt_idx)
  {
    bool is_empty = rbt_joint_names[rbt_idx].empty();

    // find matching ROS element
    size_t ros_idx = std::find(ros_joint_names.begin(), ros_joint_names.end(), rbt_joint_names[rbt_idx]) - ros_joint_names.begin();
    bool is_found = ros_idx < ros_joint_names.size();

    // error-chk: required robot joint not found in ROS joint-list
    if (!is_empty && !is_found)
    {
      ROS_ERROR("Expected joint (%s) not found in JointTrajectory.  Aborting command.", rbt_joint_names[rbt_idx].c_str());
      return false;
    }

    if (is_empty)
    {
      if (!ros_pt.positions.empty()) rbt_pt->positions.push_back(default_joint_pos_);
      if (!ros_pt.velocities.empty()) rbt_pt->velocities.push_back(-1);
      if (!ros_pt.accelerations.empty()) rbt_pt->accelerations.push_back(-1);
    }
    else
    {
      if (!ros_pt.positions.empty()) rbt_pt->positions.push_back(ros_pt.positions[ros_idx]);
      if (!ros_pt.velocities.empty()) rbt_pt->velocities.push_back(ros_pt.velocities[ros_idx]);
      if (!ros_pt.accelerations.empty()) rbt_pt->accelerations.push_back(ros_pt.accelerations[ros_idx]);
    }
  }
  return true;
}

bool JointTrajectoryInterface::calc_speed(const trajectory_msgs::JointTrajectoryPoint& pt, double* rbt_velocity, double* rbt_duration)
{
	return calc_velocity(pt, rbt_velocity) && calc_duration(pt, rbt_duration);
}

// default velocity calculation computes the %-of-max-velocity for the "critical joint" (closest to velocity-limit)
// such that 0.2 = 20% of maximum joint speed.
//
// NOTE: this calculation uses the maximum joint speeds from the URDF file, which may differ from those defined on
// the physical robot.  These differences could lead to different actual movement velocities than intended.
// Behavior should be verified on a physical robot if movement velocity is critical.
bool JointTrajectoryInterface::calc_velocity(const trajectory_msgs::JointTrajectoryPoint& pt, double* rbt_velocity)
{
  std::vector<double> vel_ratios;

  ROS_ASSERT(all_joint_names_.size() == pt.positions.size());

  // check for empty velocities in ROS topic
  if (pt.velocities.empty())
  {
    ROS_WARN("Joint velocities unspecified.  Using default/safe speed.");
    *rbt_velocity = default_vel_ratio_;
    return true;
  }

  for (size_t i=0; i<all_joint_names_.size(); ++i)
  {
    const std::string &jnt_name = all_joint_names_[i];

    // update vel_ratios
    if (jnt_name.empty())                             // ignore "dummy joints" in velocity calcs
      vel_ratios.push_back(-1);
    else if (joint_vel_limits_.count(jnt_name) == 0)  // no velocity limit specified for this joint
      vel_ratios.push_back(-1);
    else
      vel_ratios.push_back( fabs(pt.velocities[i] / joint_vel_limits_[jnt_name]) );  // calculate expected duration for this joint
  }

  // find largest velocity-ratio (closest to max joint-speed)
  int max_idx = std::max_element(vel_ratios.begin(), vel_ratios.end()) - vel_ratios.begin();
  
  if (vel_ratios[max_idx] > 0)
    *rbt_velocity = vel_ratios[max_idx];
  else
  {
    ROS_WARN_ONCE("Joint velocity-limits unspecified.  Using default velocity-ratio.");
    *rbt_velocity = default_vel_ratio_;
  }

  if ( (*rbt_velocity < 0) || (*rbt_velocity > 1) )
  {
    ROS_WARN("computed velocity (%.1f %%) is out-of-range.  Clipping to [0-100%%]", *rbt_velocity * 100);
    *rbt_velocity = std::min(1.0, std::max(0.0, *rbt_velocity));  // clip to [0,1]
  }
  
  return true;
}

bool JointTrajectoryInterface::calc_duration(const trajectory_msgs::JointTrajectoryPoint& pt, double* rbt_duration)
{
  std::vector<double> durations;
  double this_time = pt.time_from_start.toSec();
  static double last_time = 0;

  if (this_time <= last_time)  // earlier time => new trajectory.  Move slowly to first point.
    *rbt_duration = default_duration_;
  else
    *rbt_duration = this_time - last_time;

  last_time = this_time;

  return true;
}

JointTrajPtMessage JointTrajectoryInterface::create_message(int seq, std::vector<double> joint_pos, double velocity, double duration)
{
  industrial::joint_data::JointData pos;
  ROS_ASSERT(joint_pos.size() <= (unsigned int)pos.getMaxNumJoints());

  for (size_t i=0; i<joint_pos.size(); ++i)
    pos.setJoint(i, joint_pos[i]);

  rbt_JointTrajPt pt;
  pt.init(seq, pos, velocity, duration);

  JointTrajPtMessage msg;
  msg.init(pt);

  return msg;
}

void JointTrajectoryInterface::trajectoryStop()
{
  JointTrajPtMessage jMsg;
  SimpleMessage msg, reply;

  ROS_INFO("Joint trajectory handler: entering stopping state");
  jMsg.setSequence(SpecialSeqValues::STOP_TRAJECTORY);
  jMsg.toRequest(msg);
  ROS_DEBUG("Sending stop command");
  this->connection_->sendAndReceiveMsg(msg, reply);
}

bool JointTrajectoryInterface::stopMotionCB(industrial_msgs::StopMotion::Request &req,
		                                    industrial_msgs::StopMotion::Response &res)
{
  trajectoryStop();

  // no success/fail result from trajectoryStop.  Assume success.
  res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;

  return true;  // always return true.  To distinguish between call-failed and service-unavailable.
}

bool JointTrajectoryInterface::is_valid(const trajectory_msgs::JointTrajectory &traj)
{
  for (int i=0; i<traj.points.size(); ++i)
  {
    const trajectory_msgs::JointTrajectoryPoint &pt = traj.points[i];

    // check for non-empty positions
    if (pt.positions.empty())
      ROS_ERROR_RETURN(false, "Validation failed: Missing position data for trajectory pt %d", i);

    // check for joint velocity limits
    for (int j=0; j<pt.velocities.size(); ++j)
    {
      std::map<std::string, double>::iterator max_vel = joint_vel_limits_.find(traj.joint_names[j]);
      if (max_vel == joint_vel_limits_.end()) continue;  // no velocity-checking if limit not defined

      if (std::abs(pt.velocities[j]) > max_vel->second)
        ROS_ERROR_RETURN(false, "Validation failed: Max velocity exceeded for trajectory pt %d, joint '%s'", i, traj.joint_names[j].c_str());
    }

    // check for valid timestamp
    if ((i > 0) && (pt.time_from_start.toSec() == 0))
      ROS_ERROR_RETURN(false, "Validation failed: Missing valid timestamp data for trajectory pt %d", i);
  }

  return true;
}

// copy robot JointState into local cache
void JointTrajectoryInterface::jointStateCB(const sensor_msgs::JointStateConstPtr &msg)
{
  this->cur_joint_pos_ = *msg;
}

} //joint_trajectory_interface
} //industrial_robot_client

----------------------------


# LLM Response

[FILENAME: generic_robot_state_node.cpp]
/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "industrial_robot_client/robot_state_interface.h"
#include <rclcpp/rclcpp.hpp>

using industrial_robot_client::robot_state_interface::RobotStateInterface;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotStateInterface>();
  if (!node->init())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize Robot State Interface");
    rclcpp::shutdown();
    return -1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
[FILENAME: joint_trajectory_interface.cpp]
/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include "industrial_robot_client/joint_trajectory_interface.h"
#include "simple_message/joint_traj_pt.h"
#include "industrial_utils/param_utils.h"

using namespace industrial_utils::param;
using industrial::simple_message::SimpleMessage;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;
namespace SpecialSeqValues = industrial::joint_traj_pt::SpecialSeqValues;
typedef industrial::joint_traj_pt::JointTrajPt rbt_JointTrajPt;
typedef trajectory_msgs::JointTrajectoryPoint  ros_JointTrajPt;

namespace industrial_robot_client
{
namespace joint_trajectory_interface
{

#define ROS_ERROR_RETURN(rtn,...) do {RCLCPP_ERROR(rclcpp::get_logger("joint_trajectory_interface"), __VA_ARGS__); return(rtn);} while(0)

bool JointTrajectoryInterface::init(std::string default_ip, int default_port)
{
  std::string ip;
  int port;

  // override IP/port with ROS params, if available
  this->node_->declare_parameter<std::string>("robot_ip_address", default_ip);
  this->node_->declare_parameter<int>("port", default_port);
  this->node_->get_parameter("robot_ip_address", ip);
  this->node_->get_parameter("port", port);

  // check for valid parameter values
  if (ip.empty())
  {
    RCLCPP_ERROR(this->node_->get_logger(), "No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
    return false;
  }
  if (port <= 0)
  {
    RCLCPP_ERROR(this->node_->get_logger(), "No valid robot IP port found.  Please set ROS 'port' param");
    return false;
  }

  char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
  RCLCPP_INFO(this->node_->get_logger(), "Joint Trajectory Interface connecting to IP address: '%s:%d'", ip_addr, port);
  default_tcp_connection_.init(ip_addr, port);
  free(ip_addr);

  return init(&default_tcp_connection_);
}

bool JointTrajectoryInterface::init(SmplMsgConnection* connection)
{
  std::vector<std::string> joint_names;
  if (!getJointNames("controller_joint_names", "robot_description", joint_names))
  {
    RCLCPP_ERROR(this->node_->get_logger(), "Failed to initialize joint_names.  Aborting");
    return false;
  }

  return init(connection, joint_names);
}

bool JointTrajectoryInterface::init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                                    const std::map<std::string, double> &velocity_limits)
{
  this->connection_ = connection;
  this->all_joint_names_ = joint_names;
  this->joint_vel_limits_ = velocity_limits;

  // Retrieve joint names from the parameter server and finish initialization.
  // Use the "controller_joint_names" and "robot_description" parameters.
  if (this->all_joint_names_.empty())
  {
    if (!getJointNames("controller_joint_names", "robot_description", this->all_joint_names_))
    {
      RCLCPP_ERROR(this->node_->get_logger(), "Failed to retrieve joint names from parameter server");
      return false;
    }
  }

  // If velocity limits are not provided, attempt to load them from the parameter server
  if (this->joint_vel_limits_.empty())
  {
    this->node_->declare_parameter<double>("default_vel_ratio", 0.1);
    this->node_->get_parameter("default_vel_ratio", this->default_vel_ratio_);

    this->node_->declare_parameter<double>("default_duration", 5.0);
    this->node_->get_parameter("default_duration", this->default_duration_);

    this->node_->declare_parameter<double>("default_joint_pos", 0.0);
    this->node_->get_parameter("default_joint_pos", this->default_joint_pos_);

    // Load velocity limits from URDF/parameter server
    for (const auto& joint_name : this->all_joint_names_)
    {
      if (joint_name.empty()) continue;
      double limit;
      if (this->node_->get_parameter("joint_velocity_limits." + joint_name, limit))
      {
        this->joint_vel_limits_[joint_name] = limit;
      }
    }
  }

  // Create subscribers and services
  this->sub_joint_trajectory_ = this->node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "joint_path_command", 1,
      std::bind(&JointTrajectoryInterface::jointTrajectoryCB, this, std::placeholders::_1));

  this->sub_joint_state_ = this->node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1,
      std::bind(&JointTrajectoryInterface::jointStateCB, this, std::placeholders::_1));

  this->srv_joint_trajectory_ = this->node_->create_service<industrial_msgs::srv::CmdJointTrajectory>(
      "joint_trajectory_action",
      std::bind(&JointTrajectoryInterface::jointTrajectoryCB, this, std::placeholders::_1, std::placeholders::_2));

  this->srv_stop_motion_ = this->node_->create_service<industrial_msgs::srv::StopMotion>(
      "stop_motion",
      std::bind(&JointTrajectoryInterface::stopMotionCB, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->node_->get_logger(), "Joint Trajectory Interface initialized successfully");
  return true;
}

JointTrajectoryInterface::~JointTrajectoryInterface()
{  
  trajectoryStop();
  this->sub_joint_trajectory_.reset();
}

bool JointTrajectoryInterface::jointTrajectoryCB(industrial_msgs::CmdJointTrajectory::Request &req,
                                                 industrial_msgs::CmdJointTrajectory::Response &res)
{
  trajectory_msgs::JointTrajectoryPtr traj_ptr(new trajectory_msgs::JointTrajectory);
  *traj_ptr = req.trajectory;  // copy message data
  this->jointTrajectoryCB(traj_ptr);

  // no success/fail result from jointTrajectoryCB.  Assume success.
  res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;

  return true;  // always return true.  To distinguish between call-failed and service-unavailable.
}

void JointTrajectoryInterface::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  RCLCPP_INFO(this->node_->get_logger(), "Receiving joint trajectory message");

  // check for STOP command
  if (msg->points.empty())
  {
    RCLCPP_INFO(this->node_->get_logger(), "Empty trajectory received, canceling current trajectory");
    trajectoryStop();
    return;
  }

  // convert trajectory into robot-format
  std::vector<JointTrajPtMessage> robot_msgs;
  if (!trajectory_to_msgs(msg, &robot_msgs))
    return;

  // send command messages to robot
  send_to_robot(robot_msgs);
}

bool JointTrajectoryInterface::trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj, std::vector<JointTrajPtMessage>* msgs)
{
  msgs->clear();

  // check for valid trajectory
  if (!is_valid(*traj))
    return false;

  for (size_t i=0; i<traj->points.size(); ++i)
  {
    ros_JointTrajPt rbt_pt, xform_pt;
    double vel, duration;

    // select / reorder joints for sending to robot
    if (!select(traj->joint_names, traj->points[i], this->all_joint_names_, &rbt_pt))
      return false;

    // transform point data (e.g. for joint-coupling)
    if (!transform(rbt_pt, &xform_pt))
      return false;

    // reduce velocity to a single scalar, for robot command
    if (!calc_speed(xform_pt, &vel, &duration))
      return false;

    JointTrajPtMessage msg = create_message(i, xform_pt.positions, vel, duration);
    msgs->push_back(msg);
  }

  return true;
}

bool JointTrajectoryInterface::select(const std::vector<std::string>& ros_joint_names, const ros_JointTrajPt& ros_pt,
                      const std::vector<std::string>& rbt_joint_names, ros_JointTrajPt* rbt_pt)
{
  ROS_ASSERT(ros_joint_names.size() == ros_pt.positions.size());

  // initialize rbt_pt
  *rbt_pt = ros_pt;
  rbt_pt->positions.clear(); rbt_pt->velocities.clear(); rbt_pt->accelerations.clear();

  for (size_t rbt_idx=0; rbt_idx < rbt_joint_names.size(); ++rbt_idx)
  {
    bool is_empty = rbt_joint_names[rbt_idx].empty();

    // find matching ROS element
    size_t ros_idx = std::find(ros_joint_names.begin(), ros_joint_names.end(), rbt_joint_names[rbt_idx]) - ros_joint_names.begin();
    bool is_found = ros_idx < ros_joint_names.size();

    // error-chk: required robot joint not found in ROS joint-list
    if (!is_empty && !is_found)
    {
      RCLCPP_ERROR(this->node_->get_logger(), "Expected joint (%s) not found in JointTrajectory.  Aborting command.", rbt_joint_names[rbt_idx].c_str());
      return false;
    }

    if (is_empty)
    {
      if (!ros_pt.positions.empty()) rbt_pt->positions.push_back(default_joint_pos_);
      if (!ros_pt.velocities.empty()) rbt_pt->velocities.push_back(-1);
      if (!ros_pt.accelerations.empty()) rbt_pt->accelerations.push_back(-1);
    }
    else
    {
      if (!ros_pt.positions.empty()) rbt_pt->positions.push_back(ros_pt.positions[ros_idx]);
      if (!ros_pt.velocities.empty()) rbt_pt->velocities.push_back(ros_pt.velocities[ros_idx]);
      if (!ros_pt.accelerations.empty()) rbt_pt->accelerations.push_back(ros_pt.accelerations[ros_idx]);
    }
  }
  return true;
}

bool JointTrajectoryInterface::calc_speed(const trajectory_msgs::JointTrajectoryPoint& pt, double* rbt_velocity, double* rbt_duration)
{
	return calc_velocity(pt, rbt_velocity) && calc_duration(pt, rbt_duration);
}

// default velocity calculation computes the %-of-max-velocity for the "critical joint" (closest to velocity-limit)
// such that 0.2 = 20% of maximum joint speed.
//
// NOTE: this calculation uses the maximum joint speeds from the URDF file, which may differ from those defined on
// the physical robot.  These differences could lead to different actual movement velocities than intended.
// Behavior should be verified on a physical robot if movement velocity is critical.
bool JointTrajectoryInterface::calc_velocity(const trajectory_msgs::JointTrajectoryPoint& pt, double* rbt_velocity)
{
  std::vector<double> vel_ratios;

  ROS_ASSERT(all_joint_names_.size() == pt.positions.size());

  // check for empty velocities in ROS topic
  if (pt.velocities.empty())
  {
    RCLCPP_WARN(this->node_->get_logger(), "Joint velocities unspecified.  Using default/safe speed.");
    *rbt_velocity = default_vel_ratio_;
    return true;
  }

  for (size_t i=0; i<all_joint_names_.size(); ++i)
  {
    const std::string &jnt_name = all_joint_names_[i];

    // update vel_ratios
    if (jnt_name.empty())                             // ignore "dummy joints" in velocity calcs
      vel_ratios.push_back(-1);
    else if (joint_vel_limits_.count(jnt_name) == 0)  // no velocity limit specified for this joint
      vel_ratios.push_back(-1);
    else
      vel_ratios.push_back( fabs(pt.velocities[i] / joint_vel_limits_[jnt_name]) );  // calculate expected duration for this joint
  }

  // find largest velocity-ratio (closest to max joint-speed)
  int max_idx = std::max_element(vel_ratios.begin(), vel_ratios.end()) - vel_ratios.begin();
  
  if (vel_ratios[max_idx] > 0)
    *rbt_velocity = vel_ratios[max_idx];
  else
  {
    RCLCPP_WARN_ONCE(this->node_->get_logger(), "Joint velocity-limits unspecified.  Using default velocity-ratio.");
    *rbt_velocity = default_vel_ratio_;
  }

  if ( (*rbt_velocity < 0) || (*rbt_velocity > 1) )
  {
    RCLCPP_WARN(this->node_->get_logger(), "computed velocity (%.1f %%) is out-of-range.  Clipping to [0-100%%]", *rbt_velocity * 100);
    *rbt_velocity = std::min(1.0, std::max(0.0, *rbt_velocity));  // clip to [0,1]
  }
  
  return true;
}

bool JointTrajectoryInterface::calc_duration(const trajectory_msgs::JointTrajectoryPoint& pt, double* rbt_duration)
{
  std::vector<double> durations;
  double this_time = pt.time_from_start.toSec();
  static double last_time = 0;

  if (this_time <= last_time)  // earlier time => new trajectory.  Move slowly to first point.
    *rbt_duration = default_duration_;
  else
    *rbt_duration = this_time - last_time;

  last_time = this_time;

  return true;
}

JointTrajPtMessage JointTrajectoryInterface::create_message(int seq, std::vector<double> joint_pos, double velocity, double duration)
{
  industrial::joint_data::JointData pos;
  ROS_ASSERT(joint_pos.size() <= (unsigned int)pos.getMaxNumJoints());

  for (size_t i=0; i<joint_pos.size(); ++i)
    pos.setJoint(i, joint_pos[i]);

  rbt_JointTrajPt pt;
  pt.init(seq, pos, velocity, duration);

  JointTrajPtMessage msg;
  msg.init(pt);

  return msg;
}

void JointTrajectoryInterface::trajectoryStop()
{
  JointTrajPtMessage jMsg;
  SimpleMessage msg, reply;

  RCLCPP_INFO(this->node_->get_logger(), "Joint trajectory handler: entering stopping state");
  jMsg.setSequence(SpecialSeqValues::STOP_TRAJECTORY);
  jMsg.toRequest(msg);
  RCLCPP_DEBUG(this->node_->get_logger(), "Sending stop command");
  this->connection_->sendAndReceiveMsg(msg, reply);
}

bool JointTrajectoryInterface::stopMotionCB(industrial_msgs::StopMotion::Request &req,
		                                    industrial_msgs::StopMotion::Response &res)
{
  trajectoryStop();

  // no success/fail result from trajectoryStop.  Assume success.
  res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;

  return true;  // always return true.  To distinguish between call-failed and service-unavailable.
}

bool JointTrajectoryInterface::is_valid(const trajectory_msgs::JointTrajectory &traj)
{
  for (int i=0; i<traj.points.size(); ++i)
  {
    const trajectory_msgs::JointTrajectoryPoint &pt = traj.points[i];

    // check for non-empty positions
    if (pt.positions.empty())
      ROS_ERROR_RETURN(false, "Validation failed: Missing position data for trajectory pt %d", i);

    // check for joint velocity limits
    for (int j=0; j<pt.velocities.size(); ++j)
    {
      std::map<std::string, double>::iterator max_vel = joint_vel_limits_.find(traj.joint_names[j]);
      if (max_vel == joint_vel_limits_.end()) continue;  // no velocity-checking if limit not defined

      if (std::abs(pt.velocities[j]) > max_vel->second)
        ROS_ERROR_RETURN(false, "Validation failed: Max velocity exceeded for trajectory pt %d, joint '%s'", i, traj.joint_names[j].c_str());
    }

    // check for valid timestamp
    if ((i > 0) && (pt.time_from_start.toSec() == 0))
      ROS_ERROR_RETURN(false, "Validation failed: Missing valid timestamp data for trajectory pt %d", i);
  }

  return true;
}

// copy robot JointState into local cache
void JointTrajectoryInterface::jointStateCB(const sensor_msgs::JointStateConstPtr &msg)
{
  this->cur_joint_pos_ = *msg;
}

} //joint_trajectory_interface
} //industrial_robot_client