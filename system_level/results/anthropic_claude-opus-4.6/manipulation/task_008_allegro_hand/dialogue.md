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

FILE_PATH: AllegroHandDrv.cpp
----------------------------
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Wonik Robotics.
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
 *   * Neither the name of Wonik Robotics nor the names of its
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
 */

/*
 *  @file AllegroHandDrv.cpp
 *  @brief Allegro Hand Driver
 *
 *  Created on:         Nov 15, 2012
 *  Added to Project:   Jan 17, 2013
 *  Author:             Sean Yi, K.C.Chang, Seungsu Kim, & Alex Alspach
 *  Maintained by:      Sean Yi(seanyi@wonikrobotics.com)
 */

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string>
#include "ros/ros.h"
#include "candrv/candrv.h"
#include "allegro_hand_driver/AllegroHandDrv.h"

using namespace std;

#define MAX_DOF 16

#define PWM_LIMIT_ROLL 250.0*1.5
#define PWM_LIMIT_NEAR 450.0*1.5
#define PWM_LIMIT_MIDDLE 300.0*1.5
#define PWM_LIMIT_FAR 190.0*1.5

#define PWM_LIMIT_THUMB_ROLL 350.0*1.5
#define PWM_LIMIT_THUMB_NEAR 270.0*1.5
#define PWM_LIMIT_THUMB_MIDDLE 180.0*1.5
#define PWM_LIMIT_THUMB_FAR 180.0*1.5

#define PWM_LIMIT_GLOBAL_8V 800.0 // maximum: 1200
#define PWM_LIMIT_GLOBAL_24V 500.0
#define PWM_LIMIT_GLOBAL_12V 1200.0

#define allegro_V4 0

namespace allegro
{

AllegroHandDrv::AllegroHandDrv()
    : //_can_handle(0),
     _curr_position_get(0)
    , _emergency_stop(false)
{
    ROS_INFO("AllegroHandDrv instance is constructed.");
}

AllegroHandDrv::~AllegroHandDrv()
{
    if (_can_handle != 0) {
        ROS_INFO("CAN: System Off");
        CANAPI::command_set_period(_can_handle, 0);
        usleep(10000);
        ROS_INFO("CAN: Close CAN channel");
        CANAPI::command_can_close(_can_handle);
    }
}

// trim from end. see http://stackoverflow.com/a/217605/256798
static inline std::string &rtrim(std::string &s)
{
    s.erase(std::find_if(
        s.rbegin(), s.rend(),
        std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

bool AllegroHandDrv::init(int mode)
{
    string CAN_CH;
    ros::param::get("~comm/CAN_CH", CAN_CH);
    rtrim(CAN_CH);  // Ensure the ROS parameter has no trailing whitespace.

    if (CAN_CH.empty()) {
        ROS_ERROR("Invalid (empty) CAN channel, cannot proceed. Check PCAN comms.");
        return false;
    }

    if (CANAPI::command_can_open_with_name(_can_handle, CAN_CH.c_str())) {
        _can_handle = 0;
        return false;
    }

    ROS_INFO("CAN: Flush CAN receive buffer");
    CANAPI::command_can_flush(_can_handle);
    usleep(100);

    //ROS_INFO("CAN: System Off");
    CANAPI::command_servo_off(_can_handle);
    usleep(100);

    ROS_INFO("CAN: Request Hand Information");
    CANAPI::request_hand_information(_can_handle);
    usleep(100);

    ROS_INFO("CAN: Request Hand Serial");
    CANAPI::request_hand_serial(_can_handle);
    usleep(100);

    ROS_INFO("CAN: Setting loop period(:= 2ms) and initialize system");
    short comm_period[3] = {2, 0, 0}; // millisecond {position, imu, temperature}
    CANAPI::command_set_period(_can_handle, comm_period);

    ROS_INFO("CAN: System ON");
    CANAPI::command_servo_on(_can_handle);
    usleep(100);


    ROS_INFO("CAN: Communicating");

    return true;
}

int AllegroHandDrv::readCANFrames()
{
    if (_emergency_stop)
        return -1;

    _readDevices();
    //usleep(10);

    return 0;
}

int AllegroHandDrv::writeJointTorque()
{
    _writeDevices();

    if (_emergency_stop) {
        ROS_ERROR("Emergency stop in writeJointTorque()");
        return -1;
    }

    return 0;
}

bool AllegroHandDrv::isJointInfoReady()
{
    return (_curr_position_get == (0x01 | 0x02 | 0x04 | 0x08));
}

void AllegroHandDrv::resetJointInfoReady()
{
    _curr_position_get = 0;
}

void AllegroHandDrv::setTorque(double *torque)
{
    if (_hand_version == 1.0) {
        // for Allegro Hand v1.0
        for (int findex = 0; findex < 4; findex++) {
            _desired_torque[4*findex+0] = torque[4*findex+0];
            _desired_torque[4*findex+1] = torque[4*findex+1];
            _desired_torque[4*findex+2] = torque[4*findex+2];
            _desired_torque[4*findex+3] = torque[4*findex+3];
        }
    }
    else if (_hand_version >= 2.0) {
        // for Allegro Hand v2.0
#if allegro_V4
        for (int findex = 0; findex < 4; findex++) {
            _desired_torque[4*findex+0] = torque[4*findex+0];
            _desired_torque[4*findex+1] = torque[4*findex+1];
            _desired_torque[4*findex+2] = torque[4*findex+2];
            _desired_torque[4*findex+3] = torque[4*findex+3];
        }
#else
        for (int findex = 0; findex < 4; findex++) {
            _desired_torque[4*findex+0] = torque[4*findex+0]*1.43*1000;
            _desired_torque[4*findex+1] = torque[4*findex+1]*1.43*1000;
            _desired_torque[4*findex+2] = torque[4*findex+2]*1.43*1000;
            _desired_torque[4*findex+3] = torque[4*findex+3]*1.43*1000;

        }

#endif
    }
    else {
        ROS_ERROR("CAN: Can not determine proper finger CAN channels. Check the Allegro Hand version number in 'zero.yaml'");
        return;
    }
}

void AllegroHandDrv::getJointInfo(double *position)
{
    for (int i = 0; i < DOF_JOINTS; i++) {
        position[i] = _curr_position[i];
        
    }
}

void AllegroHandDrv::_readDevices()
{
    int err;
    int id;    
    int len;
    unsigned char data[8];

    err = CANAPI::can_read_message(_can_handle, &id, &len, data, FALSE, 0);
    while (!err) {
        _parseMessage(id, len, data);
        err = CANAPI::can_read_message(_can_handle, &id, &len, data, FALSE, 0);
    }
    //ROS_ERROR("can_read_message returns %d.", err); // PCAN_ERROR_QRCVEMPTY(32) from Peak CAN means "Receive queue is empty". It is not an error.
}

void AllegroHandDrv::_writeDevices()
{
// TODO [Task 2]: Implement torque-to-PWM conversion and safety saturation.
// 1. For non-V4 hands, implement a hard limit: if torque exceeds +/- 240.0, clamp it.
// 2. For Type-A hands, apply a 0.5x scaling factor to specific joints (index 1, 5, 9) 
//    as per hardware sensitivity requirements.
// 3. Convert the processed double 'pwmDouble' to short 'pwm' and send via CANAPI.
//ENDOF TODO
}

void AllegroHandDrv::_parseMessage(int id, int len, unsigned char* data)
{
    int tmppos[4];
    int lIndexBase;
    int i;

// TODO [Task 1]: Implement the raw CAN data unpacking logic for finger joint positions.
// 1. Calculate finger index: 'int findex = (id & 0x00000007);'.
// 2. Assembly 16-bit data: Use 'data[i] | (data[i+1] << 8)' format for consistency.
// 3. Apply scaling: For non-V4, multiply by (M_PI / 180.0) * 0.088.
// 4. Update bitmask: Use the hexadecimal format '_curr_position_get |= (0x01 << findex);'.
// END OF TODO
}

} // namespace allegro

----------------------------

FILE_PATH: allegro_node.cpp
----------------------------
// Common allegro node code used by any node. Each node that implements an
// AllegroNode must define the computeDesiredTorque() method.
//
// Editor: Hibo (sh-yang @ wonik.com)

#include "allegro_node.h"

std::string jointNames[DOF_JOINTS] =
        {
                "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",
                "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",
                "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",
                "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0",
        };

std::vector<std::string> frame_ids = {"link_3_0_tip", "link_7_0_tip", "link_11_0_tip", "link_15_0_tip"};
std::vector<std::string> namespaces = {"fingertip_[0]", "fingertip_[1]", "fingertip_[2]", "fingertip_[3]"};
std::vector<visualization_msgs::Marker> markers(4);

AllegroNode::AllegroNode(bool sim /* = false */) {
  mutex = new boost::mutex();
  
  // Create arrays 16 long for each of the four joint state components
  current_joint_state.position.resize(DOF_JOINTS);
  current_joint_state.velocity.resize(DOF_JOINTS);
  current_joint_state.effort.resize(DOF_JOINTS);
  current_joint_state.name.resize(DOF_JOINTS);

  // Initialize values: joint names should match URDF, desired torque and
  // velocity are both zero.
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.name[i] = jointNames[i];
    desired_torque[i] = 0.0;
    current_velocity[i] = 0.0;
    current_position_filtered[i] = 0.0;
    current_velocity_filtered[i] = 0.0;
  }

  
  // Get Allegro Hand information from parameter server
  // This information is found in the Hand-specific "zero.yaml" file from the allegro_hand_description package
  std::string robot_name, manufacturer, origin, serial;
  double version;
  ros::param::get("~hand_info/robot_name", robot_name);
  ros::param::get("~hand_info/which_hand", whichHand);
  ros::param::get("~hand_info/which_type", whichType);
  ros::param::get("~hand_info/manufacturer", manufacturer);
  ros::param::get("~hand_info/origin", origin);
  ros::param::get("~hand_info/serial", serial);
  ros::param::get("~hand_info/version", version);

  // Initialize CAN device
  canDevice = 0;
  if(!sim) {
    canDevice = new allegro::AllegroHandDrv();
    if (canDevice->init()) {
        usleep(3000);
    }
    else {
        delete canDevice;
        canDevice = 0;
    }
  }

  // Start ROS time
  tstart = ros::Time::now();
  
  // Advertise current joint state publisher and subscribe to desired joint
  // states.
  joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
  joint_cmd_sub = nh.subscribe(DESIRED_STATE_TOPIC, 1, // queue size
                                &AllegroNode::desiredStateCallback, this);
 
  time_sub = nh.subscribe("timechange", 10, &AllegroNode::ControltimeCallback,this);
  force_sub = nh.subscribe("forcechange", 10, &AllegroNode::GraspforceCallback,this);
 
  marker_pub = nh.advertise<visualization_msgs::Marker>("fingertip_arrow_markers", 1);
}

AllegroNode::~AllegroNode() {
  if (canDevice) delete canDevice;
  delete mutex;
  nh.shutdown();
}

// Get Allegro Hand desired joint position
void AllegroNode::desiredStateCallback(const sensor_msgs::JointState &msg) {
  mutex->lock();
  desired_joint_state = msg;
  mutex->unlock();
}

// Get Allegro Hand motion control time from gui
void AllegroNode::ControltimeCallback(const std_msgs::Float32::ConstPtr& msg)
{
    motion_time = msg->data;
    pBHand->SetMotiontime(motion_time);
}

// Get Allegro Hand grasping force from gui
void AllegroNode::GraspforceCallback(const std_msgs::Float32::ConstPtr& msg)
{
    force_get = msg->data;
}

// Main publisher
void AllegroNode::publishData() {
  // current position, velocity and effort (torque), fingertip_sensor (Rviz) published
  current_joint_state.header.stamp = tnow;
  
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.position[i] = current_position[i]; /// current_position_filtered[i];
    current_joint_state.velocity[i] = current_velocity[i]; /// current_velocity_filtered[i];
    current_joint_state.effort[i] = desired_torque[i];


  }
  joint_state_pub.publish(current_joint_state);

  // fingertip_sensor_Rviz
  for (const auto& marker : markers) {
    marker_pub.publish(marker);
    ros::Duration(1e-6).sleep();
    }

}


void AllegroNode::Rviz_Arrow(){ 
  //  This is the function to visualize fingertip sensors on Rviz.

    tf2::Quaternion orientation;
    orientation.setRPY(0, -M_PI/4, 0); 

  for (int i = 0; i < 4; ++i) {
    markers[i].header.frame_id = frame_ids[i];
    markers[i].ns = namespaces[i];
    markers[i].type = visualization_msgs::Marker::ARROW;
    markers[i].action = visualization_msgs::Marker::ADD;
    markers[i].pose.orientation = tf2::toMsg(orientation);
    markers[i].scale.x = fingertip_sensor[i] / 5000.0;
    markers[i].scale.y = 0.005;
    markers[i].scale.z = 0.005;
    markers[i].color.r = 1.0;
    markers[i].color.a = 1.0;
  }
}

void AllegroNode::updateController() {

// TODO [Task 3]: Synchronize CAN device data with ROS joint states.
// 1. Check if 'canDevice->readCANFrames()' is negative; if so, call 'rclcpp::shutdown()'.
// 2. If 'canDevice->isJointInfoReady()', backup 'current_position' to 'previous_position'.
// 3. Compute velocity: '(current_position[i] - previous_position[i]) / dt'.
// 4. MANDATORY: Call 'canDevice->resetJointInfoReady()' at the end of the update block.
//END OF TODO
}

// Interrupt-based control is not recommended by Wonik Robotics. I have not tested it.
void AllegroNode::timerCallback(const ros::TimerEvent &event) {
  updateController();
}

ros::Timer AllegroNode::startTimerCallback() {
  ros::Timer timer = nh.createTimer(ros::Duration(0.001),
                                    &AllegroNode::timerCallback, this);
  return timer;
}

----------------------------


# LLM Response



[FILENAME: AllegroHandDrv.cpp]
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Wonik Robotics.
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
 *   * Neither the name of Wonik Robotics nor the names of its
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
 */

/*
 *  @file AllegroHandDrv.cpp
 *  @brief Allegro Hand Driver
 *
 *  Created on:         Nov 15, 2012
 *  Added to Project:   Jan 17, 2013
 *  Author:             Sean Yi, K.C.Chang, Seungsu Kim, & Alex Alspach
 *  Maintained by:      Sean Yi(seanyi@wonikrobotics.com)
 */

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "candrv/candrv.h"
#include "allegro_hand_driver/AllegroHandDrv.h"

using namespace std;

#define MAX_DOF 16

#define PWM_LIMIT_ROLL 250.0*1.5
#define PWM_LIMIT_NEAR 450.0*1.5
#define PWM_LIMIT_MIDDLE 300.0*1.5
#define PWM_LIMIT_FAR 190.0*1.5

#define PWM_LIMIT_THUMB_ROLL 350.0*1.5
#define PWM_LIMIT_THUMB_NEAR 270.0*1.5
#define PWM_LIMIT_THUMB_MIDDLE 180.0*1.5
#define PWM_LIMIT_THUMB_FAR 180.0*1.5

#define PWM_LIMIT_GLOBAL_8V 800.0 // maximum: 1200
#define PWM_LIMIT_GLOBAL_24V 500.0
#define PWM_LIMIT_GLOBAL_12V 1200.0

#define allegro_V4 0

namespace allegro
{

AllegroHandDrv::AllegroHandDrv()
    : //_can_handle(0),
     _curr_position_get(0)
    , _emergency_stop(false)
{
    RCLCPP_INFO(rclcpp::get_logger("AllegroHandDrv"), "AllegroHandDrv instance is constructed.");
}

AllegroHandDrv::~AllegroHandDrv()
{
    if (_can_handle != 0) {
        RCLCPP_INFO(rclcpp::get_logger("AllegroHandDrv"), "CAN: System Off");
        CANAPI::command_set_period(_can_handle, 0);
        usleep(10000);
        RCLCPP_INFO(rclcpp::get_logger("AllegroHandDrv"), "CAN: Close CAN channel");
        CANAPI::command_can_close(_can_handle);
    }
}

// trim from end. see http://stackoverflow.com/a/217605/256798
static inline std::string &rtrim(std::string &s)
{
    s.erase(std::find_if(
        s.rbegin(), s.rend(),
        [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
    return s;
}

bool AllegroHandDrv::init(int mode)
{
    string CAN_CH;

    auto node = rclcpp::Node::make_shared("allegro_hand_drv_temp_param_node");
    node->declare_parameter<std::string>("comm.CAN_CH", "");
    node->get_parameter("comm.CAN_CH", CAN_CH);
    rtrim(CAN_CH);  // Ensure the ROS parameter has no trailing whitespace.

    if (CAN_CH.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("AllegroHandDrv"), "Invalid (empty) CAN channel, cannot proceed. Check PCAN comms.");
        return false;
    }

    if (CANAPI::command_can_open_with_name(_can_handle, CAN_CH.c_str())) {
        _can_handle = 0;
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("AllegroHandDrv"), "CAN: Flush CAN receive buffer");
    CANAPI::command_can_flush(_can_handle);
    usleep(100);

    //RCLCPP_INFO("CAN: System Off");
    CANAPI::command_servo_off(_can_handle);
    usleep(100);

    RCLCPP_INFO(rclcpp::get_logger("AllegroHandDrv"), "CAN: Request Hand Information");
    CANAPI::request_hand_information(_can_handle);
    usleep(100);

    RCLCPP_INFO(rclcpp::get_logger("AllegroHandDrv"), "CAN: Request Hand Serial");
    CANAPI::request_hand_serial(_can_handle);
    usleep(100);

    RCLCPP_INFO(rclcpp::get_logger("AllegroHandDrv"), "CAN: Setting loop period(:= 2ms) and initialize system");
    short comm_period[3] = {2, 0, 0}; // millisecond {position, imu, temperature}
    CANAPI::command_set_period(_can_handle, comm_period);

    RCLCPP_INFO(rclcpp::get_logger("AllegroHandDrv"), "CAN: System ON");
    CANAPI::command_servo_on(_can_handle);
    usleep(100);


    RCLCPP_INFO(rclcpp::get_logger("AllegroHandDrv"), "CAN: Communicating");

    return true;
}

int AllegroHandDrv::readCANFrames()
{
    if (_emergency_stop)
        return -1;

    _readDevices();
    //usleep(10);

    return 0;
}

int AllegroHandDrv::writeJointTorque()
{
    _writeDevices();

    if (_emergency_stop) {
        RCLCPP_ERROR(rclcpp::get_logger("AllegroHandDrv"), "Emergency stop in writeJointTorque()");
        return -1;
    }

    return 0;
}

bool AllegroHandDrv::isJointInfoReady()
{
    return (_curr_position_get == (0x01 | 0x02 | 0x04 | 0x08));
}

void AllegroHandDrv::resetJointInfoReady()
{
    _curr_position_get = 0;
}

void AllegroHandDrv::setTorque(double *torque)
{
    if (_hand_version == 1.0) {
        // for Allegro Hand v1.0
        for (int findex = 0; findex < 4; findex++) {
            _desired_torque[4*findex+0] = torque[4*findex+0];
            _desired_torque[4*findex+1] = torque[4*findex+1];
            _desired_torque[4*findex+2] = torque[4*findex+2];
            _desired_torque[4*findex+3] = torque[4*findex+3];
        }
    }
    else if (_hand_version >= 2.0) {
        // for Allegro Hand v2.0
#if allegro_V4
        for (int findex = 0; findex < 4; findex++) {
            _desired_torque[4*findex+0] = torque[4*findex+0];
            _desired_torque[4*findex+1] = torque[4*findex+1];
            _desired_torque[4*findex+2] = torque[4*findex+2];
            _desired_torque[4*findex+3] = torque[4*findex+3];
        }
#else
        for (int findex = 0; findex < 4; findex++) {
            _desired_torque[4*findex+0] = torque[4*findex+0]*1.43*1000;
            _desired_torque[4*findex+1] = torque[4*findex+1]*1.43*1000;
            _desired_torque[4*findex+2] = torque[4*findex+2]*1.43*1000;
            _desired_torque[4*findex+3] = torque[4*findex+3]*1.43*1000;

        }

#endif
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("AllegroHandDrv"), "CAN: Can not determine proper finger CAN channels. Check the Allegro Hand version number in 'zero.yaml'");
        return;
    }
}

void AllegroHandDrv::getJointInfo(double *position)
{
    for (int i = 0; i < DOF_JOINTS; i++) {
        position[i] = _curr_position[i];
        
    }
}

void AllegroHandDrv::_readDevices()
{
    int err;
    int id;    
    int len;
    unsigned char data[8];

    err = CANAPI::can_read_message(_can_handle, &id, &len, data, FALSE, 0);
    while (!err) {
        _parseMessage(id, len, data);
        err = CANAPI::can_read_message(_can_handle, &id, &len, data, FALSE, 0);
    }
    //RCLCPP_ERROR("can_read_message returns %d.", err); // PCAN_ERROR_QRCVEMPTY(32) from Peak CAN means "Receive queue is empty". It is not an error.
}

void AllegroHandDrv::_writeDevices()
{
    short pwm[MAX_DOF];
    double pwmDouble[MAX_DOF];

    for (int i = 0; i < MAX_DOF; i++) {
        pwmDouble[i] = _desired_torque[i];

#if !allegro_V4
        // Hard limit: clamp torque to +/- 240.0
        if (pwmDouble[i] > 240.0) {
            pwmDouble[i] = 240.0;
        } else if (pwmDouble[i] < -240.0) {
            pwmDouble[i] = -240.0;
        }
#endif
    }

    // For Type-A hands, apply a 0.5x scaling factor to specific joints
    // (index 1, 5, 9) as per hardware sensitivity requirements.
    if (whichType == "A" || whichType == "a") {
        pwmDouble[1] *= 0.5;
        pwmDouble[5] *= 0.5;
        pwmDouble[9] *= 0.5;
    }

    // Convert processed double to short and send via CANAPI
    for (int i = 0; i < MAX_DOF; i++) {
        pwm[i] = (short)pwmDouble[i];
    }

    CANAPI::command_set_torque(_can_handle, pwm);
}

void AllegroHandDrv::_parseMessage(int id, int len, unsigned char* data)
{
    int tmppos[4];
    int lIndexBase;
    int i;

    int findex = (id & 0x00000007);

    lIndexBase = findex * 4;

    // Assembly 16-bit data from two bytes: data[i] | (data[i+1] << 8)
    tmppos[0] = (short)(data[0] | (data[1] << 8));
    tmppos[1] = (short)(data[2] | (data[3] << 8));
    tmppos[2] = (short)(data[4] | (data[5] << 8));
    tmppos[3] = (short)(data[6] | (data[7] << 8));

    for (i = 0; i < 4; i++) {
#if allegro_V4
        _curr_position[lIndexBase + i] = (double)tmppos[i];
#else
        // Apply scaling: multiply by (M_PI / 180.0) * 0.088
        _curr_position[lIndexBase + i] = (double)tmppos[i] * (M_PI / 180.0) * 0.088;
#endif
    }

    // Update bitmask using hexadecimal format
    _curr_position_get |= (0x01 << findex);
}

} // namespace allegro

[FILENAME: allegro_node.cpp]
// Common allegro node code used by any node. Each node that implements an
// AllegroNode must define the computeDesiredTorque() method.
//
// Editor: Hibo (sh-yang @ wonik.com)

#include "allegro_node.h"

std::string jointNames[DOF_JOINTS] =
        {
                "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",
                "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",
                "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",
                "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0",
        };

std::vector<std::string> frame_ids = {"link_3_0_tip", "link_7_0_tip", "link_11_0_tip", "link_15_0_tip"};
std::vector<std::string> namespaces = {"fingertip_[0]", "fingertip_[1]", "fingertip_[2]", "fingertip_[3]"};
std::vector<visualization_msgs::msg::Marker> markers(4);

AllegroNode::AllegroNode(bool sim /* = false */)
  : Node("allegro_node") {
  mutex = new std::mutex();
  
  // Create arrays 16 long for each of the four joint state components
  current_joint_state.position.resize(DOF_JOINTS);
  current_joint_state.velocity.resize(DOF_JOINTS);
  current_joint_state.effort.resize(DOF_JOINTS);
  current_joint_state.name.resize(DOF_JOINTS);

  // Initialize values: joint names should match URDF, desired torque and
  // velocity are both zero.
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.name[i] = jointNames[i];
    desired_torque[i] = 0.0;
    current_velocity[i] = 0.0;
    current_position_filtered[i] = 0.0;
    current_velocity_filtered[i] = 0.0;
  }

  
  // Get Allegro Hand information from parameter server
  // This information is found in the Hand-specific "zero.yaml" file from the allegro_hand_description package
  std::string robot_name, manufacturer, origin, serial;
  double version;
  this->declare_parameter<std::string>("hand_info.robot_name", "");
  this->declare_parameter<std::string>("hand_info.which_hand", "");
  this->declare_parameter<std::string>("hand_info.which_type", "");
  this->declare_parameter<std::string>("hand_info.manufacturer", "");
  this->declare_parameter<std::string>("hand_info.origin", "");
  this->declare_parameter<std::string>("hand_info.serial", "");
  this->declare_parameter<double>("hand_info.version", 0.0);

  this->get_parameter("hand_info.robot_name", robot_name);
  this->get_parameter("hand_info.which_hand", whichHand);
  this->get_parameter("hand_info.which_type", whichType);
  this->get_parameter("hand_info.manufacturer", manufacturer);
  this->get_parameter("hand_info.origin", origin);
  this->get_parameter("hand_info.serial", serial);
  this->get_parameter("hand_info.version", version);

  // Initialize CAN device
  canDevice = 0;
  if(!sim) {
    canDevice = new allegro::AllegroHandDrv();
    if (canDevice->init()) {
        usleep(3000);
    }
    else {
        delete canDevice;
        canDevice = 0;
    }
  }

  // Start ROS time
  tstart = this->now();
  
  // Advertise current joint state publisher and subscribe to desired joint
  // states.
  joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(JOINT_STATE_TOPIC, 3);
  joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      DESIRED_STATE_TOPIC, 1,
      std::bind(&AllegroNode::desiredStateCallback, this, std::placeholders::_1));
 
  time_sub = this->create_subscription<std_msgs::msg::Float32>(
      "timechange", 10,
      std::bind(&AllegroNode::ControltimeCallback, this, std::placeholders::_1));
  force_sub = this->create_subscription<std_msgs::msg::Float32>(
      "forcechange", 10,
      std::bind(&AllegroNode::GraspforceCallback, this, std::placeholders::_1));
 
  marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("fingertip_arrow_markers", 1);
}

AllegroNode::~AllegroNode() {
  if (canDevice) delete canDevice;
  delete mutex;
}

// Get Allegro Hand desired joint position
void AllegroNode::desiredStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  mutex->lock();
  desired_joint_state = *msg;
  mutex->unlock();
}

// Get Allegro Hand motion control time from gui
void AllegroNode::ControltimeCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    motion_time = msg->data;
    pBHand->SetMotiontime(motion_time);
}

// Get Allegro Hand grasping force from gui
void AllegroNode::GraspforceCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    force_get = msg->data;
}

// Main publisher
void AllegroNode::publishData() {
  // current position, velocity and effort (torque), fingertip_sensor (Rviz) published
  current_joint_state.header.stamp = tnow;
  
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.position[i] = current_position[i]; /// current_position_filtered[i];
    current_joint_state.velocity[i] = current_velocity[i]; /// current_velocity_filtered[i];
    current_joint_state.effort[i] = desired_torque[i];


  }
  joint_state_pub->publish(current_joint_state);

  // fingertip_sensor_Rviz
  for (const auto& marker : markers) {
    marker_pub->publish(marker);
    rclcpp::sleep_for(std::chrono::microseconds(1));
    }

}


void AllegroNode::Rviz_Arrow(){ 
  //  This is the function to visualize fingertip sensors on Rviz.

    tf2::Quaternion orientation;
    orientation.setRPY(0, -M_PI/4, 0); 

  for (int i = 0; i < 4; ++i) {
    markers[i].header.frame_id = frame_ids[i];
    markers[i].ns = namespaces[i];
    markers[i].type = visualization_msgs::msg::Marker::ARROW;
    markers[i].action = visualization_msgs::msg::Marker::ADD;
    markers[i].pose.orientation = tf2::toMsg(orientation);
    markers[i].scale.x = fingertip_sensor[i] / 5000.0;
    markers[i].scale.y = 0.005;
    markers[i].scale.z = 0.005;
    markers[i].color.r = 1.0;
    markers[i].color.a = 1.0;
  }
}

void AllegroNode::updateController() {

  if (canDevice) {
    // Read CAN frames; if negative, shutdown
    if (canDevice->readCANFrames() < 0) {
      rclcpp::shutdown();
      return;
    }

    // If joint info is ready, update positions and compute velocities
    if (canDevice->isJointInfoReady()) {
      // Backup current_position to previous_position
      for (int i = 0; i < DOF_JOINTS; i++) {
        previous_position[i] = current_position[i];
      }

      // Get new joint positions from CAN device
      canDevice->getJointInfo(current_position);

      // Compute dt
      tnow = this->now();
      double dt = (tnow - tstart).seconds();
      tstart = tnow;

      // Compute velocity: (current_position[i] - previous_position[i]) / dt
      if (dt > 0.0) {
        for (int i = 0; i < DOF_JOINTS; i++) {
          current_velocity[i] = (current_position[i] - previous_position[i]) / dt;
        }
      }

      // Compute desired torque (pure virtual, implemented by derived classes)
      computeDesiredTorque();

      // Set torque to CAN device and write
      canDevice->setTorque(desired_torque);
      canDevice->writeJointTorque();

      // Publish data
      publishData();

      // Reset joint info ready flag
      canDevice->resetJointInfoReady();
    }
  }
}

// Timer-based control callback
void AllegroNode::timerCallback() {
  updateController();
}

rclcpp::TimerBase::SharedPtr AllegroNode::startTimerCallback() {
  auto timer = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&AllegroNode::timerCallback, this));
  return timer;
}