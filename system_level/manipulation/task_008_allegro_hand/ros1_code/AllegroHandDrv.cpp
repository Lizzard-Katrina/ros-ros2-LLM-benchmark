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
