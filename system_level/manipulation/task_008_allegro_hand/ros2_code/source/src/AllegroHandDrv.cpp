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
 *  @brief Allegro Hand Driver (ROS2 port)
 */

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
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

#define PWM_LIMIT_GLOBAL_8V 800.0
#define PWM_LIMIT_GLOBAL_24V 500.0
#define PWM_LIMIT_GLOBAL_12V 1200.0

#define allegro_V4 0

namespace allegro
{

AllegroHandDrv::AllegroHandDrv()
    : HAND_TYPE_A(false)
    , RIGHT_HAND(true)
    , _hand_version(2.0)
    , _tau_cov_const(1.0)
    , _input_voltage(12.0)
    , _curr_position_get(0)
    , _pwm_max_global(PWM_LIMIT_GLOBAL_12V)
    , _emergency_stop(false)
    , _sim_mode(false)
{
    memset(_curr_position, 0, sizeof(_curr_position));
    memset(_curr_torque, 0, sizeof(_curr_torque));
    memset(_desired_position, 0, sizeof(_desired_position));
    memset(_desired_torque, 0, sizeof(_desired_torque));
    memset(_pwm_max, 0, sizeof(_pwm_max));
    memset(_encoder_offset, 0, sizeof(_encoder_offset));
    memset(_encoder_direction, 0, sizeof(_encoder_direction));
    memset(_motor_direction, 0, sizeof(_motor_direction));

    // Set per-joint PWM limits
    for (int f = 0; f < 3; f++) {
        _pwm_max[4*f+0] = PWM_LIMIT_ROLL;
        _pwm_max[4*f+1] = PWM_LIMIT_NEAR;
        _pwm_max[4*f+2] = PWM_LIMIT_MIDDLE;
        _pwm_max[4*f+3] = PWM_LIMIT_FAR;
    }
    _pwm_max[12] = PWM_LIMIT_THUMB_ROLL;
    _pwm_max[13] = PWM_LIMIT_THUMB_NEAR;
    _pwm_max[14] = PWM_LIMIT_THUMB_MIDDLE;
    _pwm_max[15] = PWM_LIMIT_THUMB_FAR;

    RCLCPP_INFO(rclcpp::get_logger("AllegroHandDrv"), "AllegroHandDrv instance is constructed.");
}

AllegroHandDrv::~AllegroHandDrv()
{
    RCLCPP_INFO(rclcpp::get_logger("AllegroHandDrv"), "AllegroHandDrv instance is destructed.");
}

bool AllegroHandDrv::init(int mode)
{
    (void)mode;
    RCLCPP_INFO(rclcpp::get_logger("AllegroHandDrv"), "AllegroHandDrv initialized (sim mode).");
    return true;
}

void AllegroHandDrv::setSimMode(bool sim)
{
    _sim_mode = sim;
}

int AllegroHandDrv::readCANFrames()
{
    if (_emergency_stop)
        return -1;

    _readDevices();

    // In sim mode, always mark all joints as ready
    if (_sim_mode) {
        _curr_position_get = 0x01 | 0x02 | 0x04 | 0x08;
    }

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
        for (int findex = 0; findex < 4; findex++) {
            _desired_torque[4*findex+0] = torque[4*findex+0];
            _desired_torque[4*findex+1] = torque[4*findex+1];
            _desired_torque[4*findex+2] = torque[4*findex+2];
            _desired_torque[4*findex+3] = torque[4*findex+3];
        }
    }
    else if (_hand_version >= 2.0) {
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
        RCLCPP_ERROR(rclcpp::get_logger("AllegroHandDrv"),
            "CAN: Can not determine proper finger CAN channels.");
        return;
    }
}

void AllegroHandDrv::getJointInfo(double *position)
{
    for (int i = 0; i < DOF_JOINTS; i++) {
        position[i] = _curr_position[i];
    }
}

void AllegroHandDrv::injectJointPositions(double *positions)
{
    for (int i = 0; i < DOF_JOINTS; i++) {
        _curr_position[i] = positions[i];
    }
}

void AllegroHandDrv::setAllJointsReady()
{
    _curr_position_get = 0x01 | 0x02 | 0x04 | 0x08;
}

void AllegroHandDrv::_readDevices()
{
    // In simulation mode, nothing to read from CAN bus
}

void AllegroHandDrv::_writeDevices()
{
    // [Task 2]: Implement torque-to-PWM conversion and safety saturation.
    short pwm[DOF_JOINTS];

    for (int i = 0; i < DOF_JOINTS; i++) {
        double pwmDouble = _desired_torque[i];

#if !allegro_V4
        // 1. Hard limit: clamp torque to +/- 240.0
        if (pwmDouble > 240.0) pwmDouble = 240.0;
        if (pwmDouble < -240.0) pwmDouble = -240.0;
#endif

        // 2. For Type-A hands, apply 0.5x scaling to joints 1, 5, 9
        if (HAND_TYPE_A) {
            if (i == 1 || i == 5 || i == 9) {
                pwmDouble *= 0.5;
            }
        }

        // Also check RIGHT_HAND for motor direction if needed
        if (RIGHT_HAND) {
            // Standard direction for right hand
        }

        // 3. Convert to short PWM
        pwm[i] = (short)pwmDouble;
    }

    // In simulation mode, we don't actually send via CANAPI
    (void)pwm;
}

void AllegroHandDrv::_parseMessage(int id, int len, unsigned char* data)
{
    (void)len;
    int tmppos[4];
    int lIndexBase;
    int i;

    // [Task 1]: Implement the raw CAN data unpacking logic for finger joint positions.

    // 1. Calculate finger index from CAN ID
    int findex = (id & 0x00000007);

    lIndexBase = findex * 4;

    // 2. Assemble 16-bit data from two 8-bit bytes: data[i] | (data[i+1] << 8)
    for (i = 0; i < 4; i++) {
        tmppos[i] = (short)(data[i*2] | (data[i*2+1] << 8));
    }

    // 3. Apply scaling: For non-V4, multiply by (M_PI / 180.0) * 0.088
#if allegro_V4
    for (i = 0; i < 4; i++) {
        _curr_position[lIndexBase + i] = (double)tmppos[i];
    }
#else
    for (i = 0; i < 4; i++) {
        _curr_position[lIndexBase + i] = (double)tmppos[i] * (M_PI / 180.0) * 0.088;
    }
#endif

    // 4. Update bitmask using hexadecimal format
    _curr_position_get |= (0x01 << findex);
}

} // namespace allegro