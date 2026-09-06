/*
 * Copyright (c) 2021, Agilex Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "limo_driver.h"

int flag = 0;

namespace AgileX {

LimoDriver::LimoDriver() : Node("limo_node") {
    std::string port_name;
    this->declare_parameter<std::string>("port_name", "ttyTHS1");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<bool>("pub_odom_tf", false);
    this->declare_parameter<bool>("use_mcnamu", false);

    this->get_parameter("port_name", port_name);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("pub_odom_tf", pub_odom_tf_);
    this->get_parameter("use_mcnamu", use_mcnamu_);

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    motion_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 5, std::bind(&LimoDriver::twistCmdCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // connect to the serial port
    if (port_name.find("tty") != port_name.npos) {
        port_name = "/dev/" + port_name;
        connect(port_name, B460800);
        enableCommandedMode();
        if (use_mcnamu_) {
            motion_mode_ = MODE_MCNAMU;
            enableMcMode();
        }
        RCLCPP_INFO(this->get_logger(), "open the serial port: %s", port_name.c_str());
    }
}

LimoDriver::~LimoDriver() {
}

double LimoDriver::degToRad(double deg) {
    return deg / 180.0 * M_PI;
}

double LimoDriver::normalizeAngle(double angle) {
    while (angle >= M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle <= -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

void LimoDriver::connect(std::string dev_name, uint32_t bouadrate) {
    port_ = std::shared_ptr<SerialPort>(new SerialPort(dev_name, bouadrate));
    if (port_->openPort() == 0) {
        read_data_thread_ = std::shared_ptr<std::thread>(
            new std::thread(std::bind(&LimoDriver::readData, this)));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s", port_->getDevPath().c_str());
        port_->closePort();
        exit(-1);
    }
}

void LimoDriver::readData() {
    uint8_t rx_data = 0;
    while (rclcpp::ok()) {
        auto len = port_->readByte(&rx_data);
        if (len < 1)
            continue;
        processRxData(rx_data);
    }
}

void LimoDriver::processRxData(uint8_t data) {
    static LimoFrame frame;
    static int data_num = 0;
    static uint8_t checksum = 0;
    static uint8_t state = LIMO_WAIT_HEADER;

    switch (state) {
        case LIMO_WAIT_HEADER: {
            if (data == FRAME_HEADER) {
                frame.stamp = this->now().seconds();
                state = LIMO_WAIT_LENGTH;
            }
            break;
        }
        case LIMO_WAIT_LENGTH: {
            if (data == FRAME_LENGTH) {
                state = LIMO_WAIT_ID_HIGH;
            } else {
                state = LIMO_WAIT_HEADER;
            }
            break;
        }
        case LIMO_WAIT_ID_HIGH: {
            frame.id = static_cast<uint16_t>(data) << 8;
            state = LIMO_WAIT_ID_LOW;
            break;
        }
        case LIMO_WAIT_ID_LOW: {
            frame.id |= static_cast<uint16_t>(data);
            state = LIMO_WAIT_DATA;
            data_num = 0;
            break;
        }
        case LIMO_WAIT_DATA: {
            if (data_num < 8) {
                frame.data[data_num++] = data;
                checksum += data;
            } else {
                frame.count = data;
                state = LIMO_CHECK;
                data_num = 0;
            }
            break;
        }
        case LIMO_CHECK: {
            if (data == checksum) {
                parseFrame(frame);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Invalid frame! Check sum failed!");
            }

            state = LIMO_WAIT_HEADER;
            checksum = 0;
            memset(&frame.data[0], 0, 8);
            break;
        }
        default:
            break;
    }
}

void LimoDriver::parseFrame(const LimoFrame& frame) {
    switch (frame.id) {
        case MSG_MOTION_STATE_ID: {
            double linear_velocity = static_cast<int16_t>((frame.data[1] & 0xff) | (frame.data[0] << 8)) / 1000.0;
            double angular_velocity = static_cast<int16_t>((frame.data[3] & 0xff) | (frame.data[2] << 8)) / 1000.0;
            double lateral_velocity = static_cast<int16_t>((frame.data[5] & 0xff) | (frame.data[4] << 8)) / 1000.0;
            double steering_angle = static_cast<int16_t>((frame.data[7] & 0xff) | (frame.data[6] << 8)) / 1000.0;
            if (steering_angle > 0) {
                steering_angle *= left_angle_scale_;
            } else {
                steering_angle *= right_angle_scale_;
            }
            publishOdometry(frame.stamp, linear_velocity, angular_velocity,
                            lateral_velocity, steering_angle);
            break;
        }
        case MSG_SYSTEM_STATE_ID: {
            uint8_t vehicle_state = frame.data[0];
            uint8_t control_mode = frame.data[1];
            double battervyoltage = ((frame.data[3] & 0xff) | (frame.data[2] << 8)) * 0.1;
            uint16_t error_code = ((frame.data[5] & 0xff) | (frame.data[4] << 8));

            if (!use_mcnamu_) {
                motion_mode_ = frame.data[6];
            }

            processErrorCode(error_code);
            publishLimoState(frame.stamp, vehicle_state, control_mode,
                             battervyoltage, error_code, motion_mode_);
            break;
        }
        case MSG_ACTUATOR1_HS_STATE_ID:
        case MSG_ACTUATOR2_HS_STATE_ID:
        case MSG_ACTUATOR3_HS_STATE_ID:
        case MSG_ACTUATOR4_HS_STATE_ID:
        case MSG_ACTUATOR1_LS_STATE_ID:
        case MSG_ACTUATOR2_LS_STATE_ID:
        case MSG_ACTUATOR3_LS_STATE_ID:
        case MSG_ACTUATOR4_LS_STATE_ID:
            break;
        case MSG_ODOMETRY_ID: {
            int32_t left_wheel_odom = (frame.data[3] & 0xff) | (frame.data[2] << 8) |
                                      (frame.data[1] << 16) | (frame.data[0] << 24);
            int32_t right_wheel_odom = (frame.data[7] & 0xff) | (frame.data[6] << 8) |
                                       (frame.data[5] << 16) | (frame.data[4] << 24);
            (void)left_wheel_odom;
            (void)right_wheel_odom;
            break;
        }
        case MSG_IMU_ACCEL_ID: {
            imu_data_.accel_x = static_cast<int16_t>((frame.data[1] & 0xff) | (frame.data[0] << 8)) / 100.0;
            imu_data_.accel_y = static_cast<int16_t>((frame.data[3] & 0xff) | (frame.data[2] << 8)) / 100.0;
            imu_data_.accel_z = static_cast<int16_t>((frame.data[5] & 0xff) | (frame.data[4] << 8)) / 100.0;
            break;
        }
        case MSG_IMU_GYRO_ID: {
            imu_data_.gyro_x = degToRad(static_cast<int16_t>((frame.data[1] & 0xff) |
                                        (frame.data[0] << 8)) / 100.0);
            imu_data_.gyro_y = degToRad(static_cast<int16_t>((frame.data[3] & 0xff) |
                                        (frame.data[2] << 8)) / 100.0);
            imu_data_.gyro_z = degToRad(static_cast<int16_t>((frame.data[5] & 0xff) |
                                        (frame.data[4] << 8)) / 100.0);
            break;
        }
        case MSG_IMU_EULER_ID: {
            imu_data_.yaw = static_cast<int16_t>((frame.data[1] & 0xff) | (frame.data[0] << 8)) / 100.0;
            imu_data_.pitch = static_cast<int16_t>((frame.data[3] & 0xff) | (frame.data[2] << 8)) / 100.0;
            imu_data_.roll = static_cast<int16_t>((frame.data[5] & 0xff) | (frame.data[4] << 8)) / 100.0;
            publishIMUData(frame.stamp);
            break;
        }
        default:
            break;
    }
}

void LimoDriver::processErrorCode(uint16_t error_code) {
    if (error_code & 0x0001) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "LIMO: Low battery!");
    }
    if (error_code & 0x0002) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "LIMO: Low battery!");
    }
    if (error_code & 0x0004) {
        RCLCPP_WARN(this->get_logger(), "LIMO: Remote control lost connect!");
    }
    if (error_code & 0x0008) {
        RCLCPP_ERROR(this->get_logger(), "LIMO: Motor driver 1 error!");
    }
    if (error_code & 0x0010) {
        RCLCPP_ERROR(this->get_logger(), "LIMO: Motor driver 2 error!");
    }
    if (error_code & 0x0020) {
        RCLCPP_ERROR(this->get_logger(), "LIMO: Motor driver 3 error!");
    }
    if (error_code & 0x0040) {
        RCLCPP_ERROR(this->get_logger(), "LIMO: Motor driver 4 error!");
    }
    if (error_code & 0x0100) {
        RCLCPP_ERROR(this->get_logger(), "LIMO: Drive status error!");
    }
}

void LimoDriver::enableCommandedMode() {
    LimoFrame frame;
    frame.id = MSG_CTRL_MODE_CONFIG_ID;
    frame.data[0] = 0x01;
    frame.data[1] = 0;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;

    sendFrame(frame);
}

void LimoDriver::enableMcMode() {
    LimoFrame frame;
    frame.id = MSG_CTRL_MODE_CONFIG_ID;
    frame.data[0] = 0x01;
    frame.data[1] = 0;
    frame.data[2] = 0x01;
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;

    sendFrame(frame);
    RCLCPP_INFO(this->get_logger(), "use_mcnamu");
}

void LimoDriver::setMotionCommand(double linear_vel, double angular_vel,
                                  double lateral_velocity, double steering_angle) {
    LimoFrame frame;
    frame.id = MSG_MOTION_COMMAND_ID;
    int16_t linear_cmd = linear_vel * 1000;
    int16_t angular_cmd = angular_vel * 1000;
    int16_t lateral_cmd = lateral_velocity * 1000;
    int16_t steering_cmd = steering_angle * 1000;

    frame.data[0] = static_cast<uint8_t>(linear_cmd >> 8);
    frame.data[1] = static_cast<uint8_t>(linear_cmd & 0x00ff);
    frame.data[2] = static_cast<uint8_t>(angular_cmd >> 8);
    frame.data[3] = static_cast<uint8_t>(angular_cmd & 0x00ff);
    frame.data[4] = static_cast<uint8_t>(lateral_cmd >> 8);
    frame.data[5] = static_cast<uint8_t>(lateral_cmd & 0x00ff);
    frame.data[6] = static_cast<uint8_t>(steering_cmd >> 8);
    frame.data[7] = static_cast<uint8_t>(steering_cmd & 0x00ff);
    sendFrame(frame);
}

void LimoDriver::sendFrame(const LimoFrame& frame) {
    uint32_t checksum = 0;
    uint8_t frame_len = 0x0e;
    uint8_t data[14] = {0x55, frame_len};

    data[2] = static_cast<uint8_t>(frame.id >> 8);
    data[3] = static_cast<uint8_t>(frame.id & 0xff);
    for (size_t i = 0; i < 8; ++i) {
        data[i + 4] = frame.data[i];
        checksum += frame.data[i];
    }
    data[frame_len - 1] = static_cast<uint8_t>(checksum & 0xff);

    port_->writeData(data, frame_len);
}

void LimoDriver::twistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;
    double lateral_vel = msg->linear.y;

    switch (motion_mode_) {
        case MODE_ACKERMANN: {
            if (std::fabs(angular_vel) < 0.001) {
                setMotionCommand(linear_vel, 0.0, 0.0, 0.0);
            } else {
                double r = linear_vel / angular_vel;
                // Ackermann: central_angle = atan(wheelbase_ / r)
                double central_angle = std::atan(wheelbase_ / r);
                double inner_angle = convertCentralAngleToInner(central_angle);
                if (inner_angle > max_inner_angle_) {
                    inner_angle = max_inner_angle_;
                }
                if (inner_angle < -max_inner_angle_) {
                    inner_angle = -max_inner_angle_;
                }
                inner_angle = normalizeAngle(inner_angle);
                double cmd_central_angle = convertInnerAngleToCentral(inner_angle);
                setMotionCommand(linear_vel, 0.0, 0.0, cmd_central_angle);
            }
            break;
        }
        case MODE_MCNAMU: {
            setMotionCommand(linear_vel, angular_vel, lateral_vel, 0.0);
            break;
        }
        case MODE_FOUR_DIFF:
        default: {
            setMotionCommand(linear_vel, angular_vel, 0.0, 0.0);
            break;
        }
    }
}

void LimoDriver::publishIMUData(double stamp) {
    sensor_msgs::msg::Imu imu_msg;

    int32_t sec = static_cast<int32_t>(stamp);
    uint32_t nsec = static_cast<uint32_t>((stamp - sec) * 1e9);
    imu_msg.header.stamp = rclcpp::Time(sec, nsec);
    imu_msg.header.frame_id = "imu_link";

    imu_msg.linear_acceleration.x = imu_data_.accel_x;
    imu_msg.linear_acceleration.y = imu_data_.accel_y;
    imu_msg.linear_acceleration.z = imu_data_.accel_z;

    imu_msg.angular_velocity.x = imu_data_.gyro_x;
    imu_msg.angular_velocity.y = imu_data_.gyro_y;
    imu_msg.angular_velocity.z = imu_data_.gyro_z;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, degToRad(imu_data_.yaw));

    if (flag == 0) {
        present_theta_ = last_theta_ = imu_data_.yaw;
        flag = 1;
    }

    present_theta_ = imu_data_.yaw;
    delta_theta_ = present_theta_ - last_theta_;
    if (delta_theta_ < 0.1 && delta_theta_ > -0.1) delta_theta_ = 0;
    real_theta_ = real_theta_ + delta_theta_;
    last_theta_ = present_theta_;

    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();

    imu_msg.linear_acceleration_covariance[0] = 1.0f;
    imu_msg.linear_acceleration_covariance[4] = 1.0f;
    imu_msg.linear_acceleration_covariance[8] = 1.0f;

    imu_msg.angular_velocity_covariance[0] = 1e-6;
    imu_msg.angular_velocity_covariance[4] = 1e-6;
    imu_msg.angular_velocity_covariance[8] = 1e-6;

    imu_msg.orientation_covariance[0] = 1e-6;
    imu_msg.orientation_covariance[4] = 1e-6;
    imu_msg.orientation_covariance[8] = 1e-6;

    imu_publisher_->publish(imu_msg);
}

void LimoDriver::publishOdometry(double stamp, double linear_velocity,
                                 double angular_velocity, double lateral_velocity,
                                 double steering_angle) {
    double current_time = stamp;
    if (last_time_ == 0.0) {
        last_time_ = current_time;
        return;
    }
    double dt = current_time - last_time_;
    last_time_ = current_time;

    if (dt <= 0.0 || dt > 1.0) {
        return;
    }

    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;

    if (motion_mode_ == MODE_ACKERMANN) {
        vx = linear_velocity;
        vy = 0.0;
        if (std::fabs(steering_angle) > 0.001) {
            wz = linear_velocity / wheelbase_ * std::tan(steering_angle);
        } else {
            wz = 0.0;
        }
    } else if (motion_mode_ == MODE_MCNAMU) {
        vx = linear_velocity;
        vy = lateral_velocity;
        wz = angular_velocity;
    } else {
        vx = linear_velocity;
        vy = 0.0;
        wz = angular_velocity;
    }

    // project vx, vy into global frame and integrate with dt
    position_x_ += (vx * cos(theta_) - vy * sin(theta_)) * dt;
    position_y_ += (vx * sin(theta_) + vy * cos(theta_)) * dt;
    theta_ += wz * dt;
    theta_ = normalizeAngle(theta_);

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, theta_);

    if (pub_odom_tf_) {
        geometry_msgs::msg::TransformStamped odom_tf;
        int32_t sec = static_cast<int32_t>(current_time);
        uint32_t nsec = static_cast<uint32_t>((current_time - sec) * 1e9);
        odom_tf.header.stamp = rclcpp::Time(sec, nsec);
        odom_tf.header.frame_id = odom_frame_;
        odom_tf.child_frame_id = base_frame_;
        odom_tf.transform.translation.x = position_x_;
        odom_tf.transform.translation.y = position_y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation.x = odom_quat.x();
        odom_tf.transform.rotation.y = odom_quat.y();
        odom_tf.transform.rotation.z = odom_quat.z();
        odom_tf.transform.rotation.w = odom_quat.w();
        tf_broadcaster_->sendTransform(odom_tf);
    }

    nav_msgs::msg::Odometry odom_msg;
    int32_t sec = static_cast<int32_t>(current_time);
    uint32_t nsec = static_cast<uint32_t>((current_time - sec) * 1e9);
    odom_msg.header.stamp = rclcpp::Time(sec, nsec);
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = odom_quat.x();
    odom_msg.pose.pose.orientation.y = odom_quat.y();
    odom_msg.pose.pose.orientation.z = odom_quat.z();
    odom_msg.pose.pose.orientation.w = odom_quat.w();

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = wz;

    odom_publisher_->publish(odom_msg);
}

void LimoDriver::publishLimoState(double stamp, uint8_t vehicle_state, uint8_t control_mode,
                                  double battery_voltage, uint16_t error_code, int8_t motion_mode) {
    (void)stamp;
    (void)vehicle_state;
    (void)control_mode;
    (void)battery_voltage;
    (void)error_code;
    (void)motion_mode;
}

double LimoDriver::convertInnerAngleToCentral(double inner_angle) {
    double r = wheelbase_ / std::tan(fabs(inner_angle)) + track_ / 2;
    double central_angle = std::atan(wheelbase_ / r);

    if (inner_angle < 0) {
        central_angle = -central_angle;
    }

    return central_angle;
}

double LimoDriver::convertCentralAngleToInner(double central_angle) {
    double inner_angle = std::atan(2 * wheelbase_ * std::sin(fabs(central_angle)) /
                                   (2 * wheelbase_ * std::cos(fabs(central_angle)) -
                                    track_ * std::sin(fabs(central_angle))));

    if (central_angle < 0) {
        inner_angle = -inner_angle;
    }

    return inner_angle;
}

}  // namespace AgileX