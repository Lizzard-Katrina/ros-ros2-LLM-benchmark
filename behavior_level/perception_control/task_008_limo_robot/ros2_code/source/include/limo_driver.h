#ifndef LIMO_DRIVER_H_
#define LIMO_DRIVER_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>
#include <memory>
#include <thread>
#include <cmath>
#include <cstring>

#include "serial_port.h"

namespace AgileX {

// Frame protocol constants
constexpr uint8_t FRAME_HEADER = 0x55;
constexpr uint8_t FRAME_LENGTH = 0x0e;

// Parser states
constexpr uint8_t LIMO_WAIT_HEADER = 0;
constexpr uint8_t LIMO_WAIT_LENGTH = 1;
constexpr uint8_t LIMO_WAIT_ID_HIGH = 2;
constexpr uint8_t LIMO_WAIT_ID_LOW = 3;
constexpr uint8_t LIMO_WAIT_DATA = 4;
constexpr uint8_t LIMO_CHECK = 5;

// Message IDs
constexpr uint16_t MSG_MOTION_STATE_ID = 0x0311;
constexpr uint16_t MSG_SYSTEM_STATE_ID = 0x0321;
constexpr uint16_t MSG_ACTUATOR1_HS_STATE_ID = 0x0331;
constexpr uint16_t MSG_ACTUATOR2_HS_STATE_ID = 0x0332;
constexpr uint16_t MSG_ACTUATOR3_HS_STATE_ID = 0x0333;
constexpr uint16_t MSG_ACTUATOR4_HS_STATE_ID = 0x0334;
constexpr uint16_t MSG_ACTUATOR1_LS_STATE_ID = 0x0341;
constexpr uint16_t MSG_ACTUATOR2_LS_STATE_ID = 0x0342;
constexpr uint16_t MSG_ACTUATOR3_LS_STATE_ID = 0x0343;
constexpr uint16_t MSG_ACTUATOR4_LS_STATE_ID = 0x0344;
constexpr uint16_t MSG_ODOMETRY_ID = 0x0351;
constexpr uint16_t MSG_IMU_ACCEL_ID = 0x0361;
constexpr uint16_t MSG_IMU_GYRO_ID = 0x0362;
constexpr uint16_t MSG_IMU_EULER_ID = 0x0363;
constexpr uint16_t MSG_CTRL_MODE_CONFIG_ID = 0x0421;
constexpr uint16_t MSG_MOTION_COMMAND_ID = 0x0401;

// Motion modes
constexpr int8_t MODE_FOUR_DIFF = 0;
constexpr int8_t MODE_ACKERMANN = 1;
constexpr int8_t MODE_MCNAMU = 2;

struct LimoFrame {
    double stamp = 0.0;
    uint16_t id = 0;
    uint8_t data[8] = {0};
    uint8_t count = 0;
};

struct ImuData {
    double accel_x = 0.0;
    double accel_y = 0.0;
    double accel_z = 0.0;
    double gyro_x = 0.0;
    double gyro_y = 0.0;
    double gyro_z = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
};

class LimoDriver : public rclcpp::Node {
public:
    LimoDriver();
    ~LimoDriver();

private:
    double degToRad(double deg);
    double normalizeAngle(double angle);

    void connect(std::string dev_name, uint32_t bouadrate);
    void readData();
    void processRxData(uint8_t data);
    void parseFrame(const LimoFrame& frame);
    void processErrorCode(uint16_t error_code);

    void enableCommandedMode();
    void enableMcMode();

    void setMotionCommand(double linear_vel, double angular_vel,
                          double lateral_velocity, double steering_angle);
    void sendFrame(const LimoFrame& frame);

    void twistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void publishIMUData(double stamp);
    void publishOdometry(double stamp, double linear_velocity,
                         double angular_velocity, double lateral_velocity,
                         double steering_angle);
    void publishLimoState(double stamp, uint8_t vehicle_state, uint8_t control_mode,
                          double battery_voltage, uint16_t error_code, int8_t motion_mode);

    double convertInnerAngleToCentral(double inner_angle);
    double convertCentralAngleToInner(double central_angle);

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;

    // TF broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Serial port
    std::shared_ptr<SerialPort> port_;
    std::shared_ptr<std::thread> read_data_thread_;

    // Parameters
    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";
    bool pub_odom_tf_ = false;
    bool use_mcnamu_ = false;

    // Robot dimensions
    double wheelbase_ = 0.2;
    double track_ = 0.172;
    double max_inner_angle_ = 0.48869;  // ~28 degrees

    // Angle calibration
    double left_angle_scale_ = 1.0;
    double right_angle_scale_ = 1.0;

    // Motion mode
    int8_t motion_mode_ = MODE_FOUR_DIFF;

    // Odometry state
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;
    double last_time_ = 0.0;

    // IMU data
    ImuData imu_data_;
    double present_theta_ = 0.0;
    double last_theta_ = 0.0;
    double delta_theta_ = 0.0;
    double real_theta_ = 0.0;
};

}  // namespace AgileX

#endif  // LIMO_DRIVER_H_