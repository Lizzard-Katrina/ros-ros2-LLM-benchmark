#ifndef RSM_CORE_ROBOT_CONTROL_MUX_H
#define RSM_CORE_ROBOT_CONTROL_MUX_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <task_008a_robot_statemachine_scaffold/msg/operation_mode.hpp>
#include <task_008a_robot_statemachine_scaffold/srv/set_operation_mode.hpp>
#include <chrono>
#include <cmath>
#include <string>

namespace rsm {

constexpr double MOVE_THRESH = 0.1;

using OperationMode = task_008a_robot_statemachine_scaffold::msg::OperationMode;
using SetOperationMode = task_008a_robot_statemachine_scaffold::srv::SetOperationMode;

class RobotControlMux : public rclcpp::Node {
public:
    RobotControlMux();
    ~RobotControlMux();

    void publishTopics();

private:
    void publishCmdVel();
    void publishOperationMode();

    void setOperationMode(
        const std::shared_ptr<SetOperationMode::Request> request,
        std::shared_ptr<SetOperationMode::Response> response);

    void autonomyCmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr cmd_vel);

    void teleoperationCmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr cmd_vel);

    void joystickCallback(
        const sensor_msgs::msg::Joy::SharedPtr joy);

    void teleoperationIdleTimerCallback();

    bool checkJoystickCommand(
        const sensor_msgs::msg::Joy::SharedPtr joy);

    // Parameters
    std::string _autonomy_operation_cmd_vel_topic;
    std::string _teleoperation_cmd_vel_topic;
    std::string _cmd_vel_topic;
    std::string _joystick_topic;
    double _teleoperation_idle_timer_duration;
    bool _joystick_used;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;
    rclcpp::Publisher<OperationMode>::SharedPtr _operation_mode_pub;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _autonomy_cmd_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _teleoperation_cmd_vel_sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joystick_sub;

    // Service
    rclcpp::Service<SetOperationMode>::SharedPtr _set_operation_mode_service;

    // Timer
    rclcpp::TimerBase::SharedPtr _teleoperation_idle_timer;
    bool _timer_running;

    // State
    bool _emergency_stop_active;
    uint8_t _operation_mode;
    geometry_msgs::msg::Twist _autonomy_cmd_vel;
    geometry_msgs::msg::Twist _teleoperation_cmd_vel;
    sensor_msgs::msg::Joy _joystick_cmd;
};

} /* namespace rsm */

#endif