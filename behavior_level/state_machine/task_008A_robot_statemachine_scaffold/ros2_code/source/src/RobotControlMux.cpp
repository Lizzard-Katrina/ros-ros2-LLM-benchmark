#include <rsm_core/RobotControlMux.h>

namespace rsm {

RobotControlMux::RobotControlMux()
: rclcpp::Node("robot_control_mux")
{
    // TODO 1: Infrastructure Setup.
    // 1. Declare and retrieve all parameters.
    this->declare_parameter<std::string>("autonomy_cmd_vel_topic", "autonomy/cmd_vel");
    this->declare_parameter<std::string>("teleoperation_cmd_vel_topic", "teleoperation/cmd_vel");
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    this->declare_parameter<std::string>("joystick_topic", "joy");
    this->declare_parameter<double>("teleoperation_idle_timer", 0.5);
    this->declare_parameter<bool>("joystick_used", false);

    this->get_parameter("autonomy_cmd_vel_topic", _autonomy_operation_cmd_vel_topic);
    this->get_parameter("teleoperation_cmd_vel_topic", _teleoperation_cmd_vel_topic);
    this->get_parameter("cmd_vel_topic", _cmd_vel_topic);
    this->get_parameter("joystick_topic", _joystick_topic);
    this->get_parameter("teleoperation_idle_timer", _teleoperation_idle_timer_duration);
    this->get_parameter("joystick_used", _joystick_used);

    // 2. Setup Pub/Sub with a reliable QoS profile (depth 10).
    rclcpp::QoS qos_profile(10);
    qos_profile.reliable();

    _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
        _cmd_vel_topic, qos_profile);
    _operation_mode_pub = this->create_publisher<OperationMode>(
        "operationMode", qos_profile);

    _autonomy_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        _autonomy_operation_cmd_vel_topic, qos_profile,
        std::bind(&RobotControlMux::autonomyCmdVelCallback, this, std::placeholders::_1));

    _teleoperation_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        _teleoperation_cmd_vel_topic, qos_profile,
        std::bind(&RobotControlMux::teleoperationCmdVelCallback, this, std::placeholders::_1));

    _joystick_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        _joystick_topic, qos_profile,
        std::bind(&RobotControlMux::joystickCallback, this, std::placeholders::_1));

    // 3. Initialize the 'setOperationMode' service.
    _set_operation_mode_service = this->create_service<SetOperationMode>(
        "setOperationMode",
        std::bind(&RobotControlMux::setOperationMode, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Create the idle timer (initially not running)
    auto timer_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(_teleoperation_idle_timer_duration));
    _teleoperation_idle_timer = this->create_wall_timer(
        timer_duration,
        std::bind(&RobotControlMux::teleoperationIdleTimerCallback, this));
    _teleoperation_idle_timer->cancel();
    _timer_running = false;

    //END OF TODO
    _emergency_stop_active = false;
    _operation_mode = OperationMode::STOPPED;
}

RobotControlMux::~RobotControlMux() {
}

void RobotControlMux::publishTopics() {
    publishCmdVel();
    publishOperationMode();
}

void RobotControlMux::publishCmdVel() {
    geometry_msgs::msg::Twist cmd_vel;
    if (!_emergency_stop_active) {
        if (_operation_mode == OperationMode::AUTONOMOUS) {
            cmd_vel = _autonomy_cmd_vel;
        } else if (_operation_mode == OperationMode::TELEOPERATION) {
            cmd_vel = _teleoperation_cmd_vel;
        }
    }
    _cmd_vel_pub->publish(cmd_vel);
}

void RobotControlMux::publishOperationMode() {
    OperationMode msg;
    msg.emergency_stop = _emergency_stop_active;
    msg.mode = _operation_mode;
    _operation_mode_pub->publish(msg);
}

void RobotControlMux::setOperationMode(
    const std::shared_ptr<SetOperationMode::Request> request,
    std::shared_ptr<SetOperationMode::Response> response)
{
// TODO 2: Implement the service logic.
    // Update internal states based on the 'request' and fill the 'response'.
    _emergency_stop_active = request->operation_mode.emergency_stop;
    _operation_mode = request->operation_mode.mode;
    response->success = true;
    response->message = "Operation mode set";
//END OF TODO
}

void RobotControlMux::autonomyCmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr cmd_vel) {
    _autonomy_cmd_vel = *cmd_vel;
}

void RobotControlMux::teleoperationCmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr cmd_vel) {
    _teleoperation_cmd_vel = *cmd_vel;
    if (!_emergency_stop_active) {
        if (_teleoperation_cmd_vel.linear.x != 0.0
                || _teleoperation_cmd_vel.linear.y != 0.0
                || _teleoperation_cmd_vel.linear.z != 0.0
                || _teleoperation_cmd_vel.angular.x != 0.0
                || _teleoperation_cmd_vel.angular.y != 0.0
                || _teleoperation_cmd_vel.angular.z != 0.0) {
            _operation_mode = OperationMode::TELEOPERATION;
            _teleoperation_idle_timer->cancel();
            _timer_running = false;
        }
        if (_operation_mode == OperationMode::TELEOPERATION) {
            _teleoperation_idle_timer->reset();
            _timer_running = true;
        }
    }
}

void RobotControlMux::joystickCallback(
        const sensor_msgs::msg::Joy::SharedPtr joy) {
    if (!_emergency_stop_active) {
        if (checkJoystickCommand(joy)) {
            _operation_mode = OperationMode::TELEOPERATION;
            _teleoperation_idle_timer->cancel();
            _timer_running = false;
        }
        if (_operation_mode == OperationMode::TELEOPERATION) {
            _teleoperation_idle_timer->reset();
            _timer_running = true;
        }
    }
}

void RobotControlMux::teleoperationIdleTimerCallback() {
    _operation_mode = OperationMode::STOPPED;
    geometry_msgs::msg::Twist empty_cmd_vel;
    _teleoperation_cmd_vel = empty_cmd_vel;
    _teleoperation_idle_timer->cancel();
    _timer_running = false;
}

bool RobotControlMux::checkJoystickCommand(
        const sensor_msgs::msg::Joy::SharedPtr joy) {
    bool movingCommand = false;
    if (_joystick_cmd.axes.size() == joy->axes.size()) {
        for (unsigned int i = 0; i < joy->axes.size(); i++) {
            if ((std::abs(joy->axes[i]) - std::abs(_joystick_cmd.axes[i]))
                    > MOVE_THRESH) {
                movingCommand = true;
            }
        }
    }
    if (_joystick_cmd.buttons.size() == joy->buttons.size()) {
        for (unsigned int i = 0; i < joy->buttons.size(); i++) {
            if (joy->buttons[i] && !_joystick_cmd.buttons[i]) {
                movingCommand = true;
            }
        }
    }
    _joystick_cmd = *joy;
    return movingCommand;
}

} /* namespace rsm */