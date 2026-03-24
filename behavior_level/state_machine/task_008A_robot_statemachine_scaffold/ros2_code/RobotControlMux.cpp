#include <rsm_core/RobotControlMux.h>

namespace rsm {

RobotControlMux::RobotControlMux() : Node("robot_control_mux") {
    // TODO 1: Infrastructure Setup.
    // 1. Declare and retrieve all parameters.
    this->declare_parameter<int>("emergency_stop_active", 0);
    this->declare_parameter<int>("operation_mode", rsm_msgs::msg::OperationMode::STOPPED);

    _emergency_stop_active = this->get_parameter("emergency_stop_active").as_int();
    _operation_mode = this->get_parameter("operation_mode").as_int();

    // 2. Setup Pub/Sub with a reliable QoS profile (depth 10).
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();

    _autonomy_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "autonomy_cmd_vel", qos,
        std::bind(&RobotControlMux::autonomyCmdVelCallback, this, std::placeholders::_1));

    _teleoperation_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "teleoperation_cmd_vel", qos,
        std::bind(&RobotControlMux::teleoperationCmdVelCallback, this, std::placeholders::_1));

    _joystick_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "joystick", qos,
        std::bind(&RobotControlMux::joystickCallback, this, std::placeholders::_1));

    _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
    _operation_mode_pub = this->create_publisher<rsm_msgs::msg::OperationMode>("operation_mode", qos);

    // 3. Initialize the 'setOperationMode' service.
    _set_operation_mode_srv = this->create_service<rsm_msgs::srv::SetOperationMode>(
        "set_operation_mode",
        std::bind(&RobotControlMux::setOperationMode, this, std::placeholders::_1, std::placeholders::_2));

    // Timer for teleoperation idle timeout
    _teleoperation_idle_timer = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RobotControlMux::teleoperationIdleTimerCallback, this));

    _teleoperation_idle_timer.cancel();

    _joystick_cmd.axes.clear();
    _joystick_cmd.buttons.clear();
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
        if (_operation_mode == rsm_msgs::msg::OperationMode::AUTONOMOUS) {
            cmd_vel = _autonomy_cmd_vel;
        } else if (_operation_mode == rsm_msgs::msg::OperationMode::TELEOPERATION) {
            cmd_vel = _teleoperation_cmd_vel;
        }
    }
    _cmd_vel_pub->publish(cmd_vel);
}

void RobotControlMux::publishOperationMode() {
    rsm_msgs::msg::OperationMode msg;
    msg.emergency_stop = _emergency_stop_active;
    msg.mode = _operation_mode;
    _operation_mode_pub->publish(msg);
}

bool RobotControlMux::setOperationMode(
    const std::shared_ptr<rsm_msgs::srv::SetOperationMode::Request> request,
    std::shared_ptr<rsm_msgs::srv::SetOperationMode::Response> response) {
    // TODO 2: Implement the service logic.
    // Update internal states based on the 'request' and fill the 'response'.
    // [STYLE]: Access members using 'request->' and 'response->'.

    if (request->emergency_stop) {
        _emergency_stop_active = 1;
        _operation_mode = rsm_msgs::msg::OperationMode::STOPPED;
        _teleoperation_idle_timer.cancel();
    } else {
        _emergency_stop_active = 0;
        if (request->mode == rsm_msgs::msg::OperationMode::AUTONOMOUS ||
            request->mode == rsm_msgs::msg::OperationMode::TELEOPERATION ||
            request->mode == rsm_msgs::msg::OperationMode::STOPPED) {
            _operation_mode = request->mode;
            if (_operation_mode == rsm_msgs::msg::OperationMode::TELEOPERATION) {
                _teleoperation_idle_timer.reset();
                _teleoperation_idle_timer.cancel();
            }
        } else {
            response->success = false;
            response->message = "Invalid operation mode requested";
            return true;
        }
    }

    response->success = true;
    response->message = "Operation mode set successfully";
    return true;
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
            _operation_mode = rsm_msgs::msg::OperationMode::TELEOPERATION;
            _teleoperation_idle_timer.cancel();
        }
        if (_operation_mode == rsm_msgs::msg::OperationMode::TELEOPERATION) {
            _teleoperation_idle_timer.reset();
            _teleoperation_idle_timer.cancel();
            _teleoperation_idle_timer = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&RobotControlMux::teleoperationIdleTimerCallback, this));
        }
    }
}

void RobotControlMux::joystickCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
    if (!_emergency_stop_active) {
        if (checkJoystickCommand(joy)) {
            _operation_mode = rsm_msgs::msg::OperationMode::TELEOPERATION;
            _teleoperation_idle_timer.cancel();
        }
        if (_operation_mode == rsm_msgs::msg::OperationMode::TELEOPERATION) {
            _teleoperation_idle_timer.reset();
            _teleoperation_idle_timer.cancel();
            _teleoperation_idle_timer = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&RobotControlMux::teleoperationIdleTimerCallback, this));
        }
    }
}

void RobotControlMux::teleoperationIdleTimerCallback() {
    _operation_mode = rsm_msgs::msg::OperationMode::STOPPED;
    geometry_msgs::msg::Twist empty_cmd_vel;
    _teleoperation_cmd_vel = empty_cmd_vel;
    _teleoperation_idle_timer.cancel();
}

bool RobotControlMux::checkJoystickCommand(
    const sensor_msgs::msg::Joy::SharedPtr joy) {
    bool movingCommand = false;
    if (_joystick_cmd.axes.size() == joy->axes.size()) {
        for (size_t i = 0; i < joy->axes.size(); i++) {
            if ((std::abs(joy->axes[i]) - std::abs(_joystick_cmd.axes[i])) > MOVE_THRESH) {
                movingCommand = true;
            }
        }
    }
    if (_joystick_cmd.buttons.size() == joy->buttons.size()) {
        for (size_t i = 0; i < joy->buttons.size(); i++) {
            if (joy->buttons[i] && !_joystick_cmd.buttons[i]) {
                movingCommand = true;
            }
        }
    }
    _joystick_cmd = *joy;
    return movingCommand;
}

} /* namespace rsm */