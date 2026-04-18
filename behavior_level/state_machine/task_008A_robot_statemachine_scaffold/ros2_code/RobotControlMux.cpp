#include <rsm_core/RobotControlMux.h>

#include <cmath>
#include <chrono>
#include <functional>

namespace rsm {

RobotControlMux::RobotControlMux() {
    // TODO 1: Infrastructure Setup.
    // 1. Declare and retrieve all parameters.
    // 2. Setup Pub/Sub with a reliable QoS profile (depth 10).
    // 3. Initialize the 'setOperationMode' service.
    // [STYLE]: For the service callback, you MUST name the parameters 'request' and 'response'.
    // Use 'this->create_subscription' and 'this->create_publisher'.
    //END OF TODO
    const std::string autonomy_cmd_vel_topic =
        this->declare_parameter<std::string>("autonomy_cmd_vel_topic", "/autonomy/cmd_vel");
    const std::string teleoperation_cmd_vel_topic =
        this->declare_parameter<std::string>("teleoperation_cmd_vel_topic", "/teleoperation/cmd_vel");
    const std::string joystick_topic =
        this->declare_parameter<std::string>("joystick_topic", "/joy");
    const std::string cmd_vel_topic =
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const std::string operation_mode_topic =
        this->declare_parameter<std::string>("operation_mode_topic", "/operation_mode");
    const std::string set_operation_mode_service_name =
        this->declare_parameter<std::string>("set_operation_mode_service", "/set_operation_mode");
    const double teleoperation_idle_timeout_sec =
        this->declare_parameter<double>("teleoperation_idle_timeout_sec", 0.5);

    rclcpp::QoS qos(10);
    qos.reliable();

    _autonomy_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        autonomy_cmd_vel_topic, qos,
        std::bind(&RobotControlMux::autonomyCmdVelCallback, this, std::placeholders::_1));

    _teleoperation_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        teleoperation_cmd_vel_topic, qos,
        std::bind(&RobotControlMux::teleoperationCmdVelCallback, this, std::placeholders::_1));

    _joystick_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        joystick_topic, qos,
        std::bind(&RobotControlMux::joystickCallback, this, std::placeholders::_1));

    _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, qos);
    _operation_mode_pub = this->create_publisher<rsm_msgs::msg::OperationMode>(operation_mode_topic, qos);

    _set_operation_mode_srv = this->create_service<rsm_msgs::srv::SetOperationMode>(
        set_operation_mode_service_name,
        std::bind(&RobotControlMux::setOperationMode, this, std::placeholders::_1, std::placeholders::_2));

    _teleoperation_idle_timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(teleoperation_idle_timeout_sec)),
        std::bind(&RobotControlMux::teleoperationIdleTimerCallback, this));
    _teleoperation_idle_timer->cancel();

	_emergency_stop_active = 0;
	_operation_mode = rsm_msgs::msg::OperationMode::STOPPED;
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
	msg.emergencyStop = _emergency_stop_active;
	msg.mode = _operation_mode;
	_operation_mode_pub->publish(msg);
}

void RobotControlMux::setOperationMode(
		const std::shared_ptr<rsm_msgs::srv::SetOperationMode::Request> request,
		std::shared_ptr<rsm_msgs::srv::SetOperationMode::Response> response) {
// TODO 2: Implement the service logic.
    // Update internal states based on the 'request' and fill the 'response'.
    // [STYLE]: Access members using 'request->' and 'response->'.
//END OF TODO
	_emergency_stop_active = request->emergencyStop;

	if (_emergency_stop_active) {
		_operation_mode = rsm_msgs::msg::OperationMode::STOPPED;
		_teleoperation_cmd_vel = geometry_msgs::msg::Twist();
		_teleoperation_idle_timer->cancel();
		response->success = true;
		return;
	}

	if (request->mode == rsm_msgs::msg::OperationMode::STOPPED ||
	    request->mode == rsm_msgs::msg::OperationMode::AUTONOMOUS ||
	    request->mode == rsm_msgs::msg::OperationMode::TELEOPERATION) {
		_operation_mode = request->mode;
		response->success = true;
	} else {
		response->success = false;
		return;
	}

	if (_operation_mode == rsm_msgs::msg::OperationMode::TELEOPERATION) {
		_teleoperation_idle_timer->reset();
	} else {
		_teleoperation_idle_timer->cancel();
	}
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
			_teleoperation_idle_timer->cancel();
		}
		if (_operation_mode == rsm_msgs::msg::OperationMode::TELEOPERATION) {
			_teleoperation_idle_timer->reset();
		}
	}
}

void RobotControlMux::joystickCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
	if (!_emergency_stop_active) {
		if (checkJoystickCommand(joy)) {
			_operation_mode = rsm_msgs::msg::OperationMode::TELEOPERATION;
			_teleoperation_idle_timer->cancel();
		}
		if (_operation_mode == rsm_msgs::msg::OperationMode::TELEOPERATION) {
			_teleoperation_idle_timer->reset();
		}
	}
}

void RobotControlMux::teleoperationIdleTimerCallback() {
	_operation_mode = rsm_msgs::msg::OperationMode::STOPPED;
	geometry_msgs::msg::Twist empty_cmd_vel;
	_teleoperation_cmd_vel = empty_cmd_vel;
	_teleoperation_idle_timer->cancel();
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