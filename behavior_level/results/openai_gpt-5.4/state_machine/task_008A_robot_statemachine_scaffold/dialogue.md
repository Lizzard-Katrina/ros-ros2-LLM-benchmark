# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#include <rsm_core/RobotControlMux.h>

namespace rsm {

RobotControlMux::RobotControlMux() {
    // TODO 1: Infrastructure Setup.
    // 1. Declare and retrieve all parameters.
    // 2. Setup Pub/Sub with a reliable QoS profile (depth 10).
    // 3. Initialize the 'setOperationMode' service.
    // [STYLE]: For the service callback, you MUST name the parameters 'request' and 'response'.
    // Use 'this->create_subscription' and 'this->create_publisher'.
    //END OF TODO
	_emergency_stop_active = 0;
	_operation_mode = rsm_msgs::OperationMode::STOPPED;
}

RobotControlMux::~RobotControlMux() {

}

void RobotControlMux::publishTopics() {
	publishCmdVel();
	publishOperationMode();
}

void RobotControlMux::publishCmdVel() {
	geometry_msgs::Twist cmd_vel;
	if (!_emergency_stop_active) {
		if (_operation_mode == rsm_msgs::OperationMode::AUTONOMOUS) {
			cmd_vel = _autonomy_cmd_vel;
		} else if (_operation_mode == rsm_msgs::OperationMode::TELEOPERATION) {
			cmd_vel = _teleoperation_cmd_vel;
		}
	}
	_cmd_vel_pub.publish(cmd_vel);
}

void RobotControlMux::publishOperationMode() {
	rsm_msgs::OperationMode msg;
	msg.emergencyStop = _emergency_stop_active;
	msg.mode = _operation_mode;
	_operation_mode_pub.publish(msg);
}

bool RobotControlMux::setOperationMode(rsm_msgs::SetOperationMode::Request &req,
		rsm_msgs::SetOperationMode::Response &res) {
// TODO 2: Implement the service logic.
    // Update internal states based on the 'request' and fill the 'response'.
    // [STYLE]: Access members using 'request->' and 'response->'.
//END OF TODO
}

void RobotControlMux::autonomyCmdVelCallback(
		const geometry_msgs::Twist::ConstPtr& cmd_vel) {
	_autonomy_cmd_vel = *cmd_vel;
}

void RobotControlMux::teleoperationCmdVelCallback(
		const geometry_msgs::Twist::ConstPtr& cmd_vel) {
	_teleoperation_cmd_vel = *cmd_vel;
	if (!_emergency_stop_active) {
		if (_teleoperation_cmd_vel.linear.x != 0.0
				|| _teleoperation_cmd_vel.linear.y != 0.0
				|| _teleoperation_cmd_vel.linear.z != 0.0
				|| _teleoperation_cmd_vel.angular.x != 0.0
				|| _teleoperation_cmd_vel.angular.y != 0.0
				|| _teleoperation_cmd_vel.angular.z != 0.0) {
			_operation_mode = rsm_msgs::OperationMode::TELEOPERATION;
			_teleoperation_idle_timer.stop();
		}
		if (_operation_mode == rsm_msgs::OperationMode::TELEOPERATION) {
			_teleoperation_idle_timer.start();
		}
	}
}

void RobotControlMux::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	if (!_emergency_stop_active) {
		if (checkJoystickCommand(joy)) {
			_operation_mode = rsm_msgs::OperationMode::TELEOPERATION;
			_teleoperation_idle_timer.stop();
		}
		if (_operation_mode == rsm_msgs::OperationMode::TELEOPERATION) {
			_teleoperation_idle_timer.start();
		}
	}
}

void RobotControlMux::teleoperationIdleTimerCallback(
		const ros::TimerEvent& event) {
	_operation_mode = rsm_msgs::OperationMode::STOPPED;
	geometry_msgs::Twist empty_cmd_vel;
	_teleoperation_cmd_vel = empty_cmd_vel;
	_teleoperation_idle_timer.stop();
}

bool RobotControlMux::checkJoystickCommand(
		const sensor_msgs::Joy::ConstPtr& joy) {
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

----------------------------


# LLM Response

#include <rsm_core/RobotControlMux.h>
#include <rclcpp/qos.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <functional>

namespace rsm {

RobotControlMux::RobotControlMux() {
    this->declare_parameter("autonomy_cmd_vel_topic", std::string("autonomy/cmd_vel"));
    this->declare_parameter("teleoperation_cmd_vel_topic", std::string("teleoperation/cmd_vel"));
    this->declare_parameter("joystick_topic", std::string("joy"));
    this->declare_parameter("cmd_vel_topic", std::string("cmd_vel"));
    this->declare_parameter("operation_mode_topic", std::string("operation_mode"));
    this->declare_parameter("set_operation_mode_service", std::string("setOperationMode"));
    this->declare_parameter("teleoperation_idle_timeout", 1.0);

    this->get_parameter("autonomy_cmd_vel_topic", _autonomy_cmd_vel_topic);
    this->get_parameter("teleoperation_cmd_vel_topic", _teleoperation_cmd_vel_topic);
    this->get_parameter("joystick_topic", _joystick_topic);
    this->get_parameter("cmd_vel_topic", _cmd_vel_topic);
    this->get_parameter("operation_mode_topic", _operation_mode_topic);
    this->get_parameter("set_operation_mode_service", _set_operation_mode_service);
    this->get_parameter("teleoperation_idle_timeout", _teleoperation_idle_timeout);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    _autonomy_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        _autonomy_cmd_vel_topic,
        qos,
        std::bind(&RobotControlMux::autonomyCmdVelCallback, this, std::placeholders::_1));

    _teleoperation_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        _teleoperation_cmd_vel_topic,
        qos,
        std::bind(&RobotControlMux::teleoperationCmdVelCallback, this, std::placeholders::_1));

    _joystick_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        _joystick_topic,
        qos,
        std::bind(&RobotControlMux::joystickCallback, this, std::placeholders::_1));

    _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(_cmd_vel_topic, qos);
    _operation_mode_pub = this->create_publisher<rsm_msgs::msg::OperationMode>(_operation_mode_topic, qos);

    _set_operation_mode_srv = this->create_service<rsm_msgs::srv::SetOperationMode>(
        _set_operation_mode_service,
        std::bind(
            &RobotControlMux::setOperationMode,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    _teleoperation_idle_timer = this->create_wall_timer(
        std::chrono::duration<double>(_teleoperation_idle_timeout),
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

bool RobotControlMux::setOperationMode(
		const std::shared_ptr<rsm_msgs::srv::SetOperationMode::Request> request,
		std::shared_ptr<rsm_msgs::srv::SetOperationMode::Response> response) {
	_emergency_stop_active = request->emergencyStop;
	_operation_mode = request->mode;

	if (_emergency_stop_active) {
		_operation_mode = rsm_msgs::msg::OperationMode::STOPPED;
		_teleoperation_idle_timer->cancel();
	}

	response->success = true;
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