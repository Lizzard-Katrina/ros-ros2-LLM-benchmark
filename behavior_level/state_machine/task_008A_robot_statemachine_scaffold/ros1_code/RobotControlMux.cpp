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
