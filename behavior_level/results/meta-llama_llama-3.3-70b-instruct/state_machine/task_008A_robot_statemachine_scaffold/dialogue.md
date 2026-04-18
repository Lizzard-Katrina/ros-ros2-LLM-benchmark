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

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/timer.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rsm_msgs/msg/operation_mode.hpp>
#include <rsm_msgs/srv/set_operation_mode.hpp>

namespace rsm {

class RobotControlMux : public rclcpp::Node {
public:
    RobotControlMux() : Node("robot_control_mux") {
        // TODO 1: Infrastructure Setup.
        // 1. Declare and retrieve all parameters.
        // 2. Setup Pub/Sub with a reliable QoS profile (depth 10).
        // 3. Initialize the 'setOperationMode' service.
        // [STYLE]: For the service callback, you MUST name the parameters 'request' and 'response'.
        // Use 'this->create_subscription' and 'this->create_publisher'.
        this->declare_parameter("emergency_stop_active", 0);
        this->declare_parameter("operation_mode", 0);
        _emergency_stop_active = this->get_parameter("emergency_stop_active").as_int();
        _operation_mode = static_cast<rsm_msgs::msg::OperationMode::_operation_mode_type>(this->get_parameter("operation_mode").as_int());

        // Create publishers
        _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
        _operation_mode_pub = this->create_publisher<rsm_msgs::msg::OperationMode>("operation_mode", rclcpp::QoS(10));

        // Create subscribers
        _autonomy_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("autonomy_cmd_vel", rclcpp::QoS(10), std::bind(&RobotControlMux::autonomyCmdVelCallback, this, std::placeholders::_1));
        _teleoperation_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("teleoperation_cmd_vel", rclcpp::QoS(10), std::bind(&RobotControlMux::teleoperationCmdVelCallback, this, std::placeholders::_1));
        _joystick_sub = this->create_subscription<sensor_msgs::msg::Joy>("joystick", rclcpp::QoS(10), std::bind(&RobotControlMux::joystickCallback, this, std::placeholders::_1));

        // Create service
        _set_operation_mode_srv = this->create_service<rsm_msgs::srv::SetOperationMode>("set_operation_mode", std::bind(&RobotControlMux::setOperationMode, this, std::placeholders::_1, std::placeholders::_2));

        // Create timer
        _teleoperation_idle_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&RobotControlMux::teleoperationIdleTimerCallback, this));
    }

    ~RobotControlMux() {}

    void publishTopics() {
        publishCmdVel();
        publishOperationMode();
    }

    void publishCmdVel() {
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

    void publishOperationMode() {
        rsm_msgs::msg::OperationMode msg;
        msg.emergency_stop = _emergency_stop_active;
        msg.mode = _operation_mode;
        _operation_mode_pub->publish(msg);
    }

    bool setOperationMode(const std::shared_ptr<rsm_msgs::srv::SetOperationMode::Request> request, std::shared_ptr<rsm_msgs::srv::SetOperationMode::Response> response) {
        // TODO 2: Implement the service logic.
        // Update internal states based on the 'request' and fill the 'response'.
        // [STYLE]: Access members using 'request->' and 'response->'.
        _operation_mode = request->mode;
        response->success = true;
        return true;
    }

    void autonomyCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        _autonomy_cmd_vel = *msg;
    }

    void teleoperationCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        _teleoperation_cmd_vel = *msg;
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

    void joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (!_emergency_stop_active) {
            if (checkJoystickCommand(msg)) {
                _operation_mode = rsm_msgs::msg::OperationMode::TELEOPERATION;
                _teleoperation_idle_timer->cancel();
            }
            if (_operation_mode == rsm_msgs::msg::OperationMode::TELEOPERATION) {
                _teleoperation_idle_timer->reset();
            }
        }
    }

    void teleoperationIdleTimerCallback() {
        _operation_mode = rsm_msgs::msg::OperationMode::STOPPED;
        geometry_msgs::msg::Twist empty_cmd_vel;
        _teleoperation_cmd_vel = empty_cmd_vel;
        _teleoperation_idle_timer->cancel();
    }

    bool checkJoystickCommand(const sensor_msgs::msg::Joy::SharedPtr msg) {
        bool movingCommand = false;
        if (_joystick_cmd.axes.size() == msg->axes.size()) {
            for (unsigned int i = 0; i < msg->axes.size(); i++) {
                if ((std::abs(msg->axes[i]) - std::abs(_joystick_cmd.axes[i])) > MOVE_THRESH) {
                    movingCommand = true;
                }
            }
        }
        if (_joystick_cmd.buttons.size() == msg->buttons.size()) {
            for (unsigned int i = 0; i < msg->buttons.size(); i++) {
                if (msg->buttons[i] && !_joystick_cmd.buttons[i]) {
                    movingCommand = true;
                }
            }
        }
        _joystick_cmd = *msg;
        return movingCommand;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;
    rclcpp::Publisher<rsm_msgs::msg::OperationMode>::SharedPtr _operation_mode_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _autonomy_cmd_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _teleoperation_cmd_vel_sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joystick_sub;
    rclcpp::Service<rsm_msgs::srv::SetOperationMode>::SharedPtr _set_operation_mode_srv;
    rclcpp::TimerBase::SharedPtr _teleoperation_idle_timer;

    int _emergency_stop_active;
    int _operation_mode;
    geometry_msgs::msg::Twist _autonomy_cmd_vel;
    geometry_msgs::msg::Twist _teleoperation_cmd_vel;
    sensor_msgs::msg::Joy _joystick_cmd;
    const double MOVE_THRESH = 0.1;
};

} /* namespace rsm */
```