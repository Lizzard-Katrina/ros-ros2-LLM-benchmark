#include <mobile_robot_autonomous_navigation/robot_hardware_interface.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"

namespace mobile_robot_autonomous_navigation {

hardware_interface::CallbackReturn ROBOTHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    joint_position_.assign(2, 0.0);
    joint_velocity_.assign(2, 0.0);
    joint_velocity_command_.assign(2, 0.0);
    
    left_motor_pos = 0.0;
    right_motor_pos = 0.0;
    left_prev_cmd = 0;
    right_prev_cmd = 0;
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ROBOTHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &joint_position_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[1].name, hardware_interface::HW_IF_POSITION, &joint_position_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_[1]));
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ROBOTHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_command_[0]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_command_[1]));
    return command_interfaces;
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

hardware_interface::return_type ROBOTHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    uint8_t rbuff[1];
    int x;

    left_motor.readBytes(rbuff,1);
    x=(int8_t)rbuff[0];
    left_motor_pos+=angles::from_degrees((double)x);
    joint_position_[0]=left_motor_pos;

    right_motor.readBytes(rbuff,1);
    x=(int8_t)rbuff[0];
    right_motor_pos+=angles::from_degrees((double)x);
    joint_position_[1]=right_motor_pos;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ROBOTHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    uint8_t wbuff[2];
    int velocity,result;
    
    velocity=(int)angles::to_degrees(joint_velocity_command_[0]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;

    if(left_prev_cmd!=velocity)
    {
	    result = left_motor.writeData(wbuff,2);
	    left_prev_cmd=velocity;
    }
    
    velocity=(int)angles::to_degrees(joint_velocity_command_[1]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;

    if(right_prev_cmd!=velocity)
    {
	    result = right_motor.writeData(wbuff,2);
	    right_prev_cmd=velocity;
    }

    return hardware_interface::return_type::OK;
}

} // namespace mobile_robot_autonomous_navigation

PLUGINLIB_EXPORT_CLASS(mobile_robot_autonomous_navigation::ROBOTHardwareInterface, hardware_interface::SystemInterface)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mobile_robot_hardware_interface");
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}