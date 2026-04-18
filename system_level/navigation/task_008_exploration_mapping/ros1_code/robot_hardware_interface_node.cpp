#include <mobile_robot_autonomous_navigation/robot_hardware_interface.h>

//namesapce i2c_ros
/*TODO
* 1. Refactor 'ROBOTHardwareInterface' to inherit from 'hardware_interface::SystemInterface'.
 * 2. Define mandatory ROS 2 Control overrides: on_init, export_state_interfaces, 
 * export_command_interfaces, read, and write.
 * 3. Replace ROS 1 NodeHandle with ROS 2 LifecycleNode components if necessary.
 * 4. Declare storage for joint states (position, velocity) and commands (velocity).
 * Constraint: Use 'hardware_interface::CallbackReturn' for lifecycle methods.
 *END OF TODO 
*/
ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

/* TODO: [System Level Sync - Hole 2: Lifecycle Implementation] * 1. In 'on_init', parse hardware info and 
 initialize internal buffers. * 2. Implement 'export_state_interfaces' to return 'position' and 'velocity' 
 handles. * 3. Implement 'export_command_interfaces' to return 'velocity' handles. * 4. IMPORTANT: Joint names 
 MUST exactly match those in 'controllers.yaml'. *END OF TODO
*/
void ROBOTHardwareInterface::read() {
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

    //ROS_INFO("pos=%.2f x=%d ",pos,x);
	
}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
   
    velocityJointSaturationInterface.enforceLimits(elapsed_time);   

	uint8_t wbuff[2];

    int velocity,result;
    
    
    velocity=(int)angles::to_degrees(joint_velocity_command_[0]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;
	//ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    if(left_prev_cmd!=velocity)
    {
	    result = left_motor.writeData(wbuff,2);
	    //ROS_INFO("Writen successfully result=%d", result);
	    left_prev_cmd=velocity;
    }
    
    velocity=(int)angles::to_degrees(joint_velocity_command_[1]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;
	//ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    if(right_prev_cmd!=velocity)
    {
	    result = right_motor.writeData(wbuff,2);
	    //ROS_INFO("Writen successfully result=%d", result);
	    right_prev_cmd=velocity;
    }


		
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "mobile_robot_hardware_interface");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(4);  
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}
