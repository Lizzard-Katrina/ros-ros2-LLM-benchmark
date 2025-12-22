#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    ROS_INFO_STREAM("Received joint positions: ");
    for (size_t i = 0; i < msg->position.size(); i++) {
        ROS_INFO_STREAM("  joint_" << i << ": " << msg->position[i]);
    }

    // TODO: Integration-level check
    // <FILL: add range check / simple assertion>
    // END of TODO
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_get_joint_state");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("allegro_hand/joint_states", 10, jointStateCallback);

    ros::spin();
    return 0;
}
