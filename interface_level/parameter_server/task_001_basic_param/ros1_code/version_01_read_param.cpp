#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_001_basic_param");
    ros::NodeHandle nh("~");

    std::string robot_name;
    int start_mode;

    // TODO_1: Load parameters robot_name and start_mode from parameter server
    // end:TODO_1

    ROS_INFO("Node started.");
    return 0;
}
