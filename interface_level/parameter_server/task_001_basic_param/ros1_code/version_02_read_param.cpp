#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_001_basic_param");
    ros::NodeHandle nh("~");

    std::string robot_name;
    int start_mode;

    nh.getParam("robot_name", robot_name);
    nh.getParam("start_mode", start_mode);

    // TODO_2: Log the parameter values robot_name and start_mode
    // end:TODO_2

    return 0;
}
