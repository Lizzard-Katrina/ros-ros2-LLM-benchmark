#include <ros/ros.h>
#include "tm_msgs/SctResponse.h"

void SctResponseCallback(const tm_msgs::SctResponse::ConstPtr& msg)
{
   //TODO: print or process sct_response message
   //END
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SctResponseDemo");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("tm_driver/sct_response", 1000, SctResponseCallback);
    ros::spin();
    return 0;
}
