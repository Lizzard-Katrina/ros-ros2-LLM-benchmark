#include "sound_localizer.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "sound_localization_node");
    ros::NodeHandle nh;

    SoundLocalizer localizer(nh);

    ros::spin();
    return 0;
}
