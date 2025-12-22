#ifndef SOUND_LOCALIZER_H
#define SOUND_LOCALIZER_H

#include <ros/ros.h>
#include <sensor_msgs/AudioData.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <Eigen/Dense>

class SoundLocalizer {
public:
    SoundLocalizer(ros::NodeHandle& nh);

    void audioCallback(const sensor_msgs::AudioData::ConstPtr& audio_msg);

private:
    ros::Subscriber audio_sub_;
    ros::Publisher sound_dir_pub_;

    // Audio buffers and processing state
    std::vector<std::vector<float>> microphoneBuffer_;
    Eigen::Vector3f estimatedDirection_;

    // Algorithm parameters
    int sampleRate_;
    int bufferSize_;
};

#endif
