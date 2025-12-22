#include "sound_localizer.h"

SoundLocalizer::SoundLocalizer(ros::NodeHandle& nh) {
    audio_sub_ = nh.subscribe("/audio/raw", 100, &SoundLocalizer::audioCallback, this);
    sound_dir_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/sound_direction", 10);

    // TODO: Initialize sound localization algorithm (HARK skeleton)
    // 1. Initialize audio buffers
    // 2. Initialize estimated direction
    // 3. Load algorithm parameters
    // 4. Initialize any processing modules / FFT buffers
    // END
}
