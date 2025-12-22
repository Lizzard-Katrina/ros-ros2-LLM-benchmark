#include "sound_localizer.h"
#include <Eigen/Dense>
#include <std_msgs/Float32MultiArray.h>

void SoundLocalizer::audioCallback(const sensor_msgs::AudioData::ConstPtr& audio_msg) {
    // Append audio data to microphone buffer
    microphoneBuffer_.push_back(audio_msg->data);

    // TODO: Compute sound source direction from microphone array
    // --------------------------
    // - Perform FFT or HARK beamforming
    // - Estimate azimuth/elevation of dominant sound source
    // - Update estimatedDirection_
    // END

    // Publish estimated direction
    std_msgs::Float32MultiArray dirMsg;
    dirMsg.data = {estimatedDirection_(0), estimatedDirection_(1), estimatedDirection_(2)};
    sound_dir_pub_.publish(dirMsg);
}
