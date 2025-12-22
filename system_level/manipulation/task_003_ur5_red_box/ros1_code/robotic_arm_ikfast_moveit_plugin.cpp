#include "robotic_arm_ikfast_moveit_plugin/robotic_arm_ikfast_moveit_plugin.h"
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Geometry>

namespace robotic_arm
{

class RoboticArmMoveItPlugin
{
public:
    RoboticArmMoveItPlugin() {}
    ~RoboticArmMoveItPlugin() {}

    bool computeTrajectory(const Eigen::Affine3d& target_pose,
                           trajectory_msgs::JointTrajectory& traj);

private:
    // Member variables
    std::vector<std::string> joint_names_;
    double max_joint_speed_;
};

bool RoboticArmMoveItPlugin::computeTrajectory(const Eigen::Affine3d& target_pose,
                                               trajectory_msgs::JointTrajectory& traj)
{
    // TODO: Fill in the IK solution selection and trajectory generation logic
    // END
    return true;
}

} // namespace robotic_arm
