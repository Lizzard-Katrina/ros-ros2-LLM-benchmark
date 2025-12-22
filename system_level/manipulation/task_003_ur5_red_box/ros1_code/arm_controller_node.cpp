#include <ros/ros.h>
#include "robotic_arm_ikfast_moveit_plugin.h"
#include "robotic_arm_ikfast_solver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_controller_node");
    ros::NodeHandle nh;

    robotic_arm::RoboticArmMoveItPlugin plugin;
    robotic_arm::RoboticArmIKFastSolver solver;

    Eigen::Affine3d target_pose; // Target pose set by test

    trajectory_msgs::JointTrajectory traj;

    // TODO: Call plugin.computeTrajectory() and publish the trajectory
    // end: Fill-in ends

    ros::spin();
    return 0;
}
