/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <nav2_util/geometry_utils.hpp>

#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

#include <Eigen/Core>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/utils.h>

namespace base_local_planner {

  class TrajectoryPlannerROS {
  public:

    bool checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
      geometry_msgs::msg::PoseStamped global_pose;
      if(costmap_ros_->getRobotPose(global_pose)){
        if(update_map){
          //we need to give the planner some sort of global plan, since we're only checking for legality
          //we'll just give the robots current position
          std::vector<geometry_msgs::msg::PoseStamped> plan;
          plan.push_back(global_pose);
          tc_->updatePlan(plan, true);
        }

        //copy over the odometry information
        nav_msgs::msg::Odometry base_odom;
        {
          boost::recursive_mutex::scoped_lock lock(odom_lock_);
          base_odom = base_odom_;
        }

        return tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation),
            base_odom.twist.twist.linear.x,
            base_odom.twist.twist.linear.y,
            base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);

      }
      RCLCPP_WARN(rclcpp::get_logger("trajectory_planner_ros"), "Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
      return false;
    }

  };

};