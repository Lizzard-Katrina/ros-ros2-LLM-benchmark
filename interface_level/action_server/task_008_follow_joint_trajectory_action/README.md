# Task 008 — Follow Joint Trajectory Action

## Purpose
Benchmark ROS1 → ROS2 translation of an action server. The server executes FollowJointTrajectory goals, provides feedback on joint positions, and reports success/failure.

# Oracle Test Suite for TrajectoryPlannerROS::checkTrajectory (ROS2 Translation)

## Design Overview
These oracle tests verify that the ROS1 → ROS2 translation of `TrajectoryPlannerROS::checkTrajectory` retains **all key behaviors and semantic logic**, without requiring compilation or execution.

Each test validates a **concept**, independent of line numbers or exact formatting.

--

##  Source Code Origin

This task is derived from the ROS1 Navigation Stack.

**Original package:**
- `base_local_planner`

**Original file:**
-`https://github.com/ros-planning/navigation/blob/noetic-devel/base_local_planner/src/trajectory_planner_ros.cpp`

**Target function:**
```cpp
bool TrajectoryPlannerROS::checkTrajectory(
    double vx_samp,
    double vy_samp,
    double vtheta_samp,
    bool update_map
)
```

---

## Tests and Concepts

1. **Class Exists**  
   - Verifies that the `TrajectoryPlannerROS` class is defined.
   - Reference: `class TrajectoryPlannerROS` in original ROS1 code.

2. **checkTrajectory Method Exists**  
   - Ensures the method `bool checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map)` is present.
   - Reference: ROS1 `TrajectoryPlannerROS::checkTrajectory`.

3. **Uses costmap_ros_->getRobotPose**  
   - Confirms that the planner retrieves the robot pose using the costmap in ROS2.
   - Reference: `costmap_ros_->getRobotPose(global_pose)`.

4. **Handles update_map Logic**  
   - Checks that if `update_map` is true, `tc_->updatePlan()` is called to refresh the plan.
   - Reference: ROS1:
     ```cpp
     if(update_map){
         tc_->updatePlan(plan, true);
     }
     ```

5. **Uses Odometry with Lock**  
   - Ensures that `base_odom_` is copied with a `boost::recursive_mutex::scoped_lock`.
   - Reference: ROS1 code section copying odom inside lock.

6. **Calls tc_->checkTrajectory with Correct Args**  
   - Validates that `tc_->checkTrajectory` is called with robot position, orientation, odometry, and sampled velocities.
   - Reference: ROS1:
     ```cpp
     tc_->checkTrajectory(global_pose.pose.position.x,
                          global_pose.pose.position.y,
                          tf2::getYaw(global_pose.pose.orientation),
                          base_odom.twist.twist.linear.x,
                          base_odom.twist.twist.linear.y,
                          base_odom.twist.twist.angular.z,
                          vx_samp, vy_samp, vtheta_samp)
     ```

7. **Returns Boolean**  
   - Ensures that the function returns the boolean result of `tc_->checkTrajectory`.
   - Reference: ROS1 `return tc_->checkTrajectory(...);`

8. **Warns if getRobotPose Fails**  
   - Confirms that a warning is issued if `getRobotPose` fails.
   - Reference: ROS1 `ROS_WARN("Failed to get the pose of the robot...");`

9. **No ROS1 API Leftovers**  
   - Checks that all ROS1-specific types and namespaces are removed (`ros::`, `tf::`, `nav_msgs::Odometry`, `costmap_2d::Costmap2DROS`).
   - Ensures ROS2-only implementation.

