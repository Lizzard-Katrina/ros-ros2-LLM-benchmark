# Task 003: TurtleBot3 Simulation

**Source:** [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)  

## Description
This task integrates a standard mobile robot stack with Gazebo and ROS. You will:

- Spawn a TurtleBot3 (TB3) model in a Gazebo world.
- Launch simulation via `roslaunch turtlebot3_gazebo`.
- Run navigation or teleop nodes.
- Observe TF, odometry, and laser scan topics.

This task is useful for practicing ROS-Gazebo integration, robot state publishing, and sensor topic subscriptions.

## Source Directories

| Directory | Purpose |
|-----------|---------|
| `turtlebot3_gazebo/launch` | Launch files for Gazebo simulation and TB3 robot spawn |
| `turtlebot3_gazebo/urdf`   | URDF models of TB3 robots (Burger and Waffle) |
| `turtlebot3_gazebo/worlds` | Gazebo world files for simulation |

## Example Files to Work On (Scaffolded)

- `turtlebot3_gazebo/launch/turtlebot3_world.launch.py`
- `turtlebot3_gazebo/urdf/turtlebot3_burger_cam.urdf`
- `turtlebot3_gazebo/urdf/turtlebot3_waffle.urdf`

## Expected Outcome

After completing the task:

1. You can launch Gazebo with the TB3 robot in the world.
2. The robot model correctly spawns with all links/joints defined.
3. Sensor topics (LIDAR, camera, IMU) are published to ROS.
4. You can run teleop or navigation nodes and observe TF and odometry.
5. Students practice filling in missing blocks in URDF and launch files.

## Notes

- Each scaffold file has one or more TODOs marking a **complete functional block** to implement.
- Fill in each TODO according to ROS-Gazebo conventions.

