# Task 001 — LaserScan → Obstacle Detection

This task focuses on converting a ROS1 node that processes LaserScan data
and publishes obstacle information. The learner must fill in missing code
in two separate versions of the same file.

## Files with Blanks

Each part file contains the complete node, with exactly one blank:

### 1. laser_obstacle_filter_part1.py
Blank: compute minimum range from LaserScan  
Expected outcome:  
`min_range = min(msg.ranges)`

### 2. laser_obstacle_filter_part2.py
Blank: build the obstacle PointStamped message  
Expected outcome: 
```
obstacle_msg = PointStamped()
obstacle_msg.header = msg.header
obstacle_msg.point.x = min_range
obstacle_msg.point.y = 0.0
obstacle_msg.point.z = 0.0
```
## Docker Instructions

### Build:
cd docker
docker build -t laser_filter_ros1 .


### Run container:
docker run -it laser_filter_ros1 /bin/bash

