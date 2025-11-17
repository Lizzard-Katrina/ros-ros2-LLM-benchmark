#!/bin/bash
set -e

# Source ROS
source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash

# Run the subscriber first (in background)
/workspace/devel/lib/task_code/person_subscriber &

# Run the publisher
/workspace/devel/lib/task_code/person_publisher
