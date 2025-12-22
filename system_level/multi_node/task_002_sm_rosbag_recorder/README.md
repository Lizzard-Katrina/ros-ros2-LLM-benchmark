# ROS Bag Recorder

## Overview
This package provides the `rosbag` recorder node that records ROS topics to bag files for later playback.  
The recorder subscribes to specified topics, records messages with optional buffering and splitting, and supports snapshot triggers.  

### Key Features
- Record selected topics, all topics, or topics matching regex patterns.
- Snapshot recording triggered via a ROS topic.
- Automatic file splitting based on size, duration, or count.
- Optional compression and latched message handling.
- Disk space monitoring to prevent overflow.
- Supports both synchronous and asynchronous writing.

## Architecture
The recorder orchestrates the following components:

1. **Publisher Nodes**  
   ROS nodes publish messages to topics.

2. **Recorder Node (`rosbag`)**  
   - Subscribes to topics.
   - Queues incoming messages (`OutgoingQueue`).
   - Writes messages to bag files (`startWriting`, `doRecord`, `doRecordSnapshotter`).
   - Monitors disk space and handles splitting (`checkSize`, `checkDuration`).

3. **Playback / Analysis**  
   Recorded bag files can be replayed or analyzed using ROS tools.

## Installation
```bash
# Clone the repository
git clone https://github.com/ros/ros_comm.git
cd ros_comm

# Build using catkin
catkin_make
source devel/setup.bash
