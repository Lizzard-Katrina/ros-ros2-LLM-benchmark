# Task 002 — Camera → Edge Detection

This task subscribes to `/camera/image_raw` (sensor_msgs/Image), applies Canny edge detection,
and publishes edges as `/edges`. There are **two part files**, each with a single blank.

## Files with blanks

1. edge_detection_node_part1.py
   - Blank: convert ROS Image to OpenCV image
   - Expected outcome: `cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")`

2. edge_detection_node_part2.py
   - Blank: convert OpenCV edges to ROS Image
   - Expected outcome: `ros_edges = self.bridge.cv2_to_imgmsg(edges, encoding="mono8")`

## Directory structure

- `ros1_code/` : Python node + launch
- `tests/` : optional validation scripts
- `docker/` : Dockerfile + setup.sh
- `metadata.json`
- `README.md`

## Docker instructions

### Build:
```
cd docker
docker build -t camera_edge_ros1 .
```

### Run:

```
docker run -it camera_edge_ros1 /bin/bash
source /opt/ros/noetic/setup.bash
cd /root/ros1_ws
./setup.sh
```

