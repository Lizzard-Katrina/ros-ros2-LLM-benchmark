# Task 006 — Stereo Depth Map Generation
**Behavior Level → Sensor Processing**

This task converts stereo images into a depth map.

---

## 📌 Task Summary

### Inputs:
- `/left/image_raw`  (sensor_msgs/Image)
- `/right/image_raw` (sensor_msgs/Image)

### Output:
- `/depth/image`  (sensor_msgs/Image)

The processing node consumes synchronized stereo frames and produces a depth or disparity map.

---

# 📁 Directory Structure

task_006_stereo_depth_map/
│── metadata.json  
│── Dockerfile  
│── setup.sh  
│── ros1_code/
│   ├── stereo_subscriber.py      ← contains **one TODO**
│   ├── depth_proc_node.py        ← contains **one TODO**
│   └── launch/
│       └── stereo_depth.launch
│── tests/

---

# 🧩 TODO Description

### 1. stereo_subscriber.py  
Goal:  
Subscribe to `/left/image_raw` and `/right/image_raw`.

### 2. depth_proc_node.py  
Goal:  
- Subscribe to both stereo topics  
- Compute depth map  
- Publish on `/depth/image`  


---

# ⭐ Expected ROS2 Outcome (Brief)

A correct ROS2 rewrite should:

1. Use `rclpy` and `image_transport`/QoS for sensor data.
2. Subscribe to `/left/image_raw` and `/right/image_raw` using `sensor_msgs.msg.Image`.
3. Synchronize messages using `message_filters.ApproximateTimeSynchronizer`.
4. Compute depth or disparity map using stereo vision algorithms (OpenCV or similar).
5. Publish result to `/depth/image`.
6. Use ROS2 node lifecycle (`rclpy.init()`, `Node`, `rclpy.spin()`).

---

# 🐳 Docker Usage

### Build:
ocker build -t task006_stereo .


### Run:


docker run -it --net=host task006_stereo


### Inside:


roslaunch stereo_task stereo_depth.launch
